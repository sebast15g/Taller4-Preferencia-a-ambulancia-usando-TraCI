#!/usr/bin/env python3
import os
import subprocess
import json
import csv
import random
from pathlib import Path
from collections import defaultdict

SIM_SCRIPT = "/root/omnet/taller3/retot3_trac_v2.py"
METRICS_FILE = "/root/omnet/taller3/metrics.json"
CSV_FILE = "/root/omnet/taller3/benchmark_results.csv"
SEED_MANUAL = "1776857298"  

def run_simulation(mode_name, use_gui, enable_tls, enable_coop, enable_teleport=False, seed=None):
    """Ejecuta el script de simulación inyectando variables de entorno"""
    env = os.environ.copy()
    env['AMB_SIM_MODE'] = mode_name
    env['AMB_USE_GUI'] = 'true' if use_gui else 'false'
    env['AMB_ENABLE_TLS_CONTROL'] = 'true' if enable_tls else 'false'
    env['AMB_ENABLE_VEHICLE_COOPERATION'] = 'true' if enable_coop else 'false'
    env['AMB_ENABLE_TELEPORT'] = 'true' if enable_teleport else 'false'
    env['AMB_OUTPUT_METRICS'] = 'true'
    env['AMB_METRICS_FILE'] = METRICS_FILE
    
    if seed:
        env['AMB_SEED'] = str(seed)
    elif 'AMB_SEED' in env:
        del env['AMB_SEED']

    print(f"\n[{mode_name}] Iniciando simulación...")
    print(f"  -> GUI: {use_gui} | TLS: {enable_tls} | COOP: {enable_coop} | SEED: {seed or 'Random'}")
    
    try:
        # Silenciamos la salida estándar en el benchmark masivo para no saturar la consola
        stdout_dest = subprocess.DEVNULL if not use_gui else None
        subprocess.run(["python3", SIM_SCRIPT], env=env, stdout=stdout_dest)
    except KeyboardInterrupt:
        print(f"\n[!] Simulación {mode_name} abortada por el usuario.")
    
    if Path(METRICS_FILE).exists():
        with open(METRICS_FILE, 'r') as f:
            return json.load(f)
    return None

def save_to_csv(results, filepath):
    """Guarda los resultados crudos en un archivo CSV para graficar después"""
    if not results:
        return
    # Definimos las columnas que nos interesan para el CSV
    fieldnames = ["run_id", "random_seed", "mode", "ambulance_arrived", "response_time", "total_tls_preemptions", "recovery_level"]
    
    try:
        with open(filepath, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            for row in results:
                writer.writerow(row)
        print(f"\n[📁 CSV EXPORTADO] Datos crudos guardados exitosamente en: {filepath}")
    except Exception as e:
        print(f"\n[ERROR] No se pudo guardar el CSV: {e}")

def print_aggregated_metrics(results, n_runs):
    """Calcula y muestra los promedios de múltiples ejecuciones"""
    print("\n" + "="*85)
    print(f" 📊 RESULTADOS AGREGADOS (BENCHMARK - {n_runs} iteraciones) ".center(85))
    print("="*85)
    print(f"{'MODO':<25} | {'LLEGADAS':<10} | {'T. PROM RESPUESTA (s)':<22} | {'PROMEDIO SEMÁFOROS':<18}")
    print("-" * 85)

    grouped = defaultdict(list)
    for r in results:
        grouped[r['mode']].append(r)

    for mode in sorted(grouped.keys()):
        runs = grouped[mode]
        arrivals = sum(1 for r in runs if r.get('ambulance_arrived'))
        times = [r.get('response_time') for r in runs if r.get('ambulance_arrived') and r.get('response_time') is not None]
        
        avg_time = sum(times) / len(times) if times else 0.0
        avg_tls = sum(r.get('total_tls_preemptions', 0) for r in runs) / len(runs)
        
        t_resp_str = f"{avg_time:.1f}" if times else "N/A"
        print(f"{mode:<25} | {arrivals:>4}/{len(runs):<5} | {t_resp_str:<22} | {avg_tls:<18.1f}")
    print("="*85 + "\n")

def menu():
    while True:
        try:
            print("\n" + "="*50)
            print("=== MENÚ DE PRUEBAS DE AMBULANCIA ===".center(50))
            print("="*50)
            print("1. [Métricas] Benchmark Headless MÚLTIPLE (CSV export)")
            print("-" * 50)
            print("  --- GUI CON SEMILLA MANUAL ---")
            print("2. [GUI] Sin control ni coop")
            print("3. [GUI] SOLO control de Semáforos")
            print("4. [GUI] SOLO Cooperación Vehicular (con teleport)")
            print("5. [GUI] CON control y coop (Ambos)")
            print("-" * 50)
            print("  --- GUI CON SEMILLA ALEATORIA ---")
            print("6. [GUI] Sin control ni coop")
            print("7. [GUI] CON control y coop (Ambos)")
            print("0. Salir")
            
            opc = input("\nElige una opción: ")
            
            if opc == "1":
                iter_input = input("¿Cuántas simulaciones por modo quieres ejecutar? (ej. 10): ")
                try:
                    n_runs = int(iter_input)
                except ValueError:
                    print("Entrada inválida. Usando 3 simulaciones por defecto.")
                    n_runs = 3

                print(f"\n🚀 Ejecutando {n_runs} ciclos en segundo plano. Esto puede tardar un poco...")
                
                # Generamos N semillas aleatorias distintas para que todos los modos corran en las mismas N condiciones
                seeds = [random.randint(100000, 999999999) for _ in range(n_runs)]
                
                modos = [
                    ("1_SIN_NADA", False, False, False),       
                    ("2_SOLO_SEMAFOROS", True, False, False),  
                    ("3_SOLO_COOP_VEH", False, True, True),    
                    ("4_AMBOS_CONTROLES", True, True, True)    
                ]
                
                all_results = []
                for idx, current_seed in enumerate(seeds):
                    print(f"\n--- CICLO {idx+1}/{n_runs} (Semilla: {current_seed}) ---")
                    for nombre, tls, coop, teleport in modos:
                        res = run_simulation(nombre, use_gui=False, enable_tls=tls, enable_coop=coop, enable_teleport=teleport, seed=current_seed)
                        if res:
                            res['run_id'] = idx + 1
                            all_results.append(res)
                
                # Al terminar todos los ciclos, guardamos y mostramos la tabla
                save_to_csv(all_results, CSV_FILE)
                print_aggregated_metrics(all_results, n_runs)

            elif opc == "2":
                run_simulation("GUI_MANUAL_SIN_CONTROL", use_gui=True, enable_tls=False, enable_coop=False, enable_teleport=False, seed=SEED_MANUAL)
            elif opc == "3":
                run_simulation("GUI_MANUAL_SOLO_SEMAFOROS", use_gui=True, enable_tls=True, enable_coop=False, enable_teleport=False, seed=SEED_MANUAL)
            elif opc == "4":
                run_simulation("GUI_MANUAL_SOLO_COOP", use_gui=True, enable_tls=False, enable_coop=True, enable_teleport=True, seed=SEED_MANUAL)
            elif opc == "5":
                run_simulation("GUI_MANUAL_AMBOS_CONTROLES", use_gui=True, enable_tls=True, enable_coop=True, enable_teleport=True, seed=SEED_MANUAL)
            elif opc == "6":
                run_simulation("GUI_RANDOM_SIN_CONTROL", use_gui=True, enable_tls=False, enable_coop=False, enable_teleport=False, seed=None)
            elif opc == "7":
                run_simulation("GUI_RANDOM_AMBOS_CONTROLES", use_gui=True, enable_tls=True, enable_coop=True, enable_teleport=True, seed=None)
            elif opc == "0":
                print("Saliendo de las pruebas... ¡Éxito con esas métricas!")
                break
            else:
                print("Opción no válida. Intenta de nuevo.")
                
        except KeyboardInterrupt:
            print("\n\n[!] Ejecución cancelada (Ctrl+C). ¡Saliendo limpiamente!")
            break
        except EOFError:
            break

if __name__ == "__main__":
    menu()