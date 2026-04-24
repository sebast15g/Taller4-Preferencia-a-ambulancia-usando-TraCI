#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import os

# Configuración visual para que parezca de paper académico
sns.set_theme(style="whitegrid")
plt.rcParams.update({'font.size': 12, 'axes.titlesize': 14})

# Rutas de archivos
CSV_FILE = "/root/omnet/taller3/benchmark_results.csv"
OUTPUT_DIR = "/root/omnet/taller3/"

def generar_graficas():
    if not os.path.exists(CSV_FILE):
        print(f"Error: No se encuentra el archivo {CSV_FILE}.")
        return

    # Leer los datos
    df = pd.read_csv(CSV_FILE)

    # Filtrar solo los casos donde la ambulancia llegó
    df_llegadas = df[df['ambulance_arrived'] == True].copy()
    df_llegadas['mode'] = df_llegadas['mode'].str.replace('_', ' ').str.title()

    # =========================================================
    # 1. Gráfica de Barras: Promedio de Tiempo con IC explícito
    # =========================================================
    plt.figure(figsize=(10, 6))
    ax1 = sns.barplot(
        data=df_llegadas, x='mode', y='response_time', 
        palette='viridis', capsize=.1, errorbar=('ci', 95)
    )
    plt.title('Tiempo de Respuesta Promedio (con IC 95%)')
    plt.ylabel('Tiempo (segundos)')
    plt.xlabel('')
    plt.xticks(rotation=15)

    # Calcular estadísticas exactas para anotar
    stats = df_llegadas.groupby('mode')['response_time'].agg(['mean', 'std', 'count'])
    # Cálculo estándar del margen de error (aproximación Z=1.96 para 95% IC)
    stats['ci_margin'] = 1.96 * stats['std'] / np.sqrt(stats['count'])

    # Colocar los textos dentro de las barras
    for i, p in enumerate(ax1.patches):
        mode_name = ax1.get_xticklabels()[i].get_text()
        if mode_name in stats.index:
            mean_val = stats.loc[mode_name, 'mean']
            ci_val = stats.loc[mode_name, 'ci_margin']
            
            # Formatear el texto (Media ± Margen de Error)
            texto = f"{mean_val:.1f}s" if pd.isna(ci_val) else f"{mean_val:.1f}s\n(±{ci_val:.1f})"
            
            # Posicionar en el centro de la barra
            ax1.text(p.get_x() + p.get_width() / 2., p.get_height() / 2., texto, 
                     ha="center", va="center", color="white", fontweight="bold", 
                     bbox=dict(facecolor='black', alpha=0.5, edgecolor='none', boxstyle='round,pad=0.3'))

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "1_tiempo_respuesta.png"), dpi=300)
    plt.close()

    # =========================================================
    # 2. Boxplot: Distribución con Medianas anotadas
    # =========================================================
    plt.figure(figsize=(10, 6))
    ax2 = sns.boxplot(data=df_llegadas, x='mode', y='response_time', palette='Set2')
    sns.stripplot(data=df_llegadas, x='mode', y='response_time', color='black', alpha=0.5, jitter=True)
    plt.title('Distribución de Tiempos (Boxplot con Mediana Real)')
    plt.ylabel('Tiempo (segundos)')
    plt.xlabel('')
    plt.xticks(rotation=15)

    # Anotar el valor numérico de la Mediana
    medianas = df_llegadas.groupby('mode')['response_time'].median()
    for i, xtick in enumerate(ax2.get_xticks()):
        mode_name = ax2.get_xticklabels()[i].get_text()
        if mode_name in medianas.index:
            med_val = medianas[mode_name]
            ax2.text(xtick + 0.15, med_val, f'Med: {med_val:.1f}', 
                     ha='left', va='center', size='11', color='darkred', weight='semibold')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "2_distribucion_boxplot.png"), dpi=300)
    plt.close()

    # =========================================================
    # 3. Gráfica de Líneas: Evolución por Run (Semilla)
    # =========================================================
    plt.figure(figsize=(10, 6))
    ax3 = sns.lineplot(
        data=df_llegadas, x='run_id', y='response_time', hue='mode', 
        marker='o', linewidth=2.5, palette='tab10'
    )
    plt.title('Comparativa Directa por Escenario (Run)')
    plt.ylabel('Tiempo (segundos)')
    plt.xlabel('ID de Simulación (Diferentes condiciones de tráfico)')
    plt.xticks(df_llegadas['run_id'].unique())
    
    # Mover la leyenda afuera para que no tape los puntos
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "3_comparativa_escenarios.png"), dpi=300)
    plt.close()

    # =========================================================
    # 4. Gráfica de Barras: Uso de Semáforos Forzados
    # =========================================================
    plt.figure(figsize=(10, 6))
    ax4 = sns.barplot(
        data=df_llegadas, x='mode', y='total_tls_preemptions', 
        palette='magma', errorbar=None
    )
    plt.title('Intervenciones en Semáforos (Promedio por Modo)')
    plt.ylabel('Cantidad de Semáforos Modificados')
    plt.xlabel('')
    plt.xticks(rotation=15)

    # Anotar la altura exacta en la punta de cada barra
    for container in ax4.containers:
        ax4.bar_label(container, fmt='%.1f', padding=3, weight='bold', color='black')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "4_uso_semaforos.png"), dpi=300)
    plt.close()

    print("\n[ÉXITO] Las 4 gráficas han sido empaquetadas y guardadas individualmente en:")
    print(f" -> {OUTPUT_DIR}")

if __name__ == "__main__":
    generar_graficas()