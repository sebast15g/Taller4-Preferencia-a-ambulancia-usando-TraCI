# Taller 4: Preferencia a ambulancia usando TraCI

Este repositorio contiene el desarrollo de una simulación vehicular en SUMO controlada mediante TraCI, orientada a dar prioridad a una ambulancia dentro de una red urbana con tráfico saturado. El escenario representa una condición tipo hora pico, donde se genera un accidente aleatorio sobre un vehículo privado y se despacha una ambulancia hacia el lugar del incidente.

El sistema implementado permite comparar diferentes modos de control:

- Sin control ni cooperación.
- Solo prioridad semafórica.
- Solo cooperación vehicular.
- Prioridad semafórica y cooperación vehicular de forma conjunta.

El objetivo principal es evaluar si el control dinámico de semáforos y la cooperación de vehículos cercanos reducen el tiempo de respuesta de la ambulancia.

---

## Estructura del repositorio

```text
Taller4-Preferencia-a-ambulancia-usando-TraCI/
│
├── archivos_sumo/
│   ├── map5.osm
│   ├── map5.net.xml
│   ├── map5.sumocfg
│   ├── trafico_hora_pico.trips.xml
│   ├── trafico_hora_pico.rou.xml
│   └── vtypes_reto.add.xml
│
├── programas/
│   ├── retot3_trac_v2.py
│   ├── runner.py
│   └── scripts de análisis o graficación
│
├── resultados/
│   ├── benchmark_results.csv
│   ├── metrics.json
│   └── figuras generadas
│
└── README.md
```

## Comandos principales

### Generar la red SUMO desde OpenStreetMap

```bash
netconvert --osm-files map5.osm -o map5.net.xml \
  --geometry.remove \
  --junctions.join \
  --junctions.join-dist 5 \
  --no-internal-links \
  --tls.join \
  --tls.guess \
  --ramps.guess \
  --remove-edges.isolated \
  --edges.join \
  --roundabouts.guess
```

## Generación de demanda vehicular con `randomTrips.py`

La demanda vehicular del escenario se genera usando la herramienta `randomTrips.py` de SUMO. Esta herramienta crea viajes aleatorios sobre la red `map5.net.xml` y luego genera el archivo de rutas que será usado por la simulación.

Antes de ejecutar este comando, debe existir el archivo de red:

```text
map5.net.xml
```

En este archivo se define la distribución `trafico_hora_pico`, que incluye los tipos de vehículos usados en la simulación.

### Comando utilizado

```bash
python3 $SUMO_HOME/tools/randomTrips.py \
  -b 0 -e 3600 -p 1 \
  -n map5.net.xml \
  --trip-attributes 'type="trafico_hora_pico"' \
  --random-depart \
  --min-distance 300 \
  --fringe-factor 5 \
  -o trafico_hora_pico.trips.xml \
  --route-file trafico_hora_pico.rou.xml
```


## Scripts de simulación

Los scripts de Python usados para ejecutar la simulación se encuentran en la carpeta:

```text
programas/
```
Archivos principales:

programas/retot3_trac_v2.py
programas/runner.py
