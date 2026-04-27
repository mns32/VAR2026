# nav_genetic — Navegación con GA para la Práctica 2 VAR

Wall-follower con PID cuyos parámetros se **optimizan con un Algoritmo
Genético**. El controlador funciona solo (con valores por defecto razonables)
y el GA refina los 6 parámetros para que el robot complete la vuelta lo
más rápido posible sin chocar.

## Estructura

```txt
nav_genetic/
├── package.xml
├── setup.py / setup.cfg
├── resource/nav_genetic
└── nav_genetic/
    ├── controller.py   # WallFollower (PID + slowdown frontal)
    ├── monitor.py      # TrialMonitor (mide path/colisiones/vuelta)
    ├── ga_train.py     # Bucle GA (selección + cruce + mutación)
    └── run_best.py     # Ejecuta el mejor genoma encontrado
```

## Genoma evolucionado (6 genes reales)

| Gen | Significado | Rango |
|-----|-------------|-------|
| `target_distance` | Distancia objetivo a la pared derecha (m) | 0.25 – 0.70 |
| `kp` | Ganancia proporcional del PID | 1.0 – 10.0 |
| `ki` | Ganancia integral | 0.0 – 2.0 |
| `kd` | Ganancia derivativa | 0.0 – 4.0 |
| `linear_speed` | Velocidad lineal base (m/s) | 0.12 – 0.26 |
| `max_angular` | Velocidad angular máxima (rad/s) | 1.0 – 2.5 |

## Función fitness

```txt
fitness = path_length
        − 0.5 · colisiones
        − 5    si min_clearance < 0.10 m
        + 50   si completa vuelta
        + 2 · (tiempo_restante)   si completa vuelta
```

## Instalación

Dentro del contenedor (pestaña "Trabajo" de `run_practica2.sh`):

```bash
cd /home/ros2_ws
colcon build --packages-select nav_genetic --symlink-install
source install/setup.bash
```

## Uso

### A) Solo ver el wall-follower funcionar (sin GA)
```bash
ros2 run nav_genetic wall_follower
```

### B) Entrenar con GA
Con la simulación corriendo:
```bash
cd /home/ros2_ws
ros2 run nav_genetic ga_train
```
Genera `best_genome.json` y `ga_log.csv`. Tarda ≈ 60–80 min con los valores
por defecto (POP=10, GENS=12, EVAL=30s).

### C) Ejecutar el mejor genoma encontrado
```bash
ros2 run nav_genetic run_best
```

## Hiperparámetros del GA (en `ga_train.py`)

```python
POP_SIZE       = 10
N_GENERATIONS  = 12
EVAL_SECONDS   = 30.0
ELITE          = 2
TOURNAMENT_K   = 3
MUTATION_RATE  = 0.30
MUTATION_SIGMA = 0.20
```
