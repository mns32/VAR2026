#!/usr/bin/env python3
"""
ga_train – Entrenamiento por Algoritmo Genético del controlador wall-follower.

GENOMA (6 genes reales): target_distance, kp, ki, kd, linear_speed, max_angular.
FITNESS: distancia recorrida + bonus por vuelta − penalización por colisiones.

EJECUCIÓN:
  Asegúrate de que la simulación está corriendo (./run_practica2.sh).
  Luego, en la pestaña 'Trabajo':
      cd /home/ros2_ws && colcon build --packages-select nav_genetic && \\
      source install/setup.bash && \\
      ros2 run nav_genetic ga_train

El mejor genoma encontrado se va guardando en best_genome.json en la carpeta
desde la que se ejecute (típicamente /home/ros2_ws).
"""
import json
import math
import os
import random
import subprocess
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from nav_genetic.controller import WallFollower
from nav_genetic.monitor    import TrialMonitor


# ─── Hiperparámetros del GA (tunear si se quiere entrenar más/menos) ────────
POP_SIZE        = 10
N_GENERATIONS   = 12
EVAL_SECONDS    = 30.0
ELITE           = 2
TOURNAMENT_K    = 3
MUTATION_RATE   = 0.30
MUTATION_SIGMA  = 0.20    # fracción del rango del gen
SEED            = 42

# Límites de cada gen (búsqueda del GA dentro de estos rangos)
BOUNDS = {
    'target_distance': (0.25, 0.70),
    'kp':              (1.0, 10.0),
    'ki':              (0.0,  2.0),
    'kd':              (0.0,  4.0),
    'linear_speed':    (0.12, 0.26),
    'max_angular':     (1.0,  2.5),
}
KEYS = list(BOUNDS.keys())

# Pose de spawn del robot (debe coincidir con el launch)
SPAWN_X, SPAWN_Y, SPAWN_YAW = 9.05, 9.00, -3.11
WORLD_NAME = 'default'    # nombre del <world> en race.sdf

OUTPUT_FILE = 'best_genome.json'
LOG_FILE    = 'ga_log.csv'


# ─── Utilidades del GA ───────────────────────────────────────────────────────
def random_genome():
    return {k: random.uniform(*BOUNDS[k]) for k in KEYS}

def crossover(a, b):
    """Cruce uniforme: cada gen viene aleatoriamente de uno u otro padre."""
    return {k: random.choice([a[k], b[k]]) for k in KEYS}

def mutate(g):
    """Mutación gaussiana acotada al rango."""
    out = dict(g)
    for k in KEYS:
        if random.random() < MUTATION_RATE:
            lo, hi = BOUNDS[k]
            out[k] += random.gauss(0.0, MUTATION_SIGMA) * (hi - lo)
            out[k] = max(lo, min(hi, out[k]))
    return out

def tournament(pop, fits):
    idxs = random.sample(range(len(pop)), TOURNAMENT_K)
    winner = max(idxs, key=lambda i: fits[i])
    return dict(pop[winner])


# ─── Reset de la simulación: teletransportar el robot al spawn ──────────────
def teleport_robot():
    """
    Llama al servicio de Gazebo /world/<world>/set_pose para devolver el robot
    al punto de salida. Se hace por subprocess porque ros_gz no expone este
    servicio como servicio ROS por defecto.
    """
    qz = math.sin(SPAWN_YAW / 2.0)
    qw = math.cos(SPAWN_YAW / 2.0)
    req = (
        f'name: "turtlebot3", '
        f'position: {{x: {SPAWN_X}, y: {SPAWN_Y}, z: 0.05}}, '
        f'orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}'
    )
    cmd = [
        'gz', 'service',
        '-s', f'/world/{WORLD_NAME}/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req,
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, timeout=4)
        if r.returncode != 0:
            print(f"  [teleport] gz devolvió {r.returncode}: {r.stderr.decode().strip()}")
    except FileNotFoundError:
        print("  [teleport] AVISO: 'gz' no está en PATH; no se reseteará la posición")
    except subprocess.TimeoutExpired:
        print("  [teleport] AVISO: timeout llamando a gz service")


# ─── Una evaluación = un trial completo ──────────────────────────────────────
def evaluate(genome, follower, monitor, executor) -> tuple:
    # 1) Apagar controlador y parar el robot
    follower.set_enabled(False)
    follower.publish_zero()
    for _ in range(5):
        executor.spin_once(timeout_sec=0.05)

    # 2) Teleport al spawn
    teleport_robot()
    time.sleep(0.6)

    # 3) Drenar callbacks atrasados (mensajes pre-teleport en cola)
    drain_until = time.time() + 0.6
    while time.time() < drain_until:
        executor.spin_once(timeout_sec=0.02)

    # 4) Cargar nuevo genoma, resetear monitor y arrancar
    follower.update_genome(genome)
    monitor.reset()
    follower.set_enabled(True)

    # 5) Ejecutar el trial
    t_end = time.time() + EVAL_SECONDS
    while time.time() < t_end:
        executor.spin_once(timeout_sec=0.02)

    # 6) Parar y medir
    follower.set_enabled(False)
    follower.publish_zero()

    return monitor.fitness(EVAL_SECONDS), monitor.summary()


# ─── Programa principal ──────────────────────────────────────────────────────
def main():
    random.seed(SEED)
    rclpy.init()

    follower = WallFollower(genome={
        'target_distance': 0.40, 'kp': 4.0, 'ki': 0.0,
        'kd': 1.5, 'linear_speed': 0.20, 'max_angular': 1.5,
    })
    follower.set_enabled(False)
    monitor  = TrialMonitor()
    executor = SingleThreadedExecutor()
    executor.add_node(follower)
    executor.add_node(monitor)

    # Esperar a que lleguen los primeros /scan y /odom
    print(">>> Esperando datos de /odom y /scan ...")
    deadline = time.time() + 10.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if monitor.start_xy is not None:
            break

    # Inicializar log
    log_f = open(LOG_FILE, 'w')
    log_f.write('gen,ind,fitness,path,collisions,lap,'
                + ','.join(KEYS) + '\n')

    pop = [random_genome() for _ in range(POP_SIZE)]
    fits = [None] * POP_SIZE
    best_genome = None
    best_fit    = -float('inf')

    try:
        for gen in range(N_GENERATIONS):
            print(f"\n========= Generación {gen+1}/{N_GENERATIONS} =========")
            for i, g in enumerate(pop):
                if fits[i] is not None:
                    print(f"  [{i+1:2d}] (élite, fitness={fits[i]:.2f})")
                    continue
                print(f"  [{i+1:2d}] {{tgt={g['target_distance']:.2f} "
                      f"Kp={g['kp']:.2f} Ki={g['ki']:.2f} Kd={g['kd']:.2f} "
                      f"v={g['linear_speed']:.2f} ωmax={g['max_angular']:.2f}}}")
                fit, summ = evaluate(g, follower, monitor, executor)
                fits[i] = fit
                print(f"       → fitness={fit:.2f}  {summ}")

                row = [gen, i, f"{fit:.2f}",
                       f"{summ['path_length']:.2f}", summ['collisions'],
                       int(summ['lap_completed'])]
                row += [f"{g[k]:.4f}" for k in KEYS]
                log_f.write(','.join(map(str, row)) + '\n')
                log_f.flush()

                if fit > best_fit:
                    best_fit, best_genome = fit, dict(g)
                    with open(OUTPUT_FILE, 'w') as fh:
                        json.dump({'genome': best_genome,
                                   'fitness': best_fit,
                                   'generation': gen}, fh, indent=2)
                    print(f"       ★ nuevo mejor (guardado en {OUTPUT_FILE})")

            # Construir siguiente generación
            order = sorted(range(POP_SIZE), key=lambda i: fits[i], reverse=True)
            print(f"  · mejor de la gen: {fits[order[0]]:.2f}")
            print(f"  · mejor global:    {best_fit:.2f}")

            new_pop  = [dict(pop[i]) for i in order[:ELITE]]
            new_fits = [fits[i]      for i in order[:ELITE]]
            while len(new_pop) < POP_SIZE:
                a = tournament(pop, fits)
                b = tournament(pop, fits)
                new_pop.append(mutate(crossover(a, b)))
                new_fits.append(None)    # hay que evaluarlo
            pop, fits = new_pop, new_fits

    except KeyboardInterrupt:
        print("\n>>> Interrumpido por el usuario.")

    finally:
        log_f.close()
        follower.publish_zero()
        executor.shutdown()
        follower.destroy_node()
        monitor.destroy_node()
        rclpy.shutdown()
        print(f"\n=== ENTRENAMIENTO FINALIZADO ===")
        print(f"Mejor fitness: {best_fit:.2f}")
        print(f"Mejor genoma:  {best_genome}")
        print(f"Guardado en:   {os.path.abspath(OUTPUT_FILE)}")
        print(f"Log CSV:       {os.path.abspath(LOG_FILE)}")


if __name__ == '__main__':
    main()
