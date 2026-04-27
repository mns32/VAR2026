#!/usr/bin/env python3
"""
run_best – Ejecuta el wall-follower con el mejor genoma encontrado por el GA.

Carga el archivo best_genome.json que genera ga_train.py y lanza el controlador
con esos parámetros. Este es el comando que usarás para grabar el vídeo final
de la práctica.

Uso:
  ros2 run nav_genetic run_best
  ros2 run nav_genetic run_best --ros-args -p genome_file:=/ruta/a/best_genome.json
"""
import json
import os
import sys

import rclpy
from nav_genetic.controller import WallFollower, DEFAULT_GENOME


def find_genome_file():
    # 1) Si el primer argumento es una ruta a un .json, úsala
    for arg in sys.argv[1:]:
        if arg.endswith('.json') and os.path.isfile(arg):
            return arg
    # 2) Buscar en CWD
    for cand in ['best_genome.json', '/home/ros2_ws/best_genome.json']:
        if os.path.isfile(cand):
            return cand
    return None


def main(args=None):
    rclpy.init(args=args)

    path = find_genome_file()
    if path:
        with open(path) as f:
            data = json.load(f)
        genome = data.get('genome', data)
        print(f">>> Usando genoma de {path}")
        print(f"    fitness={data.get('fitness', '?')}, generación={data.get('generation', '?')}")
    else:
        genome = dict(DEFAULT_GENOME)
        print(">>> No se encontró best_genome.json; usando parámetros por defecto.")

    print(f"    genoma = {genome}")

    node = WallFollower(genome=genome)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
