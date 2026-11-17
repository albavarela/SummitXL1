# -*- coding: utf-8 -*-
from __future__ import division
import time
import numpy as np

# Pare efectos prácticos haremos dos situaciones, una donde el obstáculo esté de frente (avoid obstacle) 
# otra para que el obstáculo este al lado (tomamos que es un pared)

class Movimiento(object):
    def __init__(self, timeout=0.5, dist_deseada=1.0, kp_lin=0.5, kp_ang=1.0):
        self.timeout = timeout
        self.dist_deseada = dist_deseada
        self.kp_lin = kp_lin
        self.kp_ang = kp_ang
        self._objetivo = None   # (position, angle, t)

    # Definición de movimiento, simplemente se detiene
    def stop(self):
        return 0.0, 0.0,  0.0

    # Definición de búsqueda, simplemente gira
    def search(self, sentido):
        return 0.0, 0.0, sentido * 0.5
 
    def follow(self, position, angle=None, obstacle_distance=None, sentido_escape=1, distancia_pared=None,
            umbral=1.0):
        """Sigue position=(x, y) respecto al robot y mantiene dist_deseada.

        angle se conserva por compatibilidad, pero no se usa: la dirección se
        calcula desde la posición observada de la persona.
        """
        #if self.objetivo_caducado():
        #   return self.stop()

        distancia = np.hypot(position[0], position[1])
        rumbo = np.arctan2(position[1], position[0])
        error_distancia = distancia - self.dist_deseada

        # 1. Gira hacia la persona y solo avanza si está más lejos que la distancia deseada.
        vx = vz = 0.0
        vz = self.kp_ang * rumbo
        if error_distancia > 0.1:
            vx = self.kp_lin * error_distancia * max(0.0, np.cos(rumbo))

        # 2. Obstáculo de frente: frena y rodea en lateral
        if obstacle_distance is not None and 0.0 <= obstacle_distance < umbral:
            vx *= max(0.0, obstacle_distance / umbral)
            vz = 0.5 * sentido_escape

        # 3. Pared lateral: gira para apartarse (el lado lo da sentido_escape)
        if (distancia_pared is not None
                and 0.0 <= distancia_pared < umbral):
            vz = 0.5 * sentido_escape

        return (float(np.clip(vx, -0.2, 1.0)),
                float(np.clip(vz, -0.5, 0.5)))