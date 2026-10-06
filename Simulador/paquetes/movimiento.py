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
        return 0.0, 0.0, 0.0

    # Definición de búsqueda, simplemente gira
    def search(self, sentido):
        return 0.0, 0.0, sentido * 0.5
 
    def follow(self,, position, angle, obstacle_distance=None, sentido_escape = 1, distancia_pared=None,
            umbral=1.0):
        """Devuelve (vx, vy, vz). sentido_escape: +1 izquierda, -1 derecha."""
        #if self.objetivo_caducado():
        #   return self.stop()

        error = np.hypot(position[0], position[1]) - self.dist_deseada

        # 1. Seguimiento base (locales, sin estado)
        vx = vy = vz = 0.0
        if abs(error) > 0.1 and abs(angle) < 40:
            vx = self.kp_lin * error * np.cos(np.radians(angle))
        if abs(angle) > 5:
            vz = self.kp_ang * np.radians(angle)

        # 2. Obstáculo de frente: frena y rodea en lateral
        if obstacle_distance is not None and obstacle_distance < umbral:
            vx *= obstacle_distance / umbral      # frena suave
            vy = 0.5 * sentido_escape

        # 3. Pared lateral: se aparta (el lado lo da sentido_escape)
        if distancia_pared is not None and distancia_pared < umbral:
            vy = 0.5 * sentido_escape

        return (float(np.clip(vx, -0.2, 1.0)),
                float(np.clip(vy, -0.5, 0.5)),
                float(np.clip(vz, -0.5, 0.5)))