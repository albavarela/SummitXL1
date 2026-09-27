# SummitXL1 — Comportamiento Social de Seguimiento de Personas

Proyecto desarrollado por tres alumnos del Grado en Robótica de la Universidad de Santiago de Compostela (USC), en el marco de la asignatura **Interacción Persona-Robot**. El objetivo es programar un comportamiento social para el robot **SummitXL** capaz de detectar, seguir y reaccionar de forma segura ante la presencia de una persona.

## Índice
 
1. [Objetivos](#objetivos)
2. [Estructura del repositorio](#estructura-del-repositorio)
3. [Arquitectura del sistema](#arquitectura-del-sistema)
4. [Requisitos e instalación](#requisitos-e-instalación)
5. [Reparto de tareas](#reparto-de-tareas)
6. [Resultados y evaluación](#resultados-y-evaluación)
7. [Problemas encontrados y soluciones](#problemas-encontrados-y-soluciones)
8. [Multimedia](#multimedia)
9. [Referencias](#referencias)
10. [Autores](#autores)

---
 
## Objetivos
 
**Objetivo general:**
Desarrollar un comportamiento social de seguimiento de personas ("person following") para el robot SummitXL, que le permita detectar, seguir y reaccionar de forma segura ante la presencia de un humano.
 
**Objetivos específicos:**
- Detectar y seguir a una persona en tiempo real usando los sensores del robot (LiDAR 2D, 3D y cámara RGB-D).
- Diseñar un controlador de movimiento que mantenga una distancia y orientación social adecuadas respecto a la persona.
- Implementar una máquina de estados robusta que gestione los distintos modos de comportamiento.
- Validar el sistema completo en simulación antes de probarlo en el robot real.
- Evaluar el comportamiento social del robot según criterios propios de la Interacción Persona-Robot.
---

## Estructura del repositorio


- `Software/`: Donde se encuentra toda la programación del proyecto.
- `Simulador/`: Donde se encuentra el simulador del robot y como instalarlo.
- `README.md`: Fichero explicativo de todo el proyecto.


---
## Arquitectura del sistema
 
El sistema se divide en tres módulos independientes que se comunican mediante topics de ROS:
 
```
 ┌─────────────────┐                                   ┌──────────────────┐                     ┌────────────┐
 │   PERCEPCIÓN    │ ───────────────────────────────▶ |   CONTROL Y       │ ─────────────────▶ │  SummitXL  │
 │ (LiDAR, RGB-D)  │                                   │  MOVIMIENTO      │                     │  (robot)   │
 └─────────────────┘                                   └──────────────────┘                     └────────────┘
         ▲                                                       ▲
         │                                                       │
         └────────────────────────┬──────────────────────────────┘
                                  │
                         ┌──────────────────┐
                         │  INTEGRACIÓN     │
                         │  MÁQUINA DE      │
                         │  ESTADOS         │
                         └──────────────────┘
```
---
## Requisitos e instalación
 
**Requisitos:**
- Ubuntu `[completar versión]`
- ROS `[completar distro, p. ej. Noetic]`
- Gazebo `[completar versión]`
- Paquetes de Robotnik para el SummitXL: `[enlace al repositorio oficial]`
- Dependencias adicionales: `[OpenCV, PCL, modelo de detección de personas, etc.]`
 
Instrucciones detalladas del simulador disponibles en [`Simulador/`](./Simulador).

---

## Reparto de tareas
 
| Integrante | Módulo(s) asignado(s) | Tareas principales |
|---|---|---|
| `[Alba Varela]` | Percepción | `[detalle]` |
| `[Aarón Franco]` | Control y movimiento | `[detalle]` |
| `[Sofia Fernández]` | Integración, simulación y pruebas | `[detalle]` |
 
---
 
## Resultados y evaluación
 
Métricas consideradas para evaluar el comportamiento social del robot:
 
- Distancia media mantenida respecto a la persona:
- Tiempo de reacción ante pérdida/recuperación del objetivo.:
- Número de paradas de seguridad / colisiones evitadas:
- Suavidad de la trayectoria (variación de velocidad angular/lineal):
---
 
## Problemas encontrados y soluciones
 
`[Sin cubrir por ahora]`
 
---

 
## Multimedia
 
`[Sin cubrir por ahora]`

---
 
## Referencias
 
- Documentación oficial de Robotnik para el SummitXL: `[enlace]`
---
 
## Autores
 
- `[Alba Varela]` — Grado en Robótica, USC
- `[Sofia Fernández]` — Grado en Robótica, USC
- `[Aarón Franco]` — Grado en Robótica, USC
