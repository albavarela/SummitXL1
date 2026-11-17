# Seguimiento de personas

`roslaunch piernas follow.launch` utiliza el detector 2D. El seguimiento
requiere una transformacion TF valida entre `robot_odom` y `robot_base_link`.
Los frames se pueden cambiar con `tracking_frame:=...` y `base_frame:=...`.

El adaptador recibe las personas detectadas por `leg_detector` en
`/people_tracker_measurements`. Solo se usan detecciones que alcanzan el
umbral de fiabilidad del adaptador (por defecto 0,1). El detector puede
confundir objetos o piernas de personas diferentes con una persona.

El adaptador ignora durante el resto de la ejecucion las IDs que se desplacen
menos de 20 cm durante 8 s. Una ID nueva se trata como otra persona, aunque
este cerca de una deteccion anterior; tambien puede volver a seleccionarse
una persona ignorada si el detector le asigna una ID nueva.

Al arrancar o si se pierde una deteccion, el robot puede girar si no detecta
una persona. Despejar la zona y tener disponible la parada de emergencia
antes de ejecutar el launch.
