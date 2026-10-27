# Corrección del seguimiento con la pinza abierta — 2026-10-23

Se reprodujo el bloqueo del brazo al activar seguimiento con el gripper en la
apertura máxima. Gazebo medía 0,1500000038 rad; al inicializar la consigna con
ese valor, la validación estricta de límites del estado completo rechazaba
los comandos del brazo. La comprobación de colisiones no encontró contactos.

La consigna inicial normaliza a los límites exactos solamente errores de hasta
1e-6 rad alrededor de los extremos. La medida original sigue visible, los
objetivos mantienen el rango [-0,5; 0,15] rad y los valores claramente fuera
de rango siguen siendo inválidos. No se amplió el rango de movimiento.

También se oculta Query Goal State de MotionPlanning mientras el seguimiento
está activo. Así queda el marcador etiquetado Seguimiento en vivo sin el objetivo
naranja superpuesto. Al detener, se restaura la preferencia anterior, incluyendo
el caso en que el usuario ya lo tenía oculto. Se usan QPointer para soportar la
eliminación del display y pruebas Qt de transiciones y restauración.

Verificación:

- Prueba numérica RED por función ausente y GREEN tras implementar normalización.
- Prueba de propiedad RViz RED por guardia ausente y GREEN tras implementarla.
- Reproducción en Gazebo antes: primer joint se desplazaba solo 1,8e-7 rad y
  aparecía Detenido por colisión o límite.
- Regresión después: cambio máximo medido en articulaciones del brazo de
  0,01984 rad, error FK final 0,0002532 m y estado Seguimiento activo, con la
  pinza medida en 0,1500000038 rad.
- La regresión usa un objetivo cartesiano generado por FK; mide todo el brazo
  y la herramienta, pues IK puede repartir el movimiento entre articulaciones.
- Dos CTest y 96 tests funcionales pytest pasan; no se ejecutaron de nuevo los
  tres linters heredados, cuyos fallos están registrados en la validación anterior.
- Verificación visual en RViz: objetivo naranja oculto al activar, marcador
  propio visible y postura del brazo doblada. Aplicación recargada en dominio 42.

Reproducción con Gazebo/RViz y brazo doblado, gripper en apertura máxima:

```bash
python3 /root/mycobot_ros2_packages/mycobot_280/mycobot_280_live_control/test/gripper_open_arm_follow.py
```

El test manda un objetivo vivo, verifica los estados y deja seguimiento detenido.
