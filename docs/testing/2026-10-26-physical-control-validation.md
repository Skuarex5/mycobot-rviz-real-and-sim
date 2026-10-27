# Validación con el manipulador real — 26/10/2026

Pruebas ejecutadas en el myCobot 280 M5 de este equipo, con pinza adaptativa
original conectada al Atom. El operador confirmó pinza vacía, brazo firme y zona
despejada. Se usó un único propietario USB y las acciones y servicios públicos
de la sesión integrada. No se flasheó firmware, calibraron ceros ni liberaron
servos.

## Fallo reproducido y corrección

Al mover la pinza en vivo, GET_ANGLES dejaba de responder y vencía la medición
completa tras 0.8 s. El coordinador registró `state_age=0.805`, mientras el
heartbeat del panel y el estado del controlador seguían recientes. El backend
publicó «Se perdió el estado medido» y se verificó la parada física.

Las trazas USB mostraron respuestas vacías después de SET_GRIPPER_VALUE. Probar
dos argumentos, cuatro argumentos o la espera original del SDK tampoco resolvió
el corte. Consultar GET_GRIPPER_VALUE antes de GET_ANGLES mantuvo las respuestas
y permitió mover la pinza. Esto es un comportamiento observado en esta unidad;
no demuestra cuál es el defecto interno del firmware.

El transporte definitivo marca pendiente la lectura después de cada escritura
asíncrona de pinza. Antes de la siguiente lectura del brazo consulta y valida
la posición de pinza, manteniendo el bloqueo USB. La consulta posterior de estado
sigue obteniendo mediciones reales; no se renuevan marcas de tiempo con valores
inventados. Las comprobaciones de parada y vigilancia permanecen activas.

Las cuatro regresiones nuevas fallaron antes del cambio y pasaron después,
incluyendo lecturas ausentes, negativas y booleanas. La prueba física aislada
pasó primero con instrumentación y luego con el controlador normal, sin parches
temporales.

## Resultado completo con el controlador normal

Registro: [complete-validation.json](physical-control/2026-10-26/complete-validation.json).

| Comprobación | Resultado medido |
| --- | --- |
| Plan y Execute separados hacia postura flexionada | Pasó; error máximo 0.01782 rad, aproximadamente 1.02° |
| Plan & Execute con pequeño giro de base | Pasó; acción terminada correctamente |
| Seguimiento activado sin arrastrar | Cambio máximo 0.00314 rad, aproximadamente 0.18°, en 2 s |
| Marcador en vivo, objetivo base +0.08 rad | Cambio medido +0.06597 rad, aproximadamente 3.78° |
| Pinza en vivo, objetivo +0.06 rad | Cambio medido +0.04432 rad, sin perder sesión |
| Detener seguimiento y deseleccionar físico | Ambos servicios confirmaron éxito; destino final Gazebo |

El campo `live_max_command_lead` del JSON mide el objetivo IK en el canal ROS,
antes del limitador USB. No representa el adelanto de los pasos enviados al
firmware, limitado por el controlador a 0.03 rad.

La primera tentativa tras reiniciar no movió el robot porque aún faltaba cargar
el controlador de pinza de Gazebo. También se eliminó un coordinador huérfano del
diagnóstico anterior. La validación final se ejecutó tras confirmar un único
coordinador y un único controlador USB.

Prueba aislada: [gripper-live-validation.json](physical-control/2026-10-26/gripper-live-validation.json).
Traza diagnóstica, que incluye las variantes fallidas y la prueba instrumentada:
[usb-diagnostic-trace.jsonl](physical-control/2026-10-26/usb-diagnostic-trace.jsonl).

## Validación de software

- Python: **158 pasaron**, 3 linters heredados deseleccionados; aviso existente
  de xacro. [Salida completa](physical-control/2026-10-26/python-tests.log).
- C++/Qt: **2 de 2** pruebas CTest pasaron en dominio 48; Python usó dominio 49.
- En una ejecución anterior del 23/10 hubo un fallo de segmentación al destruir
  un ActionServer de rclpy durante pruebas concurrentes con compilación. Las
  ejecuciones completas posteriores, incluida la del 26/10, pasaron. Este dato
  no equivale a haber resuelto el fallo interno de rclpy.

## Repetición

Con una sola sesión integrada ya lista, en este equipo comprobado:

```bash
docker exec -it ros2-humble bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
python3 /root/mycobot_ros2_packages/tools/validate_physical_control.py \
  --execute-on-real-robot --report /tmp/physical-validation.json
```

El comando mueve el brazo hacia una postura flexionada, gira ligeramente la base
y abre un poco la pinza. Requiere las mismas condiciones físicas de esta prueba.
El script rechaza ejecutarse sin la opción explícita y termina desactivando el
seguimiento y seleccionando Gazebo, también ante fallos.

Son pruebas de movimientos pequeños, con pinza vacía. La tolerancia final local
es 0.04 rad, aproximadamente 2.3°; no se ha medido precisión cartesiana con un
instrumento externo. El extremo completamente abierto de la pinza sigue siendo
nominal. No se validaron agarres, carga, toda la zona de trabajo ni respuesta a
una desconexión física durante el movimiento.
