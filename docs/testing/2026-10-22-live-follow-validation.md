# Validación del seguimiento en vivo — 2026-10-22

Implementación en ROS 2 Humble dentro de `ros2-humble`, Gazebo Classic y MoveIt
Servo 2.5.10. Pruebas de simulación aisladas en dominio 46 y puerto Gazebo 11366;
sesión interactiva en dominio 42. No se enviaron comandos al robot físico.

## Comprobaciones automatizadas

- Build de siete paquetes con `colcon build --symlink-install --packages-up-to
  mycobot_280_hardware mycobot_280_pick_place_demo`.
- 96 pruebas funcionales pytest pasan (`python3 -m pytest -q -m 'not linter'`).
- Dos pruebas CTest pasan: sesión de control y panel Qt con ROS real en modo
  offscreen. Incluyen rangos, watchdog monotónico, estados de transición,
  rechazo del servicio sin activar el botón, slider oculto, ausencia de comandos
  al inicializarlo, publicación por interacción, apertura máxima exacta y cierre
  con solicitud pendiente.
- Los tres linters heredados de `mycobot_description/test` siguen fallando:
  `test_copyright`, `test_flake8`, `test_pep257`. Se ejecutaron por separado;
  no se incluyen en los 96 tests funcionales.

## Integración con Gazebo

El script reproducible es
`mycobot_280/mycobot_280_live_control/test/live_follow_integration.py`.
Necesita Gazebo y el launch de MoveIt con `use_rviz:=false`, pues un panel RViz
abierto seguiría enviando heartbeat durante la prueba de desconexión:

```bash
export ROS_DOMAIN_ID=46
export GAZEBO_MASTER_URI=http://127.0.0.1:11366
# En terminales separadas, con el entorno ROS/workspace cargado:
ros2 launch mycobot_280_gazebo gazebo.launch.py adaptive_gripper:=true gui:=false
ros2 launch mycobot_280_moveit2 moveit_gazebo.launch.py adaptive_gripper:=true use_rviz:=false
python3 /root/mycobot_ros2_packages/mycobot_280/mycobot_280_live_control/test/live_follow_integration.py --adaptive-gripper true
```

Repetir los tres últimos comandos con `false` prueba el brazo sin gripper.
El script prepara una postura doblada mediante la acción de trayectoria actual;
los diez movimientos posteriores se generan por feedback del marcador en vivo,
sin solicitar planificación. Verifica movimiento mediante `/joint_states` y FK.

Resultados finales con la misma versión del backend:

| Variante | Salida de órdenes | Error final de posición | Primera respuesta, diez objetivos |
| --- | --- | --- | --- |
| Con gripper | 50,00 Hz | 0,170 mm | 31–74 ms |
| Sin gripper | 49,99 Hz | 0,102 mm | 30–96 ms |

La primera consigna estacionaria también debe producir movimiento: la prueba no
se limita a comprobar las siguientes actualizaciones del marcador. Las entradas
con NaN o quaternion nulo se ignoran. Se prueban los dos extremos de apertura y
las cinco articulaciones mimic del gripper. Tras un objetivo inalcanzable,
reactivar conserva la postura medida (variación permitida <0,003 rad).

También se verifican ausencia de salto al activar, exclusión de los controladores,
rechazo de activación durante trayectoria, rechazo de acciones mientras está
activo, parada por pérdida de heartbeat/estados y por pausa de Gazebo,
recuperación de trayectoria, y fallo de activación al descargar un controlador
con recuperación al recargarlo. Con gripper se introduce un obstáculo en la
escena de MoveIt y se comprueba que la pinza conserva la posición hasta retirarlo.

## Ajustes derivados de mediciones

El Jacobiano mezcla metros y radianes: una postura doblada válida del myCobot
presenta condición ~72,5, y la postura recta ~1,7×10¹². Los umbrales de ejemplo
Panda 17/30 bloqueaban este brazo pequeño; se usan 100/200 para reducir velocidad
y detener, respectivamente. La postura recta sigue siendo singular. Las dos
referencias `ee_frame_name` y `robot_link_command_frame` deben apuntar a la
herramienta: PoseTracking Humble usa la segunda para medir su error.

El gripper tiene una separación normal de 1,958 mm entre dos bielas. El margen de
proximidad propia es 1 mm; el margen frente a escena sigue en 20 mm. No se
modificaron las exclusiones del SRDF ni las máscaras de contacto de Gazebo.
Además del control Servo se valida el estado combinado brazo/gripper antes de
publicarlo. El puente limita pasos articulares y cartesianos; el gripper usa una
rampa de posición.

## Límites de estas mediciones

Las latencias y errores corresponden a estos objetivos y esta carga del equipo;
no garantizan seguimiento de todas las poses, tiempo real ni precisión física.
Servo detiene o reduce velocidad ante singularidades, obstáculos y límites,
y no calcula rutas para rodearlos. La vigilancia se basa en tiempo monotónico.
Si Gazebo permanece pausado, puede detenerse la salida sin poder confirmar aún
el cambio de controladores; hay que reanudarlo y recuperar el modo.

La exactitud de masas, rozamiento, flexibilidad, agarre y calibración física sigue
fuera de esta prueba. El modo continuo solo se incorpora al launch de Gazebo.

## Inspección de RViz y entrega local

El panel se cargó en RViz real. La captura [live-follow-rviz.png](live-follow-rviz.png)
muestra el botón, el slider y el marcador etiquetado «Seguimiento en vivo» sobre
la herramienta, con el modo activo. Se verificó una suscripción de RViz al topic
`/mycobot_live/marker/update`. Humble configura este display con **Interactive
Markers Namespace**, no con la propiedad **Update Topic** de otras versiones.
La variante sin gripper oculta el slider, comprobado en Qt y en RViz.

En la sesión del dominio 42 se activó y desactivó el seguimiento y luego se
completó una trayectoria mediante la acción existente. Se preparó una postura
con el codo doblado para facilitar la primera prueba manual. La entrega deja
el seguimiento desactivado y los controladores de trayectoria activos.

La pausa de Servo en Humble conserva su última consigna de posición. Al activar
se reinicia Servo desde el estado medido y el puente mantiene esa postura hasta
recibir un objetivo válido del usuario. El trabajador vuelve a comprobar el
error medido aunque PoseTracking devuelva SUCCESS; esto evita perder un objetivo
si su suscripción todavía estaba procesando la pose anterior.
