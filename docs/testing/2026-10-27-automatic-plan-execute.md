# Seguimiento con planificación y ejecución automáticas

El marcador de seguimiento y el deslizador de pinza envían objetivos MoveGroup
con `plan_only=false`. Se mantienen activos los controladores de trayectoria.
Se ejecuta una acción por vez y se conserva el último objetivo pendiente por
grupo. Cada plan usa el estado actual (`start_state.is_diff=true`). La activación
no envía objetivos ni posiciones; solo crea el marcador en la postura medida.

En el lanzamiento integrado la entrada `/mycobot_live/move_action` comparte el
planificador interno, destino, sesión y factores físicos del coordinador. Los
botones manuales de Plan/Execute se rechazan durante seguimiento, y una petición
automática tardía tras detener se rechaza antes de reservar el destino. En modo
independiente se usa `/move_action`. Se conserva la validación de arrastres y
poses finitas. No se usa Servo ni la salida física de posiciones directas.

Detener o perder heartbeat/mediciones cancela la acción y descarta pendientes.
La reactivación espera el resultado terminal, incluso si el servidor acepta el
objetivo después de haber pulsado Detener. Los errores de planificación no se
reintentan indefinidamente. Un arrastre continuo empieza a planificar sin esperar
a soltar el ratón; el movimiento sigue trayectorias sucesivas.

## Verificación

- Compilación ROS Humble: [registro](automatic-plan-execute/2026-10-27/build.log).
- Suite Python completa: **189 pruebas pasaron**, con el aviso heredado de xacro:
  [registro](automatic-plan-execute/2026-10-27/pytest.log).
- Pruebas C++/Qt: **2 pasaron**, sin fallos. El panel de prueba emite un aviso de
  respuesta tardía al destruir el cliente, sin hacer fallar la prueba.
- Se observó primero el fallo del backend anterior al intentar activar con
  controladores de trayectoria activos. La regresión de arrastre continuo
  también falló antes de sustituir la espera de pausa por limitación de envíos.
- Doce regresiones ROS prueban el backend real con una acción MoveGroup
  controlada: activación inmóvil, plan automático, consolidación de objetivos,
  pinza independiente, parada/heartbeat/mediciones, error de planificación,
  aceptación tardía, arrastre continuo, poses no autorizadas o inválidas,
  reactivación, variante sin pinza y selección física sin USB.
- Gazebo + MoveIt + ControlCoordinator reales, sin RViz ni driver físico:
  se preparó una postura doblada, se activó sin movimiento, se arrastró mediante
  feedback ROS, se comprobó movimiento automático de brazo y pinza y rechazo de
  una petición manual mientras seguimiento estaba activo. Resultado:
  [JSON](automatic-plan-execute/2026-10-27/gazebo-result.json).

Comandos principales, dentro de un contenedor con el workspace instalado:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select mycobot_280_live_control
ROS_DOMAIN_ID=83 python3 -m pytest -q src/mycobot_ros2_packages/mycobot_280
ROS_DOMAIN_ID=85 colcon test --packages-select mycobot_280_live_control
colcon test-result --verbose
```

La integración de Gazebo se ejecuta en otro dominio aislado con
`ros2 launch mycobot_280_hardware control.launch.py adaptive_gripper:=true
launch_driver:=false use_rviz:=false gui:=false`, seguida de
`python3 src/mycobot_ros2_packages/mycobot_280/mycobot_280_live_control/test/automatic_execution_integration.py`
usando ese mismo dominio.

## Límites de lo validado

No se movió el manipulador físico: `/dev/ttyACM0` no está presente y el contenedor
habitual no puede arrancar con ese dispositivo ausente. Se utilizó un contenedor
de pruebas sin USB, con la misma imagen y workspace compartido. La selección
física se probó con un servidor de acción controlado; esto no acredita movimiento
real. Tampoco se automatizaron gestos en la ventana RViz: se ejercitó su protocolo
de feedback de marcadores ROS y las pruebas del panel Qt.

Durante la limpieza de ambos lanzamientos se observó una caída de `move_group`
en la destrucción de `rclcpp::CallbackGroup`, después de ejecutar correctamente
las trayectorias. El nodo de seguimiento terminó con código 0. El fallo de cierre
no se corrigió en este cambio; hay antecedentes del mismo patrón de descarga de
bibliotecas en la documentación del 26/10.
