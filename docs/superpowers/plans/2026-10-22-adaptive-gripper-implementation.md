# Integración del gripper adaptativo — plan de implementación

> **Para el implementador:** ejecutar las tareas en orden con `superpowers:executing-plans`, o con `superpowers:subagent-driven-development` si el usuario elige delegación. Completar las verificaciones antes de declarar resultados.

**Objetivo:** controlar el myCobot 280 M5 con gripper adaptativo en Gazebo y MoveIt, y añadir su controlador físico sin movimientos al arrancar.
**Diseño aprobado:** `docs/superpowers/specs/2026-10-22-adaptive-gripper-design.md`.
**Arquitectura:** variante opcional del modelo existente; mismo contrato `FollowJointTrajectory` en simulación y hardware; USB propiedad exclusiva de `RobotController`.
**Entorno:** ROS 2 Humble en `ros2-humble`, workspace `/root/ros2_ws`. No hay hardware conectado.

## Decisiones y contratos comunes

- `adaptive_gripper:=false` mantiene el flujo actual. Al activarlo, todos los lanzamientos de Gazebo, MoveIt, controlador y monitor seleccionan la misma variante geométrica.
- Grupo del brazo: `arm_group`; grupo de pinza: `gripper_group`; articulación principal: `gripper_controller`.
- Acción de pinza: `/mycobot280m5_gripper_controller/follow_joint_trajectory`, con una sola articulación principal.
- El URDF original limita el principal a `[-0.74, 0.15]` rad. La intersección con los seguidores limita el movimiento compatible a `[-0.5, 0.15]` rad. Usar esta intersección, sin ampliar límites de seguidores.
- Estados de simulación: `open=0.15`, `closed=-0.5`. `closed` significa el cierre permitido por este modelo, no un cierre físico completo calibrado.
- La conversión nominal conserva los extremos originales: `-0.74 rad -> 0`, `0.15 rad -> 100`. No remapear automáticamente el rango reducido al recorrido físico completo: el mínimo compatible resulta aproximadamente 27/100. Hacer configurables ambos extremos originales y documentar su calibración pendiente.
- Límite de velocidad de pinza: `0.5 rad/s`; aceleración de planificación: `1.0 rad/s²`; velocidad del firmware: `15` inicialmente; tolerancia de llegada del principal: `0.03 rad`. Son valores iniciales de software, sujetos a validación física.
- Crear `gripper_tool_frame`, fijo a `gripper_base`, con origen nominal `xyz="0 0.06 0"`, `rpy="0 0 0"`; documentar que requiere ajuste respecto de la herramienta real. Mantener el grupo del brazo con sus seis actuadores y usar ese marco como referencia terminal en la variante.
- No permitir objetivos simultáneos de brazo y pinza en el controlador físico inicial. Reservar el mismo estado de ejecución bajo `_goal_lock` para ambos servidores.

## Atención especial durante la revisión

1. URDF mimic no basta para mover seguidores en Gazebo: comprobar el estado de los seis ejes de la pinza.
2. La conversión a valores 0–100 no debe convertir el cierre reducido en un cierre físico completo.
3. No publicar órdenes como si fueran mediciones; omitir estados inválidos y rechazar ejecución sin lectura reciente.
4. La parada del brazo no es la parada del gripper. Verificar `gripper_stop`, incluida su respuesta y posterior ausencia de movimiento.
5. Con `adaptive_gripper=false`, el nodo conserva su API de seis articulaciones y pasan las pruebas actuales.

## Tarea 1 — Modelo y recursos del gripper

**Archivos:**
- Crear `mycobot_description/urdf/adaptive_gripper/adaptive_gripper.xacro` y recuperar las siete mallas `.dae` del respaldo `/home/skuarex/backups/mycobot-unused-20261022-144828/mycobot_description/urdf/adaptive_gripper`.
- Modificar `mycobot_description/setup.py` para instalar esos recursos.
- Modificar `mycobot_280/mycobot_280_gazebo/urdf/mycobot_280_m5_gazebo.urdf.xacro` para añadir un argumento `adaptive_gripper` y la variante opcional.
- Crear `mycobot_280/mycobot_280_gazebo/test/test_adaptive_gripper_model.py`.

**Produce:** macro `adaptive_gripper(parent)` que conserva las transformaciones y nombres aportados, añade inercias, dinámica, herramienta y relaciones mimic. El modelo expandido conserva `name="mycobot_280_m5"` y la fijación al mundo.

- [x] Escribir pruebas que expandan ambas variantes con Xacro. Verificar ausencia de pinza en la original, siete links añadidos en la variante más el marco de herramienta, seis articulaciones móviles de pinza, un solo principal y recursos existentes.
- [x] Verificar numéricamente las relaciones mimic en ambos extremos y puntos intermedios; comprobar que ningún seguidor excede su límite. Verificar inercias positivas y velocidades no nulas.
- [x] Ejecutar las pruebas y confirmar que fallan antes de implementar.
- [x] Implementar el modelo, instalando solo los recursos necesarios. Etiquetar las masas/inercias nuevas como estimaciones; no sustituir las transformaciones del archivo del usuario ni modificar su archivo original.
- [x] Ejecutar de nuevo las pruebas en Humble y comprobar que pasan.

## Tarea 2 — Control de pinza en Gazebo

**Archivos:**
- Crear `mycobot_280/mycobot_280_gazebo/config/ros2_controllers_gripper.yaml`.
- Modificar `mycobot_280/mycobot_280_gazebo/launch/gazebo.launch.py` y el modelo de la tarea 1.
- Crear `mycobot_280/mycobot_280_gazebo/test/test_adaptive_gripper_controllers.py`.

**Consume:** variante de la tarea 1. **Produce:** `mycobot280m5_gripper_controller`, un `JointTrajectoryController` del principal; seguidores mimic en el hardware simulado y estados publicados por el único broadcaster.

- [x] Escribir pruebas de configuración: controlador de brazo intacto, pinza de una articulación y seguidores con multiplicadores correctos en `gazebo_ros2_control`.
- [x] Confirmar fallo, implementar selección de modelo/YAML durante evaluación del launch y añadir spawner de pinza condicionado a `adaptive_gripper=true`.
- [x] Compilar y lanzar una instancia de prueba aislada con dominio ROS y puerto maestro de Gazebo propios. No detener la instancia del usuario para estas pruebas.
- [x] Enviar objetivos de apertura y cierre por la acción de pinza. Exigir resultado exitoso, principal dentro de `0.03 rad` y seguidores acordes con mimic; comprobar un movimiento del brazo y ausencia de errores de recursos.
- [x] Repetir arranque sin gripper y verificar sus dos controladores originales.

## Tarea 3 — Planificación y selección de variante en MoveIt

**Archivos:**
- Crear `mycobot_280/mycobot_280_moveit2/config/mycobot_280_m5_gripper.srdf`, `joint_limits_gripper.yaml` y `moveit_controllers_gripper.yaml`.
- Modificar `mycobot_280/mycobot_280_moveit2/launch/moveit_gazebo.launch.py`.
- Crear `mycobot_280/mycobot_280_moveit2/test/test_adaptive_gripper_moveit.py`.

**Consume:** URDF opcional y nombres de acción de tareas 1–2. **Produce:** selección `adaptive_gripper`, `gripper_group`, poses `open`/`closed` y configuración compartible por hardware.

- [x] Probar coincidencia de robot/grupos/controladores, rango de pinza, herramienta y límites de brazo. La variante original debe seguir usando sus archivos actuales.
- [x] Confirmar fallo, implementar archivos y selección al evaluar el launch. No declarar KDL para la pinza, que no es una cadena de seis ejes.
- [x] Revisar colisiones del mecanismo y del montaje con la brida. Excluir solo pares justificados por contacto del mecanismo; mantener comprobaciones entre la pinza y el resto del brazo.
- [x] Con Gazebo de prueba, lanzar MoveIt y comprobar planificación y ejecución de `gripper_group` y `arm_group`. Verificar que un objetivo inválido se rechaza.

## Tarea 4 — Conversión, USB y ejecución física

**Archivos:**
- Crear `mycobot_280/mycobot_280_hardware/mycobot_280_hardware/gripper_conversion.py`.
- Crear `mycobot_280/mycobot_280_hardware/mycobot_280_hardware/gripper_execution.py`.
- Modificar `mycobot_280/mycobot_280_hardware/mycobot_280_hardware/robot_control_node.py` y `config/robot_control.yaml`.
- Crear `test/test_gripper_conversion.py` y `test/test_gripper_control_node.py` dentro de ese paquete; modificar `test/test_read_only.py` con una lista explícita de llamadas autorizadas, sin retirar las demás prohibiciones.

**Contratos:**
- `GripperCalibration`: dataclass inmutable con extremos nominales en radianes, valores del dispositivo y límites compatibles del modelo.
- `device_value_to_radians(value: float, calibration: GripperCalibration) -> float`: validar valor finito y rango 0–100; convertir sin recortar ni inventar mediciones fuera del rango compatible del modelo.
- `radians_to_device_value(position: float, calibration: GripperCalibration) -> int`: validar límites compatibles y devolver valor redondeado del dispositivo.
- `ControlledArm.get_gripper_value()`, `set_gripper_value(value: int, speed: int)`, `gripper_stop()` e `is_gripper_moving()`: mismo bloqueo/ritmo USB y permiso que el brazo; lecturas permitidas con movimiento deshabilitado.
- `GripperTrajectoryExecutor`: recibe el `ControlledArm` del nodo, calibración, reloj monótono y acceso al estado de ejecución compartido; no crea dispositivo serie propio. Valida y ejecuta objetivos de una articulación con feedback de lectura real.

- [x] Probar primero conversiones en extremos nominales, rango reducido, valores no finitos, lecturas inválidas y consistencia ida/vuelta con redondeo de un punto del dispositivo.
- [x] Con dispositivo falso, probar: cero órdenes al arrancar; bloqueo por `allow_motion=false`; objetivo inválido; exclusión con brazo; rechazo sin parada soportada; éxito solo por convergencia medida; cancelación/timeout/pérdida de lectura con parada específica; desbloqueo tras cada resultado.
- [x] Implementar después de confirmar los fallos. Verificar firmas y retornos reales del SDK instalado antes de interpretar una respuesta como éxito. No llamar a inicialización, recalibración ni configuración de fuerza.
- [x] Añadir parámetros opcionales `adaptive_gripper=false`, calibración, velocidad, tolerancia y timeout. Activar el segundo servidor de acción solo con la opción habilitada.
- [x] Integrar lectura de pinza en el único flujo de publicación de `/joint_states`. Conservar el retorno y caché de seis posiciones del brazo para sus validadores; añadir al mensaje solo lecturas válidas de pinza.
- [x] Mantener pruebas anteriores y nuevas pasando en el contenedor. Informar que se validó un dispositivo falso, sin afirmar validación física.

## Tarea 5 — Monitor, lanzamientos de hardware y guía

**Archivos:**
- Modificar `mycobot_280/mycobot_280_hardware/mycobot_280_hardware/robot_monitor_node.py` y `config/robot_monitor.yaml`.
- Modificar `mycobot_280/mycobot_280_hardware/launch/{robot_monitor,robot_control,moveit_real}.launch.py`.
- Ampliar `test/test_robot_monitor_node.py`, `test/test_read_only.py` y pruebas de selección de variante.
- Actualizar `README.md` y `mycobot_280/mycobot_280_hardware/README.md`.

**Consume:** modelo/MoveIt de tareas 1–3 y conversión/control de tarea 4. **Produce:** opción coherente de gripper en todos los lanzamientos y monitor que solo lee.

- [x] Probar monitor habilitado/deshabilitado con dispositivo falso; lecturas de pinza inválidas no generan posiciones ficticias ni comandos. Probar que el launch físico conserva `allow_motion=false` y `use_sim_time=false`.
- [x] Implementar el monitor opcional y pasar `adaptive_gripper` desde los lanzamientos hasta los nodos y archivos del modelo/MoveIt. Conservar la inclusión de launch con ámbito propio que mantiene RViz habilitado.
- [x] Documentar comandos para ambos modelos, dominios 42/43, USB en Docker y comparación de apertura real/modelo antes de habilitar ejecución. Explicar el cierre reducido, los valores nominales y la calibración pendiente.
- [x] Compilar y ejecutar la suite completa; comprobar `--show-args` de los lanzamientos. Repetir pruebas de integración de simulación con el gripper y sin él.
- [x] Entregar resultados separados: pruebas de software, ejecución en Gazebo/MoveIt y validación física pendiente. No ejecutar comandos sobre hardware sin equipo conectado y habilitación explícita.

## Estado de ejecución

Implementado y probado en Docker y Gazebo/MoveIt. Ver resultados y limitaciones en
`docs/testing/2026-10-22-adaptive-gripper-validation.md`. Validación física pendiente.

## Verificación final

Dentro de `ros2-humble`, con ROS Humble cargado:

```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-up-to \
  mycobot_280_gazebo mycobot_280_moveit2 \
  mycobot_280_pick_place_demo mycobot_280_hardware
source install/setup.bash
python3 -m pytest -q src/mycobot_ros2_packages/mycobot_280
```

Además de las pruebas, registrar resultados de acciones en Gazebo, estados medidos
y planificación/ejecución de MoveIt para ambos grupos. Revisar `git diff --check`.
Mantener separados los cambios de esta integración y la limpieza previa aún sin commit.
