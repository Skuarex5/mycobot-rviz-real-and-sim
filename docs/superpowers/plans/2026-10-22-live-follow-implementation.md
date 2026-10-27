# Seguimiento en vivo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking. Ejecutar aquí, sin subagentes, por indicación del usuario.

**Goal:** Activar desde RViz el seguimiento continuo del objetivo del brazo y la apertura del gripper en Gazebo, sin Plan/Execute.

**Architecture:** Un paquete C++ contiene el panel RViz y un nodo que integra MoveIt Servo PoseTracking, un marcador interactivo y vigilancia de comunicación. El nodo cambia de forma exclusiva entre los controladores de trayectoria existentes y nuevos controladores de posición. Consume la escena de MoveIt y el estado medido del simulador.

**Tech Stack:** ROS 2 Humble, Gazebo Classic, ros2_control, MoveIt Servo Humble, C++17, Qt5, RViz2, interactive_markers, ament_cmake/gtest y pytest para integración.

**Spec:** [Diseño aprobado](../specs/2026-10-22-live-follow-design.md).

## Global Constraints

- El alcance es la simulación del myCobot 280 M5, con y sin gripper adaptativo.
- El trabajo se ejecutará en esta sesión, sin subagentes, conforme a la preferencia del usuario.
- Publicación nominal de órdenes a 50 Hz, limitada por el rendimiento de Gazebo.
- La pinza sigue el deslizador dentro de su rango compatible `[-0.5, 0.15] rad`.
- No hay movimiento al arrancar ni salto hacia un objetivo viejo.
- No cambiar las acciones públicas ni los nombres de los controladores existentes.
- No añadir este panel al lanzamiento de hardware.
- Con gripper deshabilitado, ocultar su deslizador y no cargar ni activar un controlador de pinza inexistente.
- Mantener la instancia interactiva en dominio 42 y las pruebas en dominio/puerto maestro separados.
- No modificar las exclusiones de colisión ya verificadas ni relajar límites para ocultar fallos.
- No crear commits ni publicar cambios durante esta implementación.

## Review Focus

1. Activación simultánea con ejecución de trayectoria: rechazar la activación si hay objetivos aceptados o ejecutándose, y resolver la carrera mediante el cambio exclusivo de interfaces (tarea 1/5).
2. Gazebo pausado o reloj retrocediendo: la vigilancia usa reloj monotónico y detiene el modo aunque el reloj ROS no avance (tarea 2/5).
3. Objetivo imposible o inicio singular: Servo informa la parada y permite retirar el objetivo de la singularidad cuando su API lo admite; no desactivar protecciones (tarea 2/5).
4. Cambios del gripper y brazo simultáneos: validar la combinación propuesta contra la escena, sin ignorar los estados mimic ni obstáculos nuevos (tarea 2/5).
5. Cierre del panel durante un cambio de controladores: la transición termina en un estado confirmado y sin seguir el último objetivo (tarea 1/3/5).

## Mapa de archivos

Paquete nuevo `mycobot_280/mycobot_280_live_control/`:

- `package.xml`, `CMakeLists.txt`: dependencias, ejecutable, biblioteca del plugin y pruebas.
- `include/mycobot_280_live_control/control_session.hpp`: máquina de estados, validación de objetivos y watchdog; lógica sin Qt ni llamadas ROS bloqueantes.
- `src/live_follow_node.cpp`: escena, Servo, marcador, estados medidos y cambio de controladores.
- `include/mycobot_280_live_control/live_follow_panel.hpp`, `src/live_follow_panel.cpp`, `plugin_description.xml`: panel Qt, heartbeat y servicios asíncronos.
- `config/servo.yaml`, `config/pose_tracking.yaml`: parámetros Humble y ganancias iniciales moderadas.
- `test/test_control_session.cpp`, `test/test_live_follow_panel.cpp`: pruebas de lógica y panel.
- `test/live_follow_integration.py`: prueba reproducible en simulación mediante las mismas interfaces del panel y del marcador.

Archivos existentes a modificar:

- `mycobot_280/mycobot_280_gazebo/config/ros2_controllers{,_gripper}.yaml`: nuevos controladores de posición.
- `mycobot_280/mycobot_280_gazebo/launch/gazebo.launch.py`: spawners inactivos.
- `mycobot_280/mycobot_280_gazebo/package.xml`: dependencia de position_controllers.
- `mycobot_280/mycobot_280_moveit2/launch/moveit_gazebo.launch.py`: nodo y parámetros de la variante.
- `mycobot_280/mycobot_280_moveit2/config/moveit_live.rviz`: copia de la configuración de simulación con el panel adicional; mantener `moveit.rviz` para hardware.
- `mycobot_280/mycobot_280_moveit2/package.xml`: dependencia del paquete nuevo, sin dependencia inversa.
- Tests existentes de selección de controladores/launch: extender sus casos con/sin gripper.
- `README.md`, `docs/testing/2026-10-22-live-follow-validation.md`: uso, resultados y límites observados.

## Task 1: Sesión exclusiva y controladores

**Interfaces:**
- `enum class Mode { Disabled, Starting, Active, Stopping, Fault };`
- `ControlSession(bool has_gripper)` expone `requestEnable(bool trajectory_busy, bool state_fresh) -> bool`, `confirmControllers(bool live_active, bool trajectory_active)`, `requestDisable()`, `fault(const std::string&)`, `mode() const -> Mode`.
- El adaptador ROS usa `/controller_manager/list_controllers` y `/controller_manager/switch_controller`, cambio STRICT y verificación posterior.
- Nuevos controladores: `mycobot280m5_live_arm_controller` (seis articulaciones en el orden existente), `mycobot280m5_live_gripper_controller` (solo `gripper_controller`); tipo `position_controllers/JointGroupPositionController`.
- Los dos conjuntos de trayectoria/posición nunca reclaman simultáneamente las mismas interfaces. JSB permanece activo.

- [x] Escribir pruebas de estados: desactivado inicial, rechazo con trayectoria en curso/estado ausente, activo solo tras confirmación, fallo de cambio, desactivación durante Starting y variante sin gripper. Escribir pruebas de YAML/spawners que comprueben posición y `--inactive`.
- [x] Ejecutarlas antes de implementar; confirmar que fallan por la funcionalidad ausente.
- [x] Crear el paquete C++17 y `ControlSession`; añadir los controladores a ambas variantes y sus spawners inactivos. Declarar dependencias e instalar en Docker `ros-humble-moveit-servo` y `ros-humble-position-controllers` después de comprobar los paquetes disponibles.
- [x] Implementar el adaptador asíncrono de cambio con tiempo máximo de 2 s por operación y confirmación por listado. Monitorizar estados de acciones existentes: ACCEPTED/EXECUTING/CANCELING bloquean activar. Si la carrera con una nueva acción coincide con el cambio estricto, abortar seguimiento, confirmar retorno y reportarlo.
- [x] Compilar con `colcon build --symlink-install --packages-up-to mycobot_280_live_control mycobot_280_gazebo`; ejecutar gtest y pruebas de configuración. Arrancar Gazebo aislado y verificar trayectoria activa/posición inactiva sin alterar el dominio 42.

## Task 2: Seguimiento continuo, escena y parada

**Interfaces:**
- Nodo `/mycobot_live_follow`, namespace público `/mycobot_live`.
- Servicio `/mycobot_live/set_enabled`: `std_srvs/srv/SetBool`; respuesta positiva solo tras confirmación de transición.
- Publicaciones `enabled` (`std_msgs/Bool`), `has_gripper` (`std_msgs/Bool`), `status` (`std_msgs/String`), `gripper_position` (`std_msgs/Float64`), con durabilidad transient_local para estado.
- Subscripciones `heartbeat` (`std_msgs/Empty`) y `gripper_target` (`std_msgs/Float64`). Estado medido de `/joint_states`.
- Parámetros: `adaptive_gripper`, `robot_description`, semántica/cinemática/límites idénticos a MoveIt, `use_sim_time=true`.
- `ControlSession::checkWatchdog(steady_time now, steady_time heartbeat, steady_time joints, steady_time servo)` produce motivo de parada si vence algún plazo.

- [x] Añadir pruebas: heartbeat vencido (0.75 s), estado medido vencido (0.5 s), salida Servo vencida durante movimiento (0.25 s), reloj ROS detenido, NaN/infinito, gripper fuera de rango, ningún comando inicial y objetivo reinicializado tras volver a activar. Ejecutar y comprobar fallo inicial.
- [x] Implementar un PlanningSceneMonitor secundario que consume `/monitored_planning_scene`, solicita la escena inicial y verifica estado completo de las articulaciones requeridas. Filtrar nombres `_mimic` del plugin al construir estados manuales; conservar mimic del modelo MoveIt.
- [x] Integrar `moveit_servo::PoseTracking` y `ServoParameters::makeServoParameters`. Ejecutar `moveToPose` en un trabajador unido al cerrar; `stopMotion()` y pausa Servo al detener. Reanudar el bucle al alcanzar la tolerancia (1 mm/0.01 rad) si llega un objetivo nuevo, sin usar tolerancias negativas para forzar una ejecución infinita.
- [x] Configurar periodo 0.02 s, posiciones solamente, escala inicial lineal 0.03 m/s y angular 0.2 rad/s, colisiones/suavizado/límites habilitados. Ganancias iniciales proporcionales XYZ 1.5/angular 0.5, integral y derivativa 0. Validar los nombres exactos contra los headers instalados de Humble.
- [x] Recibir salida de Servo en un topic interno y publicarla al controlador solo con sesión activa y datos frescos. Al mantener un objetivo alcanzado, distinguir la pausa normal de PoseTracking de una pérdida del trabajador; publicar mantenimiento de postura y exigir progreso del ciclo interno para watchdog.
- [x] Crear servidor de marcador interactivo `/mycobot_live/marker`, marcador `tool_target`, seis controles de traslación/rotación en `g_base`; herramienta `gripper_tool_frame` o `joint6_flange` según variante. Inicializar con pose medida al activar y renovar la pose objetivo a 50 Hz mientras heartbeat/estado sean válidos.
- [x] Implementar aproximación del gripper a 0.25 rad/s máximo. Validar bajo bloqueo de lectura de escena el estado combinado brazo/gripper propuesto, actualizar mimic y rechazar colisiones; conservar salida segura anterior. No publicar al controlador de pinza cuando no existe.
- [x] Detener por watchdog, fallo o desactivación: cerrar compuerta de órdenes, detener trabajador, emitir mantenimiento medido si está disponible, restaurar trayectoria y confirmar estados. No enviar ceros como postura cuando faltan mediciones. Publicar fallo si no se confirma restauración.
- [x] Ejecutar gtest y pruebas ROS de servicio con controladores falsos: espera de disponibilidad, respuesta tardía, heartbeat perdido durante Starting, escena ausente, obstáculo sobre candidato del gripper y objetivo imposible. Confirmar que nunca queda `enabled=true` tras un fallo.

## Task 3: Panel RViz

**Interfaces:**
- Plugin `mycobot_280_live_control/LiveFollowPanel`, clase derivada de `rviz_common::Panel`.
- Usa exclusivamente los servicios/topics de tarea 2. QTimer envía heartbeat cada 100 ms; callbacks ROS actualizan Qt mediante señales encoladas, sin esperar futures en el hilo de UI.
- Botón «Activar seguimiento» / «Detener seguimiento», etiqueta de estado y deslizador 0–650 convertido por `radians = -0.5 + value / 1000.0`.

- [x] Crear pruebas Qt: click envía solicitud, botón espera respuesta sin declarar activo, fallo muestra mensaje, slider oculto sin gripper, medición inicial no publica objetivo, cambio del usuario publica una sola consigna, destrucción del panel deja de emitir heartbeat. Ejecutarlas con `QT_QPA_PLATFORM=offscreen` y comprobar fallo inicial.
- [x] Implementar plugin y registro pluginlib con AUTOMOC. Mantener slider deshabilitado hasta tener estado confirmado y dato inicial; bloquear señales mientras recibe posición medida. Deshabilitar controles durante transición.
- [x] Añadir texto: «Arrastrá el marcador de seguimiento. El objetivo de MotionPlanning se usa con Plan/Execute». Mostrar los estados confirmados y motivos de límites/colisión/singularidad sin declarar que una pose imposible fue alcanzada.
- [x] Compilar y ejecutar pruebas Qt y carga pluginlib en entorno ROS. Confirmar cierre limpio del nodo/hilo asociado al panel.

## Task 4: Integración de launch y variantes

**Interfaces:**
- El launch de simulación inicia backend con el mismo URDF/SRDF/cinemática/límites que `move_group`; `use_rviz=false` mantiene backend inactivo sin heartbeat.
- El launch usa por defecto `moveit_live.rviz`. Hardware conserva `moveit.rviz` sin panel ni backend.

- [x] Extender tests de launch para comprobar ambas variantes, argumentos de herramienta, configuración Servo secundaria y ausencia de live en hardware. Ejecutar y comprobar fallo inicial.
- [x] Añadir nodo backend al launch de simulación, cargar YAML del paquete nuevo y pasar parámetros de variante. Crear configuración RViz con panel y display InteractiveMarkers suscrito al namespace del servidor.
- [x] Declarar dependencias en los paquetes consumidores sin ciclos. Actualizar el docstring del launch que ya no inicia solamente MoveIt/RViz.
- [x] Compilar paquetes afectados y ejecutar pruebas funcionales anteriores: `python3 -m pytest -q -m 'not linter'` con ambos entornos ROS cargados. Registrar fallos nuevos por separado de los linters heredados ya conocidos.

## Task 5: Prueba física de simulación y documentación

**Interfaces:**
- `test/live_follow_integration.py --adaptive-gripper true|false` usa SetBool, heartbeat, mensajes de feedback del marcador y topics públicos; observa `/joint_states` y listado de controladores.
- Se ejecuta en dominio 46 y Gazebo master `http://127.0.0.1:11366`; limpia únicamente sus procesos.

- [x] Crear prueba que primero confirme ausencia de movimiento al arrancar/activar (variación <0.01 rad durante 1 s), luego desplace marcador desde pose medida 5 mm en una dirección alcanzable y compruebe acercamiento medido al objetivo sin solicitar planificación.
- [x] Con gripper, ordenar dos aperturas interiores al rango y comprobar error <0.03 rad, relaciones mimic y parada al introducir obstáculo. Sin gripper, comprobar ausencia de controlador/objetivos de pinza y seguimiento del brazo.
- [x] Probar objetivo fuera de alcance e inicio singular, heartbeat perdido, pausa de Gazebo, pérdida de estado y cierre de panel durante cambio. Verificar cese de consignas de movimiento y estado desactivado/fallo dentro de 1 s desde watchdog vencido; distinguir restauración no posible con Gazebo pausado del retorno confirmado al reanudar.
- [x] Probar rechazo con trayectoria ejecutándose; mientras está activo, comprobar rechazo de ejecución de trayectoria. Desactivar, confirmar controladores y ejecutar una trayectoria pequeña mediante las acciones anteriores.
- [x] Medir frecuencia de órdenes y latencia desde feedback hasta primer cambio medido en diez objetivos. Documentar valores observados, sin prometer tiempo real estricto. Ajustar únicamente parámetros de control justificados por resultados y repetir escenarios afectados.
- [x] Verificar visualmente botón, marcador, slider y textos en RViz si hay una herramienta GUI disponible. Si no la hay, documentar esa limitación y complementar con prueba de plugin/Qt offscreen, sin afirmar una inspección visual realizada.
- [x] Actualizar README con build/dependencias, activación, diferencia entre ambos marcadores, parada y recuperación de Plan/Execute. Guardar evidencias en `docs/testing/2026-10-22-live-follow-validation.md`.
- [x] Reiniciar la simulación interactiva del dominio 42 con la versión probada, esperar controladores/listado y backend listos, dejar seguimiento desactivado para que el usuario lo active. Revisar diff y resumir cambios, comprobaciones y límites restantes.

## Auto-revisión

Cobertura: tareas 1/2 cubren exclusión, escena, continuidad y parada; tarea 3 cubre experiencia de RViz; tarea 4 cubre variantes y separación del hardware; tarea 5 valida movimiento medido y recuperación. Los cinco casos de Review Focus tienen pruebas asignadas. Los topics del panel coinciden con los del backend. No se requieren mensajes ROS propios ni nuevos modelos; no hay dependencias cíclicas. Ninguna comprobación usa el dominio de la sesión interactiva hasta la entrega final.
