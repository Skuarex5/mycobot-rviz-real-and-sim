# myCobot 280 M5 con gripper adaptativo: simulación y hardware

Estado: diseño aprobado por el usuario el 2026-10-22. Implementación y pruebas de software/simulación completadas; validación física pendiente.

## Objetivo y equipo

Integrar el URDF `mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5_adaptive_gripper.urdf` en el flujo existente de
Gazebo Classic, MoveIt 2 y RViz, y permitir controlar el gripper adaptativo original
Elephant Robotics conectado al Atom del myCobot 280 M5. El usuario confirmó ese
modelo. La versión de firmware y la calibración física todavía no están verificadas.

El alcance inicial comprende planificación del brazo con la geometría del gripper,
apertura y cierre controlados y lectura del estado en simulación y hardware.
No incluye todavía una escena de agarre de objetos, detección de objetos,
control de fuerza ni garantías de agarre físico. La demo de movimientos actual
continúa siendo exclusiva de simulación.

## Modelo compartido y compatibilidad

- Conservar el modelo sin gripper como opción predeterminada; añadir selección
  explícita `adaptive_gripper:=true` a los lanzamientos correspondientes.
- Recuperar únicamente las siete mallas necesarias del respaldo existente.
- Conservar los nombres y transformaciones del URDF aportado. Normalizar el
  nombre del robot y añadir la información requerida por los motores de simulación.
- Validar límites de todas las articulaciones mimic. El rango del actuador principal
  debe respetar también los límites de sus seguidores: no usar sin revisión el
  rango original completo de `gripper_controller`.
- Añadir masas e inercias estimadas y documentadas para la simulación; no tratarlas
  como mediciones del dispositivo ni como validación de la dinámica real.
- Añadir una referencia de herramienta para planificación y documentar su posición
  nominal; su coincidencia con la herramienta física requiere calibración.

## Gazebo y MoveIt

- Mantener `arm_group` y el controlador existente de seis articulaciones.
- Añadir `gripper_group` con una articulación independiente, configuraciones de
  apertura/cierre y un controlador de trayectoria independiente.
- Usar `FollowJointTrajectory` para el brazo y para el actuador del gripper, con
  nombres diferentes y el mismo contrato en simulación y hardware.
- Configurar el seguimiento mimic también en gazebo_ros2_control, no solo en la
  visualización de RViz. Comprobar el movimiento de todos los seguidores.
- Incluir la geometría del gripper en las comprobaciones de colisión. Revisar las
  exclusiones de contacto del mecanismo sin desactivar globalmente sus colisiones.
- Compartir modelo y configuración entre RViz, MoveIt y Gazebo cuando se selecciona
  el gripper, evitando modelos diferentes y publicadores duplicados.

## Controlador físico

- Ampliar el nodo `robot_control`, manteniendo un único propietario del USB y el
  bloqueo de acceso serie compartido entre lectura, brazo y gripper.
- Integrar `set_gripper_value` y `get_gripper_value` para el adaptativo de tipo 1,
  verificando firmas disponibles y comportamiento de la versión instalada.
- Configurar explícitamente la conversión entre la posición del URDF y los valores
  de apertura del dispositivo. No asumir que ese valor representa una distancia
  en milímetros ni una relación lineal exacta con la apertura física.
- Publicar la posición principal del gripper a partir de lecturas válidas del
  dispositivo. Los estados mimic se derivan de la relación del modelo. No anunciar
  como medida una posición que solo fue ordenada.
- Rechazar órdenes de brazo y gripper con `allow_motion:=false`; no mover,
  inicializar ni recalibrar el gripper al arrancar.
- Limitar velocidad, validar objetivos y rechazar órdenes simultáneas al brazo y
  gripper inicialmente, para mantener claro el comportamiento de ejecución.
- Verificar soporte de parada específica del gripper para cancelaciones y errores;
  no asumir que `stop()` del brazo detiene también la pinza. Si no está disponible,
  rechazar ejecución del gripper con un diagnóstico explícito.
- Ante lecturas inválidas, pérdida de conexión o falta de convergencia, devolver
  error sin declarar éxito ni inventar realimentación.
- Ampliar el monitor de solo lectura de forma opcional para mostrar el gripper.

## Verificación y documentación

1. Pruebas de referencias de mallas, estructura, rangos mimic y configuración de
   controladores; regresión del modelo original.
2. Pruebas del hardware con dispositivo simulado: conversión, permisos, exclusión
   de ejecución, lectura inválida, timeout, cancelación y ausencia de movimiento al
   arrancar. Adaptar las comprobaciones estáticas para permitir exclusivamente las
   nuevas llamadas de gripper autorizadas en el controlador.
3. Compilar en el contenedor Humble y probar el movimiento del brazo y del gripper
   en Gazebo, así como la selección y ejecución de ambos grupos en MoveIt.
4. Documentar comandos Docker y lanzamientos de ambos modelos; mantener dominios
   separados para simulación y hardware.
5. Validación física pendiente: comprobar firmware, lectura, sentido y extremos
   de apertura antes de habilitar movimiento. Informar por separado qué se probó
   en software, en simulación y en el dispositivo real.
