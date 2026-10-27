# Seguimiento en vivo desde RViz hacia Gazebo

Estado: diseño aprobado por el usuario («Dale»), implementado y verificado en Gazebo con y sin gripper.

## Objetivo confirmado

El usuario quiere activar un botón en RViz y arrastrar el objetivo del brazo o
ajustar el gripper para que el robot de Gazebo lo siga mientras lo hace. No debe
requerir Plan/Execute ni reproducir una animación guardada. El trabajo se ejecutará
en esta sesión, sin subagentes, conforme a la preferencia del usuario.

El alcance es la simulación del myCobot 280 M5, con y sin gripper adaptativo. El
control USB existente conserva su flujo de planificación y ejecución; este modo
continuo no se incorpora al lanzamiento del robot físico.

## Experiencia en RViz

Un panel «myCobot — Seguimiento en vivo» contiene:

- Botón con estados «Activar seguimiento» / «Detener seguimiento».
- Indicador del estado real: desactivado, iniciando, activo, detenido por límite,
  colisión o singularidad, y error de conexión.
- Deslizador de apertura del gripper cuando el modelo lo incluye.
- Mensaje breve que distingue el marcador de seguimiento del objetivo de
  planificación del panel MotionPlanning.

Al activar el modo, un marcador de seis grados de libertad aparece sobre la pose
medida de la herramienta. Gazebo sigue su posición y orientación de forma
continua. La pinza sigue el deslizador dentro de su rango compatible
`[-0.5, 0.15] rad`. No hay movimiento al arrancar ni salto hacia un objetivo viejo.
El marcador y el deslizador se inicializan a partir del estado medido.

Al desactivarlo, se detiene el seguimiento, el robot mantiene su postura y se
restablece el modo de planificación. La activación se rechaza si hay una
trayectoria planificada todavía en ejecución. El botón solo muestra «activo»
después de confirmar el cambio de controladores y la disponibilidad del control.

## Arquitectura propuesta

Crear un paquete C++ `mycobot_280_live_control` con un plugin de panel RViz y un
nodo de seguimiento. Mantener un único modelo URDF/SRDF, las mallas existentes y
la realimentación del único `/joint_states` del simulador.

Para el brazo, usar la interfaz PoseTracking de MoveIt Servo disponible en
Humble, con límites articulares, suavizado y comprobación de colisiones activados.
Servo recibe un objetivo de pose actualizado desde el marcador; el nodo traduce
su salida en consignas continuas para el controlador de posición del brazo.
No se llama a la acción de planificación para cada movimiento del ratón.

Para el gripper, limitar la velocidad de aproximación al objetivo del deslizador,
respetar los límites del principal y mantener las relaciones mimic en Gazebo.
Validar colisiones de los estados candidatos del gripper contra la escena de
MoveIt antes de aceptarlos. Las colisiones internas del mecanismo conservan las
exclusiones que ya fueron probadas, sin desactivar las colisiones externas.

El nodo toma la escena publicada por MoveIt; no se anuncia como segundo servidor
primario de escena de planificación. El panel usa una interfaz propia de
objetivos, activación y estado; no reutiliza el robot fantasma de MotionPlanning
como si fuera un controlador continuo.

## Exclusión entre modos

Añadir controladores de posición continua para brazo y gripper, inicialmente
inactivos. Los controladores de trayectoria actuales siguen activos por defecto.

Cambiar entre ambos conjuntos mediante `controller_manager/switch_controller`,
con cambio estricto y confirmación de estados. El conjunto de un modo debe estar
inactivo antes de que el otro pueda controlar sus articulaciones. Esto evita que
los botones Plan/Execute y el seguimiento escriban simultáneamente los motores.
Durante seguimiento las solicitudes de ejecución al controlador de trayectoria
inactivo se rechazan; la planificación y visualización siguen disponibles.

Al salir, reactivar los controladores de trayectoria manteniendo la postura
medida. Si el cambio falla, detener la emisión de órdenes, informar el fallo y
no mostrar un modo operativo que no haya sido confirmado.

## Continuidad y pérdida de comunicación

Publicación nominal de órdenes a 50 Hz, limitada por el rendimiento de Gazebo.
Mantener límites de velocidad del modelo; los movimientos iniciales se ajustan a
velocidades moderadas. El marcador expresa un objetivo, no garantiza alcanzar
poses imposibles ni atravesar obstáculos.

Usar vigilancia por reloj monotónico y un heartbeat del panel. Si RViz se cierra,
se pierde el estado articular o dejan de recibirse comandos de Servo, detener el
seguimiento y mantener la postura; no seguir indefinidamente el último objetivo.
Un marcador inmóvil con panel conectado conserva el objetivo, sin exigir al
usuario arrastrarlo continuamente. Los estados de parada deben ser visibles en
el panel. Al volver a activar, partir nuevamente de la pose medida.

## Integración y compatibilidad

El lanzamiento de simulación de MoveIt carga el panel y el nodo de seguimiento.
Los recursos de Gazebo incluyen los controladores inactivos necesarios. Usar el
mismo dominio ROS y `use_sim_time=true`; mantener la instancia interactiva en 42
y las pruebas en un dominio/puerto maestro separados.

No cambiar las acciones públicas ni los nombres de los controladores existentes.
No añadir este panel al lanzamiento de hardware. Con gripper deshabilitado,
ocultar su deslizador y no cargar ni activar un controlador de pinza inexistente.

## Alternativas consideradas

- MoveIt Servo con cambio exclusivo de controladores: recomendado por su
  suavizado, manejo de singularidades y controles de límites/colisiones.
- Resolver IK repetidamente y mandar posiciones desde Python: menos dependencias,
  pero requiere implementar y validar el control diferencial y sus protecciones.
- Replanificar movimientos cortos: no satisface el seguimiento continuo solicitado.

## Verificación requerida

- Tests de selección de variante, límites de pinza y ausencia de órdenes al inicio.
- Tests del ciclo activar/desactivar, rechazo con trayectoria en curso,
  desconexión del panel, pérdida de estado y fallo al cambiar controladores.
- Prueba de integración con cambios de objetivos y medición del movimiento real
  de Gazebo; confirmar que mover el marcador produce seguimiento sin Plan/Execute.
- Confirmar parada y mantenimiento de postura, así como recuperación del modo de
  trayectoria al desactivar.
- Probar los modelos con y sin gripper y conservar las pruebas anteriores.
- Verificación visual del botón, marcador y deslizador en RViz, además de los
  mensajes ROS. Documentar límites observados de latencia y rendimiento.
