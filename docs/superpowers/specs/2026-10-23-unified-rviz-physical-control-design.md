# Un único RViz para Gazebo y el myCobot 280 M5 físico

Estado: aprobado por el usuario («Dale»), implementado y verificado con Gazebo y USB falso; pendiente validación del equipo físico.

## Pedido y alcance

El usuario solicita conservar una sola ventana de RViz y agregar un botón que
active el manipulador físico conectado. La selección se aplica tanto a
«Seguimiento en vivo» como a «Plan & Execute», incluyendo el gripper adaptativo.
La aclaración más reciente reemplaza la elección anterior de un RViz dedicado
al hardware. Todo el trabajo se hará en esta sesión, sin subagentes ni commits.

Propuesta de comportamiento: seleccionar un único destino, Gazebo o brazo físico.
Al seleccionar el físico, Gazebo conserva su postura; no se promete movimiento
simultáneo de ambos. Conectar el USB no habilita movimiento automáticamente.

## Experiencia en RViz

- Mantener el panel de seguimiento y agregar «Manipulador físico» como botón
  con estado confirmado. Mostrar siempre «Destino: Gazebo» o «Destino: brazo
  físico», conexión y motivo de cualquier bloqueo.
- Arrancar en Gazebo, con el control físico desarmado. Con el botón físico
  apagado se conserva el funcionamiento de simulación.
- Aceptar la selección física solamente con el controlador disponible, lecturas
  medidas completas y recientes, límites válidos y ejecución física autorizada
  en el lanzamiento. No enviar posiciones al seleccionar el destino.
- Rechazar el cambio de destino durante una trayectoria o mientras se está
  activando/desactivando un modo. Si el seguimiento está activo, exigir detenerlo
  antes de cambiar el destino.
- Inicializar el marcador y el deslizador desde las mediciones del destino
  seleccionado; descartar objetivos y planes almacenados del destino anterior.
- Con el físico seleccionado, «Plan & Execute» usa su estado medido y sus
  acciones físicas. «Seguimiento en vivo» activa exclusivamente el control
  continuo del mismo destino. Ambos modos se excluyen mutuamente.
- Al perder comunicación, revocar el permiso de sesión y mostrar el fallo. No
  cambiar silenciosamente a Gazebo ni permitir que un comando se ejecute en otro
  destino. Reconectar requiere una nueva selección/activación del operador.

## Alternativas y elección

1. RViz dedicado al físico: menor acoplamiento, pero fue reemplazado por el
   pedido explícito de conservar la misma ventana.
2. Una ventana y selección exclusiva de destino: propuesta elegida para cumplir
   la última aclaración. Implica separar estados y canales de mando para impedir
   mezclar lecturas reales y simuladas.
3. Replicar todos los movimientos en Gazebo y en hardware: no solicitado en la
   última aclaración; requiere sincronizar posturas y gestionar divergencias
   entre dos robots. Queda fuera de esta implementación.

## Arquitectura

Mantener el modelo URDF/SRDF y las acciones públicas que consume MoveIt. Agregar
un coordinador de destino con una máquina de estados confirmada: simulación,
cambiando, físico desarmado, físico preparado y fallo. La selección del panel
es una petición; el controlador determina si puede aceptarla.

En el lanzamiento integrado, separar los canales internos de Gazebo y hardware
mediante namespaces/remapeos. Cada uno conserva sus estados, controladores y
acciones. Un único canal seleccionado alimenta el estado y TF que usa el RViz
común y el planificador. No publicar estados reales y simulados sobre el mismo
topic ni emitir TF duplicados de las mismas articulaciones.

El coordinador expone las acciones que ya usa MoveIt y reenvía cada objetivo al
destino fijado al aceptarlo. Propaga cancelación, feedback y resultado. Antes de
aceptar ejecución comprueba la generación de selección, disponibilidad y
coincidencia del inicio con el estado medido actual. Cambiar de destino invalida
los planes anteriores; no basta con que sus primeras posiciones coincidan por
casualidad. Las cancelaciones o pérdidas de backend no se reportan como éxito.

El lanzamiento integrado usa tiempo real en RViz, MoveIt y hardware. La entrada
seleccionada de Gazebo se adapta a este reloj únicamente para presentación y
planificación; conserva una comprobación independiente de avance de la
simulación para que un Gazebo pausado no parezca producir mediciones frescas.
No cambiar el reloj del hardware a `/clock`. Las vigilancias de comunicación
usan reloj monotónico.

Reutilizar MoveIt Servo, el marcador y las comprobaciones de colisión del
seguimiento actual. Separar el transporte de salida: controladores de posición
para Gazebo; sesión validada con el controlador USB para hardware. El nodo
físico sigue siendo el único dueño de la conexión serie. No abrir otro puerto
desde RViz ni publicar directamente comandos USB desde el panel.

Los lanzamientos actuales de simulación y hardware siguen disponibles. Agregar
un lanzamiento integrado y documentar sus comandos Docker y el acceso al puerto.

## Sesión de movimiento físico

La autorización estática `allow_motion` conserva su valor predeterminado falso.
El botón físico añade una autorización de sesión revocable; no sustituye ni
modifica silenciosamente ese parámetro. El mismo permiso de sesión debe proteger
las acciones físicas de planificación y el seguimiento, incluso si otro cliente
ROS intenta llamar directamente a sus endpoints.

Activar seguimiento requiere estado reciente, ninguna acción en ejecución,
ausencia de fallo enclavado y soporte de parada. Asignar un identificador nuevo
a cada activación; aceptar solo órdenes recientes y secuenciadas de esa sesión.
Rechazar mensajes retenidos, duplicados, fuera de orden o de sesiones anteriores.

Procesar únicamente la última consigna, sin acumular una cola. Empezar con una
frecuencia serie máxima de 10 Hz y un límite solicitado de 0,1 rad/s por
articulación, además de límites articulares y de desplazamiento respecto de la
medición. El porcentaje de velocidad del firmware no equivale a rad/s: estos
límites se comprueban en software y necesitan validación física. La vigilancia
debe detectar estancamiento y desviaciones, además de pérdida de lecturas.

Comprobar el modo de refresco del firmware antes de emitir órdenes continuas;
requerir ejecución de la última consigna, no cola secuencial. Si el SDK/firmware
no permite verificar ese modo, rechazar el seguimiento y explicar el bloqueo.
No cambiar ni restaurar ese modo mientras haya movimiento sin verificar.

El watchdog del controlador físico es independiente del nodo Servo. Si el panel
o el backend deja de responder, expira la sesión aunque el puerto siga abierto.
Cerrar la puerta a nuevas consignas antes de intentar la parada. Verificar parada
del brazo mediante lecturas válidas y estado no móvil; la mera ausencia de una
excepción al llamar `stop()` no es confirmación. Un fallo de parada enclava un
bloqueo de movimiento. Una desconexión USB impide garantizar la llegada de una
orden de parada; el software debe informar esa limitación y no rearmarse.

Para el gripper conservar el rango compatible `[-0.5, 0.15] rad`, la calibración
existente y `gripper_stop_confirmed=false` por defecto. No ejecutar calibración,
cero ni liberación de servos automáticamente. Con gripper solicitado y parada
específica sin confirmar, bloquear su ejecución y comunicar la causa; nunca
simular una confirmación con valores comandados.

## Verificación

Pruebas con dispositivo falso y reloj controlado para ausencia de movimiento al
inicio/selección, exclusión de destinos y modos, estados inválidos o caducados,
límites, comandos antiguos y secuencias, pérdida del panel/backend/USB, fallo de
parada, reconexión sin rearmado y firmware sin modo de refresco verificable.

Pruebas de las acciones encaminadas: rechazo durante seguimiento, cancelación,
resultado/feedback correctos, destino fijado y rechazo de planes de una selección
anterior. Verificar permisos también llamando directamente al controlador físico.

Integración con Gazebo y hardware falso: una sola ventana de RViz, selección de
estado/TF y destino correctos, movimiento de brazo y gripper, recuperación de
planificación al detener seguimiento y conservación del modo simulación normal.
Repetir las pruebas funcionales actuales y comprobar visualmente el panel.

No enviar movimientos al dispositivo real durante el desarrollo sin una prueba
física expresamente coordinada. Documentar resultados de software por separado:
no constituyen validación del firmware ni de precisión o seguridad física.

## Límites y decisiones de implementación

Las colisiones dependen de la escena modelada y de la calibración del robot.
Este trabajo no añade detección automática de personas ni una parada física
independiente del ordenador. No promete sincronización exacta con Gazebo ni
control de fuerza del gripper.

La separación exacta de namespaces, las interfaces de sesión y la implementación
del encaminamiento se detallarán en el plan después de aprobar este diseño.
Si las API disponibles de MoveIt no permiten invalidar de forma fiable un plan
ya almacenado en RViz, habrá que resolver esa limitación antes de habilitar la
ejecución física; no se sustituirá por un indicador visual sin comprobación.
