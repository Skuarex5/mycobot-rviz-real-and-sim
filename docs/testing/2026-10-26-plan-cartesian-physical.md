# Plan durante seguimiento y dirección cartesiana física

## Estado comprobado

El crasheo por solicitar Plan durante seguimiento está reproducido y corregido.
El recorte independiente de articulaciones también está corregido. **El
seguimiento cartesiano del brazo físico todavía no está validado:** las pruebas
reales de desplazamientos pequeños siguen mostrando errores de dirección.
Tampoco se puede presentar Plan → Execute como una solución de esa precisión.
La sesión terminó con seguimiento desactivado, destino Gazebo y brazo detenido.
No se aumentaron velocidades ni se cambiaron signos, offsets o tolerancias.

## Crasheo de MoveIt Humble

El coordinador rechazaba la solicitud ROS de Plan durante seguimiento. La versión
instalada, `libmoveit_move_group_interface.so.2.5.10`, intenta acceder a
`res->error_code` aunque el rechazo no devuelve un resultado. Se reprodujo sin
hardware, en dominio ROS 84, con la misma clase MoveGroupInterface que usa RViz:
[registro anterior](physical-control/2026-10-26/plan-cartesian/mgi-plan-before.log),
[backtrace](physical-control/2026-10-26/plan-cartesian/mgi-plan-before-gdb.log).
El código oficial de Humble permite seguir esa ruta:
[MoveGroupInterface::plan](https://github.com/moveit/moveit2/blob/humble/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp).

El coordinador ahora acepta el sobre de MoveGroup/ExecuteTrajectory y valida
propiedad, seguimiento, estado y procedencia del plan antes de reenviar.
Si falla la validación, devuelve una acción abortada con un resultado poblado;
no adquiere trabajo ni envía órdenes al destino. No se revoca la sesión física
por un intento de Plan mientras se está siguiendo. Cinco llamadas consecutivas
con el cliente nativo devolvieron error -1 y salieron con código 0:
[registro posterior](physical-control/2026-10-26/plan-cartesian/mgi-plan-after.log).
En la sesión física, Plan durante seguimiento también devolvió estado 6
(ABORTED), error -1, conservando la sesión activa; luego se detuvo explícitamente.

Para reproducir sin hardware: compilar el `CMakeLists.txt` de la carpeta de
registros después de sourcear Humble; iniciar `rejection_server.py --old` y el
cliente `mgi_probe` con los parámetros de robot de RViz, **ambos en un dominio
ROS aislado**. El cliente anterior termina con SIGSEGV. Reiniciar el servidor
sin `--old` y repetir: cinco resultados de fallo y salida 0. No correr el
servidor de prueba en el dominio del robot.

## Recorte que invierte el primer paso

En la postura medida del equipo, un objetivo de +3 mm Z, conservando orientación,
requiere aproximadamente estos cambios de hombro/codo/muñeca:
`[-0.013955, +0.056863, -0.042908] rad`. Recortar cada componente antes de aplicar
el presupuesto compartido cambia sus proporciones. La cinemática directa del
URDF da un primer paso de **-0.423 mm Z** con el algoritmo anterior y
**+0.320 mm Z** con el corregido. Esto reproduce una inversión de dirección del
software, independientemente del hardware:
[prueba del modelo](physical-control/2026-10-26/plan-cartesian/mycobot-z-direction-model.jsonl).

El limitador físico ahora aplica una única fracción de interpolación a todas
las articulaciones que avanzan. Esa fracción cumple el límite por articulación,
el presupuesto total de 0.012 rad/0.2 s y el adelanto medido de 0.03 rad. El gripper
se limita por separado. La previsualización de colisión del backend conserva las
mismas proporciones articulares. Permanecen la vigilancia y la retención de
articulaciones ya asentadas dentro de la tolerancia previamente verificada.
Esto conserva la dirección articular; no promete una recta cartesiana exacta.

## Pruebas reales que NO pasaron

Con zona previamente confirmada despejada, pinza vacía, sin órdenes de pinza,
velocidades anteriores y envolvente articular pequeña:

- Seguimiento: objetivo +3 mm Z; variación de Z reconstruida de encoders/FK
  **-3.976 mm**. La prueba falla correctamente y detiene/deselecciona el brazo.
  [Registro completo](physical-control/2026-10-26/plan-cartesian/mycobot-cartesian-physical-proof.json).
- Plan y Execute separados: objetivo +5 mm Z, trayectoria de 27 puntos validada
  por MoveIt y limitada a 0.13 rad respecto del inicio. Ejecución reportada como
  exitosa, pero variación reconstruida **-1.890 mm Z**. El error articular queda
  dentro de la tolerancia física existente de 0.04 rad y aun así el objetivo
  cartesiano no se cumple. La comprobación cartesiana externa al resultado ROS
  detectó el fallo y terminó en Gazebo.
  [Registro completo](physical-control/2026-10-26/plan-cartesian/mycobot-cartesian-planned-proof.json).

No son mediciones externas de la punta: usan encoders y el modelo. El operador
confirmó que las posturas estáticas física y de RViz coinciden visualmente. Eso
no sustituye la calibración de precisión ni demuestra la causa de los errores
residuales. No se aplicó compensación de motores ni un cambio de signos sin
medición. Hace falta comprobar precisión/repetibilidad y respuesta física de
hombro, codo y muñeca antes de afirmar seguimiento cartesiano correcto.

## Flujo de uso

Para revisar una trayectoria: detener seguimiento, seleccionar el destino,
poner el inicio en `<current>`, ajustar el objetivo de MotionPlanning, pulsar
**Plan**, revisar la previsualización y pulsar **Execute**. Cambiar de Gazebo a
físico invalida el plan: hay que planificar otra vez con el estado físico medido.
La revisión de MoveIt comprueba el modelo y la escena conocida; no demuestra
que el brazo real alcance la punta con precisión ni conozca obstáculos no modelados.

El seguimiento en vivo actual no preplanifica toda la ruta: recibe cambios de
objetivo, comprueba pasos y manda consignas. La velocidad del brazo físico sigue
siendo conservadora y su transporte USB es de 5 Hz; no equivale al control de
Gazebo ni a un servo industrial sincronizado.

## Verificación de software final

- **169 pruebas Python pasaron**, 3 linters heredados deseleccionados y el aviso
  existente de xacro; se ejecutaron en dominio ROS 85 aislado del equipo.
  [Salida](physical-control/2026-10-26/plan-cartesian/python-tests.log).
- Compilación C++ correcta, CTest **2/2** pasaron en dominio 86.
- Cliente nativo: **15 respuestas controladas**, cinco de Plan, cinco de
  Plan & Execute bloqueado y cinco de Execute sin plan válido; salida 0.
  [Salida](physical-control/2026-10-26/plan-cartesian/mgi-expanded-after.log).
- Ruff F/E9 y `git diff --check` correctos.
- Estado final: conectado, selected=false, live=false, fault=false,
  `Control físico desactivado; parada verificada`; RViz y Gazebo abiertos.

Estas comprobaciones de software **no convierten en exitosas** las dos pruebas
cartesianas físicas fallidas descritas arriba.
