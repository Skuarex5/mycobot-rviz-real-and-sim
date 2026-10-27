# Activación sin movimiento, ejes y cierre de RViz

Trabajo realizado tras el reporte de movimiento físico al activar seguimiento,
imposibilidad de elegir un eje y cierre inesperado de RViz. El operador volvió
a confirmar brazo detenido, pinza vacía y zona despejada antes de las pruebas
reales.

## Movimiento y selección de ejes

Antes, el backend emitía consignas de postura desde la activación y aceptaba
cualquier `POSE_UPDATE`, sin comprobar un arrastre. El controlador USB podía
corregir diferencias entre su referencia inicial y esa consigna aunque el
operador aún no hubiera movido el marcador.

Ahora `HardwareCommand` transporta `arm_enabled` y `gripper_enabled` junto con
las posiciones. El backend emite paquetes atómicos en
`/mycobot_live/physical_command`; el coordinador añade sesión, secuencia y fecha.
Los canales anteriores quedan dedicados a simulación y no autorizan USB físico.
Sin intención activa, el controlador conserva la vigilancia pero no escribe
posiciones. Un objetivo de pinza no habilita el brazo.

El marcador exige `MOUSE_DOWN` sobre uno de sus seis controles, seguido de
`POSE_UPDATE` del mismo cliente y control. `MOUSE_UP` cierra el gesto. La sesión
reinicia esa autorización al activarse. La pinza se habilita por su deslizador.
El panel selecciona Interact al activar; los ejes son visibles, están fijos al
marco base y llevan descripciones Mover/Girar X, Y, Z. Se corrigió la orientación
de los controles Y/Z.

La regresión del controlador demostró una escritura de ángulos con intención
deshabilitada antes del cambio y ninguna después. También se verificó el
rechazo de actualizaciones sin gesto, de otro cliente y de otro control.

## Pruebas reales

- [Activación sin movimiento](physical-control/2026-10-26/explicit-intent/activation-without-motion.json):
  205 paquetes, ninguno con intención activa y **cero escrituras de posición**
  de brazo o pinza durante activación y una pose enviada sin pulsar un eje.
  Variación máxima de lectura: 0.00157 rad, aproximadamente 0.09°.
- [Arrastre deliberado y ejecución](physical-control/2026-10-26/explicit-intent/real-drag-validation.json):
  Plan y Execute separados, Plan & Execute, arrastre y pinza pasaron. Giro real
  de base 0.06754 rad, aproximadamente 3.87°; cambio de pinza 0.04432 rad.
  Se confirmó parada y vuelta a Gazebo.

Se enviaron los eventos de arrastre a través de ROS. No se automatizaron clics
en la ventana: `orca-ide` no estaba instalado. La traza de activación se obtuvo
instrumentando el único propietario USB y sin sustituir sus métodos de control;
esa instrumentación terminó antes de recrear la sesión final.

Suite: **161 pruebas Python** y **2 pruebas C++/Qt** pasaron. Hay tres linters
heredados deseleccionados y el aviso previo de xacro.

La [sesión final con GPU](physical-control/2026-10-26/explicit-intent/final-session-validation.json)
también pasó planificación y ejecución simulada, arrastre, pinza y cambio de
destino sin movimiento físico. Su marcador publicó los seis controles visibles
con orientación fija y las descripciones correctas.

## Cierre de RViz

Se reprodujo SIGSEGV al enviar SIGINT. Primero se capturó una caída de la ruta
X11/Mesa, y después una caída en `rclcpp::CallbackGroup::~CallbackGroup`, también
con GPU y sin nuestro panel. Por tanto, habilitar la GPU no resolvió por sí solo
el cierre. Trazas:

- [Renderizado software](physical-control/2026-10-26/explicit-intent/rviz-software-renderer-crash.log).
- [Cierre con GPU](physical-control/2026-10-26/explicit-intent/rviz-gpu-shutdown-crash.log).
- [Sin panel propio](physical-control/2026-10-26/explicit-intent/rviz-without-panel-crash.log).

El patrón coincide con el [reporte de cierre de RViz/MoveIt](https://github.com/ros2/rviz/issues/1858).
El [cambio upstream en revisión](https://github.com/ros2/rviz/pull/1881) describe
problemas de orden de destrucción y bibliotecas descargadas antes de ejecutar
sus callbacks. No se reemplazaron las bibliotecas ROS instaladas.

Como medida local, el lanzamiento de RViz precarga las bibliotecas de
MotionPlanning y moveit_utils para mantenerlas referenciadas hasta que termine
el proceso. Solo afecta a RViz y conserva preloads del usuario. Con esa medida,
un cierre bajo GDB y otros tres ciclos de abrir/cerrar terminaron con código 0:
[registro](physical-control/2026-10-26/explicit-intent/rviz-preload-validation.json).
Esto valida la reproducción obtenida; no demuestra ausencia de cualquier otro
fallo de RViz.

## Contenedor actual

`ros2-humble` fue recreado conservando USB, red, dominio 42 y los mismos montajes,
añadiendo acceso NVIDIA y 512 MiB de memoria compartida. `glxinfo` confirma
RTX 2080 Ti y OpenGL NVIDIA 4.6:
[salida](physical-control/2026-10-26/explicit-intent/gpu-renderer.log).
Imagen local: `mycobot-humble:graphics-20261026`. Respaldo detenido del contenedor
anterior: `ros2-humble-sin-gpu-20261026`; no arrancarlo junto con la sesión actual.

El destino final es Gazebo y el seguimiento queda apagado. Las pruebas fueron
pequeñas, con pinza vacía; no validan carga ni toda la zona de trabajo.
