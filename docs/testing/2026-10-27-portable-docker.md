# Validación del entorno Docker portable

Se construyó `mycobot-rviz-live-sim:humble` desde el Dockerfile del repositorio,
usando la imagen OSRF `humble-desktop-full-jammy` disponible en el host. No se
usó como base el contenedor personalizado `ros2-humble` ni su workspace.

- `docker build -t mycobot-rviz-live-sim:humble .`: correcto; ocho paquetes ROS
  compilados. Imagen resultante: `60b92c1588d1`, aproximadamente 4.32 GB.
- Compose v2.29.7 validó la configuración base, hardware y hardware + NVIDIA.
  Se usó un ejecutable temporal porque este host no tiene instalado el plugin
  `docker compose`. No se cambió la instalación de Docker del host.
- Configuración resuelta: simulación sin dispositivos USB y con
  `launch_driver:=false`; variante hardware con `allow_motion:=false` y puerto
  del host configurable (se comprobó `/dev/ttyUSB7` → `/dev/ttyACM0`).
- Conexión X11 desde Compose: correcta. Una comprobación adicional con la imagen
  base y solo una autorización FamilyLocal confirmó el uso del hostname del host;
  no depende de que el archivo ya tenga cookies wildcard. No se modificó xhost.
- Suite Python dentro de la imagen: **193 pasaron**, con un aviso heredado de
  xacro. `colcon test` del panel: **2 pruebas, 0 errores, 0 fallos**.
- Lanzamiento integrado sin ventanas en un contenedor sin red externa, dominio
  ROS 74 y puerto Gazebo 11374: modelo generado, estados articulares recibidos,
  controladores de brazo y pinza activos, controladores live inactivos.
- Acción MoveGroup: planificación correcta desde el estado medido y después
  Plan & Execute con un desplazamiento simulado de 0.05 rad de la primera
  articulación. MoveIt y el controlador de Gazebo devolvieron éxito.
- Seleccionar el destino físico sin driver/USB devolvió rechazo.

La primera sonda consultó los controladores antes de terminar su carga y encontró
solo el broadcaster. Se corrigió la sonda para esperar a que los tres estuvieran
activos; los registros confirmaron que el lanzamiento los estaba cargando con
normalidad. No se cambió el código del robot para esta validación.

Se comprobó la sintaxis de los bloques bash del README, los enlaces locales y
el entrypoint. El contenedor temporal se detuvo y eliminó después de las pruebas;
la sesión histórica permaneció en ejecución.

No se ensayó aquí el movimiento del robot físico, el passthrough NVIDIA ni una
sesión gráfica completa de RViz/Gazebo con la nueva imagen. La prueba X11 verifica
conexión y autorización, y la integración ROS/Gazebo se ejecutó sin ventanas.
