# myCobot RViz — Live & Sim

Control del myCobot 280 M5 desde RViz: planificación con MoveIt 2, seguimiento
al arrastrar un marcador, simulación en Gazebo Classic y conexión USB opcional
al brazo físico y su pinza adaptativa.

## Inicio rápido en otra PC

Cada persona instala **Docker Engine y Docker Compose v2** en su equipo.
La imagen instala ROS 2 Humble, Gazebo, MoveIt y las dependencias del proyecto;
no hace falta instalar ROS en el host ni copiar el contenedor de otra persona.

El flujo gráfico está pensado para **Linux x86_64 con X11 o XWayland** (por
ejemplo Ubuntu 22.04). Windows/WSL y macOS necesitan otra configuración de
pantalla y USB, que no se incluye aquí. La primera construcción descarga varios
GB y compila los paquetes; reservá al menos 10 GB libres para imagen y caché.

Instalación oficial: [Docker Engine para Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
y [plugin Docker Compose](https://docs.docker.com/compose/install/linux/).
Comprobá que ambos estén disponibles y que tu usuario pueda ejecutar Docker:

```bash
docker --version
docker compose version
docker info
```

### 1. Abrir el proyecto y preparar la pantalla

Cloná este repositorio y, desde su carpeta, ejecutá en el **host Linux**:

```bash
cd mycobot-rviz-live-sim
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export MYCOBOT_X11_HOSTNAME="${XAUTHLOCALHOSTNAME:-$(hostname)}"
test -f "$XAUTHORITY" && test -n "$DISPLAY"
```

La última línea debe terminar correctamente: el archivo contiene la autorización
para abrir Gazebo y RViz en tu sesión gráfica. En Ubuntu con GDM suele estar en
`/run/user/<uid>/gdm/Xauthority` y la variable ya viene definida. Si el comando
falla, apuntá `XAUTHORITY` al archivo real de tu sesión antes de continuar.
`MYCOBOT_X11_HOSTNAME` conserva el nombre asociado a la autorización X11 dentro
del contenedor. Exportá ambas variables también en las terminales nuevas desde
las que ejecutes Compose. No hace falta ejecutar `xhost +`.

### 2. Construir y ejecutar

Desde la raíz del repositorio:

```bash
docker compose build
docker compose up
```

Se abren **Gazebo y un solo RViz** con brazo y pinza. El destino inicial es
**Gazebo**. Este comando no abre el puerto USB ni habilita movimiento físico.
Esperá a que aparezca el modelo y terminen de arrancar los controladores.

La imagen contiene una copia compilada del código. Después de modificarlo,
detené el lanzamiento con **Ctrl+C**, volvé a ejecutar `docker compose build`
y después `docker compose up`. No se usan rutas personales ni se monta el
workspace de quien creó el proyecto.

Para terminar, detené primero el seguimiento y pulsá **Ctrl+C**. Después:

```bash
docker compose down
```

Esto elimina el contenedor de esta sesión; conserva la imagen para el próximo
arranque. No borra el repositorio.

### Si tenés una GPU NVIDIA

La configuración predeterminada usa renderizado por CPU para evitar depender
de una GPU concreta. Si necesitás aceleración, instalá el
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
en el host y ejecutá:

```bash
docker compose -f compose.yaml -f compose.nvidia.yaml up
```

Usá los mismos archivos `-f` al ejecutar `down`. El renderizado por CPU puede ser
más lento; si RViz falla con el controlador gráfico de tu equipo, probá esta
variante cuando tengas una GPU NVIDIA compatible.

## Mover el robot en RViz

### Plan y Execute

1. Dejá el destino en **Gazebo** y el seguimiento desactivado.
2. En **MotionPlanning**, elegí `arm_group`.
3. Usá **Start State → <current>** y desplazá el objetivo con el marcador.
4. Pulsá **Plan**, revisá la trayectoria y después **Execute**.
5. Para la pinza, elegí `gripper_group`, mantené Start State en `<current>` y
   seleccioná **Goal State → open** o **closed**. Planificá y ejecutá de nuevo.

La animación de la trayectoria en RViz es una previsualización. Observá el robot
en Gazebo y el resultado de Execute para comprobar que se ejecutó. Si cambiás
la postura o el destino, generá un plan nuevo.

### Seguimiento al arrastrar

1. Pulsá **Activar seguimiento** en el panel **myCobot — Seguimiento en vivo**.
2. Arrastrá las flechas para trasladar el objetivo o los anillos para girarlo.
   La herramienta **Interact** se selecciona al activar el seguimiento.
3. Cada objetivo se planifica y ejecuta automáticamente. Para la pinza usá el
   deslizador del panel.
4. Pulsá **Detener seguimiento** antes de volver a los botones manuales de
   MotionPlanning o de cambiar el destino.

Activar seguimiento conserva la postura medida. El movimiento empieza al
arrastrar o cambiar el deslizador. Se ejecuta una acción por vez y se conserva
el último objetivo pendiente por grupo; el robot sigue trayectorias sucesivas
y puede tardar en alcanzar el marcador. Un objetivo fallido muestra el error
y requiere un nuevo movimiento del usuario para volver a intentarlo.

## Conectar el manipulador físico

Detené la simulación anterior. En el host, comprobá el puerto real y habilitá
el archivo adicional de USB:

```bash
ls -l /dev/ttyACM0
export MYCOBOT_PORT=/dev/ttyACM0
docker compose -f compose.yaml -f compose.hardware.yaml up
```

Si el dispositivo aparece como `/dev/ttyUSB0`, cambiá `MYCOBOT_PORT`. Dentro del
contenedor se presenta siempre como `/dev/ttyACM0`. El archivo adicional abre
el controlador USB, pero mantiene **`allow_motion:=false`**. Gazebo sigue siendo
el destino inicial y la selección física para ejecutar movimientos está bloqueada.

Antes de permitir movimiento, verificá calibración, zona de trabajo y parada
del brazo y la pinza con el firmware instalado. Consultá la
[guía de hardware](mycobot_280/mycobot_280_hardware/README.md), teniendo en cuenta
que el workspace de esta imagen está en `/opt/mycobot_ws`.

Después de esas comprobaciones, detené el servicio y ejecutá una sesión física
con permiso explícito:

```bash
docker compose -f compose.yaml -f compose.hardware.yaml down
docker compose -f compose.yaml -f compose.hardware.yaml run --rm mycobot \
  ros2 launch mycobot_280_hardware control.launch.py \
  adaptive_gripper:=true port:=/dev/ttyACM0 \
  allow_motion:=true gripper_stop_confirmed:=true
```

`gripper_stop_confirmed:=true` declara que el operador ya verificó la parada;
el lanzamiento no realiza esa prueba. Para NVIDIA, agregá también
`-f compose.nvidia.yaml` a los comandos de hardware.

En RViz, con seguimiento y ejecución detenidos, pulsá **Manipulador físico**,
confirmá y comprobá **Destino: brazo físico**. Plan/Execute y seguimiento pasan
a controlar el robot real; Gazebo conserva su postura. Volvé a planificar al
cambiar el destino. Solo un controlador debe abrir el USB: no ejecutes otro
monitor, script serie ni el contenedor anterior en paralelo sobre ese puerto.

La reconexión USB no reactiva una sesión física. Si falla la parada, revisá el
equipo y resolvé el fallo antes de reiniciar. El seguimiento automático tiene
[validación ROS/Gazebo](docs/testing/2026-10-27-automatic-plan-execute.md) y una
[prueba física acotada de pinza](docs/testing/2026-10-27-gripper-response.md).
La precisión cartesiana del brazo físico sigue teniendo
[limitaciones medidas](docs/testing/2026-10-26-plan-cartesian-physical.md).

## Opciones y comandos útiles

Para entrar al contenedor que dejó activo `docker compose up`, desde otra terminal
ubicada en la raíz del repositorio:

```bash
docker compose exec mycobot bash
```

En esa shell, cargá el entorno antes de usar ROS:

```bash
source /opt/ros/humble/setup.bash
source /opt/mycobot_ws/install/setup.bash
ros2 control list_controllers -c /simulation/controller_manager
ros2 topic echo /joint_states --once
ros2 topic echo /mycobot_control/status --once
```

Deben estar activos `joint_state_broadcaster`, `mycobot280m5_group_controller` y,
con pinza, `mycobot280m5_gripper_controller`. Los controladores `mycobot280m5_live_*`
pueden estar inactivos: el seguimiento actual usa acciones de trayectoria.

Para cambiar opciones del lanzamiento, detené primero el servicio y usá `run`:

```bash
docker compose down
# Simular solo el brazo, sin pinza.
docker compose run --rm mycobot \
  ros2 launch mycobot_280_hardware control.launch.py \
  adaptive_gripper:=false launch_driver:=false
```

| Argumento de `control.launch.py` | Predeterminado | Uso |
| --- | --- | --- |
| `adaptive_gripper` | `true` | Incluir la pinza. |
| `launch_driver` | `true` | Abrir USB; Compose base pasa `false`. |
| `port` | `/dev/ttyACM0` | Puerto serie dentro del contenedor. |
| `baudrate` | `115200` | Velocidad serie. |
| `allow_motion` | `false` | Permitir habilitar el destino físico. |
| `gripper_stop_confirmed` | `false` | Confirmación del operador sobre la parada. |
| `gui` / `use_rviz` | `true` | Mostrar Gazebo / RViz. |

Compose usa por defecto `ROS_DOMAIN_ID=52` y el puerto de Gazebo `11352` para
separarse del contenedor histórico de este equipo (dominio 42). Si los cambiás,
exportá las mismas variables en todas las terminales. Para limitar memoria al
compilar, podés usar `BUILD_JOBS=1 docker compose build`.

## Problemas frecuentes

| Problema | Qué revisar |
| --- | --- |
| `docker compose` no existe | Instalá el plugin Compose v2; no basta con Docker Engine. |
| Error de permisos al acceder a Docker | Configurá el acceso de tu usuario según la guía de instalación de Docker. |
| No abre Gazebo o RViz / error de display | Comprobá `DISPLAY`, el archivo `XAUTHORITY` y `/tmp/.X11-unix` en el host. Reexportá la autorización después de iniciar una nueva sesión gráfica. |
| `bind source path does not exist` | Revisá la ruta de `XAUTHORITY`; Compose no crea un archivo de autorización vacío. |
| Build terminado por falta de memoria | Reintentá con `BUILD_JOBS=1 docker compose build`. |
| RViz anima pero Gazebo no cambia | Revisá Execute y el destino. Con `start point deviates`, elegí Start State `<current>` y planificá de nuevo. |
| El destino físico es rechazado | Revisá USB, mediciones, permisos de movimiento y confirmación de parada. Leé el texto del panel. |
| Código cambiado sin efecto | Reconstruí la imagen y recreá el contenedor con `docker compose up`. |

## Desarrollo y pruebas

El [Dockerfile](Dockerfile) instala las dependencias y compila el código en
`/opt/mycobot_ws`. Usa la [imagen OSRF de ROS](https://hub.docker.com/r/osrf/ros)
y fija el commit de `warehouse_ros_mongo` que se utiliza en este proyecto.
Las versiones de los paquetes apt/pip y la etiqueta base pueden recibir
actualizaciones; no es una imagen con todas las versiones congeladas.

Para ejecutar las pruebas sin abrir ventanas ni acceder a USB:

```bash
docker run --rm mycobot-rviz-live-sim:humble bash -c '
  cd /opt/mycobot_ws
  python3 -m pytest -q src/mycobot/mycobot_280
'
docker run --rm mycobot-rviz-live-sim:humble bash -c '
  cd /opt/mycobot_ws
  colcon test --packages-select mycobot_280_live_control --return-code-on-test-failure &&
  colcon test-result --verbose
'
```

Las pruebas de software no sustituyen la validación física. Los estados de pinza
`open` (`0.15 rad`) y `closed` (`-0.5 rad`) son posiciones del modelo, no distancias
en milímetros. El agarre y transporte de objetos no están validados.

| Paquete | Función |
| --- | --- |
| `mycobot_description` | URDF, mallas y texturas. |
| `mycobot_280_gazebo` | Simulación y controladores. |
| `mycobot_280_moveit2` | Planificación e interfaz de RViz. |
| `mycobot_280_pick_place_demo` | Demo de movimientos del brazo en simulación. |
| `mycobot_280_hardware` | Control USB y coordinador del destino. |
| `mycobot_280_live_control` | Panel y seguimiento con Plan & Execute. |
| `mycobot_280_control_interfaces` | Interfaces de las sesiones físicas. |

## Contenedor anterior de este equipo

La configuración nueva es independiente de `ros2-humble`. Para seguir usando
ese entorno ya instalado, desde Ubuntu:

```bash
docker start ros2-humble
docker cp "$XAUTHORITY" ros2-humble:/tmp/mycobot-Xauthority
docker exec -it -e ROS_DOMAIN_ID=42 -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=/tmp/mycobot-Xauthority ros2-humble bash
```

Dentro del contenedor anterior, las rutas siguen siendo:

```bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws
colcon build --symlink-install --packages-up-to \
  mycobot_280_hardware mycobot_280_pick_place_demo
source install/setup.bash
ros2 launch mycobot_280_hardware control.launch.py \
  adaptive_gripper:=true launch_driver:=false
```

El repositorio está montado en `/root/mycobot_ros2_packages` y el workspace en
`/root/ros2_ws`. Esas rutas corresponden al entorno anterior, no a la imagen
portable. Detené esa sesión antes de conectar el USB desde la nueva.

## Origen y licencia

Basado en [Elephant Robotics mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2).
Se conservan las licencias y atribuciones de los paquetes incluidos.
