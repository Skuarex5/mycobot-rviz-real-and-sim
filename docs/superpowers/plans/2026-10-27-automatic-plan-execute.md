# Seguimiento con Plan & Execute automático

Objetivo solicitado: cada arrastre deliberado del marcador de seguimiento y
cambio de pinza se planifica y ejecuta sin cargar objetivos ni pulsar Plan/Execute.
Se mantienen el panel, sus tópicos, y el coordinador de destino existente.

Decisión: usar la acción pública /move_action con plan_only=false. Mantener
controladores de trayectoria activos; no usar Servo ni consignas directas. Una
acción a la vez; conservar solo el último objetivo pendiente por grupo y
planificar desde el estado medido actual. Activar no genera objetivos. Detener,
perder heartbeat o mediciones cancela la acción y descarta objetivos pendientes.
El coordinador conserva selección, límites físicos y procedencia de planes.

- [x] Regresión ROS con servidor MoveGroup controlado: activación inmóvil,
  arrastre sin botón, consolidación de objetivos, pinza, fallos y cancelación.
- [x] Sustituir la salida Servo del backend por la acción de planificación y
  ejecución. Mantener validación de gestos, estados y controladores.
- [x] Compilar y correr pruebas en ROS Humble, en dominio aislado sin USB.
- [x] Actualizar instrucciones del panel y README y documentar límites probados.

Entorno: el contenedor ros2-humble está detenido y no puede iniciar porque falta
/dev/ttyACM0. Usar contenedor de pruebas con la misma imagen y montajes de código
y workspace, sin dispositivos físicos; no alterar el contenedor del usuario.

Revisión: se añadió la entrada automática `/mycobot_live/move_action` en el
coordinador integrado. Usa el mismo planificador y límites, pero rechaza
Plan/Execute manual mientras seguimiento está activo y objetivos automáticos
recibidos después de detener. En modo independiente se conserva `/move_action`.
La revisión independiente no encontró problemas importantes pendientes.
