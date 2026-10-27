# Retorno a Gazebo y seguimiento físico desconectado

El manipulador está desconectado durante estas comprobaciones. No se enviaron
movimientos a hardware real. Se usó una sesión normal visible en dominio ROS 42
y otra sesión aislada en dominio 87, con Gazebo separado por puerto y un
`FakeUSB` que no abre puertos serie.

## Retorno a simulación

El coordinador exigía una confirmación del controlador USB incluso cuando su
sesión ya estaba revocada. Un fallo físico retenido impedía esa confirmación y
podía bloquear el cambio manual a Gazebo. Ahora, cuando el controlador informa
selected=false y live=false, la elección explícita de simulación elimina el
token local vencido y cambia de destino sin volver a pedir confirmación USB.
El fallo físico no se borra y la pérdida de conexión no cambia de destino por sí
sola. Si el físico sigue seleccionado y operativo, se conserva la exigencia de
parada confirmada antes del cambio.

También se aplicaban los límites físicos de 0.1 rad/s a toda la planificación de
la sesión integrada, incluida Gazebo. Combinados con el escalado de RViz de 0.1,
reducían la velocidad nominal a 0.01 rad/s. Las trayectorias largas mantenían
ocupada la ejecución y los botones; el registro previo contiene una ejecución
abortada por el límite de 180 s y peticiones de Plan que agotaban su espera.
No se atribuye todo bloqueo de interfaz a ese único motivo: el cliente ROS
permite comprobar el estado del backend, no todos los eventos del ratón de RViz.

El planificador compartido vuelve a usar los límites normales del modelo.
El coordinador escala solamente las solicitudes físicas, conservando los mismos
límites absolutos anteriores para brazo y pinza. No se aumentó la velocidad del
brazo real ni su frecuencia de órdenes. Las pruebas verifican además que no se
modifique la solicitud original y que un plan de otro destino siga invalidado.

## Seguimiento físico

Permanece sin validar la precisión física descrita en
[el informe anterior](2026-10-26-plan-cartesian-physical.md). La corrección de las
proporciones articulares no fue suficiente en las pruebas reales de +3 mm y
+5 mm; no se aplicó una compensación de motores sin medirla en el equipo.

Se añadió vigilancia cartesiana medida al seguidor físico. Después de un
arrastre válido, compara la posición medida de la punta con el objetivo. Si
acumula más de 2 mm en dirección contraria, o pasan 3 s sin progreso hacia un
objetivo que está a más de 1 mm, cierra las órdenes y solicita la parada existente.
Cambiar repetidamente el objetivo no renueva artificialmente el plazo.
La activación sin objetivo y el control exclusivo de pinza no pasan por esa
vigilancia. La orientación conserva las demás protecciones existentes.

La regresión de C++ reproduce el desplazamiento adverso de 3.976 mm registrado
antes y comprueba la parada, junto con avance correcto, llegada al objetivo y
retargeting sin progreso. Es una protección de parada, **no demuestra que el
seguimiento físico ahora reproduzca correctamente el movimiento**. Esa prueba
requiere volver a conectar el brazo. No se añadieron correcciones de encoder ni
se invirtieron signos o ejes.

## Resultados

- **174 pruebas Python pasaron**, 3 linters heredados deseleccionados y el aviso
  existente de xacro, en dominio aislado 85.
  [Salida](physical-control/2026-10-26/destination-recovery/mycobot-destination-tests.log).
- Compilación C++ correcta; CTest **2/2** pasaron en dominio 86.
- Integración completa con FakeUSB: Plan & Execute y Plan/Execute separados en
  ambos destinos, seguimiento simulado de brazo/pinza, cancelaciones, exclusión
  de acciones durante seguimiento, revocación por pérdida de heartbeat y regreso
  explícito a Gazebo después de esa revocación. Tras volver, Plan, Execute y
  Plan & Execute pasaron.
  [Salida](physical-control/2026-10-26/destination-recovery/mycobot-isolated-destination-integration-final.log).
- Prueba adicional: apagar el proceso FakeUSB, publicar el estado desconectado
  con fallo y volver manualmente a Gazebo **sin servidor físico disponible**.
  Plan y Execute pasaron; el fallo físico permaneció retenido.
  [Salida](physical-control/2026-10-26/destination-recovery/mycobot-disconnected-destination-final.log).
- En la sesión visible, con el robot real desconectado, Plan y Execute devolvieron
  éxito. La trayectoria de prueba de 0.04 rad duró 0.400 s.
  [Salida](physical-control/2026-10-26/destination-recovery/mycobot-visible-sim-plan-final.log).

También se probó intentar seleccionar el físico desconectado, recibir el rechazo
correcto, volver a Gazebo y ejecutar Plan & Execute en la sesión visible:
[registro](physical-control/2026-10-26/destination-recovery/mycobot-visible-disconnected-combined-final.log).

La sesión aislada de pruebas se detuvo. La sesión visible quedó abierta con
Gazebo seleccionado y seguimiento apagado. El controlador real informa que USB
no está disponible; eso no bloquea el funcionamiento de Gazebo.
