# Recuperación tras reabrir RViz

El operador informó bloqueo del selector físico y del seguimiento. La captura
mostró destino Gazebo, seguimiento desactivado y las acciones observadas en
estados terminales. Una orden explícita de detener fue aceptada y permitió
reactivar el seguimiento. No se determinó con esa captura qué transición había
dejado el bloqueo inicial; no se afirma corregida una causa interna no observada.

Se reprodujo además un defecto independiente: al seleccionar de nuevo el destino
actual, el servicio respondía `success=True` pero conservaba el error anterior
«Detené seguimiento y ejecución antes de cambiar destino». Se corrigió esa
respuesta para publicar el destino confirmado. La regresión falló antes del
cambio y pasó después.

Comprobaciones ejecutadas con Gazebo y el robot físico conectado:

- Plan & Execute hacia una postura flexionada en Gazebo: pasó.
- Movimiento del brazo y pinza simulados mediante seguimiento: pasó.
- Seleccionar físico y volver a Gazebo, sin enviar consignas físicas: pasó.
- Cerrar RViz durante seguimiento simulado: el backend publicó «Se perdió la
  conexión con el panel» y desactivó el seguimiento.
- Reabrir RViz, seleccionar físico y volver a Gazebo **sin una orden adicional de
  recuperación**: ambos servicios respondieron éxito con el destino correcto.
- Volver a mover brazo y pinza simulados en vivo después de reabrir: pasó.
- Estado final: seguimiento desactivado, destino Gazebo.

El reinicio de RViz conservó los parámetros de la sesión integrada. Se reinició
únicamente el coordinador para cargar la corrección Python, manteniendo un único
coordinador y un único propietario USB. Estas pruebas no ordenaron movimientos
al manipulador físico.

Suite completa: **159 pruebas Python pasaron**, 3 linters heredados
deseleccionados y el aviso existente de xacro. Ruff F/E9 y `git diff --check`
pasaron. El C++ no cambió en esta recuperación.
