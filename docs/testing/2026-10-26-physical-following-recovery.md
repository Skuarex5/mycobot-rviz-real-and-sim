# Seguimiento físico: varios movimientos y recuperación

Corrección del reporte «solo se mueve la primera vez», observado únicamente
en seguimiento del manipulador físico. Se conservaron el propietario USB único,
la activación sin consignas de posición, la frecuencia de 5 Hz y las protecciones
de colisión, límites, singularidad y pérdida de mediciones durante movimiento.

## Defectos reproducidos

1. Un objetivo sin solución IK se copiaba a `target_pose` antes de validarlo.
   La regresión envió una pose imposible a 10 m: fue rechazada, pero el seguidor
   seguía publicando esa pose. [Fallo anterior](physical-control/2026-10-26/following-recovery/rejected-target-before.json).
2. Una articulación con consigna ya alcanzada conservaba un error de encoder
   dentro de la tolerancia final verificada. El limitador intentaba acercar la
   consigna al encoder, alejándola de su objetivo fijo. La traza registró muñeca
   medida en 0.24696 rad, consigna anterior 0.20857 rad y objetivo 0.20857 rad.
   Esto podía producir correcciones repetidas y activar la parada por adelanto.
   Reducir la velocidad del firmware de 10 a 5 no resolvió el fallo:
   [traza](physical-control/2026-10-26/following-recovery/settled-joint-failure.log),
   [prueba](physical-control/2026-10-26/following-recovery/lower-speed-did-not-fix.json).
3. La verificación de parada bloqueaba brevemente el acceso USB y las lecturas.
   Al volver a trayectorias no se exigía una muestra completa nueva; el
   coordinador podía declarar una pérdida de estado durante esa transición.
   En el registro previo se observó `state_age=0.872` después de solicitar parar.

## Cambios

- Solo se publica un objetivo después de validar IK, límites y colisión. Si se
  rechaza, se cancela el avance, se devuelve el marcador a la pose medida y se
  conserva visible el motivo. El mismo seguimiento acepta otro arrastre.
- Una articulación cuya consigna prácticamente no cambia (0.0005 rad) y cuyo
  error medido está dentro de la tolerancia existente conserva esa consigna.
  No se persigue el residuo de encoder. Las consignas que avanzan siguen
  limitadas a 0.03 rad de adelanto; una desviación mayor que la admitida sigue
  provocando parada. La tolerancia final local continúa siendo 0.04 rad.
- Al detener se cierra el reenvío de órdenes antes de esperar la confirmación
  USB. Esa operación tiene el límite existente de 3 s y requiere confirmación
  de parada más una medición completa nueva. Después se renueva la sesión
  autenticada para permitir volver a activar sin perder la selección física.
  La vigilancia de 0.8 s continúa aplicándose al movimiento activo.

Las tres regresiones de Python fallaron antes de sus correcciones y pasaron
después. La regresión física también detectó la pose rechazada publicada antes
del cambio y su retorno a la pose medida después.

## Comprobaciones

En el robot real, a velocidad original y en la misma sesión de seguimiento:
rechazo inicial, movimiento positivo, rechazo entre movimientos, movimiento
negativo y otro positivo. Se midieron aproximadamente **+2.81°, −2.55°, +2.81°**.
La selección física permaneció activa después de parar:
[registro](physical-control/2026-10-26/following-recovery/repeated-motion-and-stop.json).

Tras reiniciar la sesión normal, sin controladores de diagnóstico, se repitió
la secuencia completa y además se paró y reactivó el seguimiento **sin volver a
seleccionar** el manipulador. Pasó; los giros medidos fueron +2.72°, −2.55° y
+2.82°. Registro final:
[production-validation.json](physical-control/2026-10-26/following-recovery/production-validation.json).
La sesión terminó en Gazebo, con seguimiento desactivado y sin fallo físico.

Suite completa: **164 pruebas Python pasaron**, 3 linters heredados
deseleccionados y el aviso existente de xacro. CTest: **2 de 2** pasaron.
[Salida Python](physical-control/2026-10-26/following-recovery/python-tests.log).

Herramienta reproducible, con zona despejada, brazo firme y pinza vacía:

```bash
python3 /root/mycobot_ros2_packages/tools/validate_physical_following.py \
  --execute-on-real-robot --moves --report /tmp/following-validation.json
```

Sin la opción explícita el script termina sin usar ROS ni mover el equipo.
Con `--moves` pide tres giros pequeños; también comprueba parar y reactivar
sin volver a seleccionar el destino. La limpieza desactiva seguimiento y vuelve
a Gazebo. No constituye una validación de todo el espacio de trabajo ni de carga.
