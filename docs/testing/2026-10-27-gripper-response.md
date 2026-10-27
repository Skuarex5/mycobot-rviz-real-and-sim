# Respuesta física de pinza y preparación de trayectorias

El reporte de apertura/cierre irregular y seguimiento retrasado se investigó en
la sesión integrada con el controlador USB único y la pinza original.

## Evidencia anterior

Los registros del controlador y MoveIt mostraron acciones de pinza fallidas
por convergencia o timeout, bloqueando el siguiente movimiento del brazo durante
10–23 segundos. El ejecutor enviaba cada muestra del plan como una orden física,
esperando su instante nominal y confirmación de reposo antes de continuar.
Las órdenes de posición del dispositivo tienen su propio perfil de movimiento.

Además, el compactador del brazo volvía a revisar todos los prefijos de una
trayectoria recta. En una reproducción con 1.400 puntos inspeccionó 980.699
puntos acumulados y tardó 6,91 segundos antes de producir una sola orden.
Los registros reales mostraban intervalos similares entre aceptación y envío.

## Correcciones

- El compactador comprueba primero todo el tramo restante con las mismas reglas
  de desviación y monotonía. Conserva el algoritmo anterior cuando hay curvas
  o cambios de dirección. La reproducción recta tarda ahora 0,01116 segundos.
- La pinza valida todos los puntos originales y conserva los extremos de cada
  tramo monótono, incluidos los cambios de dirección. Evita reiniciar el
  movimiento del dispositivo con cada pequeña muestra del plan.
- El último punto puede terminar al alcanzar la tolerancia y confirmar reposo,
  sin esperar además un instante nominal futuro. Los puntos de cambio de
  dirección conservan su espera y sus timestamps originales.
- Cada orden de pinza registra objetivo, valor del dispositivo y tiempo nominal;
  un timeout incluye también la medición que no convergió.

No se modificaron calibración, tolerancias, velocidades, factores del planificador,
propiedad del USB, autorización de sesión ni vigilancia de medidas y parada.

## Verificación

Las cuatro regresiones nuevas fallaron primero: exploración cuadrática,
microórdenes de pinza, espera del tiempo final pese a reposo y conservación de
extremos al invertir dirección. Después pasaron las 49 pruebas de trayectoria y
pinza, y la suite Python completa: **193 pasaron**, con un aviso heredado de xacro.
[Salida completa](response-improvements/2026-10-27/pytest.log).

Se reinició el lanzamiento integrado para cargar el código Python actualizado.
El operador confirmó pinza vacía y autorizó una apertura pequeña de 0,06 rad del
modelo seguida del regreso al objetivo inicial, sin mover el brazo. Se utilizaron
MoveGroup, el coordinador y las acciones de trayectoria normales.

| Movimiento | Inicio medido desde solicitud | Duración total | Resultado |
| --- | --- | --- | --- |
| Apertura pequeña | 0,869 s | 1,731 s | SUCCESS |
| Regreso | 0,992 s | 1,594 s | SUCCESS |

La apertura solicitada fue de −0,35227 a −0,29227 rad; la posición medida al
finalizar fue −0,30795 rad. Al volver se midieron −0,34489 rad. Ambos errores
quedaron dentro de la tolerancia existente de 0,03 rad. El inicio se define por
un cambio medido superior a 0,01 rad. El cambio máximo medido del brazo fue cero.
[Registro físico](response-improvements/2026-10-27/gripper-result.json).

La sesión quedó en Gazebo, seguimiento desactivado, USB conectado y sin fallo,
con parada verificada. No se ensayaron los extremos mecánicos completos de la
pinza ni movimientos físicos del brazo. El benchmark mide preparación de una
trayectoria recta; no mide latencia completa de seguimiento. Este sigue usando
planes sucesivos y espera que termine la acción actual antes de ejecutar el
último objetivo pendiente.
