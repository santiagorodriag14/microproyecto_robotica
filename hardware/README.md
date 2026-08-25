# 🛠️ Hardware y Diseño (hardware/)

Este directorio está destinado a los recursos físicos, diseños 3D y componentes electrónicos del robot y los sensores.

## Estructura Recomendada
*   **`/cad`**: Archivos de diseño 3D (archivos fuente como `.f3d`, `.sldprt` y exportables como `.stl`, `.step`) para la impresión o manufactura. Ejemplo: el soporte acoplado para el láser ToF y el IMU.
*   **`/esquemas`**: Diagramas de circuitos (Schematics) y diseños de placas (PCB) necesarios para el bus de tiempo común y sincronización de hardware.

## Notas para el equipo
Asegúrense de versionar correctamente los modelos 3D. Es recomendable exportar siempre un `.step` o `.stl` junto con el archivo nativo del programa CAD para que cualquiera en el equipo pueda visualizarlo rápidamente sin necesidad del software original.