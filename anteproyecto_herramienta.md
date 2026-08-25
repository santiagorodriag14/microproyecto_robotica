# Propuesta de Investigación: Metrología de alta precisión de grietas mediante láser ToF con compensación inercial en tiempo real para teleoperación

## 1. Planteamiento del Problema
### Contexto y Antecedentes
En el ámbito de los sistemas ciber-físicos y los ensayos no destructivos (END), la inspección remota de estructuras críticas mediante plataformas robóticas avanzadas —tales como robots escaladores y manipuladores teleoperados— ha cobrado vital importancia [F1, F6]. Sin embargo, la operación en entornos industriales complejos expone a estos sistemas a severas vibraciones mecánicas y perturbaciones dinámicas derivadas de los motores BLDC y del desplazamiento sobre superficies metálicas [F4, F8]. Tradicionalmente, la caracterización de daños superficiales se ha abordado mediante visión artificial y redes neuronales [F3], cuyas limitaciones en precisión métrica y susceptibilidad al ruido ambiental impiden obtener diagnósticos cuantitativos y auditables en tiempo real.

### Problema Específico
La ausencia de un marco metodológico estructurado que integre la adquisición de datos por láser Time-of-Flight (ToF) con la telemetría inercial de alta frecuencia en entornos de teleoperación genera incertidumbres métricas inaceptables. Específicamente, el sesgo de latencia en lazo cerrado, la deriva acumulativa de las unidades de medición inercial (IMU) y la falta de sincronización temporal determinista entre los sensores ópticos y los sistemas de compensación de movimiento provocan errores geométricos significativos al intentar medir la profundidad milimétrica de microfisuras.

### Soluciones Previas y Limitaciones
Los enfoques previos se han centrado en la implementación de robots móviles de inspección con arquitecturas de control cinemático convencionales [F2, F5] y en algoritmos de aprendizaje profundo para la clasificación visual de grietas [F7]. Las limitaciones de estas soluciones radican en su incapacidad para desacoplar el jitter mecánico y la vibración estructural de las lecturas espaciales, además de carecer de un pipeline determinista que transforme la distancia en perfiles tridimensionales exactos sin depender de interpretaciones subjetivas o inferencias probabilísticas opacas.

### Relevancia del Problema
Es sumamente relevante abordar este problema porque la industria requiere herramientas de END cuantitativas y confiables que eliminen la subjetividad en la teleoperación de alta criticidad [F8]. Contar con un sistema determinista de medición con compensación inercial en tiempo real garantiza la seguridad estructural, previene fallas catastróficas en infraestructuras y establece un nuevo estándar metrológico aplicable a la robótica de mantenimiento industrial [F6].

## 2. Preguntas de Investigación
### Pregunta Principal (Abierta / No Dicotómica)
¿De qué manera la integración síncrona de un arreglo de sensores láser ToF con un modelo de compensación inercial basado en estimación de estado estocástico permite alcanzar una precisión metrológica milimétrica en la medición de profundidad de grietas bajo condiciones de vibración en sistemas teleoperados?

### Preguntas Secundarias
- ¿Cuáles son los mecanismos mediante los cuales el desfase temporal y la deriva del IMU afectan la propagación de la incertidumbre métrica en el pipeline de adquisición ToF?
- ¿Cómo influye la arquitectura de sincronización determinista basada en sellos de tiempo de hardware en la reducción del jitter y la latencia de lazo cerrado durante la teleoperación?
- ¿En qué medida un protocolo de calibración y validación cruzada en banco de pruebas micrométrico garantiza la repetibilidad y reproducibilidad (R&R) del sistema metrológico propuesto?

## 2.5. Hipótesis de Investigación
### Hipótesis General (H1)
La implementación de un sistema determinista de medición basado en un arreglo de sensores láser ToF acoplado a una plataforma con compensación inercial en tiempo real mediante un Filtro de Kalman Extendido (EKF) y sincronización por hardware reduce el error métrico por debajo de 0.05 mm y mantiene una latencia de lazo cerrado inferior a 5 ms en entornos teleoperados.

### Hipótesis Específicas
- La sincronización temporal de los flujos de datos ópticos e inerciales mediante un bus de tiempo común elimina significativamente la desalineación espacial inducida por la vibración de los motores BLDC.
- El modelo de fusión sensorial predictiva compensa eficazmente la deriva del IMU, acotando la propagación de la incertidumbre en la estimación de la pose del efector final durante la inspección remota.

## 3. Justificación
### Justificación Científica
Aporta teóricamente al estado del arte en metrología óptica y sistemas ciber-físicos al proponer una alternativa determinista al procesamiento estocástico basado puramente en aprendizaje profundo [F3, F7], estableciendo modelos matemáticos rigurosos para la fusión sensorial en tiempo real bajo condiciones dinámicas severas [F1].

### Justificación Social
Genera un impacto social directo al incrementar la seguridad en la inspección de infraestructuras críticas y reducir el riesgo humano en entornos industriales peligrosos o de difícil acceso, garantizando auditorías estructurales transparentes y confiables [F6, F8].

### Justificación Tecnológica / Industrial
Proporciona una innovación tecnológica e industrial al desarrollar una arquitectura de hardware y software robusta para Ensayos No Destructivos (END) cuantitativos, elevando la madurez tecnológica (TRL) de los sistemas robóticos teleoperados de mantenimiento [F4, F5].

## 4. Alcance y Límites
### Alcance de la Propuesta
El alcance comprende el diseño, desarrollo, calibración y validación en banco de pruebas micrométrico de un prototipo funcional que integra sensores láser ToF, IMU y algoritmos de compensación inercial en tiempo real para la medición de grietas en superficies metálicas bajo teleoperación controlada.

### Limitaciones de la Propuesta
Las limitaciones metodológicas y de recursos incluyen la restricción del estudio a condiciones de vibración simuladas en laboratorio, la asunción de modelos de deriva estocástica acotados para el IMU y la dependencia de una infraestructura de red local con capacidad para cumplir umbrales de latencia estricts menores a 5 ms.

## 5. Ecuaciones de Búsqueda Bibliométrica
### 5.1 Ecuación Muy Específica (Alta Precisión)
```
"Time-of-Flight laser" AND "inertial compensation" AND "crack metrology" AND "teleoperation" AND "Extended Kalman Filter"
```

### 5.2 Ecuación Intermedia (Equilibrada)
```
("Time-of-Flight" OR "ToF laser") AND ("inertial measurement unit" OR "IMU drift" OR "inertial stabilization") AND ("crack depth measurement" OR "non-destructive testing") AND ("teleoperation" OR "tele-robotics")
```

### 5.3 Ecuación Muy Amplia (Alta Exhaustividad)
```
("optical metrology" OR "laser sensors" OR "ToF") AND ("vibration compensation" OR "sensor fusion" OR "state estimation") AND ("robotic inspection" OR "structural health monitoring" OR "END")
```

## 6. Fuentes Bibliográficas y Científicas Validadas
[F1] Diseño y Control de un Robot Cuadrúpedo Magnético Trepador
   URL: https://www.alphaxiv.org/es/abs/2504.13672
[F2] Robot Escalador que sube paredes metalicas | El Profe Garcia - Facebook
   URL: https://www.facebook.com/elprofe.garcia/videos/robot-escalador/1329650497159406/?locale=es_LA
[F3] Robótica on Instagram: "   Un robot que desafía la gravedad VertiGo es ...
   URL: https://www.instagram.com/reel/DV64iSwDH8o/
[F4] [PDF] Diseño y construcción de dos robots escaladores con normativas ...
   URL: https://nexoscientificos.vidanueva.edu.ec/index.php/ojs/article/download/6/6/20
[F5] Assembly of the UNIT Robot Climber V1 (Wall and Ceiling) - YouTube
   URL: https://www.youtube.com/watch?v=jm70W3FHgXs
[F6] REMLED I Robot escalador para mantenimiento y limpieza de edificios
   URL: https://www.researchgate.net/publication/318333556_REMLED_I_Robot_escalador_para_mantenimiento_y_limpieza_de_edificios
[F7] Metodología de diseño y control de robots escaladores
   URL: https://portalcientifico.unav.edu/documentos/5d5ba6f02999520e90d0606b?lang=gl
[F8] Diseño, construcción e implementación de un robot ...
   URL: https://dspace.ups.edu.ec/handle/123456789/3876?mode=full
