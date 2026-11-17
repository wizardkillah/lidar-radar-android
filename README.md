# LIDAR Radar (Android)

App Android que convierte un smartphone **sin LiDAR** en un escáner 3D / radar de interiores, combinando:

- **ARCore Depth** (profundidad estimada por movimiento) para construir un mapa 3D acumulado en coordenadas del mundo.
- **Sonar acústico** (chirp de 18–21 kHz con filtro adaptado vía FFT) para medir distancia al eco más cercano.
- **Fusión**: el sonar se contrasta con el obstáculo más cercano de ARCore ("CONFIRMADO").

Desarrollada y probada en un Redmi Note 13 Pro 5G.

## Características

- Nube de puntos 3D con color real de la cámara (YUV → RGB), modos de color por altura y por distancia.
- Visor OpenGL ES 2.0 orbitable (un dedo gira, pellizco hace zoom), grilla de piso y ejes de origen fijos.
- Filtros de calidad: rango 0,4–3 m, confianza dependiente de la distancia, descarte de bordes de profundidad,
  voxelización de 4 cm, confirmación por 4 observaciones y promedio ponderado por 1/z².
- Compuerta de captura: solo integra con desplazamiento real (paralaje) y sin giros bruscos.
- Piso real detectado con planos de ARCore.
- Corrección de deriva: la nube se guarda en bloques unidos a anclas de ARCore.
- Exportación a **PLY** (carpeta Descargas), compatible con CloudCompare / MeshLab.
- Radar 2D cenital en miniatura.

## Estructura

| Archivo | Rol |
|---|---|
| `MainActivity.kt` | Sesión ARCore, fusión profundidad+pose, render OpenGL, UI, exportación |
| `PointCloud.kt` | Nube de puntos con tabla hash de voxeles, promedio ponderado y filtro por hits |
| `SonarEngine.kt` | Emisión/captura de chirps y correlación por FFT |
| `RadarView.kt` | Vista de radar 2D |

## Compilar

Requisitos: JDK 17+, Android SDK (plataforma 36), dispositivo compatible con ARCore.

```bash
./gradlew assembleDebug
```

La APK queda en `app/build/outputs/apk/debug/app-debug.apk`. Crear `local.properties` con `sdk.dir=<ruta al SDK>`.

> En Windows, si Gradle falla con `Unable to establish loopback connection`, ejecutar con
> `JAVA_TOOL_OPTIONS=-Djdk.net.unixdomain.tmpdir=C:\jtmp` (carpeta existente con ruta corta).

## Uso

1. Aceptar permisos de cámara y micrófono; subir el volumen multimedia para el sonar.
2. Caminar despacio apuntando a paredes y objetos a 1–2 m; no girar en el lugar.
3. Botones: reiniciar, guardar PLY, mostrar/ocultar radar, modo de color, vista fija/mapa, centrar.

## Limitaciones

- La profundidad es **estimada** (no hay sensor ToF/LiDAR): ruidosa y con alcance útil ~3 m.
- La banda de sonar no está calibrada para todos los dispositivos.
- La deriva solo se corrige donde ARCore reconoce lugares ya vistos.
