# LIDAR Mapping System - Webots

Sistema de mapeo 2D y detección de obstáculos en tiempo real usando LIDAR.

## Quick Start

### 1. Compilar

```bash
cd controllers/lidar_mapper
set WEBOTS_HOME=D:\Webots
mingw32-make clean
mingw32-make
```

### 2. Ejecutar

1. Abrir **Webots R2025a**
2. Cargar mundo: `File → Open World → worlds/lidar_mapping.wbt`
3. Presionar ▶️ Play
4. Controlar con flechas del teclado

## Controles

| Tecla | Acción |
|-------|--------|
| ↑↓ | Avanzar/Retroceder |
| ←→ | Girar |
| Espacio | Detener |
| R | Resetear mapa |

## Visualización

- **Gris** = No explorado
- **Blanco** = Espacio libre
- **Negro** = Obstáculos
- **Azul** = Robot
- **Rojo** = Detecciones actuales

## Estructura

```
proyectos_webots/
├── controllers/
│   └── lidar_mapper/         # Controlador C++ compilado
│       ├── lidar_mapper.cpp  # Código fuente (580 líneas)
│       └── lidar_mapper.exe  # Ejecutable (53KB)
├── worlds/
│   └── lidar_mapping.wbt     # Mundo de simulación
└── LIDAR_SYSTEM_DOCUMENTATION.md  # Documentación completa
```

## Componentes

- **OccupancyGrid** - Mapa 2D (200x200 celdas, 10m x 10m)
- **LidarProcessor** - Procesa LIDAR con ray tracing
- **ObstacleDetector** - Detecta obstáculos en tiempo real
- **DisplayRenderer** - Visualización en Display

## Estado

✅ Compilado exitosamente  
✅ Ejecutable: 53KB  
✅ Listo para usar

## Documentación Completa

Ver: [LIDAR_SYSTEM_DOCUMENTATION.md](LIDAR_SYSTEM_DOCUMENTATION.md)

## Requisitos

- Webots R2025a+
- MinGW/GCC

## License

Apache 2.0
