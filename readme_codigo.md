# 🤖 LIDAR Mapping System - Webots

> Sistema de mapeo 2D y detección de obstáculos en tiempo real usando sensor LIDAR en Webots

[![Webots](https://img.shields.io/badge/Webots-R2025a-blue.svg)](https://cyberbotics.com/)
[![C++](https://img.shields.io/badge/C++-11-00599C.svg?logo=c%2B%2B)](https://isocpp.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Functional-success.svg)]()

---

## 📋 Descripción

Sistema completo de procesamiento de datos **LIDAR** para generar mapas 2D (occupancy grid) y detectar obstáculos en tiempo real. Implementado en C++ para el simulador **Webots** usando un robot **E-puck** equipado con sensor LIDAR de 360°.

### ✨ Características Principales

- 🗺️ **Mapeo 2D en Tiempo Real** - Generación de mapa de ocupación (200x200 celdas)
- 🎯 **Detección de Obstáculos** - Identificación y tracking de obstáculos
- 📊 **Visualización Integrada** - Display 400x400px con actualización en vivo
- 🎮 **Control Manual** - Control del robot mediante teclado
- 🧩 **Código Modular** - Arquitectura orientada a objetos con 4 clases principales

---

## 🎥 Demo

**Display en tiempo real:**
- Gris = No explorado
- Blanco = Espacio libre
- Negro = Obstáculos
- Azul = Robot
- Rojo = Detecciones actuales

---

## 🚀 Quick Start

### Requisitos

- [Webots R2025a+](https://cyberbotics.com/)
- MinGW/GCC (incluido con Webots)
- Windows/Linux/macOS

### Instalación

```bash
# Clonar el repositorio
git clone https://github.com/tu-usuario/lidar-mapping-webots.git
cd lidar-mapping-webots

# Compilar el controlador
cd controllers/lidar_mapper
set WEBOTS_HOME=D:\Webots  # Ajustar según tu instalación
mingw32-make
```

### Ejecución

1. Abrir **Webots R2025a**
2. `File → Open World → worlds/lidar_mapping.wbt`
3. Presionar ▶️
4. Controlar con teclado:
   - `↑↓` - Avanzar/Retroceder
   - `←→` - Girar
   - `ESPACIO` - Detener
   - `R` - Resetear mapa

---

## 🏗️ Arquitectura del Código

El controlador `lidar_mapper.cpp` (580 líneas) está estructurado en **4 clases principales**:

### 1️⃣ `OccupancyGrid` - Mapa de Ocupación

Implementa un grid probabilístico 2D para representar el entorno.

```cpp
class OccupancyGrid {
    vector<vector<double>> grid;  // Probabilidades de ocupación [0,1]

    bool worldToGrid(double x, double y, int &gridX, int &gridY);
    void updateCell(int x, int y, bool occupied);  // Actualización Bayesiana
    double getCell(int x, int y);
};
```

**Funcionamiento:**
- **Tamaño:** 200x200 celdas (10m x 10m reales)
- **Resolución:** 5cm por celda
- **Actualización:** Modelo Bayesiano probabilístico
  - `P(occupied) > 0.6` → Obstáculo (negro)
  - `P(occupied) < 0.4` → Libre (blanco)
  - `P(occupied) ≈ 0.5` → Desconocido (gris)

---

### 2️⃣ `LidarProcessor` - Procesamiento LIDAR

Procesa los datos crudos del sensor y actualiza el mapa.

```cpp
class LidarProcessor {
    void processScan(double robotX, double robotY, double robotTheta);
    void updateRay(double x0, double y0, double x1, double y1, float range);
};
```

**Funcionamiento:**
1. Lee 360 puntos del sensor LIDAR
2. Convierte coordenadas polares → cartesianas
3. **Ray Tracing** con algoritmo de Bresenham:
   - Marca celdas a lo largo del rayo como **libres**
   - Marca punto final como **ocupado**
4. Actualiza el `OccupancyGrid` con cada medición

---

### 3️⃣ `ObstacleDetector` - Detección de Obstáculos

Identifica obstáculos a partir de lecturas LIDAR.

```cpp
struct Obstacle {
    double x, y;        // Posición relativa al robot
    double distance;    // Distancia euclidiana
    double angle;       // Ángulo desde el frente
};

class ObstacleDetector {
    vector<Obstacle> detectObstacles(Lidar *lidar);
    Obstacle* findClosest(vector<Obstacle> &obstacles);
};
```

**Funcionamiento:**
- Filtra lecturas LIDAR dentro del rango configurado
- Calcula posición relativa de cada obstáculo
- Identifica el obstáculo más cercano
- Imprime en consola distancia y ángulo

---

### 4️⃣ `DisplayRenderer` - Visualización

Renderiza el mapa y datos en el Display de Webots.

```cpp
class DisplayRenderer {
    void renderGrid(const OccupancyGrid &grid);
    void drawRobot(double x, double y, double theta);
    void drawObstacles(const vector<Obstacle> &obstacles, ...);
    void drawText(const string &text, int x, int y);
};
```

**Funcionamiento:**
- Convierte probabilidades del grid a colores (escala de grises)
- Dibuja robot como círculo azul + línea de orientación
- Dibuja obstáculos detectados como puntos rojos
- Muestra información textual (contador, posición)
- **Optimización:** Actualiza cada 5 frames (~320ms)

---

## 🧮 Algoritmos Implementados

### Bresenham Line Algorithm
- **Propósito:** Ray tracing eficiente en grid discreto
- **Complejidad:** O(max(dx, dy))
- **Uso:** Trazar línea desde robot hasta punto de impacto LIDAR

### Bayesian Occupancy Update
- **Propósito:** Actualizar probabilidades con múltiples mediciones
- **Fórmula:**
  ```
  P_posterior = (likelihood × P_prior) /
                (likelihood × P_prior + (1 - likelihood) × (1 - P_prior))
  ```
- **Ventaja:** Converge asintóticamente con más observaciones

### Odometría Diferencial
- **Propósito:** Estimar posición del robot
- **Parámetros:** Radio de ruedas (2cm), distancia entre ruedas (5.2cm)
- **Limitación:** Acumula error con el tiempo (sin corrección GPS/SLAM)

---

## 📁 Estructura del Proyecto

```
lidar-mapping-webots/
├── controllers/
│   └── lidar_mapper/
│       ├── lidar_mapper.cpp        # Código principal (580 líneas)
│       ├── lidar_mapper.exe        # Ejecutable compilado
│       ├── Makefile
│       └── README.md
├── worlds/
│   └── lidar_mapping.wbt          # Mundo de simulación
├── README.md                       # Este archivo
├── LIDAR_SYSTEM_DOCUMENTATION.md  # Documentación completa
└── COMPILATION_GUIDE.md           # Guía de compilación
```

---

## ⚙️ Configuración y Parámetros

Puedes ajustar estos parámetros en `lidar_mapper.cpp`:

```cpp
// Mapa
const double MAP_SIZE = 10.0;          // Tamaño en metros
const int GRID_SIZE = 200;             // Resolución del grid
const double OBSTACLE_THRESHOLD = 0.6; // Umbral de obstáculo
const double FREE_THRESHOLD = 0.4;     // Umbral de espacio libre

// Control
const double MAX_SPEED = 3.0;          // Velocidad máxima (rad/s)
const double TURN_SPEED = 1.5;         // Velocidad de giro (rad/s)
```

Sensor LIDAR configurado en `lidar_mapping.wbt`:
```vrml
Lidar {
  horizontalResolution 360    # Puntos por escaneo
  fieldOfView 6.28           # 360° en radianes
  maxRange 8                 # Rango máximo (metros)
}
```

---

## 🔧 Tecnologías Utilizadas

- **Lenguaje:** C++ (estándar C++11)
- **Simulador:** Webots R2025a
- **Robot:** E-puck (ruedas diferenciales)
- **Sensor:** LIDAR 2D (360°, 8m rango)
- **Librerías:**
  - `webots::Robot`
  - `webots::Lidar`
  - `webots::Display`
  - `webots::Motor`
  - `webots::Keyboard`

---

## 📊 Rendimiento

| Métrica | Valor |
|---------|-------|
| Frecuencia de actualización | ~15.6 Hz |
| Puntos LIDAR por escaneo | 360 |
| Tamaño del mapa | 200x200 celdas |
| Memoria del grid | ~160 KB |
| Tamaño ejecutable | 53 KB |
| Líneas de código | 580 |

---



## 📸 Screenshots

**Consola con detección de obstáculos:**
```
=== LIDAR Mapper Controller ===
Closest obstacle: dist=1.23m, angle=45.3deg
Obstacles: 5 | Pos: (0.45, 1.23)
```
<img width="895" height="580" alt="image" src="https://github.com/user-attachments/assets/71172c73-0459-40e8-969e-0cbf9449bbfd" />

---

<div align="center">

**⭐ Si este proyecto te fue útil, considera darle una estrella ⭐**

Made with ❤️ using Webots

</div>
