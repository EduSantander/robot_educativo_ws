# 🤖 Mi Bot: Robot Diferencial Educativo (ROS2 Humble)

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Code-Python-yellow)
![License](https://img.shields.io/badge/License-Apache%202.0-green)

Este repositorio contiene un **robot educativo desarrollado en ROS 2**, que incluye:
- Definición del robot mediante **URDF**
- Lanzamiento de simulación
- Control del robot mediante **teleoperación (teleop)**

El objetivo del proyecto es servir como base para el aprendizaje de modelado, simulación y control básico de robots móviles en ROS 2. Para una explicación detallada sobre cómo se creó e implementó este proyecto desde cero, consulte el archivo [`docs/TUTORIAL.md`](docs/TUTORIAL.md).


---

## 📋 Requisitos

Antes de ejecutar el proyecto, asegúrate de tener instalado:

- Ubuntu 22.04
- ROS 2 (Humble recomendado)
- Colcon
- Git

Inicializa ROS 2 en tu terminal:

```bash
source /opt/ros/humble/setup.bash
```
---
## 📥 1. Clonar el repositorio

Clona el proyecto desde tu repositorio de GitHub:
```bash
git clone https://github.com/TU_USUARIO/robot_educativo_ws.git
```

Ingresa al workspace:
```bash
cd robot_educativo_ws
```
---
## 🔨 2.Compilar el workspace

Compila el proyecto con colcon:
```bash
colcon build
```

Luego, carga el entorno del workspace:
```bash
source install/setup.bash
```

💡 *Nota: Este comando debe ejecutarse en cada nueva terminal.*

---
## ▶️ 3. Ejecutar la simulación del robot

Lanza el archivo de simulación:
```bash
ros2 launch mi_bot sim.launch.py
```

Esto cargará el robot en el entorno de simulación configurado.

---
## 🎮 4. Control del robot con Teleop

En una nueva terminal, carga el entorno:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Ejecuta el nodo de teleoperación:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Usa las teclas indicadas en pantalla para mover el robot.

---
## 📁 Estructura del proyecto
```bash
robot_educativo_ws/
├── src/
│   └── mi_bot/
│       ├── launch/
│       ├── urdf/
│       ├── docs/
│       ├── package.xml
│       └── setup.py
├── build/
├── install/
└── log/
```
