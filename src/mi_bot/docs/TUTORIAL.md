# 📘 Tutorial: Creación e Implementación de un Robot Diferencial en ROS 2

Este documento explica **cómo se construyó este proyecto desde cero**, mostrando cómo un modelo físico descrito en URDF se convierte en un **robot móvil funcional** dentro de Gazebo utilizando ROS 2.

El tutorial mantiene un enfoque práctico: cada concepto teórico se acompaña del código que lo implementa.

---

## 1️⃣ Creación del Workspace y del Paquete

Todo proyecto en ROS 2 comienza con un **workspace**, que agrupa uno o más paquetes.

```bash
mkdir -p robot_educativo_ws/src
cd robot_educativo_ws
```
Dentro de la carpeta ```src```, se crea el paquete del robot:
```bash
cd src
ros2 pkg create mi_bot --build-type ament_python
```

Este comando genera automáticamente:
- ```package.xml```
- ```setup.py```
- ```setup.cfg```
- Estructura base del paquete

---
## 2️⃣ Organización del Paquete

Para mantener el proyecto ordenado, se crean carpetas con propósitos claros:

```bash
cd mi_bot
mkdir launch urdf docs
```
- ```urdf/``` → Modelo físico del robot
- ```launch/``` → Archivos de lanzamiento
- ```docs/``` → Documentación

Esta estructura facilita el mantenimiento y la escalabilidad del proyecto.

---
## 3️⃣ El Modelo del Robot (URDF)

El archivo ```robot.urdf``` describe la estructura física y cinemática del robot.

En ROS, un robot se define como:

- Links → partes rígidas
- Joints → uniones entre links

---
## 4️⃣ Sistema de Referencia Base
- **A.** ```base_footprint```

El robot comienza con un link vacío llamado ```base_footprint```.

**Concepto clave:**
```base_footprint``` representa el punto del robot proyectado sobre el suelo.
Es muy utilizado en navegación y evita que las transformaciones se vean afectadas por la altura del chasis.

```xml
<link name="base_footprint"/>
```

- **B. Unión con el Chasis**

Se conecta ```base_footprint``` con el cuerpo del robot mediante un joint fijo:

```xml
<joint name="base_joint" type="fixed">
  <parent link="base_footprint"/>
  <child link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

Esto indica que el chasis no se mueve respecto al suelo, solo se traslada y rota como un todo.

---
## 5️⃣ El Chasis (```base_link```)

El chasis es el cuerpo principal del robot.

**Concepto clave:**
Para que Gazebo pueda simular la física del robot, todo link debe tener propiedades inerciales.

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.4 0.3 0.1"/>
    </geometry>
    <material name="blue"/>
  </visual>

  <collision>
    <geometry>
      <box size="0.4 0.3 0.1"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.02" ixy="0" ixz="0"
             iyy="0.02" iyz="0"
             izz="0.02"/>
  </inertial>
</link>
```

- ```visual``` → apariencia
- ```collision``` → interacción física
- ```inertial``` → comportamiento dinámico

---
## 6️⃣ Ruedas del Robot

Las ruedas se definen como cilindros independientes.

```xml
<link name="left_wheel">
  <visual>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
    <material name="black"/>
  </visual>
</link>
```

Cada rueda es un link separado para permitir su rotación.

---
## 7️⃣ Articulaciones de las Ruedas

Las ruedas se conectan al chasis mediante joints de tipo ```continuous```.

**Concepto clave:**
Un joint ```continuous``` permite rotación infinita, ideal para ruedas.

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <origin xyz="0 0.175 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```
- ```parent``` → chasis
- ```child``` → rueda
- ```axis``` → eje de giro

---
## 8️⃣ Movimiento del Robot: Plugin de Gazebo

Un URDF por sí solo describe geometría, **no movimiento**.
Para controlar el robot en simulación se utiliza un **plugin de Gazebo**.

- Plugin de Tracción Diferencial

Este plugin permite que el robot se mueva a partir de comandos de velocidad.

```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">

    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>

    <wheel_separation>0.35</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>

    <max_wheel_torque>20</max_wheel_torque>

    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>

    <robot_base_frame>base_footprint</robot_base_frame>
    <odometry_frame>odom</odometry_frame>

    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>

  </plugin>
</gazebo>
```

¿Qué hace este plugin?

1. Escucha ```/cmd_vel```
2. Calcula la velocidad de cada rueda
3. Aplica torque en Gazebo
4. Publica odometría

---
## 9️⃣ Archivo de Lanzamiento (```sim.launch.py```)

El archivo launch permite ejecutar todo el sistema con un solo comando.

- A. Iniciar Gazebo

```python
ExecuteProcess(
    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
    output='screen'
)
```

Esto inicia Gazebo con soporte para ROS 2.

- B. Publicar el Estado del Robot

```python
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    arguments=[urdf_file]
)
```

Este nodo publica las transformaciones (TF) del robot.

- C. Spawnear el Robot

```python
Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-topic', 'robot_description', '-entity', 'mi_bot'],
    output='screen'
)
```

Este nodo inserta el robot dentro del mundo de Gazebo.

---
## 🔟 Compilación y Ejecución

```bash
colcon build
source install/setup.bash
```

Ejecutar la simulación:

```bash
ros2 launch mi_bot sim.launch.py
```

---
## 1️⃣1️⃣ Flujo Completo de Funcionamiento

- El teclado publica un mensaje ```Twist```
- El mensaje viaja por ```/cmd_vel```
- El plugin ```diff_drive``` calcula velocidades
- Gazebo aplica torque a las ruedas
- El robot se mueve y publica odometría

---
## 🔚 Conclusión

Este proyecto muestra el flujo completo de desarrollo de un robot móvil en ROS 2:

- Creación del paquete
- Modelado físico con URDF
- Control mediante plugins
- Lanzamiento coordinado con archivos launch

La base está lista para extenderse con sensores, navegación autónoma o interfaces gráficas.
