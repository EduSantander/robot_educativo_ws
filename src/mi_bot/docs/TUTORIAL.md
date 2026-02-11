# 🎓 Tutorial: Construyendo tu Primer Robot Diferencial en ROS2

Bienvenido. Esta guía desglosa la "magia" detrás de la simulación. Entenderás cómo conectamos la física, el código y el simulador.

---

## 1. La Estructura del Proyecto

A diferencia de C++, en Python usamos una estructura más simple. La clave es el archivo `setup.py`.

### ¿Por qué fallan muchos proyectos?
Porque olvidan decirle a Python que incluya los archivos "extra" (URDF y Launch).

**Concepto Clave:** En `setup.py`, la lista `data_files` es la encargada de mover tus modelos y lanzadores a la carpeta de instalación de ROS (`share/`).

```python
# setup.py
data_files=[
    # ...
    # Esto instala los archivos launch y urdf para que ROS los encuentre
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
],