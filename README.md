# ROS 2 AutoDRIVE F1Tenth - Mapeo Simultáneo (SLAM) y Teleoperación

Este repositorio documenta la implementación completa para lograr el mapeo simultáneo (SLAM) de un circuito utilizando el vehículo F1Tenth en el simulador **AutoDRIVE** y **ROS 2 Humble**. 

El proyecto resuelve desafíos críticos de integración que surgen al comunicar un simulador gráfico en Windows (Host) con un entorno de procesamiento de ROS 2 en Ubuntu (Guest VM en VirtualBox). Se abordan soluciones específicas para la desincronización temporal (timestamps), el cálculo de odometría dinámica y el formato regional de datos.

## 📷 Galería y Resultados

Para una mejor visualización del proyecto, en la carpeta `media` se encuentran los siguientes archivos:
1. **`mapa_final.png`**: Captura del mapa limpio del circuito óvalo.
2. **`error_ghosting.png`**: Captura del mapa desdoblado (muestra el problema inicial sin odometría).
3. **`tf_tree.png`**: Diagrama del árbol de transformadas.
4. **`demo_mapeo.gif`**: Video corto mostrando RViz2 construyendo el mapa durante la teleoperación.

---

## 🚀 Arquitectura del Sistema y Solución de Problemas

Para lograr un mapeo preciso, se desarrollaron nodos personalizados que intervienen la telemetría antes de enviarla al algoritmo de SLAM:

### 1. Puentes de Comunicación (WebSockets)
La conexión usa `Socket.IO` por el puerto `4567`.
* **Problema:** Incompatibilidad de protocolos (`EIO=3` del simulador vs versiones modernas de Python).
* **Solución:** Uso de un entorno virtual (`venv`) con versiones específicas: `python-socketio==4.2.0` y `python-engineio==3.13.0`.

### 2. Acondicionamiento de Datos (Parsing)
* **Problema:** Windows enviaba datos con comas decimales (ej. `0,75`) debido a la configuración regional, lo que causaba errores `ValueError` en Linux.
* **Solución:** Modificación del bridge de entrada para reemplazar comas por puntos dinámicamente antes de que NumPy procese las matrices.

### 3. Odometría Dinámica y Árbol TF (odom_publisher.py)
* **Problema:** El mapa se duplicaba en las curvas (efecto "fantasma") porque el sistema no sabía el ángulo de rotación del carro.
* **Solución:** Se creó un nodo que fusiona la posición del sensor **IPS** con la orientación real del **IMU** (en Cuaterniones). Esto genera un árbol TF dinámico completo: `map` -> `odom` -> `f1tenth_1`.

### 4. Sincronización Temporal (lidar_republisher.py)
* **Problema:** `slam_toolbox` descartaba masivamente los datos del láser porque los relojes de Windows y Ubuntu no estaban perfectamente sincronizados (error de datos en el pasado).
* **Solución:** Se creó un nodo interceptor que recibe los datos del LiDAR, borra el tiempo de Windows, estampa la hora exacta actual de Linux y republica los datos limpios en el tópico `/lidar_fixed`.

---

## 🛠️ Instalación y Dependencias

Es obligatorio aislar las librerías en un entorno virtual para evitar conflictos con el sistema base de Ubuntu:

```bash
cd ~/autodrive_ws
python3 -m venv venv
source venv/bin/activate
pip install --force-reinstall eventlet==0.33.3 Flask-SocketIO==4.1.0 python-socketio==4.2.0 python-engineio==3.13.0 gevent-websocket==0.10.1
```
🗺️ Guía de Ejecución (4 Terminales)
Ejecuta cada comando en una terminal diferente. Importante: Siempre activa el entorno virtual y exporta el PYTHONPATH antes de lanzar los nodos.

Terminal 1: Bridge y RViz
```bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
(Nota: Conecta el simulador en Windows Inmediatamente después de ejecutar esto).
```

Terminal 2: Nodo de Odometría
```bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ~/autodrive_ws/odom_publisher.py
```

Terminal 3: Nodo de LiDAR
```bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ~/autodrive_ws/lidar_republisher.py
```

Terminal 4: SLAM Toolbox
```bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p use_sim_time:=false \
  -p base_frame:=f1tenth_1 \
  -p odom_frame:=odom \
  -p map_frame:=map \
  -p transform_timeout:=5.0 \
  -p tf_buffer_duration:=60.0 \
  -r /scan:=/lidar_fixed
```

💾 Guardar el Mapa
Cuando termines de recorrer el circuito completo conduciendo lentamente con W, A, S, D desde el simulador en Windows, abre una nueva terminal y guarda el resultado:

```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/mapa_autodrive
