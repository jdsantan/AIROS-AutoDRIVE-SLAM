# ROS 2 AutoDRIVE F1Tenth - Mapeo Simultáneo (SLAM) y Teleoperación

Este repositorio documenta la implementación para lograr el mapeo simultáneo (SLAM) de un circuito utilizando el vehículo F1Tenth en el simulador **AutoDRIVE** y **ROS 2 Humble**. 

El objetivo principal de este proyecto es resolver los graves desafíos de integración que surgen al comunicar un simulador gráfico en Windows (Host) con un entorno de procesamiento de ROS 2 en Ubuntu (Guest VM). Se abordan soluciones específicas para la desincronización temporal (*timestamps*), el cálculo de odometría ausente y el formato regional de datos.

## 📷 Galería y Resultados (Recomendaciones Visuales)

*(Sugerencia para el repositorio: Añade aquí los siguientes archivos visuales para darle peso a tu trabajo)*
* **`mapa_final.png`**: Una captura de pantalla del mapa perfecto (la versión limpia donde se ven las curvas del óvalo).
* **`error_ghosting.png`**: Una captura del mapa "feo" o desdoblado para mostrar qué pasa cuando falla la odometría.
* **`tf_tree.pdf` o `.png`**: El diagrama de tu árbol TF generado con `ros2 run tf2_tools view_frames`.
* **`demo_mapeo.gif`**: Un pequeño GIF o video corto (acelerado) mostrando la pantalla dividida: de un lado manejando en Windows y del otro RViz2 construyendo el mapa en tiempo real.

## 🚀 Arquitectura del Sistema y Solución de Problemas

Para lograr un mapeo limpio, no bastaba con conectar los sistemas; fue necesario intervenir la telemetría "cruda" del simulador. El sistema se compone de los siguientes nodos:

### 1. Los Puentes Base (WebSocket a ROS 2)
AutoDRIVE se comunica mediante el puerto `4567` usando `Socket.IO`. 
* **El Problema:** El simulador requiere el protocolo antiguo `EIO=3`, el cual es incompatible con las versiones modernas de Python.
* **La Solución:** Creación de un entorno virtual (`venv`) estricto con `python-socketio==4.2.0` y `python-engineio==3.13.0`.

### 2. Acondicionamiento Numérico (Data Parsing)
* **El Problema:** Debido a la configuración regional de Windows (Latinoamérica/España), el simulador enviaba matrices espaciales (LIDAR, IPS) separando los decimales con comas (ej. `0,059`), provocando errores fatales (`ValueError`) al intentar procesarlos con `NumPy` en Linux.
* **La Solución:** Se intervino el `autodrive_incoming_bridge.py` aplicando un reemplazo global (parsing) a la telemetría antes de su procesamiento:
```python
# Corrección de separador decimal regional
data = {k: v.replace(",", ".") if isinstance(v, str) else v for k, v in data.items()}
3. El Problema del "Mapa Fantasma" y la Odometría Dinámica (odom_publisher.py)
El Problema: Al mapear curvas, el mapa se distorsionaba y las paredes se duplicaban (efecto "ghosting"). Esto ocurría porque el sistema carecía de odometría angular; el SLAM dependía únicamente del escaneo LiDAR sin saber cuánto había rotado el vehículo.

La Solución: Se desarrolló el nodo odom_publisher.py. Este script se suscribe a dos sensores:

El IPS (Indoor Positioning System) para obtener las coordenadas espaciales (X, Y, Z).

El IMU (Inertial Measurement Unit) para extraer la orientación real en cuaterniones (x, y, z, w).
Con estos datos fusionados, el nodo publica un paquete completo de Odometría y un TF dinámico (map -> odom -> f1tenth_1). Además, emite un TF estático para conectar el chasis con el sensor: f1tenth_1 -> lidar.

4. Filtro de Sincronización Temporal (lidar_republisher.py)
El Problema: El nodo slam_toolbox descartaba masivamente los escaneos LiDAR arrojando el error "the timestamp on the message is earlier than all the data in the transform cache". Esto se debía a una desincronización irreconciliable de milisegundos entre el reloj de Windows y el de la máquina virtual Ubuntu.

La Solución: Se programó el nodo lidar_republisher.py. Este nodo actúa como un interceptor: recibe el paquete original del LiDAR, elimina la marca de tiempo (timestamp) obsoleta de Windows, le inyecta la hora actual del reloj de Linux, y lo publica en un tópico limpio llamado /lidar_fixed.

🛠️ Requisitos e Instalación
Entorno
Windows (Host): AutoDRIVE Simulator.

Ubuntu (Guest VM): ROS 2 Humble.

Red: Conexión en Adaptador Puente entre Windows y la máquina virtual.

Dependencias (Obligatorias)
Para evitar conflictos con librerías del sistema operativo base, es estrictamente necesario aislar el entorno.

Bash
# Navegar al workspace
cd ~/autodrive_ws
# Crear y activar entorno virtual
python3 -m venv venv
source venv/bin/activate
# Instalar dependencias exactas
pip install --force-reinstall eventlet==0.33.3 Flask-SocketIO==4.1.0 python-socketio==4.2.0 python-engineio==3.13.0 gevent-websocket==0.10.1
🗺️ Ejecución Paso a Paso (4 Terminales)
Para levantar el sistema correctamente, abre 4 terminales, asegurándote de activar el entorno virtual y exportar el PYTHONPATH en cada una.

Terminal 1: Iniciar los Puentes (Bridges)

Bash
cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source install/setup.bash
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
(Nota: Conecta el simulador AutoDRIVE en Windows INMEDIATAMENTE DESPUÉS de ejecutar este comando).

Terminal 2: Publicador de Odometría Físico-Dinámica

Bash
cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source install/setup.bash
python3 ~/autodrive_ws/odom_publisher.py
Terminal 3: Filtro Temporal del LiDAR

Bash
cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source install/setup.bash
python3 ~/autodrive_ws/lidar_republisher.py
Terminal 4: Iniciar SLAM Toolbox

Bash
cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source install/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p use_sim_time:=false \
  -p base_frame:=f1tenth_1 \
  -p odom_frame:=odom \
  -p map_frame:=map \
  -p transform_timeout:=5.0 \
  -p tf_buffer_duration:=60.0 \
  -r /scan:=/lidar_fixed
Teleoperación y Guardado
Una vez inicializados los 4 terminales, conduce el vehículo manualmente utilizando los controles del simulador en Windows (W, A, S, D). Para obtener un resultado óptimo y evitar problemas de "Loop Closure", se recomienda dar una sola vuelta al circuito a baja velocidad.

Para guardar el mapa finalizado, abre una terminal adicional y ejecuta:

Bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/mapa_autodrive
