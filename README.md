
# 🏎️ ROS 2 AutoDRIVE F1Tenth - Mapeo Simultáneo (SLAM) y Teleoperación

Este repositorio documenta la implementación completa para lograr el mapeo simultáneo (SLAM) de un circuito utilizando el vehículo F1Tenth en el simulador **AutoDRIVE** y **ROS 2 Humble**. 

El proyecto resuelve desafíos críticos de integración que surgen al comunicar un simulador gráfico en Windows (Host) con un entorno de procesamiento de ROS 2 en Ubuntu (Guest VM en VirtualBox). Se abordan soluciones específicas para la desincronización temporal (*timestamps*), el cálculo de odometría dinámica y el formato regional de datos.

---

## 📷 Galería y Resultados

Para una mejor visualización del proyecto, en la carpeta `media` se encuentran los siguientes archivos:
1. **`mapa_final.png`**: Captura del mapa limpio del circuito óvalo.
2. **`error_ghosting.png`**: Captura del mapa desdoblado (muestra el problema inicial sin odometría).
3. **`tf_tree.png`**: Diagrama del árbol de transformadas.
4. **`demo_mapeo.gif`**: Video corto mostrando RViz2 construyendo el mapa durante la teleoperación.

---

## 🚀 Arquitectura del Sistema y Solución de Problemas

Para lograr un mapeo preciso, se desarrollaron nodos personalizados que intervienen la telemetría antes de enviarla al algoritmo de SLAM:

### 🔌 1. Puentes de Comunicación y WebSockets (`Socket.IO`)
La conexión bidireccional entre ROS 2 y el simulador AutoDRIVE se realiza a través del puerto TCP `4567` utilizando WebSockets.

* **El Problema (Incompatibilidad de Protocolos):** Al intentar conectar el simulador, el servidor levantado por ROS 2 rechazaba la conexión silenciosamente o arrojaba el siguiente error en la terminal:
  ```text
  [autodrive_incoming_bridge-1] The client is using an unsupported version of the Socket.IO or Engine.IO protocols
  ```
  Esto ocurre porque el simulador AutoDRIVE utiliza una arquitectura *legacy* basada en la versión 3 del protocolo Engine.IO (`EIO=3`), mientras que las instalaciones modernas de pip en Ubuntu instalan por defecto librerías que exigen `EIO=4`.

* **La Solución (Downgrade Controlado):** Para no corromper las librerías globales del sistema operativo, se implementó un entorno virtual (`venv`) para forzar la instalación de las versiones exactas que mantienen retrocompatibilidad con `EIO=3`.

  **Código utilizado para la resolución:**
  ```bash
  # 1. Navegar al workspace y crear el entorno virtual
  cd ~/autodrive_ws
  python3 -m venv venv
  source venv/bin/activate
  
  # 2. Instalar las versiones específicas que soportan EIO=3
  pip install --force-reinstall python-socketio==4.2.0 python-engineio==3.13.0 eventlet==0.33.3 Flask-SocketIO==4.1.0 gevent-websocket==0.10.1
  ```
  
  *Nota de Ingeniería:* Para garantizar que los nodos de ROS 2 consuman estas librerías específicas, es obligatorio inyectar la ruta del entorno virtual en el sistema antes de lanzar los nodos:
  ```bash
  export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
  ```

### 🧹 2. Acondicionamiento de Datos (Data Parsing y Regional Settings)
Al recibir telemetría espacial masiva (arreglos del LiDAR, cuaterniones, posiciones X-Y-Z), el sistema en Ubuntu colapsaba al intentar convertir las cadenas de texto a matrices de números flotantes.

* **El Problema (Conflicto de Formato Regional):** El sistema operativo Host (Windows) estaba configurado con formato regional latinoamericano, enviando los decimales separados por comas (ej. `0,75` o `0,000`). Las librerías en Ubuntu (`NumPy`) requieren estrictamente el estándar POSIX con punto decimal (`0.75`), provocando la caída del nodo:
  ```text
  [autodrive_incoming_bridge-1] ValueError: could not convert string to float: '0,000'
  ```

* **La Solución (Sanitización Global de Diccionarios):** Se intervino el código fuente del puente de entrada (`autodrive_incoming_bridge.py`) para interceptar el payload completo apenas entra por el WebSocket y "sanitizarlo".
  ```python
  # Corrección global de separador decimal regional para prevenir fallos en NumPy
  data = {k: v.replace(",", ".") if isinstance(v, str) else v for k, v in data.items()}
  # A partir de este punto, variables como data["V1 Throttle"] ya tienen formato correcto.
  ```

### 🧭 3. Fusión Sensorial para Odometría Dinámica (`odom_publisher.py`)
Para que un algoritmo SLAM funcione correctamente, necesita saber no solo qué "ve" el LiDAR, sino exactamente en qué posición y ángulo se encontraba el robot al momento de la captura.

* **El Problema (El Efecto "Ghosting"):** Inicialmente, el mapa generado presentaba severas distorsiones y paredes duplicadas al tomar las curvas del circuito. El diagnóstico reveló que el sistema carecía de odometría angular. Aunque el sensor IPS proporcionaba las coordenadas espaciales `(X, Y, Z)`, se estaba forzando un ángulo de rotación estático (`w=1.0`). 

* **La Solución (Fusión de IPS + IMU):** Se desarrolló desde cero un nodo de ROS 2 llamado `odom_publisher.py` para calcular la cinemática real del F1Tenth. 
  ```python
  # Fragmento clave de la fusión sensorial en odom_publisher.py
  def ips_callback(self, msg):
      self.last_x = msg.x
      self.last_y = msg.y

  def imu_callback(self, msg):
      self.ori_x = msg.orientation.x
      self.ori_y = msg.orientation.y
      self.ori_z = msg.orientation.z
      self.ori_w = msg.orientation.w
  ```
  Con estos datos fusionados, el nodo publica un mensaje `nav_msgs/Odometry` oficial en el tópico `/odom` y crea un "Árbol de Transformadas" (TF Tree) dinámico (`odom` -> `f1tenth_1`).

### ⏱️ 4. Sincronización Temporal y Filtro LiDAR (`lidar_republisher.py`)
Los algoritmos de mapeo dependen de un TF Tree estrictamente sincronizado en el tiempo. Si un sensor envía datos con una marca de tiempo desfasada, el paquete es descartado.

* **El Problema (Desincronización Host-Guest):** Al lanzar el SLAM, arrojaba repetidamente este error en la terminal:
  ```text
  [slam_toolbox]: Message Filter dropping message: frame 'lidar' at time 1777333389.765 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
  ```
  La causa raíz era una desincronización de milisegundos entre el reloj de Windows y el de la máquina virtual en Ubuntu.

* **La Solución (Nodo Interceptor de Tiempo):** Se creó el nodo `lidar_republisher.py` que actúa como interceptor. Borra la estampa de tiempo defectuosa de Windows y le inyecta la hora exacta actual de Linux.
  ```python
  # Fragmento clave de lidar_republisher.py
  def callback(self, msg):
      # 1. Sobrescribir el timestamp viejo con la hora actual de ROS 2 (Linux)
      msg.header.stamp = self.get_clock().now().to_msg()
      # 2. Asegurar que el frame_id coincida con nuestro TF Tree dinámico
      msg.header.frame_id = 'f1tenth_1'
      # 3. Republicar el mensaje limpio a un nuevo tópico
      self.pub.publish(msg)
  ```

---

## 🛠️ Instalación y Dependencias

Es obligatorio aislar las librerías en un entorno virtual para evitar conflictos con el sistema base de Ubuntu:

```bash
cd ~/autodrive_ws
python3 -m venv venv
source venv/bin/activate
pip install --force-reinstall eventlet==0.33.3 Flask-SocketIO==4.1.0 python-socketio==4.2.0 python-engineio==3.13.0 gevent-websocket==0.10.1
```

---

## 💻 Guía de Ejecución (4 Terminales)

Ejecuta cada comando en una terminal diferente. **Importante:** Siempre activa el entorno virtual y exporta el `PYTHONPATH` antes de lanzar los nodos.

### Terminal 1: Bridge y RViz
```bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
```
*(Nota: Conecta el simulador en Windows Inmediatamente después de ejecutar esto).*

### Terminal 2: Nodo de Odometría
```bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ~/autodrive_ws/odom_publisher.py
```

### Terminal 3: Nodo de LiDAR
```bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ~/autodrive_ws/lidar_republisher.py
```

### Terminal 4: SLAM Toolbox
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

---

## 💾 Guardar el Mapa

Cuando termines de recorrer el circuito completo conduciendo lentamente con `W, A, S, D` desde el simulador en Windows, abre una nueva terminal y guarda el resultado:

```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/mapa_autodrive
