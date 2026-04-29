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

### 1. Puentes de Comunicación y WebSockets (`Socket.IO`)
La conexión bidireccional entre ROS 2 y el simulador AutoDRIVE se realiza a través del puerto TCP `4567` utilizando WebSockets.

* **El Problema (Incompatibilidad de Protocolos):** Al intentar conectar el simulador, el servidor levantado por ROS 2 rechazaba la conexión silenciosamente o arrojaba el siguiente error en la terminal:
  ```text
  [autodrive_incoming_bridge-1] The client is using an unsupported version of the Socket.IO or Engine.IO protocols
Esto ocurre porque el simulador AutoDRIVE utiliza una arquitectura legacy basada en la versión 3 del protocolo Engine.IO (EIO=3), mientras que las instalaciones modernas de pip en Ubuntu instalan por defecto librerías que exigen EIO=4.

La Solución (Downgrade Controlado): Para no corromper las librerías globales del sistema operativo, se implementó un entorno virtual (venv) para forzar la instalación de las versiones exactas que mantienen retrocompatibilidad con EIO=3.

Código utilizado para la resolución:

Bash
# 1. Navegar al workspace y crear el entorno virtual
```bash
cd ~/autodrive_ws
python3 -m venv venv
source venv/bin/activate
```

# 2. Instalar las versiones específicas que soportan EIO=3
```bash

pip install --force-reinstall python-socketio==4.2.0 python-engineio==3.13.0 eventlet==0.33.3 Flask-SocketIO==4.1.0 gevent-websocket==0.10.1
```
Nota de Ingeniería: Para garantizar que los nodos de ROS 2 (incoming_bridge y outgoing_bridge) consuman estas librerías específicas y no hagan fallback a las librerías del sistema base de Ubuntu, es obligatorio inyectar la ruta del entorno virtual en el sistema antes de lanzar los nodos:

```bash
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
```
### 2. Acondicionamiento de Datos (Data Parsing y Regional Settings)
Al recibir telemetría espacial masiva (arreglos del LiDAR, cuaterniones, posiciones X-Y-Z), el sistema en Ubuntu colapsaba al intentar convertir las cadenas de texto a matrices de números flotantes.

* **El Problema (Conflicto de Formato Regional):** El sistema operativo Host (Windows) estaba configurado con formato regional latinoamericano, enviando los decimales separados por comas (ej. `0,75` o `0,000`). Sin embargo, las librerías científicas en Ubuntu (`NumPy`) requieren estrictamente el estándar POSIX con punto decimal (`0.75`). Esto provocaba la caída inmediata del nodo con el siguiente error:
  ```text
  [autodrive_incoming_bridge-1] ValueError: could not convert string to float: '0,000'
La Solución (Sanitización Global de Diccionarios): En lugar de intentar corregir cada variable individualmente, se intervino el código fuente del puente de entrada (autodrive_incoming_bridge.py) para interceptar el payload completo (el diccionario data) apenas entra por el WebSocket y "sanitizarlo" antes de que llegue a las funciones de ROS 2.

Código utilizado para la resolución:
Se agregó la siguiente línea de comprensión de diccionarios (dictionary comprehension) en Python justo al inicio de la recepción de datos en autodrive_incoming_bridge.py:

Python
```bash

# Corrección global de separador decimal regional para prevenir fallos en NumPy
data = {k: v.replace(",", ".") if isinstance(v, str) else v for k, v in data.items()}
# A partir de este punto, variables como data["V1 Throttle"] ya tienen formato correcto.
```
Nota de Ingeniería: Esta solución es mucho más escalable, ya que limpia automáticamente los más de 1000 valores que envía el escáner LiDAR por frame de un solo golpe, asegurando que las funciones como np.fromstring() no se rompan.

### 3. Fusión Sensorial para Odometría Dinámica (`odom_publisher.py`)
Para que un algoritmo SLAM funcione correctamente, necesita saber no solo qué "ve" el LiDAR, sino exactamente en qué posición y ángulo se encontraba el robot al momento de la captura.

* **El Problema (El Efecto "Ghosting"):** Inicialmente, el mapa generado presentaba severas distorsiones y paredes duplicadas o sobrepuestas al tomar las curvas del circuito (ver `error_ghosting.png`).  El diagnóstico reveló que el sistema carecía de odometría angular. Aunque el sensor IPS proporcionaba las coordenadas espaciales `(X, Y, Z)`, se estaba forzando un ángulo de rotación estático (`w=1.0`). Al girar el vehículo, el SLAM asumía que seguía apuntando recto, dibujando el nuevo escaneo del LiDAR sobre el mapa anterior de forma incorrecta.

* **La Solución (Fusión de IPS + IMU):** Se desarrolló desde cero un nodo de ROS 2 llamado `odom_publisher.py` para calcular la cinemática real del F1Tenth. 
  
  **Código y Lógica implementada:**
  El nodo se suscribe simultáneamente a dos tópicos de telemetría y fusiona sus datos:
  1. Extrae la posición `(X, Y)` del tópico del **IPS**.
  2. Extrae la orientación en Cuaterniones `(x, y, z, w)` del tópico del **IMU**.

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
Con estos datos fusionados, el nodo cumple dos funciones críticas para la arquitectura del sistema:

Publica un mensaje nav_msgs/Odometry oficial en el tópico /odom.

Publica un "Árbol de Transformadas" (TF Tree) dinámico, creando el eslabón faltante: odom -> f1tenth_1. Además, inyecta un TF estático que le dice al sistema a qué altura y posición exacta se encuentra montado el láser respecto al chasis (f1tenth_1 -> lidar). `

### 4. Sincronización Temporal y Filtro LiDAR (`lidar_republisher.py`)
En ROS 2, los algoritmos de navegación y mapeo (como `slam_toolbox`) dependen de un Árbol de Transformadas (TF Tree) estrictamente sincronizado en el tiempo. Si un sensor envía datos con una marca de tiempo (timestamp) que no coincide con el reloj del sistema, el paquete es descartado por seguridad.

* **El Problema (Desincronización Host-Guest):** Al lanzar el SLAM, el sistema no lograba construir el mapa y arrojaba repetidamente este error en la terminal:
  ```text
  [slam_toolbox]: Message Filter dropping message: frame 'lidar' at time 1777333389.765 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
La causa raíz era una desincronización de milisegundos entre el reloj del sistema base en Windows (AutoDRIVE) y el reloj de la máquina virtual en Ubuntu. Los escaneos del LiDAR llegaban con un timestamp del "pasado" en comparación con la odometría generada en Linux, provocando que el filtro de mensajes del SLAM los descartara masivamente.

La Solución (Nodo Interceptor de Tiempo): Intentar forzar la sincronización de los relojes de la VM (hwclock, VBoxService) a menudo es inestable. La solución más robusta a nivel de arquitectura de ROS fue aislar el problema creando el nodo lidar_republisher.py.

Código y Lógica implementada:
Este nodo actúa como un "interceptor" (Middleware). Se suscribe al tópico original, borra la estampa de tiempo defectuosa que viene desde Windows, le inyecta la hora exacta actual del reloj de Linux, y lo publica en un tópico limpio.

Python
# Fragmento clave de lidar_republisher.py
def callback(self, msg):
    # 1. Sobrescribir el timestamp viejo con la hora actual de ROS 2 (Linux)
    msg.header.stamp = self.get_clock().now().to_msg()

    # 2. Asegurar que el frame_id coincida con nuestro TF Tree dinámico
    msg.header.frame_id = 'f1tenth_1'

    # 3. Republicar el mensaje limpio a un nuevo tópico
    self.pub.publish(msg)
Nota de Ingeniería: Para que esta solución surtiera efecto, se modificó el lanzamiento de slam_toolbox en la Terminal 4. En lugar de escuchar el escáner original, se le pasó el parámetro --ros-args -r /scan:=/lidar_fixed, forzándolo a consumir únicamente los datos con el tiempo corregido.
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
