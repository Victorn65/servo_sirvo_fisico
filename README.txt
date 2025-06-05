# Paquete `servo_sirvo_fisico`

Este paquete en ROS 2 proporciona los nodos y recursos necesarios para controlar y localización de un Puzzlebot físico equipado con sensores (LIDAR y cámara) y ruedas impulsadas por motores. A continuación se describe qué funcionalidades reales ofrece, cómo está organizado y cómo utilizarlo.

---

## 1. Descripción general

El paquete **`servo_sirvo_fisico`** está diseñado para:

- **Obtener velocidades de las ruedas** a partir de los comandos de velocidad lineal y angular (`cmd_vel`), mediante un nodo de modelo cinemático.
- **Publicar estados de las ruedas** y emitir transformaciones (`tf`) de las articulaciones del robot.
- **Detectar marcadores ArUco** con la cámara montada en el Puzzlebot y publicar observaciones (pose de cada marcador respecto a la cámara).
- **Localizar el robot** fusionando odometría diferencial (basada en encoders) con observaciones de marcadores ArUco y publicar el tópico de odometría global (`/odom`) y los `tf` correspondientes.
- **Controlar el movimiento hacia un punto** o evadir obstáculo (implementación del algoritmo Bug 0/Bug 2), suscribiéndose a LIDAR, odometría y setpoint, y publicando velocidades de rueda (`/cmd_vel`) y un flag de meta alcanzada (`/goal_reached`).

Además, incluye recursos para la simulación en Gazebo (modelos y worlds), definiciones URDF para la parte física del Puzzlebot, y archivos de lanzamiento (`.launch.py`) que ponen todo en marcha en conjunto.

---

## 2. Estructura de carpetas y archivos principales

```
servo_sirvo_fisico/
├── config/                            ── Archivos de configuración (parametrización YAML, launch, etc.)
├── launch/                            ── Archivos de lanzamiento en Python para iniciar grupos de nodos
│   ├── bringup.launch.py              ── Lanza todos los nodos necesarios para ejecutar el Puzzlebot físico
│   ├── localization.launch.py         ── Lanza solo los nodos de localización (kinemática, ArUco, tf, odom)
│   └── navigation.launch.py           ── Lanza el nodo de control de punto y evasión (Bug0/Bug2)
├── meshes/                            ── Modelos 3D (STL) para la parte física del robot (ruedas, sensores, etc.)
│   ├── Puzzlebot_Wheel.stl
│   ├── Puzzlebot_Caster_Wheel.stl
│   ├── RPLidar.stl
│   └── Laser_Camera_Bracket.stl
├── urdf/                              ── Archivo URDF/XACRO que describe la geometría del Puzzlebot físico
│   └── puzzlebot.urdf
├── worlds/                            ── Escenarios para simulación en Gazebo (con o sin marcadores ArUco)
│   ├── world_prueba.world
│   ├── puzzlebot_arena.world
│   └── … (otros mundos de prueba)
├── src/
│   └── servo_sirvo_fisico/            ── Código fuente Python de todos los nodos
│       ├── __init__.py
│       ├── kinematic_model.py         ── Nodo `kinematic_model_node`
│       ├── joint_state_pub.py         ── Nodo `joint_state_publisher`
│       ├── puzzlebot_aruco.py         ── Nodo `puzzlebot_aruco_node`
│       ├── localisation.py            ── Nodo `localisation_node`
│       ├── point_stabilisation_control.py  
│       │                                  ── Nodo `puzzlebot_controller_node` (Bug 0 / Bug 2)
│       ├── puzzlebot_kinematic_model.py  ── (alias de kinematic_model.py)
│       ├── puzzlebot_aruco.py            ── (alias de puzzlebot_aruco_node)
│       └── utils/                      ── Módulos auxiliares (`math_utils`, `observation_utils`, etc.)
├── package.xml                        ── Definición de dependencias y metadatos del paquete
├── setup.py & setup.cfg               ── Se usan para instalar el paquete Python en ROS 2
├── README.md                          ── Este documento (se ha reescrito para reflejar las funciones reales)
└── LICENSE                            ── Licencia de uso (aparece en el repositorio)
```

---

## 3. Dependencias

Para compilar e instalar este paquete, se requieren:

1. **ROS 2 Humble (o superior)**  
2. **Paquetes ROS 2 de terceros (especificados en `package.xml`):**
   - `rclpy`
   - `geometry_msgs`
   - `sensor_msgs`
   - `nav_msgs`
   - `tf2_ros`
   - `tf2_geometry_msgs`
   - `cv_bridge`
   - `sensor_msgs`
   - `std_msgs`
   - `aruco_msgs`
   - `puzzlebot_aruco_msgs`  
   - **Nota**: `aruco_msgs` y `puzzlebot_aruco_msgs` son paquetes que definen los mensajes para la detección de ArUco. Asegúrate de instalarlos o compilarlos en tu espacio de trabajo.

3. **Librerías Python adicionales** (instaladas vía pip, si trabajas fuera de conda/venv):
   - `numpy`
   - `opencv-python` (para procesar imágenes con OpenCV)
   - `transforms3d` (para cálculos de rotaciones y transformaciones)
   - `colormap` y otras que use `observation_utils` (ver contenido de `utils/`)

4. **Gazebo y Ignition (opcional)**, si se requieren simulaciones con los `worlds` proporcionados.

---

## 4. Nodos disponibles

A continuación se describe cada nodo Python junto a su propósito, tópicos principales y parámetros más relevantes:

### 4.1. Nodo `kinematic_model_node`

- **Archivo**: `src/servo_sirvo_fisico/kinematic_model.py`
- **Nombre del nodo**: `kinematic_model_node`
- **Propósito**:  
  - Recibe el tópico `/cmd_vel` (`geometry_msgs/Twist`) con velocidades lineales y angulares deseadas.
  - Calcula las **velocidades de rueda** derecha e izquierda (Float32) en rad/s, basándose en el modelo diferencial del Puzzlebot (radio de rueda, distancia entre ruedas).
  - Publica en:
    - `/VelocityEncR` (`std_msgs/Float32`): velocidad angular de la rueda derecha.
    - `/VelocityEncL` (`std_msgs/Float32`): velocidad angular de la rueda izquierda.

- **Parámetros internos** (definidos en el código):
  - `r` = 0.05 [m] – Radio de las ruedas.
  - `L` = 0.19 [m] – Distancia entre ruedas.

- **Uso típico**:  
  ```bash
  ros2 run servo_sirvo_fisico kinematic_model_node
  ```  
  (Se suele lanzar junto con los demás nodos en un `launch`).

---

### 4.2. Nodo `joint_state_publisher`

- **Archivo**: `src/servo_sirvo_fisico/joint_state_pub.py`
- **Nombre del nodo**: `joint_state_publisher`
- **Propósito**:  
  - Suscribe a los tópicos `/VelocityEncR` y `/VelocityEncL` (ambos `std_msgs/Float32`).
  - Calcula la posición angular acumulada de cada rueda (integrando vel).  
  - Publica:
    - `/joint_states` (`sensor_msgs/JointState`): posición (en radianes) y velocidad (rad/s) de ambas ruedas.
  - Publica transformaciones `/tf`:
    - Broadcast de la pose de cada rueda con respecto a `base_link`.
    - Opcionalmente, broadcast de otras articulaciones definidas en el URDF.

- **Parámetros/Datos**:  
  - Se asume que, a partir de la velocidad de cada rueda, se integra para conocer la posición angular.
  - Calibra las articulaciones definidas en `urdf/puzzlebot.urdf`.

---

### 4.3. Nodo `puzzlebot_aruco_node`

- **Archivo**: `src/servo_sirvo_fisico/puzzlebot_aruco.py`
- **Nombre del nodo**: `puzzlebot_aruco_node`
- **Propósito**:  
  - Procesa imágenes de la cámara para detectar marcadores **ArUco**.
  - Suscribe a:
    - `/camera/image_raw` (`sensor_msgs/Image`).
  - Publica:
    - `/aruco_image` (`sensor_msgs/Image`, opcional, con las detecciones dibujadas).
    - `/aruco_observation` (`puzzlebot_aruco_msgs/ArucoObservation`): lista de marcadores detectados y sus poses relativas a la cámara.
  - Utiliza parámetros:
    - `aruco_side_length` (float, en metros) – Tamaño lateral del marcador.
    - `camera_matrix` (lista de 9 floats) – Matriz intrínseca de la cámara.
    - `camera_distortion` (lista de 5 floats) – Coeficientes de distorsión.
    - `camera_optical_frame` – Frame ID de la cámara en el URDF (por defecto: `camera_link_optical`).

- **Dependencias**:  
  - `cv_bridge` para convertir entre `sensor_msgs/Image` y `cv2`.
  - OpenCV (`cv2`) con módulo ArUco.
  - `tf2_ros.Buffer` y `TransformListener` para transformar puntos entre frames.

- **Uso típico** (ejemplo):  
  ```bash
  ros2 run servo_sirvo_fisico puzzlebot_aruco_node
  ```

---

### 4.4. Nodo `localisation_node`

- **Archivo**: `src/servo_sirvo_fisico/localisation.py`
- **Nombre del nodo**: `localisation_node`
- **Propósito**:  
  - Fusiona la odometría diferencial (basada en `/VelocityEncR` y `/VelocityEncL`) con observaciones de marcadores ArUco.
  - Publica:
    - `/odom` (`nav_msgs/Odometry`): pose y velocidad lineal/ángular del robot en el frame `odom`.
    - Transforms `/tf`:  
      - de `base_link` a `odom` – basado en odometría cinemática.  
      - de `map` a `odom` – usando correcciones cuando se visualiza un marcador ArUco (si está a la vista).
  - Suscribe a:
    - `/VelocityEncR` y `/VelocityEncL` (`std_msgs/Float32`).
    - `/aruco_observation` (`puzzlebot_aruco_msgs/ArucoObservation`).
  - Parámetros internos:
    - Rates de publicación, covarianzas de odometría, offsets de cámara, etc. (ver configuración por defecto en el código).

- **Flujo de datos**:  
  1. Cuando llega una velocidad de rueda, integra para actualizar pose odométrica base.  
  2. Cuando recibe una observación ArUco (pose de marcador), transforma ese punto al frame `map` (asumiendo que la posición global de cada marcador en el mapa está preconfigurada), y ajusta el **correlacionamiento** entre `map` y `odom` para corregir la deriva de odometría.

- **Uso típico**:
  ```bash
  ros2 run servo_sirvo_fisico localisation_node
  ```
  Se recomienda lanzar en conjunto con el nodo de ArUco y el nodo de modelo cinemático.

---

### 4.5. Nodo `puzzlebot_controller_node` (Point Stabilisation / Bug0 / Bug2)

- **Archivo**: `src/servo_sirvo_fisico/point_stabilisation_control.py`
- **Nombre del nodo**: `puzzlebot_controller_node`
- **Propósito**:  
  - Implementa **control reactivo** para moverse hacia un punto (`Pose2D`) y evadir obstáculos usando LIDAR (implementación de Bug 0 y Bug 2).
  - Suscribe a:
    - `/scan` (`sensor_msgs/LaserScan`): datos del LIDAR frontal 360°.
    - `/odom` (`nav_msgs/Odometry`): pose actual del robot.
    - `/setpoint` (`geometry_msgs/Pose2D`): coordenadas del objetivo en coordenadas del mapa o `odom`.
  - Publica:
    - `/cmd_vel` (`geometry_msgs/Twist`): velocidades lineal y angular para el robot.
    - `/goal_reached` (`std_msgs/Bool`): indica si el robot llegó al `setpoint` (dentro de tolerancias).

- **Parámetros más relevantes** (definidos en el código):
  - `controller_update_rate` (Hz) – Frecuencia de control (por defecto, 25 Hz).
  - `distance_tolerance` (m) – Tolerancia para considerar meta alcanzada.
  - `angular_tolerance` (rad) – Tolerancia angular para meta.
  - **Control P2P (Point-to-Point)**:
    - `p2p_v_Kp` – Ganancia proporcional velocidad lineal.
    - `p2p_w_Kp` – Ganancia proporcional velocidad angular.
  - **Seguimiento de pared (Wall Follower)**:
    - `following_walls_distance` (m) – Distancia objetivo al muro.
    - `fw_w_Kp`, `fw_e_Kp`, `fw_linear_speed` – Parámetros de controlador de pared.
  - **Bug 2**:
    - `lookahead_distance` (m) – Para controlar cuándo regresar a la línea meta.

- **Lógica básica**:
  1. Mientras no hay obstrucción, se mueve en línea recta al `setpoint` usando control P2P.
  2. Si detecta obstáculo (umbral `front_stop_distance`), calcula un lado (izquierda o derecha) para rodear la pared y cambia a modo “following_walls” (Bug 0).
  3. En modo “following_walls”, se mantiene a distancia `following_walls_distance` del muro hasta poder retomar la línea directa al objetivo (Bug 2).
  4. Cuando vuelve a la línea objetivo, regresa al modo P2P.

- **Uso típico**:
  ```bash
  ros2 run servo_sirvo_fisico puzzlebot_controller_node
  ```
  Se usa normalmente junto con:
  - Nodo de odometría (`localisation_node`)
  - Nodo de LIDAR (publica `/scan`)
  - Nodo de setpoint (o herramienta externa que publique en `/setpoint`).

---

## 5. Archivos de lanzamiento (`launch/`)

Existen varios archivos de lanzamiento para facilitar la inicialización conjunta de todos los nodos:

1. **`bringup.launch.py`**  
   - Incluye:
     - Nodo `kinematic_model_node`
     - Nodo `joint_state_publisher`
     - Nodo `puzzlebot_aruco_node`
     - Nodo `localisation_node`
     - Nodo `puzzlebot_controller_node`
     - Carga del archivo URDF (publica en `/robot_description`)
     - Parámetros de calibración de cámara, LIDAR, etc.
   - Uso:
     ```bash
     ros2 launch servo_sirvo_fisico bringup.launch.py
     ```

2. **`localization.launch.py`**  
   - Solo enciende:
     - Nodo `kinematic_model_node`
     - Nodo `joint_state_publisher`
     - Nodo `puzzlebot_aruco_node`
     - Nodo `localisation_node`
   - Ideal para probar la fusión odom + ArUco sin controlador.
   - Uso:
     ```bash
     ros2 launch servo_sirvo_fisico localization.launch.py
     ```

3. **`navigation.launch.py`**  
   - Enciende:
     - Nodo `puzzlebot_controller_node`
     - Nodo `kinematic_model_node`
     - Nodo `localisation_node`
   - Para pruebas de navegación autónoma (despliegue Bug 0/Bug 2).
   - Uso:
     ```bash
     ros2 launch servo_sirvo_fisico navigation.launch.py
     ```

> **Tip:** Verifica que todos los paquetes de mensajes necesarios (`aruco_msgs`, `puzzlebot_aruco_msgs`) estén compilados y en el `ROS_PACKAGE_PATH`. Ajusta en cada launch las rutas al URDF y parámetros según tu entorno.

---

## 6. Modelos y mundos para simulación (Opcional)

Aunque el objetivo principal de este paquete es controlar un Puzzlebot físico, se incluyen recursos de simulación en **Gazebo**:

- Carpeta `worlds/`:
  - `world_prueba.world`: Mundo vacío con planos de suelo.
  - `puzzlebot_arena.world`: Arena cerrada para pruebas de navegación.
  - `puzzlebot_arena_markers.world`: Arena con marcadores ArUco incrustados en el piso/pared.
  - Otros mundos basados en SDF: `world_box.sdf`, `world_empty.sdf`, etc.

- Carpeta `meshes/` y `models/`:
  - Modelos 3D para Gazebo (STL y archivos SDF), que replican la estructura física del Puzzlebot (incluyendo base, ruedas, soporte de LIDAR, cámara).
  - Ejemplo de uso:
    ```bash
    gazebo gazebo worlds/puzzlebot_arena.world
    ```
  - En la simulación, se puede cargar el URDF `urdf/puzzlebot.urdf` para visualizar el robot dentro de esos mundos.

> **Importante**: Para que los nodos funcionen en simulación, debes remapear tópicos físicos a los que expone Gazebo (por ejemplo, `/scan` desde un plugin de Gazebo, `/camera/image_raw`, `/joint_states_sim`, etc.). Ajusta los argumentos en los archivos en `launch/` según tu configuración de simulación.

---

## 7. Instalación

1. **Clona o extrae** este paquete en tu espacio de trabajo de ROS 2 (por ejemplo, `~/ros2_ws/src/servo_sirvo_fisico`).  

2. **Compila el espacio de trabajo**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select servo_sirvo_fisico
   ```

3. **Fuente del setup**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

4. **Verifica que los nodos estén instalados**:
   ```bash
   ros2 pkg executables servo_sirvo_fisico
   ```
   Deberías ver listados al menos:
   ```
   kinematic_model_node
   joint_state_publisher
   puzzlebot_aruco_node
   localisation_node
   puzzlebot_controller_node
   ```

---

## 8. Cómo usar paso a paso

A continuación se ejemplifica un flujo mínimo para probar el paquete en un robot real (o simulación adaptada):

1. **Inicia el middleware de ROS 2** (no suele requerir comando extra; basta con que `ros2` esté corriendo).

2. **Lanza la localización básica**:
   ```bash
   ros2 launch servo_sirvo_fisico localization.launch.py
   ```
   - Esto:
     - Publica `/VelocityEncR` y `/VelocityEncL` (cinemática).
     - Publica `/joint_states` y `/tf` (articulaciones).
     - Detecta ArUco desde `/camera/image_raw`.
     - Publica `/odom` y corrige deriva con marcadores.

3. **Publica un setpoint de prueba** (en otro terminal):
   ```bash
   ros2 topic pub /setpoint geometry_msgs/Pose2D "{x: 1.0, y: 0.0, theta: 0.0}"
   ```
   - Esto envía un objetivo a 1 m en el eje X del frame `odom`.

4. **Inicia el controlador de navegación**:
   ```bash
   ros2 launch servo_sirvo_fisico navigation.launch.py
   ```
   - El robot intentará moverse hacia el setpoint publicado. Observa:
     - Topico `/cmd_vel` para ver las órdenes de velocidad.
     - Topico `/goal_reached` (`std_msgs/Bool`): `true` cuando llegue.

5. **Siéntate y observa** cómo el robot gira, ve obstáculos con LIDAR (`/scan`) y sigue la lógica Bug 0/Bug 2.

6. **Para simulación en Gazebo** (ajusta nombres de tópicos si es necesario):
   - Lanza Gazebo con un mundo:
     ```bash
     gazebo ~/ros2_ws/src/servo_sirvo_fisico/worlds/puzzlebot_arena.world
     ```
   - En otro terminal, lanza la localización remapeando temas de sensores:
     ```bash
     ros2 launch servo_sirvo_fisico localization.launch.py        use_sim_time:=true        image_topic:=/gazebo/camera/image_raw        scan_topic:=/scan
     ```
   - Ajusta parámetros de remapeo para que el nodo reciba `/gazebo/...` en vez de los tópicos físicos.

---

## 9. Detalles adicionales

- **URDF y TF**:  
  - El archivo `urdf/puzzlebot.urdf` describe las articulaciones (`wheel_left_joint`, `wheel_right_joint`, etc.) y define los frames básicos (`base_link`, `camera_link_optical`, `laser_link`, etc.).  
  - Asegúrate de que el `frame_id` de la cámara (`camera_optical_frame`) en `puzzlebot_aruco_node` coincida con el del URDF.

- **Módulos auxiliares** (`src/servo_sirvo_fisico/utils/`):  
  - **`math_utils.py`**: Cálculos de cinemática y transformaciones (e.g., pasar velocidad de rueda a odometría).  
  - **`observation_utils.py`**: Funciones para procesar las detecciones de ArUco y calcular poses en distintos frames.

- **Calibración de cámara y LIDAR**:  
  - Los parámetros de la cámara (matriz intrínseca, distorsión) se encuentran por defecto en `puzzlebot_aruco_node`. Ajusta dichos valores si usas otra cámara.  
  - Para LIDAR, si se requiere cambiar ángulos de corte o resoluciones, edita directamente los parámetros de lanzamiento del nodo `puzzlebot_controller_node`.

- **Pruebas unitarias**:  
  - Carpeta `test/` contiene scripts PyTest para comprobar partes aisladas del paquete (e.g., funciones de `math_utils`).  
  - Ejecuta:
    ```bash
    colcon test --packages-select servo_sirvo_fisico
    colcon test-result --verbose
    ```

---

## 10. Ejemplos de uso

- **Prueba de modelo cinemático**:
  ```bash
  ros2 run servo_sirvo_fisico kinematic_model_node
  # En otro terminal:
  ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
  # Observa en:
  ros2 topic echo /VelocityEncR
  ros2 topic echo /VelocityEncL
  ```
- **Prueba de detección ArUco** (con cámara conectada):
  ```bash
  ros2 run servo_sirvo_fisico puzzlebot_aruco_node
  # En otro terminal:
  # Muestra los marcadores detectados
  ros2 topic echo /aruco_observation
  ```
- **Ver TF de odometría**:
  ```bash
  ros2 run servo_sirvo_fisico localization_node
  # En otro terminal:
  rviz2  # Configura para ver TF y Odometry
  ```

---

## 11. Notas finales

- Este **README.md** reemplaza la descripción genérica anterior (que hacía referencia a otro repositorio, “ServoYSirvo_nav2_puzzlebot”).  
- Aquí se explica con precisión **lo que hace realmente** el paquete `servo_sirvo_fisico`: sus nodos, topologías de tópicos, parámetros y recursos de simulación.
- Cualquier cambio en la estructura de carpetas (por ejemplo, actualización de `launch/` o parámetros) debe reflejarse en este documento.
- Para problemas específicos de calibración, hardware o integración, revisa los comentarios en los scripts de Python dentro de `src/servo_sirvo_fisico/`.

¡Listo! Con este README tendrás una guía clara de cómo usar y mantener el paquete `servo_sirvo_fisico`.
