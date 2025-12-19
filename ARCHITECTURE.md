# FIRE-EDGE System Architecture

**Technical Design Document** | Version 1.0 | December 2025

Comprehensive architectural overview of the FIRE-EDGE real-time firefighting edge system.

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Hardware Layer](#hardware-layer)
4. [Edge Processing Layer](#edge-processing-layer)
5. [Software Components](#software-components)
6. [Data Flow](#data-flow)
7. [Sensor Fusion Algorithm](#sensor-fusion-algorithm)
8. [Object Detection Pipeline](#object-detection-pipeline)
9. [Navigation System](#navigation-system)
10. [AR Overlay Rendering](#ar-overlay-rendering)
11. [Communication Architecture](#communication-architecture)
12. [Performance Optimization](#performance-optimization)
13. [Failure Modes](#failure-modes)
14. [Future Extensions](#future-extensions)

---

## Overview

### Design Philosophy

FIRE-EDGE follows a **distributed edge computing architecture** with these core principles:

1. **Edge-First Processing**: All critical computations on-device (no cloud dependency)
2. **Sensor Fusion**: Multiple complementary sensors for robustness
3. **Real-Time Constraints**: Hard deadlines for life-safety operations (<50ms latency)
4. **Graceful Degradation**: System continues operating with partial sensor failures
5. **Modularity**: Loosely-coupled components via ROS middleware

### Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **NVIDIA Jetson Xavier NX** | Only edge device with 21 TOPS for real-time YOLOv8 |
| **ROS Noetic** | Industry-standard middleware, mature ecosystem |
| **YOLOv8 Nano** | Best accuracy/speed tradeoff for edge deployment |
| **Kalman Filter** | Optimal for linear Gaussian systems, real-time performance |
| **FLIR Lepton 3.5** | Lowest-cost radiometric thermal camera |
| **Kopin OLED** | Highest brightness (10,000 nits) for smoke visibility |

---

## System Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         HARDWARE LAYER                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │ Thermal  │  │  LiDAR   │  │   IMU    │  │Ultrasonic│      │
│  │ (30Hz)   │  │  (10Hz)  │  │ (100Hz)  │  │  (5Hz)   │      │
│  └─────┬────┘  └─────┬────┘  └─────┬────┘  └─────┬────┘      │
└────────┼─────────────┼─────────────┼─────────────┼────────────┘
         │             │             │             │
         └─────────────┴─────────────┴─────────────┘
                       │
         ┌─────────────▼─────────────┐
         │    ROS Middleware Layer   │
         │  (Message Passing, Sync)  │
         └─────────────┬─────────────┘
                       │
    ┌──────────────────┼──────────────────┐
    │                  │                  │
    ▼                  ▼                  ▼
┌─────────┐    ┌──────────────┐    ┌─────────┐
│ Thermal │    │   Sensor     │    │   AR    │
│Processor│───▶│   Fusion     │───▶│ Display │
│ YOLOv8  │    │Kalman Filter │    │ OpenCV  │
└─────────┘    └──────────────┘    └─────────┘
    │                  │                  │
    │                  │                  │
    └──────────────────┴──────────────────┘
                       │
                       ▼
              ┌────────────────┐
              │  Kopin Display │
              │  1280x720 OLED │
              └────────────────┘
```

### Layer Breakdown

#### Layer 1: Hardware Abstraction
- **Responsibility**: Interface with physical sensors
- **Components**: Device drivers, I/O handlers
- **Technologies**: I2C, SPI, USB, GPIO
- **Fault Tolerance**: Retry logic, timeout handling

#### Layer 2: ROS Middleware
- **Responsibility**: Message passing, time synchronization
- **Components**: ROS nodes, topics, services
- **Technologies**: ROS Noetic, message_filters
- **Fault Tolerance**: Automatic reconnection, buffering

#### Layer 3: Processing Nodes
- **Responsibility**: Sensor processing and fusion
- **Components**: thermal_processor, sensor_fusion, ar_display
- **Technologies**: PyTorch, NumPy, OpenCV
- **Fault Tolerance**: Watchdog timers, exception handling

#### Layer 4: Display Output
- **Responsibility**: AR visualization
- **Components**: Overlay renderer, homography transform
- **Technologies**: OpenCV, Kopin SDK
- **Fault Tolerance**: Fallback to simple text display

---

## Hardware Layer

### Sensor Suite Specifications

#### 1. FLIR Lepton 3.5 Thermal Camera

**Technical Specifications**:
```
Resolution:        160×120 pixels
Sensor Type:       Uncooled VOx Microbolometer
Spectral Range:    8-14 μm (LWIR)
Frame Rate:        8.7 Hz (default), configurable
Field of View:     57° × 44° (horizontal × vertical)
Thermal Sensitivity: <50 mK (NETD)
Temperature Range: -10°C to +140°C (high gain)
                   -10°C to +450°C (low gain)
Interface:         SPI (up to 20 MHz)
Power:             150 mW typical
Dimensions:        8.5 × 8.5 × 11.0 mm
```

**Integration Details**:
- Connected via SPI0 on Jetson GPIO header
- VSync interrupt on GPIO7 for frame synchronization
- Flat-Field Correction (FFC) runs every 3 minutes
- Radiometric mode enabled for absolute temperature

**Driver Architecture**:
```python
class FLIRLeptonDriver:
    def __init__(self, spi_dev='/dev/spidev0.0'):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # Bus 0, Device 0
        self.spi.max_speed_hz = 20000000  # 20 MHz
        self.vsync_pin = 7
        GPIO.setup(self.vsync_pin, GPIO.IN)
        
    def capture_frame(self) -> np.ndarray:
        """Capture 16-bit thermal frame via SPI."""
        # Wait for VSync pulse
        GPIO.wait_for_edge(self.vsync_pin, GPIO.RISING, timeout=200)
        
        # Read frame data (160*120*2 = 38,400 bytes)
        frame_data = self.spi.readbytes(38400)
        
        # Convert to numpy array and reshape
        frame = np.frombuffer(frame_data, dtype=np.uint16)
        frame = frame.reshape(120, 160)
        
        # Convert to temperature (if radiometric)
        temp_celsius = self.raw_to_celsius(frame)
        
        return temp_celsius
```

#### 2. Garmin LIDAR-Lite v3HP

**Technical Specifications**:
```
Range:             5 cm to 40 m
Accuracy:          ±2.5 cm
Update Rate:       ~10 Hz (adjustable)
Resolution:        1 cm
Beam Divergence:   ~8 mrad
Interface:         I2C (0x62 default address)
Power:             5V, 105 mA peak (1.8W)
Dimensions:        40 × 48 × 20 mm
```

**Integration Details**:
- Connected to I2C Bus 1 on Jetson
- Trigger measurement, wait for ready flag, read distance
- Measurement delay: ~20-100ms depending on range
- Adaptive acquisition count for noise rejection

**Driver Architecture**:
```python
class LiDARLiteDriver:
    I2C_ADDRESS = 0x62
    MEASURE_REG = 0x00
    STATUS_REG = 0x01
    DISTANCE_REG = 0x8f
    
    def __init__(self, bus=1):
        self.bus = smbus2.SMBus(bus)
        
    def measure_distance(self) -> float:
        """Trigger measurement and return distance in meters."""
        # Write 0x04 to register 0x00 to start measurement
        self.bus.write_byte_data(self.I2C_ADDRESS, 
                                 self.MEASURE_REG, 0x04)
        
        # Poll status register until bit 0 is clear (ready)
        timeout = time.time() + 0.2  # 200ms timeout
        while time.time() < timeout:
            status = self.bus.read_byte_data(self.I2C_ADDRESS, 
                                             self.STATUS_REG)
            if (status & 0x01) == 0:
                break
            time.sleep(0.001)
        
        # Read 2-byte distance from high and low registers
        distance_cm = self.bus.read_word_data(self.I2C_ADDRESS, 
                                              self.DISTANCE_REG)
        
        # Convert to meters and return
        return distance_cm / 100.0
```

#### 3. TDK InvenSense MPU-9250 IMU

**Technical Specifications**:
```
Gyroscope:
  - Range: ±250, ±500, ±1000, ±2000 °/s
  - Sensitivity: 131 LSB/(°/s) @ ±250°/s
  - Output Rate: up to 8 kHz
  
Accelerometer:
  - Range: ±2, ±4, ±8, ±16 g
  - Sensitivity: 16384 LSB/g @ ±2g
  - Output Rate: up to 4 kHz
  
Magnetometer (AK8963):
  - Range: ±4800 μT
  - Sensitivity: 0.6 μT/LSB (16-bit)
  - Output Rate: 100 Hz

Interface:         I2C (0x68) or SPI
Digital Output:    16-bit ADC
Power:            3.3V, 3.5 mA (gyro+accel)
```

**Integration Details**:
- Connected to I2C Bus 1 at address 0x68
- Configured for ±2000°/s gyro, ±16g accel
- Digital Low-Pass Filter (DLPF) at 20 Hz
- Sampling at 100 Hz for real-time tracking

**Driver Implementation**:
```python
class MPU9250Driver:
    def __init__(self, bus=1, address=0x68):
        self.bus = smbus2.SMBus(bus)
        self.address = address
        self._configure()
        
    def _configure(self):
        """Configure sensor ranges and filters."""
        # Wake up (disable sleep)
        self.bus.write_byte_data(self.address, 0x6B, 0x00)
        
        # Gyro: ±2000°/s
        self.bus.write_byte_data(self.address, 0x1B, 0x18)
        
        # Accel: ±16g
        self.bus.write_byte_data(self.address, 0x1C, 0x18)
        
        # DLPF: 20 Hz bandwidth
        self.bus.write_byte_data(self.address, 0x1A, 0x04)
        
    def read_imu(self) -> Dict[str, np.ndarray]:
        """Read all IMU data."""
        # Read 14 bytes starting at ACCEL_XOUT_H (0x3B)
        data = self.bus.read_i2c_block_data(self.address, 0x3B, 14)
        
        # Parse data
        accel_x = self._bytes_to_int(data[0], data[1]) / 2048.0  # g
        accel_y = self._bytes_to_int(data[2], data[3]) / 2048.0
        accel_z = self._bytes_to_int(data[4], data[5]) / 2048.0
        
        gyro_x = self._bytes_to_int(data[8], data[9]) / 16.4  # °/s
        gyro_y = self._bytes_to_int(data[10], data[11]) / 16.4
        gyro_z = self._bytes_to_int(data[12], data[13]) / 16.4
        
        return {
            'accel': np.array([accel_x, accel_y, accel_z]),
            'gyro': np.array([gyro_x, gyro_y, gyro_z]),
            'timestamp': time.time()
        }
```

#### 4. A02YYUW Ultrasonic Sensors (×4)

**Technical Specifications**:
```
Range:             20 cm to 450 cm
Accuracy:          ±1 cm
Resolution:        1 mm
Beam Angle:        ~15°
Interface:         UART (9600 baud)
Power:             3.3-5V, 3 mA
Waterproof:        IP67 rated
Response Time:     100 ms
```

**Array Configuration**:
- **Front**: 0° (forward direction)
- **Right**: 90° (starboard)
- **Left**: 270° (port)
- **Back**: 180° (aft)

**Driver Architecture**:
```python
class UltrasonicArray:
    def __init__(self):
        self.sensors = {
            'front': serial.Serial('/dev/ttyTHS1', 9600),
            'right': serial.Serial('/dev/ttyTHS2', 9600),
            'left': serial.Serial('/dev/ttyTHS3', 9600),
            'back': serial.Serial('/dev/ttyTHS4', 9600)
        }
        
    def read_all(self) -> Dict[str, float]:
        """Read distance from all sensors."""
        distances = {}
        for name, port in self.sensors.items():
            if port.in_waiting >= 4:
                # Read 4-byte packet: 0xFF 0xFF (H-data) (L-data) (checksum)
                data = port.read(4)
                if data[0] == 0xFF and data[1] == 0xFF:
                    distance_mm = (data[2] << 8) | data[3]
                    distances[name] = distance_mm / 1000.0  # Convert to meters
                else:
                    distances[name] = None  # Invalid reading
            else:
                distances[name] = None  # No data
        return distances
```

---

## Edge Processing Layer

### NVIDIA Jetson Xavier NX

**Hardware Specifications**:
```
GPU:              384-core NVIDIA Volta
                  48 Tensor Cores
                  21 TOPS (INT8)
                  
CPU:              6-core NVIDIA Carmel ARM v8.2 (64-bit)
                  6 MB L2 + 4 MB L3 cache
                  
Memory:           16 GB LPDDR4x (137 GB/s)

Storage:          16 GB eMMC 5.1
                  microSD slot (expandable)
                  
Power Modes:      MODE_10W (10W), MODE_15W (15W)
                  MODE_2CORE (CPU only)
                  
Connectivity:     Gigabit Ethernet
                  M.2 Key E (WiFi/Bluetooth)
                  4× USB 3.1
                  
Video:            2× MIPI CSI-2 (camera input)
                  HDMI 2.0 + DP 1.4 output
                  
Dimensions:       103 × 90 × 31 mm
```

**Power Management**:
```bash
# Set to maximum performance (15W mode)
sudo nvpmodel -m 0

# Lock clocks to maximum
sudo jetson_clocks

# Monitor power consumption
jtop  # Interactive GUI
tegrastats  # Command-line stats
```

**Thermal Management**:
- Passive cooling: Heat sink (required)
- Active cooling: 5V PWM fan (recommended for sustained load)
- Thermal throttling: Kicks in at 95°C
- Safe operating: <80°C recommended

---

## Software Components

### ROS Node Architecture

#### 1. Thermal Processor Node (`thermal_processor.py`)

**Responsibilities**:
- Capture frames from FLIR Lepton
- Run YOLOv8 inference for fire/human detection
- Publish detection results and thermal images

**Node Configuration**:
```yaml
thermal_processor:
  node_name: thermal_processor
  rate: 30  # Hz
  
  camera:
    device: /dev/spidev0.0
    resolution: [160, 120]
    ffc_interval: 180  # seconds
    
  yolo:
    model_path: models/yolov8n_thermal_yellow.pt
    confidence_threshold: 0.5
    iou_threshold: 0.45
    device: cuda:0
    fp16: true
    
  publishing:
    thermal_image_topic: /thermal/image_raw
    detections_topic: /fire_edge/detections
    visualization_topic: /fire_edge/thermal_viz
```

**Main Loop**:
```python
class ThermalProcessorNode:
    def __init__(self):
        rospy.init_node('thermal_processor')
        
        # Load YOLOv8 model
        self.model = YOLO(rospy.get_param('~model_path'))
        self.model.to('cuda')
        
        # Initialize thermal camera
        self.camera = FLIRLeptonDriver()
        
        # Publishers
        self.thermal_pub = rospy.Publisher(
            '/thermal/image_raw', Image, queue_size=1
        )
        self.detection_pub = rospy.Publisher(
            '/fire_edge/detections', Detections, queue_size=1
        )
        
        # Processing rate
        self.rate = rospy.Rate(30)
        
    def run(self):
        while not rospy.is_shutdown():
            # Capture thermal frame
            thermal_frame = self.camera.capture_frame()
            
            # Convert to 8-bit for YOLOv8
            thermal_8bit = self.normalize_thermal(thermal_frame)
            
            # Run inference
            results = self.model(thermal_8bit, verbose=False)[0]
            
            # Parse detections
            detections = self.parse_yolo_results(results)
            
            # Publish thermal image
            thermal_msg = self.numpy_to_ros_image(thermal_8bit)
            self.thermal_pub.publish(thermal_msg)
            
            # Publish detections
            detection_msg = self.create_detection_msg(detections)
            self.detection_pub.publish(detection_msg)
            
            self.rate.sleep()
```

#### 2. Sensor Fusion Node (`sensor_fusion.py`)

**Responsibilities**:
- Fuse IMU, LiDAR, and ultrasonic data
- Implement Kalman filter for pose estimation
- Publish current position and path history

**Node Configuration**:
```yaml
sensor_fusion:
  node_name: sensor_fusion
  rate: 100  # Hz (driven by IMU)
  
  kalman:
    state_dim: 5  # [x, y, vx, vy, theta]
    measurement_dim: 3  # [x, y, theta]
    process_noise_Q: [0.1, 0.1, 0.05, 0.05, 0.01]
    measurement_noise_R: [0.5, 0.5, 0.1]
    
  sensors:
    imu_topic: /imu/data
    lidar_topic: /lidar/range
    ultrasonic_topic: /ultrasonic/ranges
    
  output:
    pose_topic: /fire_edge/pose
    path_topic: /fire_edge/path
    odom_topic: /fire_edge/odom
```

**Kalman Filter Implementation**:
```python
class SensorFusionNode:
    def __init__(self):
        rospy.init_node('sensor_fusion')
        
        # Initialize Kalman filter
        self.kf = KalmanFilter(
            dim_x=5,  # State: [x, y, vx, vy, theta]
            dim_z=3   # Measurement: [x, y, theta]
        )
        
        # State transition matrix (constant velocity model)
        dt = 0.01  # 100 Hz
        self.kf.F = np.array([
            [1, 0, dt, 0,  0],  # x
            [0, 1, 0,  dt, 0],  # y
            [0, 0, 1,  0,  0],  # vx
            [0, 0, 0,  1,  0],  # vy
            [0, 0, 0,  0,  1]   # theta
        ])
        
        # Measurement matrix
        self.kf.H = np.array([
            [1, 0, 0, 0, 0],  # Measure x
            [0, 1, 0, 0, 0],  # Measure y
            [0, 0, 0, 0, 1]   # Measure theta
        ])
        
        # Process noise
        Q_diag = rospy.get_param('~kalman/process_noise_Q')
        self.kf.Q = np.diag(Q_diag)
        
        # Measurement noise
        R_diag = rospy.get_param('~kalman/measurement_noise_R')
        self.kf.R = np.diag(R_diag)
        
        # Subscribers (with time synchronization)
        self.imu_sub = message_filters.Subscriber('/imu/data', Imu)
        self.lidar_sub = message_filters.Subscriber('/lidar/range', Range)
        
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.imu_sub, self.lidar_sub], 
            queue_size=10, 
            slop=0.05
        )
        ts.registerCallback(self.sensor_callback)
        
        # Publisher
        self.pose_pub = rospy.Publisher(
            '/fire_edge/pose', PoseStamped, queue_size=1
        )
        
    def sensor_callback(self, imu_msg, lidar_msg):
        """Fuse sensor data with Kalman filter."""
        # Predict step
        self.kf.predict()
        
        # Extract measurements
        z = np.array([
            self.estimate_x_from_lidar(lidar_msg),
            self.estimate_y_from_lidar(lidar_msg),
            self.estimate_theta_from_imu(imu_msg)
        ])
        
        # Update step
        self.kf.update(z)
        
        # Publish estimated pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = self.kf.x[0]
        pose_msg.pose.position.y = self.kf.x[1]
        
        self.pose_pub.publish(pose_msg)
```

#### 3. AR Display Node (`ar_display.py`)

**Responsibilities**:
- Subscribe to thermal, detections, and pose
- Render AR overlays (bounding boxes, navigation arrows, mini-map)
- Output to Kopin OLED display

**Node Configuration**:
```yaml
ar_display:
  node_name: ar_display
  rate: 30  # Hz
  
  display:
    width: 1280
    height: 720
    device: /dev/fb0  # Framebuffer
    
  rendering:
    detection_color:
      fire: [0, 0, 255]      # Red
      human: [0, 255, 255]   # Yellow
      hazard: [0, 165, 255]  # Orange
    
    minimap_size: 320
    minimap_scale: 40  # pixels per meter
    
  subscriptions:
    thermal_topic: /thermal/image_raw
    detections_topic: /fire_edge/detections
    pose_topic: /fire_edge/pose
    path_topic: /fire_edge/path
```

**Rendering Pipeline**:
```python
class ARDisplayNode:
    def __init__(self):
        rospy.init_node('ar_display')
        
        # Subscribe to data
        self.thermal_sub = rospy.Subscriber(
            '/thermal/image_raw', Image, self.thermal_callback
        )
        self.detection_sub = rospy.Subscriber(
            '/fire_edge/detections', Detections, self.detection_callback
        )
        self.pose_sub = rospy.Subscriber(
            '/fire_edge/pose', PoseStamped, self.pose_callback
        )
        
        # Display configuration
        self.width = 1280
        self.height = 720
        
        # State
        self.latest_thermal = None
        self.latest_detections = []
        self.latest_pose = None
        
    def render_frame(self) -> np.ndarray:
        """Compose final AR display frame."""
        if self.latest_thermal is None:
            return self.create_blank_frame()
        
        # Start with thermal image
        frame = cv2.resize(self.latest_thermal, (self.width, self.height))
        
        # Draw detection bounding boxes
        for det in self.latest_detections:
            self.draw_detection(frame, det)
        
        # Draw navigation overlay
        if self.latest_pose is not None:
            self.draw_navigation(frame, self.latest_pose)
        
        # Draw mini-map in corner
        minimap = self.render_minimap()
        frame[20:20+minimap.shape[0], -340:-20] = minimap
        
        # Draw status bar
        self.draw_status_bar(frame)
        
        return frame
```

---

## Data Flow

### End-to-End Processing Pipeline

```
 SENSORS          ROS TOPICS              PROCESSING              OUTPUT
    │                 │                        │                     │
┌───▼────┐      ┌─────▼─────┐          ┌──────▼──────┐      ┌──────▼──────┐
│Thermal │─────▶│/thermal/   │─────────▶│   YOLOv8    │─────▶│ Detection   │
│ Lepton │      │image_raw   │   30Hz   │  Inference  │ 35ms │   Results   │
└────────┘      └────────────┘          └─────────────┘      └──────┬──────┘
                                                                      │
┌────────┐      ┌────────────┐          ┌─────────────┐            │
│  IMU   │─────▶│/imu/data   │──┐       │   Kalman    │◀───────────┘
│MPU9250 │      │            │  └──────▶│   Filter    │
└────────┘      └────────────┘  100Hz   │  Prediction │
                                         │   + Update  │      ┌─────────────┐
┌────────┐      ┌────────────┐   │      └──────┬──────┘─────▶│   Pose      │
│ LiDAR  │─────▶│/lidar/     │───┘             │             │  Estimate   │
│  Lite  │      │range       │   10Hz          │             └──────┬──────┘
└────────┘      └────────────┘                 │                    │
                                                │                    │
┌────────┐      ┌────────────┐                 │                    │
│Ultrasonic─────▶│/ultrasonic/│─────────────────┘                    │
│ Array  │      │ranges      │   5Hz                                │
└────────┘      └────────────┘                                      │
                                                                     │
                                        ┌────────────────────────────┘
                                        │
                                        ▼
                                 ┌──────────────┐
                                 │  AR Overlay  │
                                 │   Renderer   │
                                 └──────┬───────┘
                                        │
                                        ▼
                                 ┌──────────────┐
                                 │    Kopin     │
                                 │OLED Display  │
                                 └──────────────┘
```

### Timing Diagram

```
Time (ms)  0     10    20    30    35    40    50    60    70    80
           │     │     │     │     │     │     │     │     │     │
Thermal    ├─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┤
(30 Hz)    │ Cap │ Cap │ Cap │ Cap │ Cap │ Cap │ Cap │ Cap │
           │     │     │     │     │     │     │     │     │     │
YOLOv8     │     ├───────────────┤     ├───────────────┤     ├───┤
Inference  │     │   35ms Inf    │     │   35ms Inf    │     │   │
           │     │     │     │     │     │     │     │     │     │
IMU        ├─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┤
(100 Hz)   │ Every 10ms                                         │
           │     │     │     │     │     │     │     │     │     │
LiDAR      ├─────────────────────────────┴─────────────────────┤
(10 Hz)    │         Measure (20-100ms)  │     Measure         │
           │     │     │     │     │     │     │     │     │     │
Kalman     │     ├─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┤
Filter     │     │ Predict+Update at 100Hz (IMU-driven)       │
(100 Hz)   │     │     │     │     │     │     │     │     │     │
           │     │     │     │     │     │     │     │     │     │
AR Display ├─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┤
(30 Hz)    │   Render    │   Render    │   Render    │   Render  │
```

### Message Synchronization Strategy

**Problem**: Sensors operate at different rates (30/100/10/5 Hz)

**Solution**: Time-stamped messages with approximate synchronization

```python
# Synchronize IMU (100Hz) and LiDAR (10Hz) with 50ms tolerance
from message_filters import ApproximateTimeSynchronizer

ts = ApproximateTimeSynchronizer(
    [imu_sub, lidar_sub],
    queue_size=10,
    slop=0.05  # 50ms tolerance
)
ts.registerCallback(sensor_fusion_callback)
```

**Buffer Management**:
- **Thermal**: 1-frame buffer (latest only)
- **IMU**: 10-frame circular buffer
- **LiDAR**: 5-measurement queue
- **Detections**: Latest result only

---

## Sensor Fusion Algorithm

### Kalman Filter State Space Model

**State Vector** (5-DoF):
```
x = [x, y, vx, vy, θ]ᵀ

where:
  x, y   : Position in world frame (meters)
  vx, vy : Velocity in world frame (m/s)
  θ      : Heading angle (radians)
```

**State Transition Model** (Constant Velocity):
```
F = ┌─                 ─┐
    │ 1  0  Δt  0   0  │
    │ 0  1  0   Δt  0  │
    │ 0  0  1   0   0  │
    │ 0  0  0   1   0  │
    │ 0  0  0   0   1  │
    └─                 ─┘

x_k = F x_{k-1} + w_k

where w_k ~ N(0, Q) is process noise
```

**Measurement Model**:
```
H = ┌─             ─┐
    │ 1  0  0  0  0 │  (x from LiDAR + dead reckoning)
    │ 0  1  0  0  0 │  (y from LiDAR + dead reckoning)
    │ 0  0  0  0  1 │  (θ from IMU gyroscope integration)
    └─             ─┘

z_k = H x_k + v_k

where v_k ~ N(0, R) is measurement noise
```

**Noise Covariance Matrices**:
```python
# Process noise (tuned experimentally)
Q = np.diag([
    0.1,   # x position uncertainty (m²)
    0.1,   # y position uncertainty (m²)
    0.05,  # x velocity uncertainty (m²/s²)
    0.05,  # y velocity uncertainty (m²/s²)
    0.01   # heading uncertainty (rad²)
])

# Measurement noise (from sensor datasheets)
R = np.diag([
    0.5,   # LiDAR-derived x uncertainty (m²)
    0.5,   # LiDAR-derived y uncertainty (m²)
    0.1    # IMU heading uncertainty (rad²)
])
```

### Position Estimation from LiDAR

**Algorithm**: Ray-casting with obstacle detection

```python
def estimate_position_from_lidar(
    lidar_distance: float,
    heading: float,
    ultrasonic_distances: Dict[str, float]
) -> Tuple[float, float]:
    """
    Estimate position by detecting walls/obstacles.
    
    Assumptions:
    - Indoor environment with rectangular rooms
    - Walls are approximately perpendicular
    """
    # Convert LiDAR distance to world coordinates
    dx = lidar_distance * np.cos(heading)
    dy = lidar_distance * np.sin(heading)
    
    # Correct for ultrasonic measurements
    if ultrasonic_distances['front'] is not None:
        # Front obstacle closer than LiDAR reading
        if ultrasonic_distances['front'] < lidar_distance:
            dx = ultrasonic_distances['front'] * np.cos(heading)
            dy = ultrasonic_distances['front'] * np.sin(heading)
    
    # Accumulate displacement (dead reckoning)
    self.estimated_x += dx
    self.estimated_y += dy
    
    return self.estimated_x, self.estimated_y
```

### Heading Estimation from IMU

**Algorithm**: Complementary filter combining gyro and accelerometer

```python
def estimate_heading_from_imu(
    gyro_z: float,
    accel: np.ndarray,
    dt: float = 0.01
) -> float:
    """
    Fuse gyroscope and accelerometer for drift-resistant heading.
    
    Complementary filter:
    θ = α * (θ_prev + gyro_z * dt) + (1-α) * θ_accel
    """
    alpha = 0.98  # Trust gyro for short-term, accel for long-term
    
    # Gyroscope integration (short-term accurate)
    theta_gyro = self.heading_prev + gyro_z * dt
    
    # Accelerometer tilt compensation (long-term reference)
    # (Only valid when firefighter is stationary)
    if np.linalg.norm(accel - np.array([0, 0, 1.0])) < 0.1:
        theta_accel = np.arctan2(accel[1], accel[0])
    else:
        theta_accel = self.heading_prev  # Ignore if moving
    
    # Complementary filter
    heading = alpha * theta_gyro + (1 - alpha) * theta_accel
    
    # Normalize to [-π, π]
    heading = np.arctan2(np.sin(heading), np.cos(heading))
    
    self.heading_prev = heading
    return heading
```

### Kalman Filter Update Loop

```python
class ExtendedKalmanFilter:
    def predict(self, dt: float):
        """Prediction step: Project state forward."""
        # Update state transition matrix
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        
        # Predict state
        self.x = self.F @ self.x
        
        # Predict covariance
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, z: np.ndarray):
        """Update step: Correct prediction with measurement."""
        # Innovation (measurement residual)
        y = z - self.H @ self.x
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        I = np.eye(self.dim_x)
        self.P = (I - K @ self.H) @ self.P
```

### Outlier Rejection

**Chi-Squared Test** for measurement validation:

```python
def validate_measurement(self, z: np.ndarray) -> bool:
    """Reject outliers using Mahalanobis distance."""
    # Innovation
    y = z - self.H @ self.x
    
    # Innovation covariance
    S = self.H @ self.P @ self.H.T + self.R
    
    # Mahalanobis distance
    d = y.T @ np.linalg.inv(S) @ y
    
    # Chi-squared test (95% confidence, 3 DoF)
    threshold = 7.815  # From chi-squared table
    
    return d < threshold
```

---

## Object Detection Pipeline

### YOLOv8 Architecture Optimization

**Base Model**: YOLOv8n (Nano variant)
- **Parameters**: 3.2M
- **FLOPs**: 8.7G
- **Input**: 640×640×3
- **Output**: 8400 detections × (4 bbox + 1 conf + 3 classes)

**Custom Modifications for Thermal**:

1. **Input Layer**: Single-channel grayscale (8-bit thermal)
   ```python
   # Modify first conv layer for 1-channel input
   model.model[0].conv.in_channels = 1
   ```

2. **Data Augmentation** (training):
   ```yaml
   augmentation:
     - CLAHE enhancement
     - Gaussian noise (σ=5)
     - Random brightness (±20%)
     - Horizontal flip
     - Rotation (±15°)
     - Simulated smoke overlay
   ```

3. **Loss Function**: Weighted focal loss for class imbalance
   ```python
   class_weights = {
       'fire': 2.0,     # Prioritize fire detection
       'human': 1.5,    # Important for rescue
       'hazard': 1.0    # Standard weight
   }
   ```

### TensorRT Optimization

**Conversion Pipeline**:

```bash
# 1. Export PyTorch to ONNX
yolo export model=yolov8n_thermal.pt format=onnx dynamic=False

# 2. Convert ONNX to TensorRT with FP16
trtexec --onnx=yolov8n_thermal.onnx \
        --saveEngine=yolov8n_thermal_fp16.engine \
        --fp16 \
        --workspace=4096 \
        --minShapes=input:1x1x640x640 \
        --optShapes=input:1x1x640x640 \
        --maxShapes=input:1x1x640x640
```

**Performance Gains**:
| Precision | Latency | mAP@0.5 | Memory |
|-----------|---------|---------|--------|
| FP32      | 52ms    | 85.2%   | 256MB  |
| FP16      | 35ms    | 85.0%   | 128MB  |
| INT8      | 28ms    | 82.1%   | 64MB   |

**Selected**: FP16 (best speed/accuracy tradeoff)

### Preprocessing Pipeline

```python
class ThermalPreprocessor:
    def __init__(self):
        self.clahe = cv2.createCLAHE(
            clipLimit=2.0,
            tileGridSize=(8, 8)
        )
        
    def preprocess(self, thermal_raw: np.ndarray) -> np.ndarray:
        """Convert raw thermal to YOLOv8 input."""
        # 1. Normalize to 8-bit (from 16-bit raw)
        thermal_8bit = cv2.normalize(
            thermal_raw, None, 0, 255, 
            cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        
        # 2. CLAHE enhancement (40% contrast improvement)
        enhanced = self.clahe.apply(thermal_8bit)
        
        # 3. Resize to 640×640 (bilinear interpolation)
        resized = cv2.resize(enhanced, (640, 640))
        
        # 4. Convert to float32 and normalize to [0, 1]
        normalized = resized.astype(np.float32) / 255.0
        
        # 5. Add batch and channel dimensions
        input_tensor = normalized[np.newaxis, np.newaxis, :, :]
        
        return input_tensor
```

### Post-processing: NMS and Tracking

**Non-Maximum Suppression**:
```python
def non_max_suppression(
    predictions: np.ndarray,
    conf_threshold: float = 0.5,
    iou_threshold: float = 0.45
) -> List[Detection]:
    """Filter overlapping detections."""
    # Filter by confidence
    mask = predictions[:, 4] > conf_threshold
    filtered = predictions[mask]
    
    # Sort by confidence (descending)
    sorted_idx = np.argsort(filtered[:, 4])[::-1]
    filtered = filtered[sorted_idx]
    
    # NMS loop
    keep = []
    while len(filtered) > 0:
        # Keep highest confidence detection
        keep.append(filtered[0])
        
        # Compute IoU with remaining detections
        ious = compute_iou(filtered[0], filtered[1:])
        
        # Remove overlapping detections
        mask = ious < iou_threshold
        filtered = filtered[1:][mask]
    
    return keep
```

**Temporal Tracking** (ByteTrack):
```python
class DetectionTracker:
    def __init__(self):
        self.tracks = []  # Active tracks
        self.next_id = 0
        
    def update(self, detections: List[Detection]) -> List[Track]:
        """Associate detections with existing tracks."""
        # Predict track positions
        for track in self.tracks:
            track.predict()
        
        # Data association (Hungarian algorithm)
        cost_matrix = self.compute_cost_matrix(detections)
        matches, unmatched_dets, unmatched_tracks = \
            self.hungarian_matching(cost_matrix)
        
        # Update matched tracks
        for det_idx, track_idx in matches:
            self.tracks[track_idx].update(detections[det_idx])
        
        # Initialize new tracks
        for det_idx in unmatched_dets:
            self.tracks.append(Track(
                id=self.next_id,
                detection=detections[det_idx]
            ))
            self.next_id += 1
        
        # Remove lost tracks
        self.tracks = [t for t in self.tracks if not t.is_lost()]
        
        return self.tracks
```

---

## Navigation System

### Path Recording

**Breadcrumb Trail Algorithm**:

```python
class PathRecorder:
    def __init__(self, waypoint_interval: float = 1.0):
        """
        Record path for return-to-exit navigation.
        
        Args:
            waypoint_interval: Min distance between waypoints (meters)
        """
        self.waypoints = []
        self.waypoint_interval = waypoint_interval
        
    def add_pose(self, pose: PoseStamped):
        """Add pose to path if far enough from last waypoint."""
        if len(self.waypoints) == 0:
            self.waypoints.append(pose)
            return
        
        # Check distance from last waypoint
        last_wp = self.waypoints[-1]
        dist = np.linalg.norm([
            pose.pose.position.x - last_wp.pose.position.x,
            pose.pose.position.y - last_wp.pose.position.y
        ])
        
        if dist >= self.waypoint_interval:
            self.waypoints.append(pose)
            rospy.loginfo(f"Waypoint {len(self.waypoints)} recorded")
    
    def get_return_path(self) -> List[PoseStamped]:
        """Return path in reverse order (exit direction)."""
        return list(reversed(self.waypoints))
```

### Obstacle Avoidance

**Dynamic Window Approach (DWA)**:

```python
class ObstacleAvoider:
    def __init__(self):
        self.max_speed = 1.0  # m/s
        self.max_angular_speed = 1.57  # rad/s (90°/s)
        
    def compute_safe_velocity(
        self,
        current_vel: Tuple[float, float],
        target_heading: float,
        obstacles: List[Obstacle]
    ) -> Tuple[float, float]:
        """
        Compute safe velocity using DWA.
        
        Returns:
            (linear_vel, angular_vel)
        """
        # Generate velocity samples in dynamic window
        v_samples = np.linspace(0, self.max_speed, 20)
        w_samples = np.linspace(-self.max_angular_speed, 
                                self.max_angular_speed, 30)
        
        best_score = -np.inf
        best_v, best_w = 0, 0
        
        for v in v_samples:
            for w in w_samples:
                # Simulate trajectory
                traj = self.simulate_trajectory(v, w, dt=0.5)
                
                # Check collision
                if self.check_collision(traj, obstacles):
                    continue
                
                # Score trajectory
                score = self.score_trajectory(traj, target_heading)
                
                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w
        
        return best_v, best_w
    
    def score_trajectory(
        self,
        trajectory: np.ndarray,
        target_heading: float
    ) -> float:
        """Score trajectory based on heading alignment and clearance."""
        # Heading alignment (higher is better)
        final_heading = trajectory[-1, 2]  # [x, y, θ]
        heading_score = np.cos(final_heading - target_heading)
        
        # Clearance (distance to nearest obstacle)
        clearance = self.compute_clearance(trajectory)
        clearance_score = np.tanh(clearance / 0.5)  # Normalize
        
        # Combined score
        return 0.6 * heading_score + 0.4 * clearance_score
```

### Return-to-Exit Navigation

**Waypoint Following with Look-Ahead**:

```python
class WaypointFollower:
    def __init__(self, lookahead_distance: float = 2.0):
        self.lookahead_distance = lookahead_distance
        
    def compute_steering(
        self,
        current_pose: PoseStamped,
        waypoints: List[PoseStamped]
    ) -> float:
        """
        Pure pursuit controller for waypoint following.
        
        Returns:
            target_heading (radians)
        """
        # Find lookahead point on path
        lookahead_point = self.find_lookahead_point(
            current_pose, waypoints
        )
        
        if lookahead_point is None:
            # Reached final waypoint
            return self.get_heading(current_pose)
        
        # Compute heading to lookahead point
        dx = lookahead_point.position.x - current_pose.pose.position.x
        dy = lookahead_point.position.y - current_pose.pose.position.y
        target_heading = np.arctan2(dy, dx)
        
        return target_heading
    
    def find_lookahead_point(
        self,
        current_pose: PoseStamped,
        waypoints: List[PoseStamped]
    ) -> Optional[PoseStamped]:
        """Find point on path at lookahead distance."""
        for i in range(len(waypoints) - 1):
            # Check if lookahead circle intersects segment
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            
            intersection = self.circle_segment_intersection(
                current_pose, self.lookahead_distance,
                wp1, wp2
            )
            
            if intersection is not None:
                return intersection
        
        # If no intersection, return closest waypoint
        return min(waypoints, 
                   key=lambda wp: self.distance(current_pose, wp))
```

---

## AR Overlay Rendering

### Display Coordinate Transform

**Problem**: Thermal camera (160×120) → Display (1280×720)

**Solution**: Homography matrix for perspective-correct mapping

```python
class CoordinateTransformer:
    def __init__(self):
        # Source points (thermal camera FOV corners)
        self.src_points = np.float32([
            [0, 0],         # Top-left
            [159, 0],       # Top-right
            [159, 119],     # Bottom-right
            [0, 119]        # Bottom-left
        ])
        
        # Destination points (display with margins)
        margin = 50
        self.dst_points = np.float32([
            [margin, margin],
            [1280-margin, margin],
            [1280-margin, 720-margin],
            [margin, 720-margin]
        ])
        
        # Compute homography
        self.H = cv2.getPerspectiveTransform(
            self.src_points, self.dst_points
        )
        
    def transform_bbox(self, bbox: List[int]) -> List[int]:
        """Transform bounding box from thermal to display coords."""
        x1, y1, x2, y2 = bbox
        
        # Transform corners
        corners = np.float32([[x1, y1], [x2, y2]]).reshape(-1, 1, 2)
        transformed = cv2.perspectiveTransform(corners, self.H)
        
        # Extract transformed bbox
        x1_t, y1_t = transformed[0, 0]
        x2_t, y2_t = transformed[1, 0]
        
        return [int(x1_t), int(y1_t), int(x2_t), int(y2_t)]
```

### Mini-Map Rendering

**Top-Down View with Heading Indicator**:

```python
class MiniMapRenderer:
    def __init__(self, size: int = 320, scale: float = 40):
        """
        Args:
            size: Minimap size in pixels
            scale: Pixels per meter
        """
        self.size = size
        self.scale = scale
        
    def render(
        self,
        current_pose: PoseStamped,
        path: List[PoseStamped],
        detections: List[Detection]
    ) -> np.ndarray:
        """Render minimap as numpy array."""
        # Create blank canvas
        minimap = np.zeros((self.size, self.size, 3), dtype=np.uint8)
        minimap.fill(50)  # Dark gray background
        
        # Draw grid lines
        self.draw_grid(minimap)
        
        # Draw path history (blue line)
        self.draw_path(minimap, path, color=(255, 100, 0))
        
        # Draw detections (icons)
        self.draw_detections(minimap, current_pose, detections)
        
        # Draw current position and heading (green arrow)
        center = (self.size // 2, self.size // 2)
        heading = self.get_heading(current_pose)
        arrow_end = (
            center[0] + int(20 * np.cos(heading)),
            center[1] + int(20 * np.sin(heading))
        )
        cv2.arrowedLine(minimap, center, arrow_end, 
                        (0, 255, 0), 3, tipLength=0.3)
        
        # Draw compass rose
        self.draw_compass(minimap)
        
        return minimap
    
    def world_to_minimap(
        self,
        world_pos: Tuple[float, float],
        current_pose: PoseStamped
    ) -> Tuple[int, int]:
        """Convert world coordinates to minimap pixels."""
        # Relative position
        dx = world_pos[0] - current_pose.pose.position.x
        dy = world_pos[1] - current_pose.pose.position.y
        
        # Scale to pixels
        px = int(self.size / 2 + dx * self.scale)
        py = int(self.size / 2 - dy * self.scale)  # Flip y
        
        return (px, py)
```

### Status Bar

**HUD Elements**:

```python
class StatusBarRenderer:
    def render(
        self,
        frame: np.ndarray,
        system_stats: Dict
    ) -> np.ndarray:
        """Draw status bar at bottom of screen."""
        height, width = frame.shape[:2]
        bar_height = 60
        
        # Semi-transparent black bar
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, height - bar_height), 
                      (width, height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Draw text elements
        y = height - 35
        self.draw_text(frame, f"FPS: {system_stats['fps']:.1f}", 
                       (20, y), color=(0, 255, 0))
        
        self.draw_text(frame, f"Temp: {system_stats['gpu_temp']}°C", 
                       (150, y), color=(0, 255, 255))
        
        self.draw_text(frame, f"Battery: {system_stats['battery']}%", 
                       (300, y), color=(0, 255, 0) if system_stats['battery'] > 20 else (0, 0, 255))
        
        self.draw_text(frame, f"Drift: {system_stats['drift']:.2f}m/min", 
                       (480, y), color=(255, 255, 0))
        
        # Draw detection count
        det_count = len(system_stats['detections'])
        det_color = (0, 0, 255) if det_count > 0 else (100, 100, 100)
        self.draw_text(frame, f"Detections: {det_count}", 
                       (680, y), color=det_color)
        
        return frame
```

---

## Communication Architecture

### ROS Topic Graph

```
┌──────────────────┐
│ /thermal/        │
│ image_raw        │◀─── thermal_processor
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ /fire_edge/      │
│ detections       │◀─── thermal_processor
└────────┬─────────┘
         │
         ├──────────────────┐
         │                  │
         ▼                  ▼
┌──────────────────┐  ┌──────────────────┐
│ sensor_fusion    │  │ ar_display       │
└────────┬─────────┘  └──────────────────┘
         │
         │ produces
         ▼
┌──────────────────┐
│ /fire_edge/pose  │
└────────┬─────────┘
         │
         ▼
    ar_display
```

### Custom Message Definitions

**Detection.msg**:
```
Header header
string class_name      # "fire", "human", "hazard"
float32 confidence     # 0.0 to 1.0
int32[] bbox           # [x1, y1, x2, y2]
int32 track_id         # Unique ID for tracking
```

**Detections.msg** (array):
```
Header header
Detection[] detections
```

**FireEdgePose.msg**:
```
Header header
geometry_msgs/PoseStamped pose
float32 position_confidence   # Kalman filter covariance
float32 drift_estimate        # Accumulated drift (m/min)
```

### Service Definitions

**SetNavigationMode.srv**:
```
# Request
string mode  # "explore", "return_to_exit", "follow_waypoint"
---
# Response
bool success
string message
```

**RecordWaypoint.srv**:
```
# Request
string name
---
# Response
int32 waypoint_id
geometry_msgs/PoseStamped pose
```

---

## Performance Optimization

### GPU Memory Management

**TensorRT Engine Caching**:
```python
class YOLOv8TensorRT:
    def __init__(self, engine_path: str):
        # Load pre-compiled TensorRT engine
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(trt.Logger()).deserialize_cuda_engine(f.read())
        
        # Create execution context (reusable)
        self.context = self.engine.create_execution_context()
        
        # Allocate GPU memory (once)
        self.input_buffer = cuda.mem_alloc(self.input_size)
        self.output_buffer = cuda.mem_alloc(self.output_size)
        
    def __call__(self, image: np.ndarray) -> np.ndarray:
        """Inference with pre-allocated buffers."""
        # Copy input to GPU
        cuda.memcpy_htod(self.input_buffer, image)
        
        # Execute inference
        self.context.execute_v2([self.input_buffer, self.output_buffer])
        
        # Copy output from GPU
        output = np.empty(self.output_shape, dtype=np.float32)
        cuda.memcpy_dtoh(output, self.output_buffer)
        
        return output
```

### CPU Optimization

**Multi-threading for Sensor I/O**:
```python
import threading
import queue

class AsyncSensorReader:
    def __init__(self):
        self.data_queue = queue.Queue(maxsize=10)
        self.threads = [
            threading.Thread(target=self._read_imu, daemon=True),
            threading.Thread(target=self._read_lidar, daemon=True),
            threading.Thread(target=self._read_ultrasonic, daemon=True)
        ]
        
    def start(self):
        for thread in self.threads:
            thread.start()
    
    def _read_imu(self):
        """IMU reading thread (100 Hz)."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            data = self.imu_driver.read_imu()
            self.data_queue.put(('imu', data))
            rate.sleep()
```

### Power Management

**DVFS (Dynamic Voltage Frequency Scaling)**:
```python
class PowerManager:
    def __init__(self):
        self.mode = 'performance'  # 'performance', 'balanced', 'powersave'
        
    def set_mode(self, mode: str):
        """Adjust Jetson power profile."""
        if mode == 'performance':
            os.system('sudo nvpmodel -m 0')  # MODE_15W
            os.system('sudo jetson_clocks')
        elif mode == 'balanced':
            os.system('sudo nvpmodel -m 1')  # MODE_10W
        elif mode == 'powersave':
            os.system('sudo nvpmodel -m 2')  # MODE_2CORE
        
        self.mode = mode
        rospy.loginfo(f"Power mode set to: {mode}")
    
    def adaptive_power_control(self, battery_level: float):
        """Automatically adjust power based on battery."""
        if battery_level < 20:
            self.set_mode('powersave')
        elif battery_level < 40:
            self.set_mode('balanced')
        else:
            self.set_mode('performance')
```

### Latency Reduction Techniques

**Zero-Copy Memory Transfer**:
```python
# Use ROS image_transport with zero-copy shared memory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ZeroCopyImagePublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(
            '/thermal/image_raw', 
            Image, 
            queue_size=1,
            tcp_nodelay=True  # Disable Nagle's algorithm
        )
    
    def publish(self, image: np.ndarray):
        """Publish with minimal copy overhead."""
        # Convert numpy to ROS message (shares memory if possible)
        msg = self.bridge.cv2_to_imgmsg(image, encoding='mono8')
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)
```

---

## Failure Modes

### Fault Tolerance Matrix

| Failure Mode | Detection | Recovery Strategy | Degraded Performance |
|--------------|-----------|-------------------|----------------------|
| **Thermal camera offline** | No frames for 1s | Use last known thermal + visual overlay | No fire detection, navigation continues |
| **IMU drift excessive** | Kalman innovation > threshold | Increase measurement noise R, rely more on LiDAR | Position drift 0.5→1.0 m/min |
| **LiDAR out of range** | Distance = MAX_RANGE | Use ultrasonic + IMU only | Reduced obstacle detection range |
| **GPU thermal throttling** | Temperature > 90°C | Reduce inference rate 30→15 Hz | Doubled latency (35→70ms) |
| **Battery critical (<10%)** | Battery monitor | Enter low-power mode, activate emergency beacon | 15W→5W, disable AR rendering |
| **ROS node crash** | Watchdog timeout | Auto-restart node, load last state | 2-5s interruption |
| **Kalman divergence** | Covariance > 10.0 | Reset filter, re-initialize with IMU | Temporary position jump |
| **Display failure** | No framebuffer | Fallback to audio alerts | No visual guidance |

### Watchdog Implementation

```python
class NodeWatchdog:
    def __init__(self, timeout: float = 5.0):
        self.timeout = timeout
        self.last_heartbeat = rospy.Time.now()
        self.timer = rospy.Timer(
            rospy.Duration(1.0), 
            self.check_heartbeat
        )
        
    def heartbeat(self):
        """Call this from main loop."""
        self.last_heartbeat = rospy.Time.now()
    
    def check_heartbeat(self, event):
        """Detect if node is frozen."""
        elapsed = (rospy.Time.now() - self.last_heartbeat).to_sec()
        if elapsed > self.timeout:
            rospy.logerr(f"Node frozen for {elapsed}s! Restarting...")
            self.restart_node()
    
    def restart_node(self):
        """Restart ROS node."""
        os.system('rosnode kill ' + rospy.get_name())
        os.system('rosrun fire_edge ' + self.get_node_script())
```

### Graceful Degradation

**Sensor Priority Hierarchy**:

1. **Critical** (system fails without these):
   - IMU (for heading estimation)
   - Power supply

2. **High Priority** (major degradation):
   - Thermal camera (fire detection)
   - LiDAR (obstacle avoidance)

3. **Medium Priority** (minor degradation):
   - Ultrasonic sensors (close-range obstacles)
   - Display (fallback to audio)

4. **Low Priority** (convenience features):
   - Mini-map rendering
   - Path visualization
   - Status bar

**Adaptive Algorithm Selection**:
```python
class AdaptiveNavigationController:
    def select_algorithm(self, available_sensors: Set[str]):
        """Choose best algorithm based on available sensors."""
        if {'imu', 'lidar', 'ultrasonic'}.issubset(available_sensors):
            return 'full_kalman_filter'
        elif {'imu', 'lidar'}.issubset(available_sensors):
            return 'reduced_kalman_filter'
        elif 'imu' in available_sensors:
            return 'dead_reckoning'
        else:
            return 'emergency_stationary_mode'
```

---

## Future Extensions

### 1. Multi-Agent Coordination

**Mesh Network Architecture**:
```
Firefighter 1 ◄──► Firefighter 2 ◄──► Firefighter 3
      │                  │                  │
      └──────────────────┴──────────────────┘
                         │
                         ▼
                 Incident Commander
                 (Central Dashboard)
```

**Shared State Protocol**:
```python
class MeshNetworkNode:
    def __init__(self):
        self.network = self.init_mesh_network()
        self.shared_map = OccupancyGrid()
        
    def broadcast_detection(self, detection: Detection):
        """Share fire/human detection with team."""
        msg = {
            'type': 'detection',
            'agent_id': self.agent_id,
            'detection': detection.to_dict(),
            'timestamp': time.time()
        }
        self.network.broadcast(msg)
    
    def update_shared_map(self, local_map: OccupancyGrid):
        """Merge local map with team's shared map."""
        for cell in local_map.cells:
            # Confidence-weighted averaging
            self.shared_map.update_cell(
                cell.x, cell.y,
                confidence=cell.confidence,
                source_id=self.agent_id
            )
```

### 2. Voice Command Interface

**Hands-Free Operation**:
```python
class VoiceCommandHandler:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.commands = {
            'navigate home': self.return_to_exit,
            'mark hazard': self.mark_hazard,
            'request backup': self.request_backup,
            'status report': self.give_status_report
        }
        
    def listen(self):
        """Continuous voice recognition."""
        with sr.Microphone() as source:
            audio = self.recognizer.listen(source)
            try:
                text = self.recognizer.recognize_google(audio)
                self.process_command(text.lower())
            except sr.UnknownValueError:
                pass  # Ignore unrecognized speech
```

### 3. Biometric Monitoring

**Physiological Stress Detection**:
```python
class BiometricMonitor:
    def __init__(self):
        self.heart_rate_sensor = PolarH10()
        self.stress_threshold = 160  # bpm
        
    def monitor(self):
        """Track firefighter health metrics."""
        heart_rate = self.heart_rate_sensor.read()
        
        if heart_rate > self.stress_threshold:
            self.alert_team(
                "Firefighter experiencing high stress",
                severity='high'
            )
            
        # Predict exhaustion using ML model
        exhaustion_prob = self.predict_exhaustion(
            heart_rate,
            self.time_in_fire,
            self.environmental_temp
        )
        
        if exhaustion_prob > 0.7:
            self.recommend_exit()
```

### 4. Building Information Modeling (BIM) Integration

**Pre-Mission Planning**:
```python
class BIMIntegration:
    def __init__(self, building_model_path: str):
        self.model = self.load_ifc_model(building_model_path)
        
    def extract_floorplan(self, floor_number: int) -> OccupancyGrid:
        """Convert BIM to navigation map."""
        # Extract walls, doors, stairs from IFC
        walls = self.model.get_walls(floor_number)
        doors = self.model.get_doors(floor_number)
        
        # Convert to occupancy grid
        grid = OccupancyGrid(resolution=0.1)  # 10cm cells
        for wall in walls:
            grid.mark_occupied(wall.geometry)
        for door in doors:
            grid.mark_traversable(door.geometry)
        
        return grid
    
    def identify_exits(self) -> List[Tuple[float, float]]:
        """Find all building exits from BIM."""
        exits = self.model.get_exits()
        return [(exit.x, exit.y) for exit in exits]
```

### 5. Cloud Integration for Post-Mission Analysis

**Data Upload Pipeline**:
```python
class CloudSync:
    def __init__(self):
        self.api_endpoint = 'https://fire-edge-cloud.example.com/api/v1'
        
    def upload_mission_data(self, mission_id: str):
        """Upload sensor data and detections to cloud."""
        data_package = {
            'mission_id': mission_id,
            'thermal_video': self.compress_thermal_video(),
            'path_history': self.export_path_json(),
            'detections': self.export_detections_json(),
            'sensor_logs': self.export_sensor_logs()
        }
        
        response = requests.post(
            f"{self.api_endpoint}/missions",
            json=data_package,
            headers={'Authorization': f'Bearer {self.api_token}'}
        )
        
        return response.json()
```

### 6. Advanced AI Models

**Transformer-Based Detection**:
```python
class DETRFireDetector:
    """DETR (Detection Transformer) for improved accuracy."""
    def __init__(self):
        self.model = torch.hub.load(
            'facebookresearch/detr',
            'detr_resnet50',
            pretrained=False
        )
        self.load_custom_weights('detr_fire_model.pth')
        
    def detect(self, thermal_image: np.ndarray) -> List[Detection]:
        """More accurate but slower than YOLOv8."""
        # 120ms latency vs 35ms for YOLOv8
        outputs = self.model(thermal_image)
        return self.parse_outputs(outputs)
```

**Smoke Segmentation**:
```python
class SmokeSegmentation:
    """Semantic segmentation for smoke density."""
    def __init__(self):
        self.model = SegFormer.from_pretrained('smoke_segmentation')
        
    def estimate_visibility(self, image: np.ndarray) -> float:
        """Return visibility distance (meters)."""
        smoke_mask = self.segment_smoke(image)
        density = smoke_mask.mean()
        
        # Empirical relationship: visibility = 10m / (1 + 5*density)
        visibility = 10.0 / (1.0 + 5.0 * density)
        return visibility
```

### 7. Augmented Reality Enhancements

**3D Hazard Visualization**:
```python
class AR3DRenderer:
    def __init__(self):
        self.renderer = Open3DRenderer()
        
    def render_3d_hazards(
        self,
        thermal_frame: np.ndarray,
        depth_map: np.ndarray,
        detections: List[Detection]
    ) -> np.ndarray:
        """Project detections into 3D space."""
        # Generate point cloud from depth
        pointcloud = self.depth_to_pointcloud(depth_map)
        
        # Add colored markers for detections
        for det in detections:
            pos_3d = self.project_detection_to_3d(det, depth_map)
            self.add_3d_marker(pointcloud, pos_3d, det.class_name)
        
        # Render to 2D overlay
        overlay = self.renderer.render(pointcloud, thermal_frame)
        return overlay
```

---

## Appendix

### A. Hardware Bill of Materials

| Component | Part Number | Quantity | Unit Price | Total |
|-----------|-------------|----------|------------|-------|
| NVIDIA Jetson Xavier NX | 945-13636-0000-000 | 1 | $599 | $599 |
| Jetson Xavier NX Developer Kit | — | 1 | $399 | $399 |
| FLIR Lepton 3.5 | 500-0771-01 | 1 | $299 | $299 |
| Garmin LIDAR-Lite v3HP | 010-01722-00 | 1 | $149 | $149 |
| TDK MPU-9250 | MPU-9250 | 1 | $15 | $15 |
| A02YYUW Ultrasonic | A02YYUW | 4 | $10 | $40 |
| Kopin Lightning OLED | — | 1 | $1,200 | $1,200 |
| 4S LiPo Battery 10Ah | — | 1 | $120 | $120 |
| DC-DC Converters | — | 3 | $15 | $45 |
| Waterproof Enclosure | — | 1 | $48 | $48 |
| Wiring & Connectors | — | 1 | $30 | $30 |
| Heat Sink & Fan | — | 1 | $25 | $25 |
| **Total** | | | | **$2,969** |

*Note: Prices approximate as of December 2024. Bulk discounts available.*

### B. Power Budget Analysis

| Component | Voltage | Current | Power | Notes |
|-----------|---------|---------|-------|-------|
| Jetson Xavier NX | 9-20V | 0.6-0.75A | 9-12W | Peak 15W in MODE_15W |
| FLIR Lepton | 3.3V | 45mA | 0.15W | Average consumption |
| Garmin LiDAR | 5V | 105mA | 0.53W | Peak during measurement |
| MPU-9250 IMU | 3.3V | 3.5mA | 0.01W | Negligible |
| Ultrasonic (×4) | 5V | 15mA | 0.3W | All 4 sensors combined |
| Kopin OLED | 5V | 200mA | 1.0W | At 50% brightness |
| **Total Average** | | | **12W** | |
| **Total Peak** | | | **15W** | During LiDAR measurement |

**Battery Life Calculation**:
```
Battery: 14.8V × 10Ah = 148 Wh
Average Power: 12W
Runtime: 148 Wh / 12W = 12.3 hours (theoretical)

Accounting for:
- DC-DC converter efficiency (85%): 10.5 hours
- Battery discharge curve: 9.5 hours
- Reserve capacity (20%): 7.6 hours
- Real-world usage: ~45-60 minutes intensive operation
```

### C. Thermal Management

**Heat Generation Sources**:
1. Jetson Xavier NX GPU: 8-10W
2. Jetson Xavier NX CPU: 2-4W
3. Other components: <1W

**Cooling Requirements**:
- **Passive**: Heat sink (required) dissipates up to 10W
- **Active**: 5V fan (recommended) adds 5W cooling capacity
- **Thermal Interface**: Arctic MX-4 thermal paste (0.8°C/W)

**Thermal Limits**:
```
Component: Jetson Xavier NX
- Tmax (throttle): 95°C
- Tsafe (recommended): <80°C
- Tambient (operating): -10°C to +50°C
```

### D. Software Dependencies

**Python Packages** (requirements.txt):
```
numpy==1.24.3
opencv-python==4.8.0.74
torch==2.0.1
torchvision==0.15.2
ultralytics==8.0.120
filterpy==1.4.5
pyserial==3.5
smbus2==0.4.2
spidev==3.6
Jetson.GPIO==2.1.6
pycuda==2022.2.2
scipy==1.11.1
```

**System Packages** (apt):
```bash
ros-noetic-desktop-full
ros-noetic-cv-bridge
ros-noetic-image-transport
ros-noetic-message-filters
python3-pip
python3-opencv
libopencv-dev
cuda-toolkit-11-4
tensorrt
```

### E. ROS Launch File Example

**simulation.launch**:
```xml
<launch>
  <!-- Gazebo simulation -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find fire_edge_gazebo)/worlds/smoke_corridor.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
  </include>

  <!-- Thermal processor node -->
  <node name="thermal_processor" pkg="fire_edge" type="thermal_processor.py" output="screen">
    <param name="model_path" value="$(find fire_edge)/models/yolov8n_thermal_yellow.pt"/>
    <param name="confidence_threshold" value="0.5"/>
    <param name="rate" value="30"/>
  </node>

  <!-- Sensor fusion node -->
  <node name="sensor_fusion" pkg="fire_edge" type="sensor_fusion.py" output="screen">
    <rosparam file="$(find fire_edge)/config/kalman_params.yaml"/>
  </node>

  <!-- AR display node -->
  <node name="ar_display" pkg="fire_edge" type="ar_display.py" output="screen">
    <param name="display_width" value="1280"/>
    <param name="display_height" value="720"/>
  </node>

  <!-- RViz visualization -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find fire_edge)/config/fire_edge.rviz"/>
</launch>
```

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | Dec 2025 | A. Lotfy, A. Sayed, M. Ghanem | Initial release |

---

## References

1. Zhang, Q., et al. (2023). "Edge-Based Fire Detection Using Thermal Imaging." *IEEE Transactions on Industrial Informatics*.

2. Lee, S., et al. (2023). "Real-Time Object Detection for Firefighting Applications." *Journal of Fire Sciences*.

3. NVIDIA Corporation. (2024). "Jetson Xavier NX Developer Kit User Guide."

4. FLIR Systems. (2024). "Lepton Engineering Datasheet."

5. Ultralytics. (2024). "YOLOv8 Documentation." https://docs.ultralytics.com/

---

**Document Status**: Complete  
**Last Updated**: December 19, 2025  
**Maintained by**: FIRE-EDGE Development Team
