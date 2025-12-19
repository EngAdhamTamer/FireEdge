# 🔥 FIRE-EDGE: Real-Time Firefighting Edge System

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](http://wiki.ros.org/noetic)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-orange.svg)](https://github.com/ultralytics/ultralytics)
[![Paper](https://img.shields.io/badge/Paper-Conference-red.svg)](docs/FIRE-EDGE.pdf)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

> **A Real-Time Firefighting Edge System with Thermal Imaging, Sensor Fusion, and Augmented Reality**

An advanced augmented reality (AR) navigation system designed to enhance firefighter safety in smoke-filled, zero-visibility environments. FIRE-EDGE leverages edge AI, multi-sensor fusion, and thermal imaging to provide real-time hazard detection and drift-resistant indoor navigation.

**🎓 Research Project** | New Ismailia National University, Egypt  
**👥 Authors**: Ahmed S. Lotfy, Adham T. Sayed, Mena R. Ghanem  
**📊 Performance**: 85% mAP@0.5 | 35ms latency | $2,470 hardware cost

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Hardware Components](#-hardware-components)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Performance Metrics](#-performance-metrics)
- [Research Paper](#-research-paper)
- [Contributing](#-contributing)
- [Citation](#-citation)

---

## 🎯 Overview

Firefighters face life-threatening disorientation in smoke-filled environments, contributing to **28% of on-duty fatalities globally**. FIRE-EDGE addresses this critical safety challenge through:

- **Edge AI Processing**: Real-time fire and human detection using YOLOv8n on NVIDIA Jetson Xavier NX
- **Multi-Sensor Fusion**: Kalman filter integration of thermal, LiDAR, IMU, and ultrasonic data
- **Low-Cost Design**: Complete system for under $2,500 (vs. $10,000+ commercial alternatives)
- **Zero Cloud Dependency**: 35ms latency vs. 500-800ms for cloud-based systems
- **Drift-Resistant Navigation**: 0.3m/min drift (40% better than IMU-only systems)

### Problem Statement

Existing AR systems for firefighters face critical limitations:
- ❌ Cloud dependency introduces dangerous latency (>500ms)
- ❌ High costs ($10,000+) limit accessibility
- ❌ Poor accuracy in smoke-filled environments
- ❌ GPS unavailability indoors

### Our Solution

✅ **Edge AI**: All processing on-device, no cloud required  
✅ **Thermal Vision**: See through smoke using FLIR Lepton 3.5  
✅ **Sensor Fusion**: Kalman filter combines 4 sensor types for robust navigation  
✅ **Affordable**: Total hardware cost <$2,500  
✅ **Open Source**: Reproducible research with public code and datasets

---

## ✨ Key Features

### 🔥 Fire & Human Detection
- **YOLOv8n Nano Model** optimized for thermal imagery
- **85% mAP@0.5** accuracy on fire/human/hazard detection
- **35ms inference latency** on Jetson Xavier NX
- Trained on 5,000+ thermal images with synthetic smoke augmentation

### 🧭 Drift-Resistant Navigation
- **Kalman Filter** fuses IMU (100Hz), LiDAR (10Hz), and ultrasonic (5Hz) data
- **0.3m/min drift** - 40% better than IMU-only baselines
- Real-time obstacle mapping in 3D space (40m range)
- Automatic return-to-exit path recording

### 🌡️ Thermal Imaging
- **FLIR Lepton 3.5** radiometric thermal camera (160×120, 8-14μm)
- See through smoke by detecting heat signatures
- Custom CLAHE enhancement for 40% contrast improvement
- Hot spot detection with adaptive thresholding

### 📱 AR Display
- **Kopin Lightning OLED** 0.5" micro-display (1280×720)
- **10,000 nits** brightness visible in any lighting condition
- Real-time hazard overlays and navigation arrows
- Live mini-map with current position and heading

### ⚡ Power Efficiency
- **12W total power** consumption (vs. 15W baseline)
- **45-minute runtime** on 10,000mAh battery
- DVFS power management on Jetson NX
- Ultra-low-power sensors (<5W combined)

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     HARDWARE LAYER                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │  FLIR    │  │  Garmin  │  │   MPU    │  │Ultrasonic│       │
│  │ Lepton   │  │  LiDAR   │  │  9250    │  │  Sensors │       │
│  │  3.5     │  │ Lite v3HP│  │   IMU    │  │(A02YYUW) │       │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘       │
└───────┼─────────────┼─────────────┼─────────────┼──────────────┘
        │             │             │             │
        ▼             ▼             ▼             ▼
┌─────────────────────────────────────────────────────────────────┐
│                    EDGE PROCESSING                               │
│              NVIDIA Jetson Xavier NX (16GB)                      │
│     384-core Volta GPU • 6-core ARM CPU • 21 TOPS                │
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              ROS NOETIC MIDDLEWARE                        │  │
│  │  ┌─────────────────┐  ┌─────────────────┐               │  │
│  │  │ /thermal_       │  │ /sensor_        │               │  │
│  │  │  processor      │  │  fusion         │               │  │
│  │  │                 │  │                 │               │  │
│  │  │  YOLOv8n +      │  │  Kalman Filter  │               │  │
│  │  │  TensorRT       │  │  (5-DoF state)  │               │  │
│  │  │  FP16 Quant     │  │                 │               │  │
│  │  └────────┬────────┘  └────────┬────────┘               │  │
│  │           │                     │                        │  │
│  │           └──────────┬──────────┘                        │  │
│  │                      ▼                                    │  │
│  │           ┌─────────────────────┐                        │  │
│  │           │   /ar_display       │                        │  │
│  │           │                     │                        │  │
│  │           │  OpenCV Pipeline    │                        │  │
│  │           │  Homography Align   │                        │  │
│  │           └──────────┬──────────┘                        │  │
│  └────────────────────────┼───────────────────────────────┘  │
└─────────────────────────────┼───────────────────────────────────┘
                              ▼
                    ┌─────────────────────┐
                    │  Kopin Lightning    │
                    │  OLED (0.5")        │
                    │  1280×720 | 10K nits│
                    └─────────────────────┘
                              │
                              ▼
                    ┌─────────────────────┐
                    │   FIREFIGHTER       │
                    │   AR OVERLAY        │
                    │  • Hazard Alerts    │
                    │  • Navigation Arrows│
                    │  • Mini-Map         │
                    │  • Return Path      │
                    └─────────────────────┘
```

### Data Flow

1. **Sensor Acquisition** (100Hz IMU, 10Hz LiDAR, 30Hz Thermal)
2. **Thermal Processing**: YOLOv8n detects fire/humans with 35ms latency
3. **Sensor Fusion**: Kalman filter estimates position with 0.3m/min drift
4. **AR Rendering**: OpenCV generates overlays aligned to OLED display
5. **Display Output**: Real-time visualization on Kopin micro-display

---

## 🔧 Hardware Components

| Component | Model | Specifications | Purpose | Cost |
|-----------|-------|----------------|---------|------|
| **Edge Computer** | NVIDIA Jetson Xavier NX | 384-core Volta GPU<br>6-core ARM CPU<br>16GB RAM<br>21 TOPS | Real-time AI inference | $599 |
| **Thermal Camera** | FLIR Lepton 3.5 | 160×120 resolution<br>57°×44° FOV<br>8-14μm spectrum | See through smoke | $299 |
| **LiDAR** | Garmin LIDAR-Lite v3HP | 40m range<br>±2.5cm accuracy<br>10Hz update rate | 3D obstacle mapping | $149 |
| **IMU** | TDK MPU-9250 | 9-DoF (gyro/accel/mag)<br>100Hz sample rate | Motion tracking | $15 |
| **Ultrasonic** | A02YYUW (×4) | 4.5m range<br>±1cm accuracy<br>Waterproof | Close-range obstacles | $40 |
| **Display** | Kopin Lightning OLED | 1280×720 HD<br>10,000 nits<br>0.5" diagonal | AR visualization | $1,200 |
| **Battery** | Custom 4S LiPo | 14.8V, 10,000mAh<br>45min runtime | Power supply | $120 |
| **Enclosure** | Custom | Waterproof, heat-resistant | System housing | $48 |
| **Total** | | | | **$2,470** |

### Why These Components?

- **Jetson Xavier NX**: Only edge device with sufficient GPU power (21 TOPS) for real-time YOLOv8 at <50ms latency
- **FLIR Lepton 3.5**: Lowest-cost radiometric thermal camera (competitors: $2,000+)
- **Kopin OLED**: Ultra-high brightness (10,000 nits) critical for smoke visibility
- **MPU-9250**: Industry-standard 9-DoF IMU with proven reliability

---

## 🚀 Installation

### System Requirements

**Hardware (Full System)**:
- NVIDIA Jetson Xavier NX Developer Kit
- USB webcam (for simulation/testing)
- 16GB+ microSD card
- Host PC with Ubuntu 20.04 (for development)

**Software**:
- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+
- CUDA 11.4+ (for Jetson)

### Option 1: Simulation Environment (Recommended for Development)

```bash
# 1. Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full

# 2. Install Gazebo 11
sudo apt install gazebo11 ros-noetic-gazebo-ros-pkgs

# 3. Clone repository
git clone https://github.com/yourusername/fire-edge.git
cd fire-edge

# 4. Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/fire-edge .
cd ~/catkin_ws
catkin_make

# 5. Install Python dependencies
pip3 install -r requirements.txt

# 6. Download YOLOv8 weights (trained on thermal dataset)
python3 scripts/download_models.py
```

### Option 2: Jetson Xavier NX Deployment

```bash
# 1. Flash JetPack 5.1 to Jetson NX
# Download from: https://developer.nvidia.com/embedded/jetpack

# 2. Install dependencies
sudo apt install python3-pip libopencv-dev
pip3 install jetson-stats

# 3. Install PyTorch with CUDA
wget https://nvidia.box.com/shared/static/[...].whl
pip3 install torch-*.whl

# 4. Clone and build
git clone https://github.com/yourusername/fire-edge.git
cd fire-edge
python3 setup.py install

# 5. Test hardware
python3 tests/test_sensors.py
```

### Option 3: Demo Mode (No Hardware)

```bash
# Quick start with pre-recorded data
git clone https://github.com/yourusername/fire-edge.git
cd fire-edge
pip install -r requirements.txt
python3 demo/run_demo.py --dataset sample_fire_scenario
```

---

## 🎮 Quick Start

### 1. Run Simulation (Gazebo)

```bash
# Terminal 1: Launch ROS and Gazebo
source ~/catkin_ws/devel/setup.bash
roslaunch fire_edge simulation.launch

# Terminal 2: Start thermal processor
rosrun fire_edge thermal_processor.py

# Terminal 3: Start sensor fusion
rosrun fire_edge sensor_fusion.py

# Terminal 4: Launch AR display
rosrun fire_edge ar_display.py
```

**Gazebo Controls**:
- `W/A/S/D` - Move firefighter forward/left/back/right
- `Arrow Keys` - Rotate view
- `SPACE` - Add waypoint
- `N` - Toggle navigation mode

### 2. Run Basic Demo Scripts

**Thermal Vision Demo**:
```bash
python ar_firefighter_helmet_cv.py
# Shows: Grayscale | Infrared Simulation | Contours
```

**AR Navigation Demo**:
```bash
python ar_navigation.py
# Move camera sideways to initialize tracking
# Press SPACE to add waypoints, N for navigation
```

**Full Simulator**:
```bash
python firefighter_simulator.py
# W/S: Move forward/back
# A/D: Turn left/right 90°
# N: Activate return-to-exit path
```

### 3. Test with Pre-Recorded Data

```bash
# Run YOLOv8 on sample thermal video
python scripts/test_detection.py --video data/smoke_corridor.mp4

# Test Kalman filter with sensor logs
python scripts/test_fusion.py --log data/sensor_log.csv

# Benchmark full pipeline
python scripts/benchmark.py --scenario collapsing_structure
```

---

## 📊 Performance Metrics

### Object Detection

| Model | mAP@0.5 | Precision | Recall | Latency | Power |
|-------|---------|-----------|--------|---------|-------|
| **FIRE-EDGE (Yellow)** | **85.0%** ± 1.8 | **89.2%** ± 2.3 | **86.1%** ± 2.7 | **35ms** ± 5 | **12W** |
| FIRE-EDGE (Red) | 83.2% ± 2.1 | 85.4% ± 3.1 | 82.3% ± 3.5 | 42ms ± 6 | 12W |
| Zhang et al. [1] | 83.2% | - | - | 800ms | - |
| Lee et al. [2] | 82.5% | - | - | 50ms | 15W |

### Navigation Accuracy

| Metric | FIRE-EDGE | Baseline [1] | Improvement |
|--------|-----------|--------------|-------------|
| **Navigation Drift** | 0.3 ± 0.1 m/min | 0.5 ± 0.2 m/min | **40% better** |
| **System Power** | 12 ± 0.5 W | 15 ± 1.0 W | 20% reduction |
| **Battery Life** | 45 ± 3 min | 30 ± 5 min | 50% longer |
| **Total Cost** | $2,470 | $5,200 | 52% cheaper |

### Key Achievements

✅ **85% mAP@0.5** - State-of-the-art for thermal fire detection  
✅ **35ms latency** - 16.7% faster than baseline, enables real-time response  
✅ **0.3m/min drift** - 40% more accurate than IMU-only systems  
✅ **12W power** - 50% longer battery life than commercial systems  
✅ **$2,470 cost** - 52% cheaper than cloud-dependent alternatives  

### Limitations

- ⚠️ **Battery Life**: 45 minutes may be insufficient for extended operations
- ⚠️ **Thermal Range**: Accuracy drops to 63% beyond 15m (FLIR Lepton limitation)
- ⚠️ **False Positives**: 8% misclassification of steam as fire in high humidity
- ⚠️ **Computation**: Jetson NX throttles under sustained 100% GPU load

---

## 📄 Research Paper

This system is based on our conference paper:

**"FIRE-EDGE: A Real-Time Firefighting Edge System with Thermal Imaging, Sensor Fusion, and Augmented Reality"**

**Authors**: Ahmed S. Lotfy, Adham T. Sayed, Mena R. Ghanem  
**Institution**: New Ismailia National University, Ismailia, Egypt  
**Conference**: 1st Student Research Conference - Role of University in Sustainable Development

📥 [**Download Full Paper (PDF)**](docs/FIRE-EDGE.pdf)

### Abstract

Firefighters face life-threatening challenges in smoke-filled environments, with disorientation contributing to 28% of fatalities. This paper presents FIRE-EDGE, a low-cost augmented reality (AR) system integrating edge AI and multi-sensor fusion. Leveraging YOLOv8 for fire detection and a Kalman filter for navigation, the system achieves 85% mAP@0.5 accuracy with 35ms latency on an NVIDIA Jetson Xavier NX. Simulations in ROS/Gazebo demonstrate superior performance over baseline models, offering a sustainable solution for firefighter safety.

---

## 🗂️ Project Structure

```
fire-edge/
│
├── README.md                          # This file
├── LICENSE                            # MIT License
├── requirements.txt                   # Python dependencies
├── setup.py                          # Package installation
│
├── docs/                             # Documentation
│   ├── FIRE-EDGE.pdf                 # Research paper
│   ├── ARCHITECTURE.md               # System architecture
│   ├── SETUP_GUIDE.md               # Detailed setup
│   ├── USER_MANUAL.md               # Operation guide
│   └── images/                       # Diagrams & screenshots
│
├── src/                              # ROS packages
│   ├── fire_edge/                    # Main package
│   │   ├── nodes/                    # ROS nodes
│   │   │   ├── thermal_processor.py  # YOLOv8 inference
│   │   │   ├── sensor_fusion.py      # Kalman filter
│   │   │   └── ar_display.py         # AR overlay
│   │   ├── launch/                   # Launch files
│   │   │   └── simulation.launch     # Gazebo simulation
│   │   └── config/                   # Parameters
│   │       ├── yolov8_config.yaml
│   │       └── kalman_params.yaml
│   └── fire_edge_gazebo/            # Simulation
│       ├── worlds/                   # Fire scenarios
│       │   ├── smoke_corridor.world
│       │   └── collapsing_structure.world
│       └── plugins/                  # Custom plugins
│           └── smoke_dynamics.cpp
│
├── scripts/                          # Utility scripts
│   ├── download_models.py            # Get YOLOv8 weights
│   ├── test_detection.py             # Test detection
│   ├── test_fusion.py                # Test Kalman filter
│   └── benchmark.py                  # Performance testing
│
├── demo/                             # Standalone demos
│   ├── ar_firefighter_helmet_cv.py   # Basic thermal vision
│   ├── ar_navigation.py              # Navigation system
│   └── firefighter_simulator.py      # Full simulator
│
├── data/                             # Sample data
│   ├── smoke_corridor.mp4            # Test video
│   ├── sensor_log.csv                # Sensor recordings
│   └── thermal_images/               # Sample thermals
│
├── models/                           # Trained models
│   ├── yolov8n_thermal_yellow.pt     # Yellow model (85%)
│   ├── yolov8n_thermal_red.pt        # Red model (83%)
│   └── README.md                     # Model info
│
└── tests/                            # Unit tests
    ├── test_sensors.py               # Hardware tests
    ├── test_yolo.py                  # Detection tests
    ├── test_kalman.py                # Fusion tests
    └── test_ros_nodes.py             # Integration tests
```

---

## 🛠️ Development

### Training YOLOv8 on Custom Thermal Dataset

```bash
# 1. Download foduucom thermal dataset
python scripts/download_dataset.py

# 2. Prepare annotations (convert to YOLO format)
python scripts/prepare_annotations.py

# 3. Train YOLOv8n
python scripts/train_yolo.py --epochs 120 --batch 32 --img 640

# 4. Export to TensorRT for Jetson
python scripts/export_tensorrt.py --model runs/train/exp/weights/best.pt
```

### Calibrating Kalman Filter

```bash
# Collect sensor data in controlled environment
roslaunch fire_edge calibration.launch

# Tune Q and R matrices
python scripts/tune_kalman.py --data calibration_data.bag

# Validate on test scenarios
python scripts/validate_kalman.py
```

### Running Tests

```bash
# Unit tests
pytest tests/ -v

# ROS integration tests
rostest fire_edge test_nodes.test

# Hardware tests (requires actual sensors)
python tests/test_sensors.py

# Full system benchmark
python scripts/benchmark.py --scenario all
```

---

## 🤝 Contributing

We welcome contributions! This is an open-source research project.

### How to Contribute

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/AmazingFeature`)
3. **Commit** your changes (`git commit -m 'Add some AmazingFeature'`)
4. **Push** to the branch (`git push origin feature/AmazingFeature`)
5. **Open** a Pull Request

### Areas We Need Help

- 🔥 **Real Thermal Camera Integration**: FLIR Lepton 3.5 drivers
- 🌐 **Multi-Agent Systems**: Mesh networking for team coordination
- 🗣️ **Voice Interface**: Hands-free command system
- 📱 **Mobile App**: Incident commander dashboard
- 🏢 **BIM Integration**: Building information system integration
- 🧪 **Testing**: Real-world fire department trials

### Development Guidelines

- Follow PEP 8 style guidelines
- Add unit tests for new features
- Update documentation
- Ensure ROS messages are properly documented

---

## 📚 Citation

If you use FIRE-EDGE in your research, please cite:

```bibtex
@inproceedings{lotfy2024fireedge,
  title={FIRE-EDGE: A Real-Time Firefighting Edge System with Thermal Imaging, Sensor Fusion, and Augmented Reality},
  author={Lotfy, Ahmed S. and Sayed, Adham T. and Ghanem, Mena R.},
  booktitle={Proceedings of the 1st Student Research Conference on the Role of University in Sustainable and Community Development},
  year={2024},
  organization={New Ismailia National University},
  address={Ismailia, Egypt}
}
```

---

## 📧 Contact

**Research Team**:
- **Ahmed S. Lotfy** - [ahmedsheriflotfy06@gmail.com](mailto:ahmedsheriflotfy06@gmail.com)
- **Adham T. Sayed** - [adhamt864@gmail.com](mailto:adhamt864@gmail.com) | [LinkedIn](https://www.linkedin.com/in/adhamtamer/)
- **Mena R. Ghanem** - [menaghanim863@gmail.com](mailto:menaghanim863@gmail.com)

**Institution**: New Ismailia National University, Ismailia, Egypt

---

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### Open Source Commitment

FIRE-EDGE is fully open-source to democratize access to life-saving technologies. We believe firefighter safety should not be limited by cost or proprietary systems.

---

## 🙏 Acknowledgments

- **New Ismailia National University** for project support
- **NVIDIA** for Jetson Developer Program
- **Ultralytics** for YOLOv8 framework
- **ROS Community** for robotics middleware
- **FLIR Systems** for thermal imaging technology
- **Firefighter community** for requirements and feedback

---

## ⚠️ Disclaimer

**This is a research prototype for academic and development purposes.**

FIRE-EDGE is **NOT** certified for actual firefighting operations. It has not undergone:
- NFPA (National Fire Protection Association) certification
- UL (Underwriters Laboratories) testing
- Real-world fire department validation

**Always follow proper safety protocols and use certified equipment in real emergencies.**

For deployment in operational environments, this system must undergo:
1. Extensive field testing with fire departments
2. Safety certification by relevant authorities
3. Hardware ruggedization for extreme conditions
4. Redundancy systems for mission-critical operation

---

## 🗺️ Roadmap

### Phase 1: Research Validation ✅ (Current)
- [x] YOLOv8 training on thermal dataset
- [x] Kalman filter sensor fusion
- [x] ROS/Gazebo simulation
- [x] Conference paper publication
- [x] Open-source code release

### Phase 2: System Integration 🚧 (Q1 2025)
- [ ] Full hardware prototype assembly
- [ ] Real FLIR Lepton 3.5 integration
- [ ] Kopin OLED display integration
- [ ] Field testing in controlled environments
- [ ] Power optimization (target: 60min battery)

### Phase 3: Pilot Program 📅 (Q2-Q3 2025)
- [ ] Partnership with local fire department
- [ ] Real-world testing in training facilities
- [ ] User feedback and iterative improvement
- [ ] Safety certification process initiation

### Phase 4: Advanced Features 🔮 (Future)
- [ ] Multi-firefighter mesh networking
- [ ] Voice command interface (hands-free)
- [ ] Heart rate & biometric monitoring
- [ ] Cloud-based incident replay
- [ ] Mobile app for incident commanders
- [ ] Digital twin integration
- [ ] 5G edge computing support

---

**Made with ❤️ for firefighter safety by the FIRE-EDGE Team**

*Empowering first responders through open-source innovation*
