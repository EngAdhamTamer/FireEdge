# Contributing to FIRE-EDGE

🔥 **Thank you for your interest in improving firefighter safety!** 🔥

We're excited that you want to contribute to FIRE-EDGE. This document provides guidelines for contributing to the project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [How Can I Contribute?](#how-can-i-contribute)
- [Development Setup](#development-setup)
- [Pull Request Process](#pull-request-process)
- [Style Guidelines](#style-guidelines)
- [Testing Requirements](#testing-requirements)
- [Areas We Need Help](#areas-we-need-help)

---

## Code of Conduct

### Our Pledge

FIRE-EDGE is a research project focused on saving lives. We are committed to providing a welcoming and inclusive environment for all contributors, regardless of background, experience level, gender identity, sexual orientation, disability, ethnicity, religion, age, or nationality.

### Expected Behavior

- **Be Respectful**: Treat all contributors with respect and courtesy
- **Be Collaborative**: Work together constructively
- **Focus on Safety**: Remember this is life-saving technology
- **Give Credit**: Acknowledge others' contributions
- **Be Professional**: Keep discussions technical and constructive

### Unacceptable Behavior

- Harassment or discrimination of any kind
- Trolling, insulting, or derogatory comments
- Publishing others' private information
- Any conduct that would be inappropriate in a professional setting

**Violations**: Contact the team at [adhamt864@gmail.com](mailto:adhamt864@gmail.com)

---

## How Can I Contribute?

### 🐛 Reporting Bugs

Before submitting a bug report:
1. **Check existing issues** to avoid duplicates
2. **Use the latest version** of the code
3. **Verify the bug** can be reproduced

**Good Bug Reports Include**:
```markdown
**Environment**:
- OS: Ubuntu 20.04
- Python: 3.8.10
- CUDA: 11.4 (if applicable)
- Hardware: Jetson Xavier NX / x86 PC / Other

**Steps to Reproduce**:
1. Run `roslaunch fire_edge simulation.launch`
2. Start thermal processor
3. Observe error in terminal

**Expected Behavior**:
YOLOv8 should detect fire objects in thermal feed

**Actual Behavior**:
Segmentation fault after 10 frames

**Error Logs**:
[Paste full error traceback here]

**Additional Context**:
- Attached thermal_processor.log
- Screenshot of Gazebo window
```

### 💡 Suggesting Features

We welcome feature suggestions! Before submitting:
1. Check if it aligns with project goals (firefighter safety)
2. Consider performance impact (edge device constraints)
3. Look for similar existing requests

**Good Feature Requests Include**:
- **Use case**: How would this help firefighters?
- **Implementation idea**: Rough technical approach
- **Alternatives considered**: Other solutions you've thought of
- **Priority**: Critical / High / Medium / Low

### 📝 Improving Documentation

Documentation improvements are always welcome:
- Fix typos or unclear explanations
- Add examples or tutorials
- Translate documentation (especially to Arabic for local fire departments)
- Create diagrams or visualizations
- Write blog posts or guides

### 🔬 Submitting Code

See [Pull Request Process](#pull-request-process) below.

---

## Development Setup

### Prerequisites

- Ubuntu 20.04 (or compatible)
- Python 3.8+
- ROS Noetic
- Git

### Initial Setup

```bash
# 1. Fork the repository on GitHub
# (Click "Fork" button at https://github.com/yourorg/fire-edge)

# 2. Clone your fork
git clone https://github.com/YOUR_USERNAME/fire-edge.git
cd fire-edge

# 3. Add upstream remote
git remote add upstream https://github.com/yourorg/fire-edge.git

# 4. Create virtual environment
python3 -m venv venv
source venv/bin/activate

# 5. Install development dependencies
pip install -r requirements.txt
pip install -r requirements-dev.txt  # If exists

# 6. Set up pre-commit hooks (optional but recommended)
pip install pre-commit
pre-commit install

# 7. Create catkin workspace (for ROS development)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/fire-edge .
cd ~/catkin_ws
catkin_make

# 8. Run tests to verify setup
pytest tests/
```

### Keeping Your Fork Updated

```bash
# Fetch upstream changes
git fetch upstream

# Merge into your local main
git checkout main
git merge upstream/main

# Push to your fork
git push origin main
```

---

## Pull Request Process

### 1. Create a Branch

```bash
# Create descriptive branch name
git checkout -b feature/thermal-camera-calibration
# or
git checkout -b fix/kalman-filter-drift
# or
git checkout -b docs/add-deployment-guide
```

Branch naming conventions:
- `feature/` - New features
- `fix/` - Bug fixes
- `docs/` - Documentation
- `test/` - Test additions/fixes
- `refactor/` - Code refactoring
- `perf/` - Performance improvements

### 2. Make Your Changes

- Write clean, readable code
- Follow style guidelines (see below)
- Add tests for new features
- Update documentation
- Keep commits atomic and well-described

**Good Commit Messages**:
```
Add FLIR Lepton 3.5 driver integration

- Implement SPI communication protocol
- Add radiometric temperature conversion
- Include auto-calibration on startup
- Update hardware tests with Lepton support

Closes #42
```

### 3. Test Your Changes

```bash
# Run all tests
pytest tests/ -v

# Run specific test file
pytest tests/test_yolo.py -v

# Run with coverage
pytest tests/ --cov=src/fire_edge --cov-report=html

# Test ROS nodes
rostest fire_edge test_nodes.test

# Lint your code
flake8 src/ --max-line-length=100
black src/ --check
```

### 4. Submit Pull Request

1. Push to your fork: `git push origin feature/your-branch-name`
2. Go to GitHub and create Pull Request
3. Fill out the PR template completely
4. Link related issues (e.g., "Closes #42")
5. Request review from maintainers

**Good PR Descriptions**:
```markdown
## Description
Adds FLIR Lepton 3.5 thermal camera driver with SPI interface.

## Motivation
Current system uses simulated thermal imaging. Real Lepton integration
enables actual fire detection in smoke-filled environments.

## Changes
- Added `src/fire_edge/drivers/flir_lepton.py`
- Updated `thermal_processor.py` to use real camera
- Added unit tests in `tests/test_flir_lepton.py`
- Updated documentation in `docs/HARDWARE.md`

## Testing
- [x] Unit tests pass (`pytest tests/test_flir_lepton.py`)
- [x] Integration test with YOLOv8
- [x] Tested on Jetson Xavier NX with actual Lepton 3.5
- [x] Temperature readings verified with IR thermometer

## Screenshots
[Attach thermal image captures]

## Checklist
- [x] Code follows style guidelines
- [x] Tests added/updated
- [x] Documentation updated
- [x] No breaking changes (or documented if unavoidable)
- [x] Commits are clean and descriptive

## Related Issues
Closes #42
Related to #38 (thermal calibration)
```

### 5. Code Review Process

- Maintainers will review within 3-5 days
- Address feedback promptly
- Be open to suggestions
- Update PR based on review
- Once approved, maintainers will merge

---

## Style Guidelines

### Python Code Style

We follow **PEP 8** with some modifications:

```python
# Line length: 100 characters (not 79)
# Use Black formatter for automatic formatting

# Good variable names (descriptive)
thermal_image = camera.capture()
kalman_state_vector = np.zeros(5)
fire_detection_confidence = 0.85

# Bad variable names (ambiguous)
img = cam.get()
x = np.zeros(5)
c = 0.85

# Type hints (encouraged for new code)
def detect_fire(thermal_frame: np.ndarray, 
                threshold: float = 0.5) -> List[Detection]:
    """
    Detect fire objects in thermal image.
    
    Args:
        thermal_frame: Thermal camera frame (160x120)
        threshold: Confidence threshold (0.0-1.0)
    
    Returns:
        List of Detection objects with bounding boxes
    """
    pass

# Constants (UPPER_CASE)
THERMAL_WIDTH = 160
THERMAL_HEIGHT = 120
FIRE_DETECTION_THRESHOLD = 0.85

# Classes (PascalCase)
class ThermalProcessor:
    pass

# Functions/methods (snake_case)
def process_thermal_frame():
    pass
```

### ROS Node Style

```python
#!/usr/bin/env python3
"""
Thermal Processor Node

Processes thermal camera feed using YOLOv8 for fire detection.
"""

import rospy
from sensor_msgs.msg import Image
from fire_edge.msg import Detection

class ThermalProcessorNode:
    def __init__(self):
        rospy.init_node('thermal_processor')
        
        # Parameters
        self.model_path = rospy.get_param('~model_path', 'models/yolov8n.pt')
        self.confidence_threshold = rospy.get_param('~threshold', 0.5)
        
        # Publishers
        self.detection_pub = rospy.Publisher(
            '/fire_edge/detections', 
            Detection, 
            queue_size=10
        )
        
        # Subscribers
        self.thermal_sub = rospy.Subscriber(
            '/thermal/image_raw',
            Image,
            self.thermal_callback
        )
        
        rospy.loginfo("Thermal processor initialized")
    
    def thermal_callback(self, msg):
        """Process incoming thermal images."""
        # Implementation
        pass

if __name__ == '__main__':
    try:
        node = ThermalProcessorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

### Documentation Style

```python
def kalman_filter_update(state: np.ndarray,
                        measurement: np.ndarray,
                        Q: np.ndarray,
                        R: np.ndarray) -> np.ndarray:
    """
    Update Kalman filter with new measurement.
    
    Implements standard Kalman filter equations:
    - Prediction: x_k = F * x_{k-1}
    - Update: x_k = x_k + K * (z_k - H * x_k)
    
    Args:
        state: Current state vector [x, y, vx, vy, theta]
        measurement: Sensor measurement [x, y, theta]
        Q: Process noise covariance matrix (5x5)
        R: Measurement noise covariance matrix (3x3)
    
    Returns:
        Updated state vector after incorporating measurement
    
    Raises:
        ValueError: If matrix dimensions don't match
    
    Example:
        >>> state = np.array([0, 0, 0, 0, 0])
        >>> measurement = np.array([1.0, 0.5, 0.1])
        >>> Q = np.diag([0.1, 0.1, 0.05, 0.05, 0.01])
        >>> R = np.diag([0.5, 0.5, 0.1])
        >>> new_state = kalman_filter_update(state, measurement, Q, R)
    """
    # Implementation
    pass
```

### Git Commit Style

```bash
# Format: <type>(<scope>): <subject>

# Types:
# feat: New feature
# fix: Bug fix
# docs: Documentation
# style: Formatting (no code change)
# refactor: Code restructure
# test: Adding tests
# chore: Maintenance

# Examples:
git commit -m "feat(thermal): add FLIR Lepton driver"
git commit -m "fix(kalman): correct covariance matrix update"
git commit -m "docs(readme): add installation instructions for Jetson"
git commit -m "test(yolo): add unit tests for fire detection"
```

---

## Testing Requirements

### Test Coverage

All new features should include tests:
- **Unit tests**: Test individual functions/classes
- **Integration tests**: Test component interactions
- **ROS tests**: Test node communication
- **Hardware tests**: Test with actual sensors (if applicable)

### Running Tests

```bash
# All tests
pytest tests/ -v

# Specific module
pytest tests/test_kalman.py -v

# With coverage
pytest tests/ --cov=src/fire_edge --cov-report=html

# ROS integration tests
rostest fire_edge test_nodes.test

# Hardware tests (requires actual hardware)
python tests/test_sensors.py --hardware
```

### Writing Tests

```python
# tests/test_thermal_processor.py
import pytest
import numpy as np
from fire_edge.thermal_processor import ThermalProcessor

class TestThermalProcessor:
    @pytest.fixture
    def processor(self):
        """Create processor instance for testing."""
        return ThermalProcessor(model_path='models/yolov8n.pt')
    
    def test_initialization(self, processor):
        """Test processor initializes correctly."""
        assert processor.model is not None
        assert processor.confidence_threshold == 0.5
    
    def test_fire_detection(self, processor):
        """Test fire detection on sample thermal image."""
        # Create sample thermal image
        thermal_frame = np.random.randint(0, 255, (120, 160), dtype=np.uint8)
        
        # Run detection
        detections = processor.detect(thermal_frame)
        
        # Verify output format
        assert isinstance(detections, list)
        for det in detections:
            assert 'bbox' in det
            assert 'confidence' in det
            assert 'class' in det
    
    def test_invalid_input(self, processor):
        """Test handling of invalid input."""
        with pytest.raises(ValueError):
            processor.detect(None)
```

---

## Areas We Need Help

### 🔥 High Priority

1. **Real Hardware Integration**
   - FLIR Lepton 3.5 driver development
   - LiDAR data processing optimization
   - IMU calibration procedures
   - Ultrasonic sensor array management

2. **Performance Optimization**
   - YOLOv8 TensorRT optimization
   - Kalman filter CUDA acceleration
   - Memory usage reduction
   - Battery life improvements

3. **Safety & Testing**
   - Real fire department field testing
   - Safety certification preparation
   - Stress testing under extreme conditions
   - Failure mode analysis

### 🌐 Medium Priority

4. **Multi-Agent Systems**
   - Mesh networking for team coordination
   - Position sharing between firefighters
   - Incident commander dashboard
   - Team heatmap visualization

5. **User Interface**
   - Voice command system (hands-free)
   - Intuitive menu navigation
   - Audio feedback for alerts
   - Accessibility features

6. **Advanced Features**
   - Smoke density estimation
   - Heat map generation
   - Structural integrity warnings
   - Evacuation route optimization

### 📚 Community Contributions

7. **Documentation**
   - Video tutorials
   - Arabic translations (for Egyptian fire departments)
   - Deployment guides for different hardware
   - Troubleshooting FAQs

8. **Education & Outreach**
   - University course materials
   - Workshop presentations
   - Demo videos
   - Blog posts on Medium/dev.to

### 🔬 Research Collaboration

9. **Academic Partnerships**
   - Joint research papers
   - Dataset collection
   - Algorithm improvements
   - Real-world validation studies

10. **Open Problems**
    - Better smoke penetration algorithms
    - Lower-cost thermal camera alternatives
    - Extended battery life solutions
    - Ruggedization for extreme heat

---

## Questions?

- 📧 **Email**: [adhamt864@gmail.com](mailto:adhamt864@gmail.com)

---

**Thank you for contributing to firefighter safety! Every improvement brings us closer to saving lives.** 🔥🚒

*FIRE-EDGE Team*  
*New Ismailia National University*
