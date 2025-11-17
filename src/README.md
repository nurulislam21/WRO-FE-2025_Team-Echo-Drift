## Folder Structure
```
.
│
├── arduino
│   ├── ITG3200_gyro_test
│   │   └── ITG3200_gyro_test.ino
│   ├── platformio.ini
│   └── src
│       └── main.cpp
│
├── raspberrypi
│   ├── __init__.py
│   ├── adjust_pid_values.py
│   ├── camera_settings.json
│   ├── contour_workers.py
│   ├── img_processing_functions.py
│   ├── main.py
│   ├── odometry_log.py
│   ├── odometry_sample_generator.py
│   ├── odometry.py
│   ├── parking.py
│   ├── playground.py
│   ├── requirements.txt
│   └── test.py
├── requirements.txt
│
│
└── tools
    ├── adjust_camera_settings.py
    ├── camera_settings.json
    ├── car_driving.py
    ├── color_picker.py
    ├── color_ranges.json
    ├── contour_analysis.py
    ├── danger_zone_visualizer.py
    ├── masking_test_pi.py
    ├── masking_test.py
    └── record.py    
```