# Carla Based Urban Drone Simulation

# DroneSimulation

![alt Text](https://github.com/meysamraz/Carla-Based-Urban-Drone-Simulation/blob/master/src/demo_day.gif)
![alt Text](https://github.com/meysamraz/Carla-Based-Urban-Drone-Simulation/blob/master/src/demo_day_rainy.gif)
![alt Text](https://github.com/meysamraz/Carla-Based-Urban-Drone-Simulation/blob/master/src/demo_night.gif)



Simulate urban drone flights in  [CARLA Simulator](https://carla.org/) with ease—ideal for computer vision, urban analytics, and autonomous systems projects.

**Perfect for vision-based projects in urban environments:**
- Test and develop algorithms for object detection, tracking, and scene understanding
- Simulate smart city scenarios with realistic traffic, weather, and lighting
- Collect synthetic data for deep learning and AI models
- Experiment with drone navigation and perception in complex cityscapes


Easily configure drone camera, speed, number of cars, weather, and time of day. Visualize the drone's camera in real time with OpenCV.

---

## Features
- Modular Python class for drone simulation in CARLA
- Real-time RGB camera view with OpenCV
- User-configurable:
  - Drone speed, height, and camera angle
  - Number of moving and parked cars
  - Video quality (360p, 480p, 720p, 1080p)
  - Weather: clear or rainy
  - Time of day: day or night
  - Traffic light timing
  - FPS overlay (optional)
- Real-time access to drone speed and road ID
- **Simulated battery drain and speed reduction as battery drops**
- **Console output of battery and speed in real time**
- Clean, extensible codebase

---

## Installation

### 1. Install Python (3.8+ recommended)

Download from [python.org](https://www.python.org/downloads/).

### 2. Install CARLA Simulator
- Download CARLA from [carla.org/downloads](https://carla.org/)
- Extract and run the simulator:
  - On Windows: double-click `CarlaUE4.exe` or run from terminal:
    ```sh
    CarlaUE4.exe -carla-port=4000
    ```
  - On Linux:
    ```sh
    ./CarlaUE4.sh -carla-port=4000
    ```
- Wait for the CARLA window to fully load before running the Python script.

### 3. Install Python dependencies

Create a `requirements.txt` file with:

```
carla
opencv-python
numpy
pandas
```

Install with pip:
```sh
pip install -r requirements.txt
```

> **Note:**
> - The `carla` Python package must match your CARLA server version. If you have issues, see [CARLA Python API installation guide](https://carla.readthedocs.io/en/latest/python_api_tutorial/).

---

## Usage

1. **Edit the configuration in `DroneSimulation.py` (bottom of the file):**

```python
config = DroneSimConfig(
    drone_view_angle=-55.0,      # Camera pitch angle
    drone_height=60.0,           # Drone flight height
    drone_speed=1.0,             # Drone speed (m/s)
    num_moving_cars=10,          # Number of moving cars
    num_parked_cars=5,           # Number of parked cars
    traffic_lights={"all": (5.0, 12.0, 2.0)},
    carla_host="localhost",
    carla_port=4000,
    video_quality="good",       # "low", "medium", "good", "very good"
    display_fps=True,            # Show FPS overlay
    time_of_day="night",        # "day" or "night"
    weather_mode="rainy"        # "clear" or "rainy"
)
sim = DroneSimulation(config)
sim.run()
```

2. **Run the simulation for test:**

```sh
python test.py
```

3. **Control:**
- Press `q` in the OpenCV window or `Ctrl+C` in the terminal to stop.
- The console will show the drone's current position, battery percentage, and speed in real time.
- When the battery drops below 30%, the drone will slow down automatically.

---

## Example Console Output

```
Drone position: x=10.00, y=20.15, z=60.00, yaw=-55.0, cam_yaw=0.0 | Battery: 98.50% | Speed: 1.00 m/s
Drone position: x=..., ... | Battery: 28.00% | Speed: 0.93 m/s
```

---

## Configuration Options

| Parameter         | Type    | Description                                      | Example         |
|-------------------|---------|--------------------------------------------------|-----------------|
| drone_view_angle  | float   | Camera pitch angle (degrees)                     | -55.0           |
| drone_height      | float   | Drone flight height (z)                          | 60.0            |
| drone_speed       | float   | Drone speed (m/s)                                | 1.0             |
| num_moving_cars   | int     | Number of moving cars                            | 10              |
| num_parked_cars   | int     | Number of parked cars                            | 5               |
| traffic_lights    | dict    | Traffic light timing (all or by ID)              | {"all": (5,12,2)}|
| carla_host        | str     | CARLA server host                                | "localhost"     |
| carla_port        | int     | CARLA server port                                | 4000            |
| video_quality     | str     | Camera resolution: "low", "medium", "good", "very good" | "good"   |
| display_fps       | bool    | Show FPS overlay in OpenCV window                | True            |
| time_of_day       | str     | "day" or "night"                                 | "night"         |
| weather_mode      | str     | "clear" or "rainy"                               | "rainy"         |
| battery/speed     | auto    | Simulated battery drains over time, speed drops as battery gets low | (automatic)     |

---

## Example requirements.txt

```
carla
opencv-python
numpy
pandas
```

---

## Troubleshooting
- **CARLA connection errors:**
  - Make sure the CARLA server is running and fully loaded before starting the Python script.
  - Ensure the `carla_port` matches the port used to launch CARLA.
  - The Python `carla` package version must match the CARLA server version.
- **No OpenCV window:**
  - Make sure you have a display (not running headless).
- **Performance:**
  - Lower the video quality or number of cars for better performance.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

