import random
import time
import carla
import numpy as np
import cv2
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple, Any

@dataclass
class DroneSimConfig:
    drone_view_angle: float = -60.0  # Camera pitch angle in degrees
    drone_height: float = 50.0       # Drone flight height (z)
    drone_speed: float = 0.5         # Drone speed (m/s)
    num_moving_cars: int = 24        # Number of moving cars
    num_parked_cars: int = 20        # Number of parked cars
    traffic_lights: Dict[Any, Tuple[float, float, float]] = field(default_factory=lambda: {"all": (4.0, 14.0, 1.0)})
    carla_host: str = "localhost"
    carla_port: int = 4000           # Default CARLA port
    video_quality: str = "medium"   # Options: "low", "medium", "good", "very good"
    display_fps: bool = False        # Show FPS overlay in display
    time_of_day: str = "day"        # Options: "day", "night"
    weather_mode: str = "clear"     # Options: "clear", "rainy"

class DroneSimulation:
    def __init__(self, config: Optional[DroneSimConfig] = None):
        self.config = config or DroneSimConfig()
        self.client = None
        self.world = None
        self.blueprint_library = None
        self.parked_vehicles = []
        self.moving_vehicles = []
        self.rgb_camera = None
        # Set resolution based on video_quality
        self.quality_map = {
            "low": (640, 360),        # 360p
            "medium": (640, 480),     # 480p
            "good": (1280, 720),      # 720p
            "very good": (1920, 1080) # 1080p
        }
        self.width, self.height = self.quality_map.get(self.config.video_quality, (640, 480))
        self.rgb_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        # Drone path endpoints (x-axis)
        self.start_x = -121.69
        self.end_x = 108.02
        self.fixed_y = 20.15
        self.fixed_z = self.config.drone_height
        # Real-time attributes
        self.current_speed = 0.0
        self.current_road_id = None
        self._setup_simulation()

    def get_parking_lane_waypoints(self):
        parking_waypoints = []
        carla_map = self.world.get_map()
        for waypoint in carla_map.generate_waypoints(2.0):
            if waypoint.lane_type == carla.LaneType.Parking:
                parking_waypoints.append(waypoint)
        return parking_waypoints

    def _spawn_rgb_camera(self):
        # Remove previous camera if exists
        if self.rgb_camera:
            self.rgb_camera.destroy()
            self.rgb_camera = None

        width, height = self.width, self.height
        rgb_cam_bp = self.blueprint_library.find('sensor.camera.rgb')
        rgb_cam_bp.set_attribute('image_size_x', str(width))
        rgb_cam_bp.set_attribute('image_size_y', str(height))
        rgb_cam_bp.set_attribute('fov', '90')

        camera_location = carla.Location(x=self.current_x, y=self.current_y, z=self.current_z)
        camera_rotation = carla.Rotation(pitch=self.yaw_angle, yaw=self.current_yaw)
        camera_transform = carla.Transform(camera_location, camera_rotation)

        def rgb_callback(image):
            arr = np.frombuffer(image.raw_data, dtype=np.uint8)
            arr = arr.reshape((height, width, 4))
            self.rgb_frame = arr[:, :, :3][:, :, ::-1]  # Convert BGRA to RGB

        self.rgb_camera = self.world.spawn_actor(rgb_cam_bp, camera_transform)
        self.rgb_camera.listen(rgb_callback)

    def _setup_simulation(self):
        print(f"[DroneSimulation] Connecting to CARLA at {self.config.carla_host}:{self.config.carla_port}...")
        self.client = carla.Client(self.config.carla_host, self.config.carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()

        # Set weather and time of day
        weather = self.world.get_weather()
        # Set time of day
        if self.config.time_of_day == "night":
            weather.sun_altitude_angle = -10.0  # Below horizon for night
        else:
            weather.sun_altitude_angle = 70.0   # High for day
        # Set weather mode
        if self.config.weather_mode == "rainy":
            print("[DroneSimulation] Setting weather to rainy mode.")
            weather.precipitation = 100.0
            weather.precipitation_deposits = 100.0
            weather.cloudiness = 50.0
            weather.fog_density = 15.0
            weather.wetness = 50.0
        else:
            weather.precipitation = 0.0
            weather.precipitation_deposits = 0.0
            weather.cloudiness = 10.0
            weather.fog_density = 0.0
            weather.wetness = 0.0
        self.world.set_weather(weather)

        # Clean up existing vehicles and sensors
        print("[DroneSimulation] Cleaning up existing actors...")
        for actor in self.world.get_actors():
            if actor.type_id.startswith(('vehicle.', 'sensor.')):
                actor.destroy()

        # Spawn parked cars in parking lanes
        print(f"[DroneSimulation] Spawning {self.config.num_parked_cars} parked cars...")
        parking_waypoints = self.get_parking_lane_waypoints()
        random.shuffle(parking_waypoints)
        car_blueprints = [bp for bp in self.blueprint_library.filter('vehicle.*')
                          if bp.has_attribute('number_of_wheels') and int(bp.get_attribute('number_of_wheels')) == 4]
        for i in range(min(self.config.num_parked_cars, len(parking_waypoints))):
            wp = parking_waypoints[i]
            bp = random.choice(car_blueprints)
            vehicle = self.world.try_spawn_actor(bp, wp.transform)
            if vehicle:
                self.parked_vehicles.append(vehicle)

        # Spawn moving cars (use general spawn points)
        print(f"[DroneSimulation] Spawning {self.config.num_moving_cars} moving cars...")
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        moving_points = spawn_points[:self.config.num_moving_cars]
        for sp in moving_points:
            bp = random.choice(car_blueprints)
            vehicle = self.world.try_spawn_actor(bp, sp)
            if vehicle:
                vehicle.set_autopilot(True)
                self.moving_vehicles.append(vehicle)

        # Set traffic lights
        print("[DroneSimulation] Configuring traffic lights...")
        traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
        for tl in traffic_lights:
            if "all" in self.config.traffic_lights:
                red, green, yellow = self.config.traffic_lights["all"]
            elif tl.id in self.config.traffic_lights:
                red, green, yellow = self.config.traffic_lights[tl.id]
            else:
                continue
            tl.set_red_time(red)
            tl.set_green_time(green)
            tl.set_yellow_time(yellow)

        # Set up drone (camera) initial position and direction
        self.current_x = self.start_x
        self.current_y = self.fixed_y
        self.current_z = self.fixed_z
        self.direction = 1
        self.yaw_angle = self.config.drone_view_angle
        self.current_yaw = 0.0  # Will be set in run
        print(f"[DroneSimulation] Video quality set to '{self.config.video_quality}' ({self.width}x{self.height})")
        print("[DroneSimulation] Setup complete.")
        self._spawn_rgb_camera()

    def run(self):
        print("[DroneSimulation] Starting simulation loop. Press Ctrl+C or 'q' in the window to stop.")
        prev_time = time.time()
        fps = 0.0
        prev_x, prev_y, prev_z = self.current_x, self.current_y, self.current_z
        prev_fps_time = time.time()
        try:
            # Set initial yaw for forward direction
            self.current_yaw = 0.0
            while True:
                # Move drone (camera) along a path and reverse at endpoints
                self.current_x += self.config.drone_speed * self.direction
                if self.direction == 1 and self.current_x >= self.end_x:
                    self.current_x = self.end_x
                    self.direction = -1
                    self.current_yaw = 180.0  # Look backward
                elif self.direction == -1 and self.current_x <= self.start_x:
                    self.current_x = self.start_x
                    self.direction = 1
                    self.current_yaw = 0.0    # Look forward
                # (You can add more complex movement logic here)
                print(f"Drone position: x={self.current_x:.2f}, y={self.current_y:.2f}, z={self.current_z:.2f}, yaw={self.yaw_angle}, cam_yaw={self.current_yaw}")

                # Update camera transform to follow the drone and direction
                if self.rgb_camera:
                    camera_location = carla.Location(x=self.current_x, y=self.current_y, z=self.current_z)
                    camera_rotation = carla.Rotation(pitch=self.yaw_angle, yaw=self.current_yaw)
                    camera_transform = carla.Transform(camera_location, camera_rotation)
                    self.rgb_camera.set_transform(camera_transform)

                # --- Update real-time speed ---
                current_time = time.time()
                distance = ((self.current_x - prev_x) ** 2 + (self.current_y - prev_y) ** 2 + (self.current_z - prev_z) ** 2) ** 0.5
                self.current_speed = distance / (current_time - prev_time) if (current_time - prev_time) > 0 else 0.0
                prev_x, prev_y, prev_z = self.current_x, self.current_y, self.current_z
                prev_time = current_time

                # --- Update real-time road id ---
                waypoint = self.world.get_map().get_waypoint(
                    carla.Location(x=self.current_x, y=self.current_y, z=self.current_z),
                    project_to_road=True, lane_type=carla.LaneType.Any
                )
                self.current_road_id = waypoint.road_id if waypoint else None

                # Display the camera image
                if hasattr(self, 'rgb_frame') and self.rgb_frame is not None:
                    self.rgb_frame = cv2.cvtColor(self.rgb_frame,cv2.COLOR_BGR2RGB)
                    display_frame = self.rgb_frame.copy()
                    if self.config.display_fps:
                        current_fps_time = time.time()
                        fps = 1.0 / (current_fps_time - prev_fps_time) if (current_fps_time - prev_fps_time) > 0 else 0.0
                        prev_fps_time = current_fps_time
                        cv2.putText(display_frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                    cv2.imshow("Drone RGB Camera", display_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("[DroneSimulation] Simulation interrupted by user.")
        finally:
            self.stop()

    def stop(self):
        print("[DroneSimulation] Cleaning up actors and sensors...")
        if self.rgb_camera:
            self.rgb_camera.destroy()
        for v in self.parked_vehicles + self.moving_vehicles:
            if v.is_alive:
                v.destroy()
        cv2.destroyAllWindows()
        print("[DroneSimulation] Cleanup complete.")

    def reset(self, new_config: Optional[DroneSimConfig] = None):
        print("[DroneSimulation] Resetting simulation.")
        self.stop()
        if new_config:
            self.config = new_config
            self.width, self.height = self.quality_map.get(self.config.video_quality, (640, 480))
            self.rgb_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self._setup_simulation()

if __name__ == "__main__":
    config = DroneSimConfig(
        drone_view_angle=-55.0,
        drone_height=60.0,
        drone_speed=1.0,
        num_moving_cars=10,
        num_parked_cars=5,
        traffic_lights={"all": (5.0, 12.0, 2.0)},
        carla_host="localhost",
        carla_port=4000,
        video_quality="good",  # Choose from: "low", "medium", "good", "very good"
        display_fps=True,      # Set to True to show FPS overlay
        time_of_day="day",  # "day" or "night"
        weather_mode="clear"  # "clear" or "rainy"
    )
    sim = DroneSimulation(config)
    sim.run()

