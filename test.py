from DroneSimulation import DroneSimulation, DroneSimConfig
config = DroneSimConfig(
        drone_view_angle=-55.0,
        drone_height=60.0,
        drone_speed=1.0,
        num_moving_cars=30,
        num_parked_cars=20,
        traffic_lights={"all": (5.0, 12.0, 2.0)},
        carla_host="localhost",
        carla_port=4000,
        video_quality="good",  # Choose from: "low", "medium", "good", "very good"
        display_fps=False,      # Set to True to show FPS overlay
        time_of_day="day",  # "day" or "night"
        weather_mode="rainy"  # "clear" or "rainy"
    )
sim = DroneSimulation(config)
sim.run()