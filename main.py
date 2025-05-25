import time
from simulation import SimulationEnvironment
from sensors import GPSSensor
from robot_controller import RobotController
from visualization import TrajectoryVisualizer

def main():
    """Main function to run the robot simulation."""
    print('Program started')
    
    # Initialize simulation
    sim_env = SimulationEnvironment()
    
    # Get handles for all required objects
    handles = sim_env.get_handles({
        'left_motor': '/PioneerP3DX/leftMotor',
        'right_motor': '/PioneerP3DX/rightMotor',
        'robot': '/PioneerP3DX',
        'gps': '/PioneerP3DX/GPS',
        # 'trailer': '/PioneerP3DX/Revolute_joint/Cuboid/Revolute_joint/PioneerP3DX2',
        # 'trailer_left_motor': '/PioneerP3DX/Revolute_joint/Cuboid/Revolute_joint/PioneerP3DX2/leftMotor2',
        # 'trailer_right_motor': '/PioneerP3DX/Revolute_joint/Cuboid/Revolute_joint/PioneerP3DX2/rightMotor2',
        # 'hitch_joint': '/PioneerP3DX/Revolute_joint',
        # 'connection_joint': '/PioneerP3DX/Revolute_joint/Cuboid/Revolute_joint'
    })
    
    # Print handles for debugging
    for name, handle in handles.items():
        print(f"Handle {name}: {handle}")
    
    # Initialize GPS sensor
    gps = GPSSensor(
        sim_env.sim,
        handles['gps'],
        handles['robot'],
        # handles['trailer'],
    )
    
    # Initialize robot controller
    robot = RobotController(sim_env, gps)
    
    # Initialize visualizer
    visualizer = TrajectoryVisualizer()
    
    # Define waypoints [x, y, z]
    waypoints = [
        [5, 0.0, 0.1],    # First waypoint
        [1.0, 5, 0.1],    # Second waypoint
        [0.0, 5.0, 0.1],  # Third waypoint
        [0.0, 0.0, 0.1]   # Return to origin
    ]
    
    print("Starting GPS navigation with the following waypoints:")
    for i, wp in enumerate(waypoints):
        print(f"Waypoint {i+1}: {wp}")
    
    # Start simulation
    sim_env.start()
    
    try:
        # Get initial time
        start_time = sim_env.get_simulation_time()
        robot.set_start_time(start_time)
        
        # Navigate to each waypoint
        for waypoint_idx, target_position in enumerate(waypoints):
            print(f"\nNavigating to waypoint {waypoint_idx+1}: {target_position}")
            
            # Reset controllers for new waypoint
            robot.reset_controllers()
            
            # Control loop for current waypoint
            while True:
                # Control timing for consistent dt
                current_time = sim_env.get_simulation_time()
                if current_time - robot.prev_time < 0.05:
                    time.sleep(0.001)
                    continue
                
                # Navigate to waypoint
                waypoint_reached = robot.navigate_to_waypoint(
                    target_position, visualizer
                )
                
                # If waypoint reached, go to next one
                if waypoint_reached:
                    elapsed_time = current_time - start_time
                    visualizer.record_waypoint_reached(elapsed_time)
                    print(f"Waypoint {waypoint_idx+1} reached!")
                    break
        
        print("\nRoute complete! Robot has returned to starting position.")
            
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        # Stop motors
        if 'left_motor' in handles and 'right_motor' in handles:
            sim_env.set_joint_velocity(handles['left_motor'], 0)
            sim_env.set_joint_velocity(handles['right_motor'], 0)
        
        # Stop simulation
        sim_env.stop()
        
        # Plot results
        visualizer.plot_results(waypoints)
    
    print('Program finished')

if __name__ == "__main__":
    main() 