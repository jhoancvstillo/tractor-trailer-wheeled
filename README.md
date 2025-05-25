# Robot Navigation Simulation

This project simulates a robot navigating through waypoints using direct GPS readings from CoppeliaSim.

## Project Structure

The code has been reorganized into a modular structure:

- `main.py`: Main program entry point
- `simulation.py`: CoppeliaSim connection and simulation management
- `sensors.py`: GPS sensor for robot position and orientation
- `controllers.py`: PID controller implementation
- `robot_controller.py`: Robot control logic
- `utils.py`: Utility functions for distance and angle calculations
- `visualization.py`: Trajectory visualization and plotting

## How to Run

To run the simulation:

```bash
python main.py
```

## Requirements

- CoppeliaSim (V-REP)
- Python 3.x
- numpy
- matplotlib
- coppeliasim_zmqremoteapi_client

## File Descriptions

- `PioneerG1T.ttt`: CoppeliaSim scene file with the robot
- `basico.ttt`: Basic CoppeliaSim scene
- `Trailer_Cin.py`: Separate trailer kinematics simulation (not used in main simulation)

## Implementation Details

The implementation uses PID controllers to navigate the robot through a series of waypoints. The robot uses:

1. A distance-based PID controller to regulate forward velocity
2. An angle-based PID controller to adjust orientation toward the target

The GPS reading comes directly from CoppeliaSim's position reporting, and the simulation visualizes the path taken.
