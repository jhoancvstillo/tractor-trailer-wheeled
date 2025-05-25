from controllers import PIDController
from utils import distance, angle_to_target

class RobotController:
    """Class to control a robot with a trailer."""
    
    def __init__(self, simulation, sensors):
        """Initialize robot controller.
        
        Args:
            simulation: SimulationEnvironment instance
            sensors: GPSSensor instance
        """
        self.kp = 10
        self.ki = 0.1
        self.kd = 0
        self.sim = simulation
        self.sensors = sensors
        
        # PID controllers
        self.pid_distance = PIDController(self.kp, self.ki, self.kd)  # PID for distance
        self.pid_angle = PIDController(self.kp, self.ki, self.kd)     # PID for angle
        
        # Maximum acceptable error
        self.max_error = 0.1
        
    def reset_controllers(self):
        """Reset all PID controllers."""
        self.pid_distance.reset()
        self.pid_angle.reset()
        
    def navigate_to_waypoint(self, target_position, visualizer=None):
        """Navigate robot to a target waypoint.
        
        Args:
            target_position: Target position [x, y, z]
            visualizer: Optional TrajectoryVisualizer
            
        Returns:
            bool: True if waypoint reached, False otherwise
        """
        sim = self.sim.sim
        handles = self.sim.handles
        
        # Get current positions and orientations
        gps_position = self.sensors.get_position()
        real_position = self.sensors.get_real_position()  # Same as gps_position now
        trailer_position = self.sensors.get_trailer_position()  # May be [0,0,0] if no trailer
        current_orientation = self.sensors.get_orientation()
        
        # Calculate distance error using GPS position
        dist_error = distance(gps_position, target_position)
        
        # If we've reached the waypoint
        if dist_error < self.max_error:
            # Stop the motors
            if 'left_motor' in handles and 'right_motor' in handles:
                self.sim.set_joint_velocity(handles['left_motor'], 0)
                self.sim.set_joint_velocity(handles['right_motor'], 0)
            return True
        
        # Calculate angle error using GPS position
        angle_error = angle_to_target(gps_position, target_position, current_orientation)
        
        # Get time delta
        current_time = sim.getSimulationTime()
        delta_time = 0.05  # Default value if first iteration
        if hasattr(self, 'prev_time'):
            delta_time = current_time - self.prev_time
        self.prev_time = current_time
        
        # Calculate control signals
        velocity_signal = self.pid_distance.compute(dist_error, delta_time)
        angular_signal = self.pid_angle.compute(angle_error, delta_time)
        
        # Limit maximum velocity
        velocity_signal = min(velocity_signal, 10)
        
        # Convert control signals to motor velocities
        left_velocity = velocity_signal - angular_signal
        right_velocity = velocity_signal + angular_signal
        
        # Apply velocities to tractor motors
        if 'left_motor' in handles and 'right_motor' in handles:
            self.sim.set_joint_velocity(handles['left_motor'], left_velocity)
            self.sim.set_joint_velocity(handles['right_motor'], right_velocity)
        
        # Print debug information
        print(f"GPS: {[round(p, 2) for p in gps_position]}, "
              f"Distance: {round(dist_error, 2)}m, "
              f"Vel: L={round(left_velocity, 2)}, R={round(right_velocity, 2)}")
        
        # Record data for visualization if provided
        if visualizer:
            elapsed_time = current_time - self.start_time
            visualizer.record_data(elapsed_time, real_position, gps_position, trailer_position)
        
        return False
        
    def set_start_time(self, start_time):
        """Set the simulation start time.
        
        Args:
            start_time: Simulation start time
        """
        self.start_time = start_time
        self.prev_time = start_time 