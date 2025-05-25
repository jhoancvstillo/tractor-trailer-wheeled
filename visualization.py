import matplotlib.pyplot as plt
import numpy as np

class TrajectoryVisualizer:
    """Class for visualizing robot trajectories."""
    
    def __init__(self):
        """Initialize trajectory visualizer."""
        # Data storage
        self.time_points = []
        self.real_trajectory_x = []
        self.real_trajectory_y = []
        self.gps_trajectory_x = []
        self.gps_trajectory_y = []
        self.trailer_trajectory_x = []
        self.trailer_trajectory_y = []
        self.waypoint_times = [0]  # Time stamps for waypoint changes
        self.has_trailer_data = False
        
    def record_data(self, elapsed_time, real_position, gps_position, trailer_position):
        """Record position data at current timestamp.
        
        Args:
            elapsed_time: Current simulation time
            real_position: Real robot position [x, y, z]
            gps_position: GPS position [x, y, z]
            trailer_position: Trailer position [x, y, z] or [0,0,0] if no trailer
        """
        self.time_points.append(elapsed_time)
        self.real_trajectory_x.append(real_position[0])
        self.real_trajectory_y.append(real_position[1])
        self.gps_trajectory_x.append(gps_position[0])
        self.gps_trajectory_y.append(gps_position[1])
        
        # Check if we have valid trailer data
        if trailer_position[0] != 0 or trailer_position[1] != 0:
            self.has_trailer_data = True
            self.trailer_trajectory_x.append(trailer_position[0])
            self.trailer_trajectory_y.append(trailer_position[1])
        
    def record_waypoint_reached(self, elapsed_time):
        """Record time when a waypoint is reached.
        
        Args:
            elapsed_time: Current simulation time
        """
        self.waypoint_times.append(elapsed_time)
        
    def plot_results(self, waypoints, filename='trayectoria_robot.png'):
        """Plot trajectory results and save to file.
        
        Args:
            waypoints: List of waypoints [[x, y, z], ...]
            filename: Output file name
        """
        plt.figure(figsize=(15, 12))
        
        # Create array of waypoints for plotting
        waypoints_x = [wp[0] for wp in waypoints] + [waypoints[0][0]]  # Add first at end to close polygon
        waypoints_y = [wp[1] for wp in waypoints] + [waypoints[0][1]]
        
        # Plot reference (waypoints) and trajectories
        plt.subplot(2, 1, 1)
        plt.plot(waypoints_x, waypoints_y, 'r--', linewidth=2, label='Reference (Waypoints)')
        plt.plot(waypoints_x, waypoints_y, 'ro', markersize=8)
        
        # Plot trajectories
        plt.plot(self.real_trajectory_x, self.real_trajectory_y, 'b-', linewidth=1.5, label='Robot Trajectory')
        
        # Plot GPS trajectory if different from real
        if self.real_trajectory_x != self.gps_trajectory_x or self.real_trajectory_y != self.gps_trajectory_y:
            plt.plot(self.gps_trajectory_x, self.gps_trajectory_y, 'g:', linewidth=1, label='GPS readings')
            
        # Plot trailer trajectory if available
        if self.has_trailer_data:
            plt.plot(self.trailer_trajectory_x, self.trailer_trajectory_y, 'm-', linewidth=1.5, label='Trailer Trajectory')
        
        # Mark starting position
        plt.plot(self.real_trajectory_x[0], self.real_trajectory_y[0], 'ko', markersize=10, label='Start')
        
        # Add waypoint numbers
        for i, wp in enumerate(waypoints):
            plt.text(wp[0], wp[1], f' WP{i+1}', fontsize=12)
        
        plt.title('Robot Trajectory')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        
        # Plot X and Y positions vs time
        plt.subplot(2, 1, 2)
        
        # Plot position X vs time
        plt.plot(self.time_points, self.real_trajectory_x, 'b-', label='X Robot')
        plt.plot(self.time_points, self.real_trajectory_y, 'r-', label='Y Robot')
        
        # Plot trailer position if available
        if self.has_trailer_data:
            plt.plot(self.time_points[:len(self.trailer_trajectory_x)], self.trailer_trajectory_x, 'g-', label='X Trailer')
            plt.plot(self.time_points[:len(self.trailer_trajectory_y)], self.trailer_trajectory_y, 'm-', label='Y Trailer')
        
        # Mark waypoint changes
        for i, wt in enumerate(self.waypoint_times):
            if i > 0:  # Don't mark initial time
                plt.axvline(x=wt, color='k', linestyle='--', alpha=0.5)
                plt.text(wt, max(max(self.real_trajectory_x), max(self.real_trajectory_y)), 
                         f' WP{i}', fontsize=10)
        
        # Plot waypoint references with thickness 2
        for i in range(len(waypoints)):
            plt.plot([self.waypoint_times[i], self.waypoint_times[min(i+1, len(self.waypoint_times)-1)]], 
                     [waypoints[i][0], waypoints[i][0]], 'b--', alpha=0.5, linewidth=2)
            plt.plot([self.waypoint_times[i], self.waypoint_times[min(i+1, len(self.waypoint_times)-1)]], 
                     [waypoints[i][1], waypoints[i][1]], 'r--', alpha=0.5, linewidth=2)
        
        plt.title('Position vs Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig(filename)
        plt.show() 