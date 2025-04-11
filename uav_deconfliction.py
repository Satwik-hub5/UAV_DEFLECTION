import numpy as np
from scipy.spatial import distance
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from datetime import datetime, timedelta
import json
from typing import List, Dict, Tuple, Optional

class Waypoint:
    """Class to represent a single waypoint in a drone's mission"""
    def __init__(self, x: float, y: float, z: float = 0, timestamp: Optional[datetime] = None):
        self.x = x
        self.y = y
        self.z = z
        self.timestamp = timestamp

    def to_array(self) -> np.ndarray:
        """Convert waypoint to numpy array"""
        return np.array([self.x, self.y, self.z])

    def __repr__(self):
        return f"Waypoint(x={self.x}, y={self.y}, z={self.z}, time={self.timestamp})"

class DroneMission:
    """Class to represent a complete drone mission with waypoints"""
    def __init__(self, drone_id: str, waypoints: List[Waypoint]):
        self.drone_id = drone_id
        self.waypoints = waypoints
        self.trajectory = self._create_trajectory()
        
    def _create_trajectory(self) -> Dict[str, np.ndarray]:
        """Create interpolated trajectory function for the mission"""
        if len(self.waypoints) < 2:
            raise ValueError("At least 2 waypoints are needed to create a trajectory")
            
        # Extract times and positions
        times = np.array([wp.timestamp.timestamp() for wp in self.waypoints])
        positions = np.array([wp.to_array() for wp in self.waypoints])
        
        # Create interpolation functions for each dimension
        trajectory = {
            'x': interp1d(times, positions[:, 0], kind='linear', fill_value='extrapolate'),
            'y': interp1d(times, positions[:, 1], kind='linear', fill_value='extrapolate'),
            'z': interp1d(times, positions[:, 2], kind='linear', fill_value='extrapolate'),
            'start_time': min(times),
            'end_time': max(times)
        }
        return trajectory
    
    def get_position_at_time(self, timestamp: datetime) -> np.ndarray:
        """Get drone's position at a specific time"""
        time_float = timestamp.timestamp()
        if time_float < self.trajectory['start_time'] or time_float > self.trajectory['end_time']:
            return None
            
        x = self.trajectory['x'](time_float)
        y = self.trajectory['y'](time_float)
        z = self.trajectory['z'](time_float)
        return np.array([x, y, z])
        
    def __repr__(self):
        return f"DroneMission(drone_id={self.drone_id}, waypoints={len(self.waypoints)})"

class ConflictDetector:
    """Main class for detecting spatial and temporal conflicts"""
    def __init__(self, safety_buffer: float = 5.0):
        self.safety_buffer = safety_buffer
        self.drone_missions = {}
        
    def add_mission(self, mission: DroneMission):
        """Add a drone mission to the system"""
        self.drone_missions[mission.drone_id] = mission
        
    def check_for_conflicts(self, primary_mission: DroneMission) -> Tuple[bool, List[Dict]]:
        """Check for conflicts between primary mission and other drones"""
        conflicts = []
        
        # Get time samples for the primary mission
        start_time = primary_mission.waypoints[0].timestamp
        end_time = primary_mission.waypoints[-1].timestamp
        time_samples = self._generate_time_samples(start_time, end_time)
        
        for sample_time in time_samples:
            primary_pos = primary_mission.get_position_at_time(sample_time)
            if primary_pos is None:
                continue
                
            for drone_id, other_mission in self.drone_missions.items():
                if drone_id == primary_mission.drone_id:
                    continue
                    
                other_pos = other_mission.get_position_at_time(sample_time)
                if other_pos is None:
                    continue
                    
                # Calculate distance between drones
                dist = distance.euclidean(primary_pos, other_pos)
                if dist < self.safety_buffer:
                    conflicts.append({
                        'time': sample_time,
                        'primary_position': primary_pos,
                        'other_drone_id': drone_id,
                        'other_position': other_pos,
                        'distance': dist
                    })
                    
        return len(conflicts) > 0, conflicts
        
    def _generate_time_samples(self, start_time: datetime, end_time: datetime, sample_rate: float = 1.0) -> List[datetime]:
        """Generate time samples between start and end time"""
        samples = []
        current_time = start_time
        delta = timedelta(seconds=sample_rate)
        
        while current_time <= end_time:
            samples.append(current_time)
            current_time += delta
            
        return samples

class Visualization:
    """Class for visualizing drone missions and conflicts"""
    @staticmethod
    def plot_2d_trajectories(primary_mission: DroneMission, other_missions: List[DroneMission], conflicts: List[Dict] = None):
        """Create 2D plot of drone trajectories"""
        plt.figure(figsize=(10, 8))
        
        # Plot primary mission
        primary_x = [wp.x for wp in primary_mission.waypoints]
        primary_y = [wp.y for wp in primary_mission.waypoints]
        plt.plot(primary_x, primary_y, 'b-o', label=f'Primary: {primary_mission.drone_id}', linewidth=2)
        
        # Plot other missions
        for mission in other_missions:
            other_x = [wp.x for wp in mission.waypoints]
            other_y = [wp.y for wp in mission.waypoints]
            plt.plot(other_x, other_y, '--o', label=f'Other: {mission.drone_id}', alpha=0.7)
            
        # Plot conflicts if any
        if conflicts:
            conflict_x = [c['primary_position'][0] for c in conflicts]
            conflict_y = [c['primary_position'][1] for c in conflicts]
            plt.scatter(conflict_x, conflict_y, c='red', s=100, marker='x', label='Conflicts', zorder=10)
            
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Drone Trajectories and Conflicts')
        plt.legend()
        plt.grid(True)
        plt.savefig('2d_trajectories.png')  # Save as an image file
        plt.show()  # This ensures the plot is displayed
        
    @staticmethod
    def plot_3d_trajectories(primary_mission: DroneMission, other_missions: List[DroneMission], conflicts: List[Dict] = None):
        """Create 3D plot of drone trajectories (extra credit)"""
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot primary mission
        primary_x = [wp.x for wp in primary_mission.waypoints]
        primary_y = [wp.y for wp in primary_mission.waypoints]
        primary_z = [wp.z for wp in primary_mission.waypoints]
        ax.plot(primary_x, primary_y, primary_z, 'b-o', label=f'Primary: {primary_mission.drone_id}', linewidth=2)
        
        # Plot other missions
        for mission in other_missions:
            other_x = [wp.x for wp in mission.waypoints]
            other_y = [wp.y for wp in mission.waypoints]
            other_z = [wp.z for wp in mission.waypoints]
            ax.plot(other_x, other_y, other_z, '--o', label=f'Other: {mission.drone_id}', alpha=0.7)
            
        # Plot conflicts if any
        if conflicts:
            conflict_x = [c['primary_position'][0] for c in conflicts]
            conflict_y = [c['primary_position'][1] for c in conflicts]
            conflict_z = [c['primary_position'][2] for c in conflicts]
            ax.scatter(conflict_x, conflict_y, conflict_z, c='red', s=100, marker='x', label='Conflicts', zorder=10)
            
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.set_zlabel('Altitude (Z)')
        ax.set_title('3D Drone Trajectories and Conflicts')
        ax.legend()
        plt.savefig('3d_trajectories.png')  # Save as an image file
        plt.show()  # This ensures the plot is displayed
        
    @staticmethod
    def create_animation(primary_mission: DroneMission, other_missions: List[DroneMission], conflicts: List[Dict]):
        """Create animated visualization of drone movements"""
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Initialize lines and points
        primary_line, = ax.plot([], [], 'b-o', linewidth=2)
        other_lines = [ax.plot([], [], '--o', alpha=0.7)[0] for _ in other_missions]
        conflict_points = ax.scatter([], [], c='red', s=100, marker='x', zorder=10)
        
        # Set plot limits
        all_x = [wp.x for wp in primary_mission.waypoints]
        all_y = [wp.y for wp in primary_mission.waypoints]
        for mission in other_missions:
            all_x.extend([wp.x for wp in mission.waypoints])
            all_y.extend([wp.y for wp in mission.waypoints])
            
        padding = max(max(all_x) - min(all_x), max(all_y) - min(all_y)) * 0.2
        ax.set_xlim(min(all_x) - padding, max(all_x) + padding)
        ax.set_ylim(min(all_y) - padding, max(all_y) + padding)
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.set_title('Drone Movement Animation')
        ax.grid(True)
        
        # Create legend
        legend_items = [plt.Line2D([0], [0], color='b', marker='o', linestyle='-', label=f'Primary: {primary_mission.drone_id}')]
        for i, mission in enumerate(other_missions):
            legend_items.append(plt.Line2D([0], [0], marker='o', linestyle='--', label=f'Other: {mission.drone_id}'))
        legend_items.append(plt.Line2D([0], [0], color='red', marker='x', linestyle='None', label='Conflicts'))
        ax.legend(handles=legend_items)
        
        # Animation update function
        def update(frame):
            current_time = primary_mission.waypoints[0].timestamp + timedelta(seconds=frame)
            
            # Update primary drone position
            primary_pos = primary_mission.get_position_at_time(current_time)
            if primary_pos is not None:
                primary_line.set_data([primary_pos[0]], [primary_pos[1]])
            
            # Update other drones positions
            for i, mission in enumerate(other_missions):
                other_pos = mission.get_position_at_time(current_time)
                if other_pos is not None:
                    other_lines[i].set_data([other_pos[0]], [other_pos[1]])
            
            # Update conflicts
            frame_conflicts = [c for c in conflicts if abs((c['time'] - current_time).total_seconds()) < 1]
            if frame_conflicts:
                conflict_x = [c['primary_position'][0] for c in frame_conflicts]
                conflict_y = [c['primary_position'][1] for c in frame_conflicts]
                conflict_points.set_offsets(np.column_stack((conflict_x, conflict_y)))
            else:
                conflict_points.set_offsets(np.empty((0, 2)))
            
            return [primary_line] + other_lines + [conflict_points]
        
        # Create animation
        total_seconds = (primary_mission.waypoints[-1].timestamp - primary_mission.waypoints[0].timestamp).total_seconds()
        frames = int(total_seconds) + 1
        ani = animation.FuncAnimation(fig, update, frames=frames, interval=200, blit=True)
        
        plt.close()
        return ani

def load_sample_data() -> Tuple[DroneMission, List[DroneMission]]:
    """Create sample data for testing"""
    # Create timestamps
    base_time = datetime.now()
    
    # Primary mission
    primary_waypoints = [
        Waypoint(0, 0, 0, base_time + timedelta(seconds=0)),
        Waypoint(5, 10, 0, base_time + timedelta(seconds=10)),
        Waypoint(10, 10, 0, base_time + timedelta(seconds=20)),
        Waypoint(15, 5, 0, base_time + timedelta(seconds=30)),
        Waypoint(20, 0, 0, base_time + timedelta(seconds=40))
    ]
    primary_mission = DroneMission("Primary-1", primary_waypoints)
    
    # Other missions
    other_missions = []
    
    # Mission that will conflict
    conflict_waypoints = [
        Waypoint(10, 0, 0, base_time + timedelta(seconds=0)),
        Waypoint(10, 5, 0, base_time + timedelta(seconds=10)),
        Waypoint(10, 10, 0, base_time + timedelta(seconds=20)),
        Waypoint(10, 15, 0, base_time + timedelta(seconds=30)),
        Waypoint(10, 20, 0, base_time + timedelta(seconds=40))
    ]
    other_missions.append(DroneMission("Other-1", conflict_waypoints))
    
    # Mission that won't conflict
    safe_waypoints = [
        Waypoint(0, 20, 0, base_time + timedelta(seconds=0)),
        Waypoint(5, 15, 0, base_time + timedelta(seconds=10)),
        Waypoint(10, 10, 0, base_time + timedelta(seconds=25)),  # Different time at intersection
        Waypoint(15, 15, 0, base_time + timedelta(seconds=35)),
        Waypoint(20, 20, 0, base_time + timedelta(seconds=45))
    ]
    other_missions.append(DroneMission("Other-2", safe_waypoints))
    
    return primary_mission, other_missions

def main():
    """Main function to demonstrate the system"""
    # Load sample data
    primary_mission, other_missions = load_sample_data()
    
    # Initialize conflict detector
    detector = ConflictDetector(safety_buffer=3.0)
    for mission in other_missions:
        detector.add_mission(mission)
    
    # Check for conflicts
    has_conflict, conflicts = detector.check_for_conflicts(primary_mission)
    
    # Print results
    if has_conflict:
        print("🚨 CONFLICT DETECTED!")
        for i, conflict in enumerate(conflicts, 1):
            print(f"\nConflict #{i}:")
            print(f"Time: {conflict['time']}")
            print(f"Primary drone position: {conflict['primary_position']}")
            print(f"Conflicting drone: {conflict['other_drone_id']}")
            print(f"Conflicting position: {conflict['other_position']}")
            print(f"Distance between drones: {conflict['distance']:.2f} units")
    else:
        print("✅ No conflicts detected")
    
    # Visualize results
    print("\nVisualizing results...")
    Visualization.plot_2d_trajectories(primary_mission, other_missions, conflicts)
    Visualization.plot_3d_trajectories(primary_mission, other_missions, conflicts)
    
    # Create and save animation
    print("Creating animation...")
    ani = Visualization.create_animation(primary_mission, other_missions, conflicts)
    
    # Try different writers depending on what's available
    try:
        print("Trying to save with FFmpeg...")
        ani.save('drone_animation.mp4', writer='ffmpeg', fps=2)
        print("Animation saved as 'drone_animation.mp4'")
    except Exception as e:
        print(f"FFmpeg error: {e}")
        try:
            print("Trying to save as GIF instead...")
            ani.save('drone_animation.gif', writer='pillow', fps=2)
            print("Animation saved as 'drone_animation.gif'")
        except Exception as e2:
            print(f"Failed to save animation: {e2}")
    
    # Display the animation directly
    print("Displaying animation in window...")
    plt.figure()
    ani = Visualization.create_animation(primary_mission, other_missions, conflicts)
    plt.show()

if __name__ == "__main__":
    main()