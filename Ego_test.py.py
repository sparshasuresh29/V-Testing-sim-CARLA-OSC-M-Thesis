#!/usr/bin/env python

import carla
import math
import time
import sys

def find_ego_vehicle(world):
    """Find the ego vehicle based on role name"""
    world.tick()
    vehicles = world.get_actors().filter('vehicle.*')
    for vehicle in vehicles:
        role = vehicle.attributes.get('role_name', '')
        if role == 'ego_vehicle' or role == 'hero':
            return vehicle
    for vehicle in vehicles:
        if 'tesla' in vehicle.type_id.lower():
            return vehicle
    return None

def find_adversary_vehicles(world, ego_vehicle):
    """Find all adversary vehicles (not the ego vehicle)"""
    if ego_vehicle is None:
        return []
    vehicles = world.get_actors().filter('vehicle.*')
    adversaries = [vehicle for vehicle in vehicles if vehicle.id != ego_vehicle.id]
    return adversaries

class WaypointFollower:
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.map = world.get_map()
        self.waypoint_separation = 2.0  # meters
        self.waypoints = []
        self.current_waypoint_index = 0
        self.lookahead_distance = 5.0  # meters
        self.previous_steering = 0.0  # For smoothing steering
        
    def generate_path(self, distance=500, lane_change=None):
        vehicle_loc = self.vehicle.get_location()
        current_waypoint = self.map.get_waypoint(vehicle_loc)
        if lane_change == 'left' and current_waypoint.get_left_lane():
            current_waypoint = current_waypoint.get_left_lane()
        elif lane_change == 'right' and current_waypoint.get_right_lane():
            current_waypoint = current_waypoint.get_right_lane()
        self.waypoints = [current_waypoint]
        distance_traveled = 0
        while distance_traveled < distance:
            next_waypoints = current_waypoint.next(self.waypoint_separation)
            if not next_waypoints:
                break
            current_waypoint = next_waypoints[0]
            self.waypoints.append(current_waypoint)
            distance_traveled += self.waypoint_separation
        print(f"Generated {len(self.waypoints)} waypoints covering {distance_traveled:.1f}m in {'left' if lane_change == 'left' else 'right' if lane_change == 'right' else 'current'} lane")
        return self.waypoints
    
    def update_waypoints(self):
        if not self.waypoints:
            return
        vehicle_loc = self.vehicle.get_location()
        min_distance = float('inf')
        closest_index = self.current_waypoint_index
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            waypoint = self.waypoints[i]
            distance = vehicle_loc.distance(waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        self.current_waypoint_index = max(closest_index, self.current_waypoint_index)  # Prevent going backward
        if self.current_waypoint_index >= len(self.waypoints) - 20:  # Increased buffer
            self.generate_path(distance=500)  # Consistent distance
    
    def get_target_waypoint(self):
        self.update_waypoints()
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return None
        vehicle_loc = self.vehicle.get_location()
        cumulative_distance = 0
        target_index = self.current_waypoint_index
        for i in range(self.current_waypoint_index + 1, len(self.waypoints)):
            waypoint = self.waypoints[i]
            prev_waypoint = self.waypoints[i-1]
            segment_distance = waypoint.transform.location.distance(prev_waypoint.transform.location)
            cumulative_distance += segment_distance
            if cumulative_distance >= self.lookahead_distance:
                target_index = i
                break
        if target_index >= len(self.waypoints):
            target_index = len(self.waypoints) - 1
        return self.waypoints[target_index]
    
    def calculate_steering(self):
        target_waypoint = self.get_target_waypoint()
        if not target_waypoint:
            return 0.0
        vehicle_transform = self.vehicle.get_transform()
        target_loc = target_waypoint.transform.location
        vehicle_loc = vehicle_transform.location
        direction_vector = target_loc - vehicle_loc
        vehicle_forward = vehicle_transform.get_forward_vector()
        vehicle_right = vehicle_transform.get_right_vector()
        forward_dot = direction_vector.x * vehicle_forward.x + direction_vector.y * vehicle_forward.y
        right_dot = direction_vector.x * vehicle_right.x + direction_vector.y * vehicle_right.y
        steering_angle = math.atan2(right_dot, forward_dot)
        smoothed_steering = 0.7 * self.previous_steering + 0.3 * steering_angle
        self.previous_steering = smoothed_steering
        return max(-1.0, min(1.0, smoothed_steering))

def main():
    client = None
    ego_vehicle = None
    adversary_vehicles = []
    try:
        # Connect to CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Get the spectator
        spectator = world.get_spectator()
        
        print("Connected to CARLA. Searching for vehicles...")
        
        # Wait for the ego vehicle to be spawned
        max_attempts = 30
        for attempt in range(max_attempts):
            ego_vehicle = find_ego_vehicle(world)
            if ego_vehicle:
                print(f"Found ego vehicle: ID={ego_vehicle.id}, Type={ego_vehicle.type_id}")
                break
            print(f"Searching for ego vehicle... Attempt {attempt+1}/{max_attempts}")
            time.sleep(0.5)
        
        if ego_vehicle is None:
            print("Failed to find ego vehicle. Exiting.")
            return
        
        # Find all adversary vehicles
        adversary_vehicles = find_adversary_vehicles(world, ego_vehicle)
        if adversary_vehicles:
            for adv in adversary_vehicles:
                print(f"Found adversary vehicle: ID={adv.id}, Type={adv.type_id}")
        else:
            print("Warning: No adversary vehicles found")
        
        # Initialize waypoint follower for ego vehicle
        follower = WaypointFollower(world, ego_vehicle)
        follower.generate_path(distance=500)
        
        # State variables
        lane_changed = False
        lane_return_detected = False
        original_lane_id = None
        start_time = time.time()
        lane_change_time = None
        state = "normal"  # States: normal, overtaking_left, returning_right
        previous_spectator_loc = None
        
        print("Starting waypoint following...")
        
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Get vehicle states
            ego_transform = ego_vehicle.get_transform()
            ego_loc = ego_transform.location
            ego_velocity = ego_vehicle.get_velocity()
            ego_speed_kmh = 3.6 * math.sqrt(ego_velocity.x**2 + ego_velocity.y**2 + ego_velocity.z**2)
            current_waypoint = world.get_map().get_waypoint(ego_loc)
            current_lane_id = current_waypoint.lane_id
            
            # Update spectator view
            spectator_transform = ego_transform
            target_loc = spectator_transform.location + carla.Location(z=5) - spectator_transform.get_forward_vector() * 10
            if previous_spectator_loc is None:
                previous_spectator_loc = target_loc
            smoothed_loc = carla.Location(
                x=0.9 * previous_spectator_loc.x + 0.1 * target_loc.x,
                y=0.9 * previous_spectator_loc.y + 0.1 * target_loc.y,
                z=0.9 * previous_spectator_loc.z + 0.1 * target_loc.z
            )
            spectator_transform.location = smoothed_loc
            spectator_transform.rotation.pitch = -20
            spectator.set_transform(spectator_transform)
            previous_spectator_loc = smoothed_loc
            
            if original_lane_id is None:
                original_lane_id = current_lane_id
                print(f"Original lane ID: {original_lane_id}")
            
            # Check distances to all adversary vehicles
            adversary_speed_kmh = 28.8  # Default if no adversaries
            trigger_lane_change = False
            closest_adversary_distance = float('inf')
            closest_adversary_behind_distance = float('inf')
            if adversary_vehicles:
                adversary_speed_kmh = 0
                for adv in adversary_vehicles:
                    if not adv.is_alive:
                        continue  # Skip destroyed adversaries
                    adv_loc = adv.get_location()
                    adv_velocity = adv.get_velocity()
                    adv_speed_kmh = 3.6 * math.sqrt(adv_velocity.x**2 + adv_velocity.y**2 + adv_velocity.z**2)
                    distance_to_adv = ego_loc.distance(adv_loc)
                    adv_waypoint = world.get_map().get_waypoint(adv_loc)
                    adv_lane_id = adv_waypoint.lane_id
                    
                    adversary_speed_kmh = max(adversary_speed_kmh, adv_speed_kmh)
                    # Check for lane change trigger (any adversary ahead within 27m)
                    if adv_lane_id == current_lane_id and distance_to_adv <= 28:
                        trigger_lane_change = True
                        closest_adversary_distance = min(closest_adversary_distance, distance_to_adv)
                    # Check distance to adversary behind in same lane
                    ego_yaw = math.radians(ego_transform.rotation.yaw)
                    relative_x = adv_loc.x - ego_loc.x
                    relative_y = adv_loc.y - ego_loc.y
                    longitudinal_dist = relative_x * math.cos(ego_yaw) + relative_y * math.sin(ego_yaw)
                    if adv_lane_id == current_lane_id and longitudinal_dist < 0:  # Behind ego
                        closest_adversary_behind_distance = min(closest_adversary_behind_distance, distance_to_adv)
            
            # Calculate steering
            steering = follower.calculate_steering()
            control = carla.VehicleControl()
            control.steer = steering
            
            # State machine for lane changing
            if state == "normal" and trigger_lane_change and not lane_changed:
                print(f"Adversary within {closest_adversary_distance:.1f}m in same lane, changing to left lane")
                follower.generate_path(distance=500, lane_change='left')
                lane_changed = True
                lane_change_time = current_time
                state = "overtaking_left"
            
            elif state == "overtaking_left" and (current_time - lane_change_time) >= 17:  # Changed from 20 to 15
                print("15 seconds in left lane completed, returning to original lane")
                follower.generate_path(distance=500, lane_change='right')
                lane_return_detected = False
                lane_change_time = current_time
                state = "returning_right"
            
            elif state == "returning_right" and (current_time - lane_change_time) >= 17:
                print("15 seconds in original lane completed, returning to normal state")
                state = "normal"
                lane_changed = False
                lane_return_detected = True
            
            # Check distance to adversary behind after overtaking
            if state == "normal" and lane_return_detected and closest_adversary_behind_distance >= 35:
                print(f"Distance to adversary behind is {closest_adversary_behind_distance:.1f}m (>= 35m), ending scenario")
                break
            
            # Speed control
            target_speed_kmh = adversary_speed_kmh + 19  # Increased by 4 km/h from +15 to +19
            speed_error = target_speed_kmh - ego_speed_kmh
            if speed_error > 0:
                control.throttle = min(0.7, max(0.05, speed_error / 30.0))
                control.brake = 0.0
            else:
                control.throttle = 0.0
                control.brake = min(0.5, max(0.05, -speed_error / 30.0))
            
            ego_vehicle.apply_control(control)
            
            # Print status periodically
            if int(elapsed_time) % 2 == 0 and int(elapsed_time) != int(elapsed_time - 0.02):
                lane_status = "Initial Lane" if current_lane_id == original_lane_id else "Overtaking Lane"
                print(f"Time: {elapsed_time:.1f}s | Speed: {ego_speed_kmh:.1f} km/h | Adv Speed: {adversary_speed_kmh:.1f} km/h | "
                      f"Target Speed: {target_speed_kmh:.1f} km/h | Closest Adv Distance: {closest_adversary_distance:.1f}m | "
                      f"Behind Adv Distance: {closest_adversary_behind_distance:.1f}m | Lane: {lane_status} | State: {state} | Steering: {steering:.2f}")
            
            time.sleep(0.02)
            
            if elapsed_time > 120:
                print("2-minute scenario duration reached. Exiting.")
                break
                
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'ego_vehicle' in locals() and ego_vehicle:
            ego_vehicle.apply_control(carla.VehicleControl())
            print("Control released")

if __name__ == '__main__':
    main()
