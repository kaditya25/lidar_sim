import pygame
import math
import numpy as np

def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1-point2)

class Robot:
    
    def __init__(self, startpos, width):
        # Meters to pixel conversion
        self.m2p = 400
        # Robot width (used to figure calculate heading from both side wheel speeds)
        self.w = width
        # Initializes robot's position
        self.x = startpos[0]
        self.y = startpos[1]
        self.heading = 0
        # Initializes robot's initial speed
        self.vl = 0.5*self.m2p
        self.vr = 0.5*self.m2p
        # Put limits on robot's wheel speeds
        self.maxspeed = 2*self.m2p
        self.minspeed = 0.5*self.m2p
        # Controls robot's obstacle avoidance
        self.min_obs_dist = 100
        self.count_down = 2

    def avoid_obstacles(self, point_cloud, dt):
        closest_obs = None
        dist = np.inf

        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x, self.y], point)
                    closest_obs = (point, dist)
            if closest_obs[1] < self.min_obs_dist and self.count_down > 0:
                self.count_down -= dt
                self.move_back()
            else:
                self.count_down = 2
                self.move_front()

    def move_back(self):
        self.vr = -self.minspeed
        self.vl = -self.minspeed/2

    def move_front(self):
        self.vr = self.minspeed
        self.vl = self.minspeed

    def kinematics(self, dt):
        self.x += ((self.vl + self.vr)/2)*math.cos(self.heading)*dt
        self.y -= ((self.vl + self.vr)/2)*math.sin(self.heading)*dt
        self.heading += (self.vr - self.vl)/self.w*dt

        if self.heading > 2*math.pi or self.heading < -2*math.pi:
            self.heading = 0
        
        self.vr = max(min(self.maxspeed, self.vr), self.minspeed)
        self.vl = max(min(self.maxspeed, self.vl), self.minspeed)

class Sim:

    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()
        # Loads images for robot and environment
        self.robot = pygame.image.load(robot_img_path)
        self.map_img = pygame.image.load(map_img_path)
        # Loads environment dimensions for display
        self.height, self.width = dimensions
        
        pygame.display.set_caption("LIDAR Obstacle Avoidance Sim")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.map_img, (0, 0))

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center = (x, y))
        self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, (255, 0, 0), point, 3, 0)


class DepthSensor:
    
    def __init__(self, sensor_range, min_obs_dist, map):
        # Loads sensor details and parameters for obstacle detection
        self.sensor_range = sensor_range
        self.min_obs_dist = min_obs_dist
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.resolution = 25
        self.map = map
        self.noise = 5

    def sense_obstacles(self, x, y, heading):
        obstacles = []
        x1, y1 = x, y
        start_angle = heading - self.sensor_range[1]
        end_angle = heading + self.sensor_range[1]
        for angle in np.linspace(start_angle, end_angle, self.resolution, False):
            x2 = x1 + self.sensor_range[0]*math.cos(angle)
            y2 = y1 - self.sensor_range[0]*math.sin(angle)
            for i in range(0, self.min_obs_dist):
                u = i/100
                x = int(x2*u + x1*(1 - u))
                y = int(y2*u + y1*(1 - u))
                if 0<x<self.map_width and 0<y<self.map_height:
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 255, 0))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obstacles.append([x, y])
                        break
        return obstacles
    
    def sense_obstacles_noisy(self, x, y, heading):
        obstacles = []
        x1, y1 = x, y
        start_angle = heading - self.sensor_range[1]
        end_angle = heading + self.sensor_range[1]
        for angle in np.linspace(start_angle, end_angle, self.resolution, False):
            x2 = x1 + self.sensor_range[0]*math.cos(angle)
            y2 = y1 - self.sensor_range[0]*math.sin(angle)
            for i in range(0, self.min_obs_dist):
                u = i/100
                x = int(x2*u + x1*(1 - u))
                y = int(y2*u + y1*(1 - u))
                if 0<x<self.map_width and 0<y<self.map_height:
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 255, 0))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obstacles.append([x, y])
                        break
        noise = np.random.normal(0, self.noise, len(obstacles))
        for i in range(len(obstacles)):
            obstacles[i][0] += noise[i]
            obstacles[i][1] += noise[i]
        return obstacles