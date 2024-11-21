import argparse
from os import path, getcwd
import math
import pygame
from robot import Robot, Sim, DepthSensor

def main():

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map", default="./maps/map_1.png", help="File path to map for robot to traverse"
    )
    parser.add_argument(
        "--no_noise", action='store_true', help="Determines if LIDAR measurements are noisy or not"
    )
    args = parser.parse_args()

    MAP_DIMENSIONS = (600, 1200)

    simulation = Sim(MAP_DIMENSIONS, path.join(getcwd(), "robot_assets", "base.png"), args.map)

    start = (200, 200)
    bot = Robot(start, 0.01*400)

    sensor_range = 250, math.radians(40)
    lidar = DepthSensor(sensor_range, bot.min_obs_dist, simulation.map)

    dt = 0
    last_time = pygame.time.get_ticks()

    running = True

    lidar_refresh = 5

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                return None
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    running = not running
        dt = (pygame.time.get_ticks() - last_time)/1000
        last_time = pygame.time.get_ticks()
        simulation.map.blit(simulation.map_img, (0, 0))
        if running:
            bot.kinematics(dt)
            simulation.draw_robot(bot.x, bot.y, bot.heading)
            if lidar_refresh == 5:
                point_cloud = lidar.sense_obstacles(bot.x, bot.y, bot.heading) if args.no_noise else lidar.sense_obstacles_noisy(bot.x, bot.y, bot.heading)
            lidar_refresh -= 1
            if lidar_refresh < 0:
                lidar_refresh = 5
            bot.avoid_obstacles(point_cloud, dt)
            simulation.draw_sensor_data(point_cloud)
            pygame.display.update()

if __name__ == "__main__":
    main()