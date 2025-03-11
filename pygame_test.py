import pygame
import numpy as np
from pygame.locals import *

# Define physical constants in SI units
M = 1.0  
m = 0.5  
g = -9.81  
l = 0.5  
dt = 0.01  
cart_width = 0.3  
cart_height = 0.1  
cart_y_coord = 800

# Define scaling factors to convert SI units to pixels
scale_factor = 600  

# Pygame settings
WIDTH = 1000
HEIGHT = 1000
FPS = 1 / dt  

# Initial conditions in SI units
x1_SI = 0.0  
theta_SI = np.pi  
x1_dot_SI = 0.0  
theta_dot_SI = 0.0  
x1_ddot_SI = 0.0
theta_ddot_SI = 0.0
x1_ref_SI = 0.0
prev_x_SI = 0.0

K = 1000
D = 0.01

# Score variables
score = 0.0
high_score = 0.0  

# Game state variables
game_over = False  
simulation_time = 0.0  # Track elapsed time

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Define font for score display
font = pygame.font.Font(None, 36)

def convert_to_pixels(value, scale_factor):
    return int(value * scale_factor)

def update_physics(theta, theta_dot, F):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    denominator = M + m - m * sin_theta ** 2

    theta_ddot = (-m * l * cos_theta * sin_theta * theta_dot ** 2 + F * cos_theta + (M + m) * g * sin_theta) / (3 * l * denominator)
    x1_ddot = (-m * l * sin_theta * theta_dot ** 2 + m * g * cos_theta * sin_theta + F) / denominator

    return x1_ddot, theta_ddot

def draw_system(x1, theta, score, high_score, game_over):
    rod_length_SI = convert_to_pixels(l, scale_factor)  
    cart_width_SI = convert_to_pixels(cart_width, scale_factor)
    cart_height_SI = convert_to_pixels(cart_height, scale_factor)
    pendulum_x = x1 - rod_length_SI * np.sin(theta)
    pendulum_y = cart_y_coord + rod_length_SI * np.cos(theta)  

    screen.fill((255, 255, 255))

    pygame.draw.line(screen, (0, 0, 0), (x1, cart_y_coord), (pendulum_x, pendulum_y), 3)  
    pygame.draw.circle(screen, (255, 0, 0), (int(pendulum_x), int(pendulum_y)), 10)  
    pygame.draw.rect(screen, (0, 0, 255), (x1 - cart_width_SI / 2, cart_y_coord, cart_width_SI, cart_height_SI))  

    score_text = font.render(f"Score: {score:.2f}", True, (0, 0, 0))
    high_score_text = font.render(f"High Score: {high_score:.2f}", True, (0, 0, 0))
    screen.blit(score_text, (20, 20))
    screen.blit(high_score_text, (20, 50))

    if game_over:
        game_over_text = font.render("GAME OVER - Press 'R' to Restart", True, (255, 0, 0))
        screen.blit(game_over_text, (WIDTH // 2 - 200, HEIGHT // 2))

def main():
    global x1_SI, theta_SI, x1_dot_SI, theta_dot_SI, prev_x_SI, score, high_score, game_over, simulation_time
    running = True
    simulation_running = False

    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    x1_SI = x1_ref_SI
                    theta_SI = np.pi
                    x1_dot_SI = 0.0
                    theta_dot_SI = 0.0
                    x1_ddot_SI = 0.0
                    theta_ddot_SI = 0.0
                    score = 0.0  
                    simulation_time = 0.0  
                    game_over = False  
                if event.key == pygame.K_q:
                    running = False
                if event.key == pygame.K_e and not game_over:
                    simulation_running = True

        if simulation_running and not game_over:
            mouse_x, _ = pygame.mouse.get_pos()
            x1_ref_SI = (mouse_x - WIDTH // 2) / scale_factor

            x1_dot_SI = (x1_SI - prev_x_SI) / dt
            F = K * (x1_ref_SI - x1_SI) - D * x1_dot_SI

            x1_ddot_SI, theta_ddot_SI = update_physics(theta_SI, theta_dot_SI, F)
            
            x1_dot_SI += x1_ddot_SI * dt
            x1_SI += x1_dot_SI * dt
            theta_dot_SI += theta_ddot_SI * dt
            theta_SI += theta_dot_SI * dt

            prev_x_SI = x1_SI
            
            # **Check if pendulum crosses the horizontal**
            if theta_SI <= np.pi / 2 or theta_SI >= 3 * np.pi / 2:
                game_over = True
                print("GAME OVER! Pendulum fell past horizontal.")

            # **Score Calculation:**
            score += dt * (1 - abs(np.pi - theta_SI) / np.pi)  
            if score > high_score:
                high_score = score  

            # **Game Over After 15 Seconds**
            simulation_time += dt
            if simulation_time >= 15.0:
                game_over = True
                print("GAME OVER! Time limit reached.")

            x1_pixels = convert_to_pixels(x1_SI, scale_factor)
            print(x1_pixels + WIDTH // 2, theta_SI, score, high_score, game_over)
            draw_system(x1_pixels + WIDTH / 2, theta_SI, score, high_score, game_over)
            pygame.display.flip()

            clock.tick(FPS)

    pygame.quit()

if __name__ == '__main__':
    main()
