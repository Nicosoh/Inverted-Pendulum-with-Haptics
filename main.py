from NetworkHandler import NetworkHandler
from PendulumSimulator import PendulumSimulator
from MpcController import MPCController
from Display import ScreenManager, StartPage, GameScreen, GameOverScreen
import numpy as np
import pygame
import pandas as pd
import os

def reset_game():
    global pendulum, time_left, score, mpc, state
    
    # Reset the state array for the next round
    state = []

    # Reset pendulum state
    pendulum = PendulumSimulator(M, m, g, l, dt, d_cart, d_theta)

    # Reset time and score
    time_left = 15.0  # Reset time
    score = 0.0

    # Delete the existing MPC object
    del mpc

    # Reinitialize the MPC controller
    mpc = MPCController(M, m, g, l, dt, d_cart, d_theta, Fmax, Tf, N_horizon)

    for _ in range(3):  
        network.receive_position()

def save_state_to_csv(state, base_filename="saved_states/states_Trial.csv"):
    if not state:  # If state is empty, print a message and skip saving
        print("State is empty, not saving to CSV.")
        return
    
    # Convert the state to a Pandas DataFrame
    # Assuming each entry in the state is a list or array of values
    df = pd.DataFrame(state, columns=[
        'x_ref', 'x', 'x_dot', 'x_ddot', 'theta', 'theta_dot', 'theta_ddot', 
        'score', 'F', 'control_input', 'random_force', 'time_left'
    ])

    # Get the file extension and base name
    base_filename, extension = os.path.splitext(base_filename)
    
    # Generate the initial filename
    filename = f"{base_filename}_1{extension}"
    
    # Check if the file already exists and increment the filename if necessary
    i = 1
    while os.path.exists(filename):
        filename = f"{base_filename}_{i}{extension}"
        i += 1

    # Save the DataFrame to CSV
    df.to_csv(filename, index=False)
    
    print(f"State has been saved to {filename}")

# Initialize pygame
pygame.init()

# Define network parameters
UDP_IP = "127.0.0.1"
SEND_PORT = 5005
RECV_PORT = 5006

# Create an instance of NetworkHandler
network = NetworkHandler(UDP_IP, SEND_PORT, RECV_PORT)

# Define physical constants in SI units
M = 0.5  
m = 1.0  
g = -9.81
l = 1.0  
dt = 0.005
cart_width = 0.3  
cart_height = 0.1
cart_y_coord = 350
d_cart = 1.0 # Cart Damping
d_theta = 0.3 # Pendulum Damping

# Define Gaussian noise parameters
noise_mean = 0  # Mean of the noise
noise_std = 200  # Standard deviation of the noise

# Initialize counter for random force application
random_force_counter = 0
random_force = 0

# Impedance Controller
K = 4000
D = 50

# Define scaling factors to convert SI units to pixels
scale_factor = 300

# Score variables
score = 0.0
high_score = 0.0
time_left = 15.0

# State Tracking
state = []

# Create an instance of PendulumSimulator
pendulum = PendulumSimulator(M, m, g, l, dt, d_cart, d_theta)

# MPC Settings
Fmax = 400
N_horizon = 30
Tf = dt * N_horizon
force_scaling = 50

# Create instance of the controller and setup
mpc = MPCController(M, m, g, l, dt, d_cart, d_theta, Fmax, Tf, N_horizon)

# Initialize pygame window and clock
WIDTH = 600
HEIGHT = 400
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Initialize screen manager
screen_manager = ScreenManager(network)
start_screen = StartPage(screen_manager)
game_screen = GameScreen(screen_manager,scale_factor, cart_y_coord)
game_over_screen = GameOverScreen(screen_manager, reset_game)

screen_manager.add_screen("game_over", game_over_screen)
screen_manager.add_screen("start", start_screen)
screen_manager.add_screen("game", game_screen)

# Start with the start screen
screen_manager.set_screen("start")
running = True
# Main game loop
while running:
    events = pygame.event.get()
    
    for event in events:
        if event.type == pygame.QUIT:
            running = False  # Quit game

    screen_manager.handle_events(events)

    if isinstance(screen_manager.current_screen, GameScreen):
        if time_left > 0:
            time_left -= dt  # Countdown
            if pendulum.theta >= np.pi / 2 and pendulum.theta <= 3 * np.pi / 2:
                position = network.receive_position()
                if position is not None:
                    mpc_enabled = start_screen.mpc_enabled
                    disturbance_enbaled = start_screen.disturbance_enabled
                    x_ref = (position - WIDTH // 2) / scale_factor
                    x, x_dot, theta = pendulum.x, pendulum.x_dot, pendulum.theta
                    
                    if disturbance_enbaled:
                        # Apply Gaussian random force
                        if random_force_counter >= 400:
                            random_force = np.random.normal(noise_mean, noise_std)
                            random_force_counter = 0  # Reset counter
                        else:
                            random_force = 0
                            random_force_counter += 1  # Increment counter
                    else:
                        random_force = 0

                    F = K * (x_ref - x) - D * x_dot + random_force
                    pendulum.update_physics(F)

                    if mpc_enabled:
                        u = mpc.compute_control(np.array([pendulum.x, pendulum.theta, pendulum.x_dot, pendulum.theta_dot]))
                        u = np.array([-u[0] / force_scaling, 0])
                    else:
                        u = np.array([0, 0])

                    network.send_force(u)

                    deviation = abs(np.pi - pendulum.theta)
                    score += np.exp(-50 * deviation)

                    if score > high_score:
                        high_score = score

                    state.append([x_ref, pendulum.x, pendulum.x_dot, pendulum.x_ddot, pendulum.theta, pendulum.theta_dot, pendulum.theta_ddot, score, F, u[0], random_force, time_left])

                    fps = clock.get_fps()
                    screen_manager.draw(screen, x, theta, score, high_score, fps, time_left)
            else:
                save_state_to_csv(state)
                screen_manager.set_screen("game_over")  # Switch to Game Over
        else:
            save_state_to_csv(state)
            screen_manager.set_screen("game_over")  # Switch to Game Over

    elif isinstance(screen_manager.current_screen, GameOverScreen):
        screen_manager.draw(screen, score, high_score)  # Draw Game Over screen

    else:
        screen_manager.draw(screen)

    pygame.display.flip()
    clock.tick(1 / dt)

pygame.quit()


