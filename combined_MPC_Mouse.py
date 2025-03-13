# --- Acados and MPC Imports ---
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from pendulum_model import export_pendulum_ode_model
from casadi import vertcat

import pygame
import numpy as np
from pygame.locals import *
import socket
import json
import matplotlib.pyplot as plt

# Define physical constants in SI units
M = 0.5  
m = 1.0  
g = -9.81
l = 1.0  
dt = 0.005  
cart_width = 0.3  
cart_height = 0.1
cart_y_coord = 350

# Define scaling factors to convert SI units to pixels
scale_factor = 300

# Pygame settings
WIDTH = 600
HEIGHT = 400
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

K = 4000
D = 10

d_cart = 0.1 # Cart Damping
d_theta = 0.5 # Pendulum Damping

# Score variables
score = 0.0
high_score = 0.0  

# Game state variables
game_over = False  
simulation_time = 0.0  # Track elapsed time
mpc_ready = True

state = []

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Define font for score display
font = pygame.font.Font(None, 36)

# Ports
SEND_PORT = 5005  # Send to Haptic
RECV_PORT = 5006  # Receive from Haptic
UDP_IP = "127.0.0.1"  # Localhost

# Create UDP sockets
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # For sending
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # For receiving

recv_sock.bind((UDP_IP, RECV_PORT))  # Bind receiving socket
recv_sock.setblocking(False)  # Set non-blocking

def display_message(message):
    """Displays a message on the screen."""
    screen.fill((255, 255, 255))  # Clear screen
    text = font.render(message, True, (0, 0, 0))
    screen.blit(text, (WIDTH // 2 - 150, HEIGHT // 2))
    pygame.display.flip()

def convert_to_pixels(value, scale_factor):
    return value * scale_factor

def update_physics(theta, theta_dot, x1_dot, F):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    # denominator = M + m - m * sin_theta ** 2
    denominator = M + m - m * cos_theta ** 2

    # theta_ddot = (-m * l * cos_theta * sin_theta * theta_dot ** 2 + F * cos_theta + (M + m) * g * sin_theta) / (l * denominator)
    # x1_ddot = (-m * l * sin_theta * theta_dot ** 2 + m * g * cos_theta * sin_theta + F) / denominator

    theta_ddot = ((m + M ) * g * sin_theta - cos_theta * (m * l * theta_dot**2 * sin_theta - d_cart * x1_dot) + cos_theta * F - theta_dot * d_theta) / denominator / l
    x1_ddot = (-m * g * cos_theta * sin_theta + m * l * theta_dot**2 * sin_theta - d_cart * x1_dot + F) / denominator 
    
    return x1_ddot, theta_ddot

def draw_system(x1, theta, score, high_score, game_over, fps, time_left):
    rod_length_SI = convert_to_pixels(l, scale_factor)  
    cart_width_SI = convert_to_pixels(cart_width, scale_factor)
    cart_height_SI = convert_to_pixels(cart_height, scale_factor)
    pendulum_x = x1 - rod_length_SI * np.sin(theta)
    pendulum_y = cart_y_coord + rod_length_SI * np.cos(theta)  

    screen.fill((255, 255, 255))

    pygame.draw.line(screen, (0, 0, 0), (x1, cart_y_coord), (pendulum_x, pendulum_y), 3)  
    pygame.draw.circle(screen, (255, 0, 0), (int(pendulum_x), int(pendulum_y)), 10)  
    pygame.draw.rect(screen, (0, 0, 255), (x1 - cart_width_SI / 2, cart_y_coord, cart_width_SI, cart_height_SI))  

    # --- Display Text ---
    score_text = font.render(f"Score: {score:.2f}", True, (0, 0, 0))
    high_score_text = font.render(f"High Score: {high_score:.2f}", True, (0, 0, 0))
    fps_text = font.render(f"FPS: {fps:.1f}", True, (0, 0, 0))
    timer_text = font.render(f"Time Left: {time_left:.1f} s", True, (255, 0, 0))  

    # Positioning Text
    screen.blit(score_text, (20, 20))
    screen.blit(high_score_text, (20, 50))
    screen.blit(fps_text, (20, 80))  
    screen.blit(timer_text, (WIDTH - 200, 20))  

    if game_over:
        game_over_text = font.render("GAME OVER - Press 'R' to Restart", True, (255, 0, 0))
        screen.blit(game_over_text, (WIDTH // 2 - 200, HEIGHT // 2))


# --- MPC Setup Function using acados ---
def setup_mpc(x0, Fmax, N_horizon, Tf, RTI=False):
    # Create the OCP object
    ocp = AcadosOcp()

    # Set the model (pendulum)
    model = export_pendulum_ode_model(M, m ,g, l, d_cart, d_theta)
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    # Set cost module (using a nonlinear least-squares cost)
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    Q_mat = 2 * np.diag([1e1, 1e3, 1e-2, 1e-2])
    R_mat = 2 * np.diag([1e-2])
    from scipy.linalg import block_diag  # Block-diagonal weight matrix
    ocp.cost.W = block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat

    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x

    yref = np.zeros((ny,))
    yref[1] = np.pi  # Target: Pendulum upright

    yref_e = np.zeros((ny_e,))
    yref_e[1] = np.pi  # Terminal reference

    ocp.cost.yref = yref
    ocp.cost.yref_e = yref_e

    # Set constraints on the control input
    ocp.constraints.lbu = np.array([-Fmax])
    ocp.constraints.ubu = np.array([+Fmax])
    ocp.constraints.idxbu = np.array([0])
    ocp.constraints.x0 = x0

    # Set prediction horizon
    ocp.solver_options.N_horizon = N_horizon
    ocp.solver_options.tf = Tf

    # Solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'IRK'
    ocp.solver_options.sim_method_newton_iter = 10

    if RTI:
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    else:
        ocp.solver_options.nlp_solver_type = 'SQP'
        ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
        ocp.solver_options.nlp_solver_max_iter = 150

    ocp.solver_options.qp_solver_cond_N = N_horizon

    # Save to a JSON file (required by acados)
    solver_json = 'acados_ocp_' + model.name + '.json'
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file=solver_json)
    acados_integrator = AcadosSimSolver(ocp, json_file=solver_json)

    return acados_ocp_solver, acados_integrator

def plot_states(state, dt):
    """ Plots the recorded states and force over time. """
    state = np.array(state)  # Convert list to NumPy array
    num_vars = 9  # Now includes force

    if state.shape[1] != num_vars:
        print("Warning: State array size mismatch!")
        return

    # Extract individual states
    x1_SI = state[:, 0]         # Cart Position
    x1_ref_SI = state[:, 1]     # Reference Position
    x1_dot_SI = state[:, 2]     # Cart Velocity
    x1_ddot_SI = state[:, 3]    # Cart Acceleration
    theta_SI = state[:, 4] / np.pi * 180       # Pendulum Angle (degrees)
    theta_dot_SI = state[:, 5] / np.pi * 180   # Angular Velocity (deg/s)
    theta_ddot_SI = state[:, 6] / np.pi * 180  # Angular Acceleration (deg/s²)
    force_SI = state[:, 7]      # Applied Force
    MPC_output = state[:, 8]/100

    time = np.arange(len(x1_SI)) * dt  # Create time array

    # --- Plot ---
    fig, axs = plt.subplots(8, 1, figsize=(10, 18), sharex=True)

    axs[0].plot(time, x1_SI, label="Cart Position (m)")
    axs[0].plot(time, x1_ref_SI, '--', label="Reference Position", alpha=0.7)
    axs[0].set_ylabel("Position (m)")
    axs[0].grid()

    axs[1].plot(time, theta_SI, label="Pendulum Angle (°)", color='orange')
    axs[1].axhline(y=180, linestyle="--", color="gray", label="180° (π rad)")
    axs[1].set_ylabel("Angle (°)")
    axs[1].grid()

    axs[2].plot(time, x1_dot_SI, label="Cart Velocity (m/s)", color='green')
    axs[2].set_ylabel("Velocity (m/s)")
    axs[2].grid()

    axs[3].plot(time, x1_ddot_SI, label="Cart Acceleration (m/s²)", color='blue')
    axs[3].set_ylabel("Acceleration (m/s²)")
    axs[3].grid()

    axs[4].plot(time, theta_dot_SI, label="Pendulum Angular Velocity (°/s)", color='red')
    axs[4].set_ylabel("Angular Vel (°/s)")
    axs[4].grid()

    axs[5].plot(time, theta_ddot_SI, label="Pendulum Angular Acceleration (°/s²)", color='brown')
    axs[5].set_ylabel("Angular Accel (°/s²)")
    axs[5].grid()

    axs[6].plot(time, force_SI, label="Applied Force (N)", color='purple')
    axs[6].axhline(y=0, linestyle="--", color="gray", label="Zero Force")
    axs[6].set_xlabel("Time (s)")
    axs[6].set_ylabel("Force (N)")
    axs[6].grid()

    axs[7].plot(time, MPC_output, label="MPC Output (N)", color='blue')
    axs[7].axhline(y=0, linestyle="--", color="gray", label="Zero Force")
    axs[7].set_xlabel("Time (s)")
    axs[7].set_ylabel("Force (N)")
    axs[7].grid()

    plt.suptitle("System States and Applied Force Over Time")
    plt.show()

def main():
    global x1_SI, theta_SI, x1_dot_SI, theta_dot_SI, prev_x_SI, score, high_score, game_over, simulation_time, mpc_ready
    running = True
    simulation_running = False

    # Step 1: Show Initial Waiting Screen
    display_message("Initializing MPC... Please wait.")

    x0 = np.array([0.0, 0.0, 0.0, 0.0])  # Initial state
    Fmax = 400
    N_horizon = 30
    Tf = dt * N_horizon
    assert(Tf/N_horizon == dt)
    
    F_send = np.array([0, 0])
    # Setup MPC using acados
    ocp_solver, integrator = setup_mpc(x0, Fmax, N_horizon, Tf, RTI=True)

    # Step 3: Once MPC is ready, show "Press E to Start"
    display_message("Press 'E' to start.")
    
    # Initialize simulation state
    current_state = np.array([x1_SI, theta_SI, x1_dot_SI, theta_dot_SI])


    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    # Show loading message
                    display_message("Rebuilding MPC... Please wait.")

                    # Reset all simulation variables
                    
                    x1_dot_SI = 0.0
                    x1_ddot_SI = 0.0
                    theta_SI = np.pi
                    theta_dot_SI = 0.0
                    theta_ddot_SI = 0.0
                    score = 0.0  
                    simulation_time = 0.0  
                    game_over = False

                    # Set flag to prevent simulation updates
                    mpc_ready = False  

                    message = json.dumps([0, 0])  # Reset forces
                    send_sock.sendto(message.encode(), (UDP_IP, SEND_PORT))

                    data, addr = recv_sock.recvfrom(1024)
                    position, _ = json.loads(data.decode())

                    x1_ref_SI = (position - WIDTH // 2) / scale_factor
                    prev_x_SI = x1_ref_SI

                    # Rebuild the MPC model
                    x0 = np.array([x1_ref_SI, np.pi, 0.0, 0.0])  # Reset initial state
                    ocp_solver, integrator = setup_mpc(x0, Fmax, N_horizon, Tf, RTI=True)

                    # Set flag to indicate MPC is ready
                    mpc_ready = True  

                    # Show message to press 'E' to continue
                    display_message("Press 'E' to start.")
                if event.key == pygame.K_q:
                    running = False
                if event.key == pygame.K_e and mpc_ready and not game_over:
                    simulation_running = True
                    message = json.dumps([0, 0])  # Reset forces
                    send_sock.sendto(message.encode(), (UDP_IP, SEND_PORT))

        if simulation_running and not game_over:
            try:
                data, addr = recv_sock.recvfrom(1024)
                position, _ = json.loads(data.decode())

                x1_ref_SI = (position - WIDTH // 2) / scale_factor

                x1_dot_SI = (x1_SI - prev_x_SI) / dt
                F = K * (x1_ref_SI - x1_SI) - D * x1_dot_SI

                x1_ddot_SI, theta_ddot_SI = update_physics(theta_SI, theta_dot_SI, x1_dot_SI, F)
                
                x1_dot_SI += x1_ddot_SI * dt
                x1_SI += x1_dot_SI * dt
                theta_dot_SI += theta_ddot_SI * dt
                theta_SI += theta_dot_SI * dt

                current_state = np.array([x1_SI, theta_SI, x1_dot_SI, theta_dot_SI])
                ocp_solver.set(0, "lbx", current_state)
                ocp_solver.set(0, "ubx", current_state)
                status = ocp_solver.solve()
                if status != 0:
                    print("MPC solver returned status", status)
                
                u = ocp_solver.get(0, "u")  # Get optimal control input

                F_send = np.array([-u[0]/80, 0])
                message = json.dumps(F_send.tolist())  # Convert to JSON string
                send_sock.sendto(message.encode(), (UDP_IP, SEND_PORT))

                # state.append([x1_SI, x1_ref_SI, x1_dot_SI, x1_ddot_SI, theta_SI, theta_dot_SI, theta_ddot_SI, F])



                state.append([x1_SI, x1_ref_SI, x1_dot_SI, x1_ddot_SI, theta_SI, theta_dot_SI, theta_ddot_SI, F, u[0]])

                prev_x_SI = x1_SI
            except BlockingIOError:
                pass  # No data available, continue execution
            
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
            time_left = max(15.0 - simulation_time, 0)  # Calculate remaining time

            # Calculate FPS
            true_fps = clock.get_fps()

            # Draw System
            x1_pixels = convert_to_pixels(x1_SI, scale_factor)
            draw_system(x1_pixels + WIDTH // 2, theta_SI, score, high_score, game_over, true_fps, time_left)
            pygame.display.flip()

            clock.tick(FPS)

    pygame.quit()
    plot_states(state, dt)

if __name__ == '__main__':
    main()
