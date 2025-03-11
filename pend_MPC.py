import pygame
import pygame.freetype  # For FPS text rendering
import numpy as np
from pygame.locals import *

# --- Acados and MPC Imports ---
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from pendulum_model import export_pendulum_ode_model
from casadi import vertcat
# from utils import plot_pendulum  # not used in realtime simulation

# --- Pygame & Simulator Settings ---
# Physical constants and simulation parameters (SI units)
dt = 0.01                # time step (s)
scale_factor = 100       # 1 meter = 100 pixels
cart_y_coord = 700       # fixed vertical position for drawing the cart

# Pygame window dimensions and FPS
WIDTH = 1000
HEIGHT = 1000
FPS = int(1/dt)          # e.g. 100 FPS

# --- MPC Setup Function using acados ---
def setup_mpc(x0, Fmax, N_horizon, Tf, RTI=False):
    # Create the OCP object
    ocp = AcadosOcp()

    # Set the model (pendulum)
    model = export_pendulum_ode_model()
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    # Set cost module (using a nonlinear least-squares cost)
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    Q_mat = 2 * np.diag([1e3, 1e3, 1e-2, 1e-2])
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

# --- Drawing Function ---
def convert_to_pixels(value, scale_factor):
    return value * scale_factor

def draw_system(x1_pixel, theta):
    """Draw the cart-pendulum system."""
    rod_length_pixel = convert_to_pixels(1.0, scale_factor)  # l = 1.0 m
    cart_width_pixel = convert_to_pixels(0.5, scale_factor)
    cart_height_pixel = convert_to_pixels(0.3, scale_factor)
    
    # Pendulum bob position
    pendulum_x = x1_pixel - rod_length_pixel * np.sin(theta)
    pendulum_y = cart_y_coord + rod_length_pixel * np.cos(theta)

    # Draw the pendulum rod
    pygame.draw.line(screen, (0, 0, 0), (x1_pixel, cart_y_coord), (pendulum_x, pendulum_y), 3)
    # Draw the pendulum bob
    pygame.draw.circle(screen, (255, 0, 0), (int(pendulum_x), int(pendulum_y)), 10)
    # Draw the cart
    pygame.draw.rect(screen, (0, 0, 255), (x1_pixel - cart_width_pixel/2, cart_y_coord, cart_width_pixel, cart_height_pixel))

# --- Main Simulation Loop ---
def main():
    x0 = np.array([0.0, 0.0, 0.0, 0.0])  # Initial state
    Fmax = 80
    Tf = 0.6
    N_horizon = 30

    # Setup MPC using acados
    ocp_solver, integrator = setup_mpc(x0, Fmax, N_horizon, Tf, RTI=True)
    
    # Initialize simulation state
    sim_state = x0.copy()  # [x, theta, x_dot, theta_dot]

    # Pygame initialization
    pygame.init()
    pygame.freetype.init()  # Initialize font module
    global screen, clock  # so draw_system can use them
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.freetype.SysFont("Arial", 24)  # Font for FPS display

    running = True
    while running:
        # Event handling
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    sim_state = x0.copy()
                    print("Simulation reset!")
                if event.key == pygame.K_g:
                    sim_state[1] -= np.pi/6  # Apply counterclockwise disturbance
                    print("Theta decreased:", sim_state[1])
                if event.key == pygame.K_h:
                    sim_state[1] += np.pi/6  # Apply clockwise disturbance
                    print("Theta increased:", sim_state[1])
                if event.key == pygame.K_q:
                    running = False

        # --- Solve MPC ---
        ocp_solver.set(0, "lbx", sim_state)
        ocp_solver.set(0, "ubx", sim_state)
        status = ocp_solver.solve()
        if status != 0:
            print("MPC solver returned status", status)
        u = ocp_solver.get(0, "u")  # Get optimal control input

        # --- Simulate using acados integrator ---
        sim_state = integrator.simulate(x=sim_state, u=u)

        # Convert cart position to pixels
        x1_pixel = convert_to_pixels(sim_state[0], scale_factor) + WIDTH / 2

        # Draw system
        screen.fill((255, 255, 255))  # Clear screen
        draw_system(x1_pixel, sim_state[1])

        # Display FPS
        true_fps = clock.get_fps()
        font.render_to(screen, (10, 10), f"FPS: {true_fps:.1f}", (0, 0, 0))

        pygame.display.flip()  # Update screen
        clock.tick(FPS)  # Maintain 100 FPS

    pygame.quit()

if __name__ == '__main__':
    main()
