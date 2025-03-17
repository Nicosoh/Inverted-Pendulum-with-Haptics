from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from pendulum_model import export_pendulum_ode_model
from casadi import vertcat
import numpy as np
from scipy.linalg import block_diag  # Block-diagonal weight matrix

class MPCController:
    def __init__(self, M, m, g, l, dt, d_cart, d_theta, Fmax, Tf, N_horizon):
        x0 = np.array([0.0,np.pi,0.0,0.0])
        self.M, self.m, self.g, self.l = M, m, g, l
        self.dt, self.d_cart, self.d_theta = dt, d_cart, d_theta
        self.Fmax, self.Tf, self.N_horizon = Fmax, Tf, N_horizon
        self.setup_mpc(x0)

    def setup_mpc(self, x0):
        # Create the OCP object
        ocp = AcadosOcp()

        # Set the model (pendulum)
        model = export_pendulum_ode_model(self.M, self.m ,self.g, self.l, self.d_cart, self.d_theta)

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
        ocp.constraints.lbu = np.array([-self.Fmax])
        ocp.constraints.ubu = np.array([+self.Fmax])
        ocp.constraints.idxbu = np.array([0])
        ocp.constraints.x0 = x0

        # Set prediction horizon
        ocp.solver_options.N_horizon = self.N_horizon
        ocp.solver_options.tf = self.Tf

        # Solver options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'IRK'
        ocp.solver_options.sim_method_newton_iter = 10
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.qp_solver_cond_N = self.N_horizon

        # Save to a JSON file (required by acados)
        solver_json = 'acados_ocp_' + model.name + '.json'
        self.ocp_solver = AcadosOcpSolver(ocp, json_file=solver_json)
        self.integrator = AcadosSimSolver(ocp, json_file=solver_json)


    def compute_control(self, state):
        self.ocp_solver.set(0, "lbx", state)
        self.ocp_solver.set(0, "ubx", state)
        status = self.ocp_solver.solve()
        if status != 0:
            print("MPC solver returned status", status)
        
        return self.ocp_solver.get(0, "u")