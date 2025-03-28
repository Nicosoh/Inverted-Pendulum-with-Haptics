#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_pendulum_ode_model(M, m ,g, l, d_cart, d_theta) -> AcadosModel:

    model_name = 'pendulum_ode'

    # set up states & controls
    x1      = SX.sym('x1')
    theta   = SX.sym('theta')
    v1      = SX.sym('v1')
    dtheta  = SX.sym('dtheta')

    x = vertcat(x1, theta, v1, dtheta)

    F = SX.sym('F')
    u = vertcat(F)

    # xdot
    x1_dot      = SX.sym('x1_dot')
    theta_dot   = SX.sym('theta_dot')
    v1_dot      = SX.sym('v1_dot')
    dtheta_dot  = SX.sym('dtheta_dot')

    xdot = vertcat(x1_dot, theta_dot, v1_dot, dtheta_dot)

    # dynamics
    cos_theta = cos(theta)
    sin_theta = sin(theta)
    denominator = M + m - m * cos_theta ** 2
    # denominator = M + m - m * sin_theta ** 2
    f_expl = vertcat(v1,
                    dtheta,
                    (-m * g * cos_theta * sin_theta + m * l * theta_dot**2 * sin_theta - d_cart * x1_dot + F) / denominator,
                    ((m + M ) * g * sin_theta - cos_theta * (m * l * theta_dot**2 * sin_theta - d_cart * x1_dot) + cos_theta * F - theta_dot * d_theta) / denominator / l
                    )
    
    # f_expl = vertcat(v1,
    #                 dtheta,
    #                 (-m * l * sin_theta * theta_dot ** 2 + m * g * cos_theta * sin_theta + F) / denominator,
    #                 (-m * l * cos_theta * sin_theta * theta_dot ** 2 + F * cos_theta + (M + m) * g * sin_theta) / (l * denominator)
    #                 )

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    # store meta information
    model.x_labels = ['$x$ [m]', r'$\theta$ [rad]', '$v$ [m]', r'$\dot{\theta}$ [rad/s]']
    model.u_labels = ['$F$']
    model.t_label = '$t$ [s]'

    return model