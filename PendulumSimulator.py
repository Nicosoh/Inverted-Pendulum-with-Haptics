import numpy as np

class PendulumSimulator:
    def __init__(self, M, m, g, l, dt, d_cart, d_theta):
        self.M, self.m, self.g, self.l = M, m, g, l
        self.dt, self.d_cart, self.d_theta = dt, d_cart, d_theta
        self.reset()
    
    def reset(self):
        self.x, self.x_dot, self.x_ddot = 0.0, 0.0, 0.0
        self.theta, self.theta_dot, self.theta_ddot = np.pi, 0.0, 0.0

    
    def update_physics(self, F):
        cos_theta = np.cos(self.theta)
        sin_theta = np.sin(self.theta)
        denominator = self.M + self.m - self.m * cos_theta ** 2

        theta_ddot = ((self.m + self.M ) * self.g * sin_theta - cos_theta * 
                      (self.m * self.l * self.theta_dot**2 * sin_theta - self.d_cart * self.x_dot) + 
                      cos_theta * F - self.theta_dot * self.d_theta) / denominator / self.l
        
        x_ddot = (-self.m * self.g * cos_theta * sin_theta + self.m * self.l * self.theta_dot**2 * 
                   sin_theta - self.d_cart * self.x_dot + F) / denominator
        
        self.x_dot += x_ddot * self.dt
        self.x += self.x_dot * self.dt
        self.theta_dot += theta_ddot * self.dt
        self.theta += self.theta_dot * self.dt