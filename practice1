import numpy as np
from scipy.integrate import solve_ivp


class Car:
    def __init__(self, length=2.3, velocity=5, x=0, y=0, theta=0):
        self.length = length
        self.velocity = velocity
        self.x = x
        self.y = y
        self.theta = theta

    def move(self, steering_angle_rad, dt):
        """
        This function computes and updates the new position and
        orientation of the car if we apply the given steering action
        for a time "dt"
        """

        def system_dynamics(t, z):
            theta = z[2]
            return [self.velocity * np.cos(theta),
                    self.velocity * np.sin(theta),
                    self.velocity * np.tan(steering_angle_rad) / self.length]

        # next, we need to solve the IVP:
        z_initial = [self.x, self.y, self.theta]
        solution = solve_ivp(system_dynamics,
                             [0, dt],
                             z_initial)
        self.x = solution.y[0][-1]
        self.y = solution.y[1][-1]
        self.theta = solution.y[2][-1]


murphy = Car()
murphy.move(0.01, 0.1)
print(murphy.theta)
