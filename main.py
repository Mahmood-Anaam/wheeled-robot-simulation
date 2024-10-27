import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from wheeledRobot import *


# Parameters

r = 0.01  # Wheel radius [m]
a = 0.5   # Position along y-axis [m]
b = 0.2   # Position along x-axis [m]
phi_dot = np.array([[10],
                    [10],
                    [10],
                    [10]])  # Wheel angular velocities [rad/s]

F = np.array([[1.0],
              [1.0],
              [1.0],
              [1.0]])  # Forces on wheels [N]
m = 6.0   # Mass [kg]
I = 0.25  # Moment of inertia [kg*m^2]

xi_init = np.array([[0],
                    [0],
                    [0]])  # Initial pose [x, y, theta]
zeta_init = np.array([[0],
                      [0],
                      [0]])  # Initial velocities [u, v, omega]

t_end = 30.0  # Simulation end time [s]
dt = 1.0  # Time step [s]


if __name__ == '__main__':
    # Simulate kinematics
    xi_history_kinematics = simulate_kinematics(r, a, b, phi_dot, xi_init, t_end, dt)
    print("Kinematics Simulation:")
    drawRobot(xi_history_kinematics)
   

    # Simulate dynamics
    xi_history_dynamics = simulate_dynamics(F, a, b, m, I, xi_init, zeta_init, t_end, dt)
    print("Dynamics Simulation:")
    drawRobot(xi_history_dynamics)
 

    # Answer circle trajectory question
    print("Circle Trajectory Wheel Velocities:")
    phi_dot_circle = answer_circle_trajectory()

    # Simulate kinematics with circle trajectory
    xi_history_circle = simulate_kinematics(r, a, b, phi_dot_circle, xi_init, t_end, dt)
    print("Circular Trajectory Simulation:")
    drawRobot(xi_history_circle)
  

    # Answer wheel wear question
    print("Wheel Wear Effect:")
    answer_wheel_wear()

   
