import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb



def wheel_matrix(r, a, b) -> np.ndarray :
    """
    This function returns the wheel matrix of the robot with 4 Swedish wheels.
    Refer to the picture of the robot in wheeledRobot.pdf.

    Parameters
    ----------
    r : float
        radius of the wheel [m]
    a : float
        position of the wheel from the center of the robot along the y-axis [m]
    b : float
        position of the wheel from the center of the robot along the x-axis [m]

    Returns
    -------
    W
        the wheel matrix of the robot as a numpy array
    """
    # Angles of the rollers for each wheel in radians (±45 degrees)
    beta = np.array([-45, 45, -45, 45]) * np.pi / 180  # Convert degrees to radians

    # Positions of the wheels (x_i, y_i)
    positions = np.array([
        [-b, -a],
        [-b,  a],
        [ b,  a],
        [ b, -a]
    ])  # Shape: (4, 2)

    # Compute L_i = x_i * sin(beta_i) - y_i * cos(beta_i)
    L = positions[:, 0] * np.sin(beta) - positions[:, 1] * np.cos(beta)

    # Wheel matrix W
    W = (1 / r) * np.array([
        [np.cos(beta[0]), np.sin(beta[0]), L[0]],
        [np.cos(beta[1]), np.sin(beta[1]), L[1]],
        [np.cos(beta[2]), np.sin(beta[2]), L[2]],
        [np.cos(beta[3]), np.sin(beta[3]), L[3]],
    ])

    return W


def simulate_kinematics(r:float, a:float, b:float, phi_dot:np.ndarray, xi_init:np.ndarray, t_end:float, dt:float) -> np.ndarray:
    """
    This function simulates the kinematics of a robot with 4 Swedish wheels.
    Refer to the picture of the robot in wheeledRobot.pdf.

    Parameters
    ----------
    r : float
        radius of the wheel [m]
    a : float
        position of the wheel from the center of the robot along the y-axis [m]
    b : float
        position of the wheel from the center of the robot along the x-axis [m]
    phi_dot : np.ndarray
        angular velocities of the wheels as a numpy array of shape (4,1) [rad/s]
    xi_init : np.ndarray
        initial pose of the robot as a numpy array of shape (3,1) [x, y, theta] [m, m, rad]
    t_end : float
        end time of simulation [s]
    dt : float
        time step [s]

    Returns
    -------
    xi_history
        the history of the robot pose as a numpy array of shape (3,N) [x, y, theta] [m, m, rad]
    """

    # get wheel matrix
    W = wheel_matrix(r, a, b)


    W_pinv = np.linalg.pinv(W)
    zeta = W_pinv @ phi_dot  

    xi = xi_init
    xi_history = xi
    time = 0
    time+=dt
    while time < t_end:
        theta = xi[2, 0]
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])
        xi_dot = R @ zeta  

        # update xi
        xi = xi + xi_dot * dt
        xi_history = np.concatenate((xi_history, xi), axis=1)
        time += dt

    return xi_history


def compute_forces_on_chassis(F:np.ndarray, a:float, b:float) -> np.ndarray:
    """
    This function computes the forces on the chassis of the robot given the forces on the wheels.
    Refer to the picture of the robot in wheeledRobot.pdf.

    Parameters
    ----------
    F : np.ndarray
        forces on the wheels as a numpy array of shape (4,1) [N]
    a : float
        position of the wheel from the center of the robot along the y-axis [m]
    b : float
        position of the wheel from the center of the robot along the x-axis [m]

    Returns
    -------
    np.array
        forces on the chassis as a numpy array of shape (3,1) [C_f_x, C_f_y, C_n_z]
    """

   # Positions of wheels relative to robot center
    positions = np.array([
        [-b, -a],
        [-b,  a],
        [ b,  a],
        [ b, -a]
    ])  # Shape: (4, 2)

    # Angles of the rollers for each wheel in radians (±45 degrees)
    beta = np.array([-45, 45, -45, 45]) * np.pi / 180

    # Direction vectors of forces at wheels
    force_dirs = np.vstack((np.cos(beta), np.sin(beta))).T  # Shape: (4, 2)

    # Initialize net force and torque
    net_force = np.zeros(2)
    net_torque = 0

    # Calculate net force and torque
    for i in range(4):
        F_i = F[i, 0] * force_dirs[i]  # Force vector at wheel i
        net_force += F_i
        r_i = positions[i]  # Position vector of wheel i
        torque_i = np.cross(r_i, F_i)  # Torque contributed by wheel i
        net_torque += torque_i

    C_f_x = net_force[0] 
    C_f_y = net_force[1]  
    C_n_z = net_torque 

    return np.array([[C_f_x], [C_f_y], [C_n_z]])


def simulate_dynamics(F:np.ndarray, a:float, b:float, m:float, I:float, xi_init:np.ndarray, zeta_init:np.array, t_end:float, dt:float) -> np.ndarray:
    """
    This function simulates the dynamics of a robot with 4 Swedish wheels.
    Refer to the picture of the robot in wheeledRobot.pdf.

    Parameters
    ----------
    F : np.ndarray
        forces on the wheels as a numpy array of shape (4,1) [N]
    a : float
        position of the wheel from the center of the robot along the y-axis [m]
    b : float
        position of the wheel from the center of the robot along the x-axis [m]
    m : float
        mass of the robot [kg]
    I : float
        moment of inertia of the robot [kg m^2]
    xi_init : np.ndarray
        initial pose of the robot as a numpy array of shape (3,1) [x, y, theta] [m, m, rad]
    zeta_init : np.array
        initial velocities of the robot as a numpy array of shape (3,1) [u, v, theta_dot] [m/s, m/s, rad/s]
    t_end : float
        end time of simulation [s]
    dt : float
        time step [s]

    Returns
    -------
    xi_history
        the history of the robot pose as a numpy array of shape (3,N) [x, y, theta] [m, m, rad]
    """

    Q = compute_forces_on_chassis(F, a, b)

    M = np.diag([m, m, I])

    xi = xi_init
    zeta = zeta_init
    xi_history = xi

    time = 0
    time += dt

    while time < t_end:
        
        zeta_dot = np.linalg.inv(M) @ Q 
        # Update velocities
        zeta = zeta + zeta_dot * dt

        # Get current orientation
        theta = xi[2, 0]

        # Rotation matrix from robot frame to global frame
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])


        xi_dot = R @ zeta  

        # update xi
        xi = xi + xi_dot * dt
        xi_history = np.concatenate((xi_history, xi), axis=1)
        time += dt

    return xi_history


def answer_circle_trajectory():
    # Parameters
    r = 0.01  # Wheel radius [m]
    a = 0.5   # Position along y-axis [m]
    b = 0.2   # Position along x-axis [m]

    # Desired linear and angular velocities
    v = 1.0      # Linear speed [m/s]
    R_circle = 2.0  # Radius of circle [m]
    omega = v / R_circle  # Angular speed [rad/s]

    # Robot velocities in robot frame
    zeta = np.array([[v], [0], [omega]])  # [u; v; omega]

    # Wheel matrix
    W = wheel_matrix(r, a, b)

    # Compute wheel velocities
    phi_dot = W @ zeta

    # Print the wheel velocities
    print("A suitable set of wheel velocities to make the robot move in a circle is:")
    print(f"phi_dot = np.array([[95.45941546],[45.96194078],[45.96194078],[95.45941546]])")
    return phi_dot


def answer_wheel_wear():
    print("Initially, with equal forces of 1N from all four wheels, the robot moves in a straight line on the moon's surface.")
    print("After one wheel experiences a 0.01% reduction in force due to wear, an unbalanced torque is introduced.")
    print("This causes the robot to gradually deviate from its straight path and begin to follow a slight curved trajectory.")


############################################################################################################
# helper function to visualize the robot
def drawRobot(xi_history):
    """
    This function draws the robot at different poses.
    It places a blue dot at the center of the robot and an arrow to indicate the orientation of the robot.

    Parameters
    ----------
    xi_history : np.ndarray
        the history of the robot pose as a numpy array of shape (3,N) [x, y, theta] [m, m, rad]

    Returns
    -------
    None
    """

    plt.figure()
    plt.plot(xi_history[0, :], xi_history[1, :], 'b')
    plt.quiver(xi_history[0, :], xi_history[1, :], np.cos(xi_history[2, :]), np.sin(xi_history[2, :]))
    plt.axis('equal')
    plt.show()
