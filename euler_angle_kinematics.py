import numpy as np
import matplotlib.pyplot as plt

def rotationX(theta):
    """
    Create a rotation matrix for rotation around the X-axis.
    
    Args:
        theta: Rotation angle in radians
    
    Returns:
        3x3 rotation matrix
    """
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])

def rotationY(theta):
    """
    Create a rotation matrix for rotation around the Y-axis.
    
    Args:
        theta: Rotation angle in radians
    
    Returns:
        3x3 rotation matrix
    """
    # Fixed: Added missing brackets for np.array
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])
    
def rotationZ(theta):
    """
    Create a rotation matrix for rotation around the Z-axis.
    
    Args:
        theta: Rotation angle in radians
    
    Returns:
        3x3 rotation matrix
    """
    # Fixed: Corrected sign error
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])
    
def rotationEulerXYZ(attitude_xyz):
    """
    Create a rotation matrix from Euler angles using XYZ sequence.
    
    Args:
        attitude_xyz: Array of [phi, theta, psi] in radians
    
    Returns:
        3x3 rotation matrix
    """
    Rx = rotationX(attitude_xyz[0])
    Ry = rotationY(attitude_xyz[1])
    Rz = rotationZ(attitude_xyz[2])
    R = np.matmul(Rx, np.matmul(Ry, Rz))
    return R

def radToDeg(rad):
    """Convert radians to degrees."""
    return np.array(rad) * (180.0/np.pi)

def degToRad(deg):
    """Convert degrees to radians."""
    return np.array(deg) * (np.pi/180.0)

def eulerAngleRatesXYZ(attitude, omega_body):
    """
    Calculate Euler angle rates from body angular velocities.
    
    Args:
        attitude: Array of [phi, theta, psi] in radians
        omega_body: Body angular velocity vector [p, q, r] in rad/s
    
    Returns:
        Euler angle rates [phi_dot, theta_dot, psi_dot]
    """
    phi = attitude[0]
    theta = attitude[1]
    # Transformation matrix from body rates to Euler angle rates
    E = np.array([[1, np.tan(theta)*np.sin(phi), np.tan(theta)*np.cos(phi)],
                  [0, np.cos(phi), -np.sin(phi)],  # Fixed: Sign error
                  [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])  # Fixed: Sign error
    return np.matmul(E, omega_body)

def eulerIntegration(X, Xdot, dt):
    """
    Perform Euler integration (forward integration).
    
    Args:
        X: Current state
        Xdot: State derivative
        dt: Time step
    
    Returns:
        Updated state
    """
    return X + Xdot * dt

# Initial attitude in radians [roll, pitch, yaw]
attitude = degToRad([0.1, 0, 0])

# Body angular velocity in rad/s [p, q, r]
omega_body = degToRad([0, 20, 0])

# Time step
dt = 0.01

# Initialize lists for storing results
time = []
phi = []
theta = []
psi = []

for t in np.arange(0, 20, dt):
    # Calculate Euler angle rates
    attitude_dot = eulerAngleRatesXYZ(attitude, omega_body)
    
    # Integrate to get new attitude
    attitude = eulerIntegration(attitude, attitude_dot, dt)
    
    # Store results
    time.append(t)
    phi.append(radToDeg(attitude)[0])
    theta.append(radToDeg(attitude)[1])
    psi.append(radToDeg(attitude)[2])

# Plot results
plt.figure(figsize=(10, 6))
plt.plot(time, phi, label='Roll (φ)')
plt.plot(time, theta, label='Pitch (θ)')
plt.plot(time, psi, label='Yaw (ψ)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.title('Attitude Evolution Over Time')
plt.legend()
plt.grid(True)
plt.savefig('Attitude.png')
plt.show()
