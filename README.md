# Attitude Simulation with Euler Angles (XYZ) and Body Rates

This script simulates the evolution of an object's attitude (orientation) over time using Euler angles in the **XYZ (roll–pitch–yaw)** convention, given constant body angular rates. It numerically integrates the Euler angle kinematics and plots roll, pitch, and yaw versus time.

## Description

The code:

1. Defines basic 3D rotation matrices about the X, Y, and Z axes.
2. Builds a combined rotation matrix from **XYZ Euler angles**.
3. Implements the kinematic relationship between **body angular rates** `[p, q, r]` and **Euler angle rates** `[φ̇, θ̇, ψ̇]` for the XYZ convention.
4. Uses simple forward Euler integration to propagate the attitude over time.
5. Plots roll (φ), pitch (θ), and yaw (ψ) in degrees as functions of time.

The default simulation case:

- Initial attitude (in degrees):

  `attitude = degToRad([0, 0, 0])   # [roll φ, pitch θ, yaw ψ]`

- Constant body rates (in deg/s):

  `omega_body = degToRad([0, 0, 5]) # [p, q, r]`

- Simulation time: `0` to `20` seconds  
- Time step: `dt = 0.01 s`

### Customizing the Simulation

You can **easily change the initial conditions and angular rates**:

- To start from a different attitude, edit:

  `attitude = degToRad([phi_deg, theta_deg, psi_deg])`  
  Example:  
  `attitude = degToRad([10, 5, -20])`

- To use different body angular rates, edit:

  `omega_body = degToRad([p_deg_s, q_deg_s, r_deg_s])`  
  Example:  
  `omega_body = degToRad([1, 2, 0])  # 1°/s roll rate, 2°/s pitch rate`

These two variables (`attitude` and `omega_body`) control the initial orientation and how the body rotates over time.

## Requirements

- Python 3.x
- NumPy
- Matplotlib

To install dependencies:

`pip install numpy matplotlib`

## Main Functions

- `rotationX(theta)`  
  Returns 3×3 rotation matrix for a rotation about the X-axis by `theta` (radians).

- `rotationY(theta)`  
  Returns 3×3 rotation matrix for a rotation about the Y-axis by `theta` (radians).

- `rotationZ(theta)`  
  Returns 3×3 rotation matrix for a rotation about the Z-axis by `theta` (radians).

- `rotationEulerXYZ(attitude_xyz)`  
  Builds the overall rotation matrix from Euler angles `[φ, θ, ψ]` (radians) using the XYZ sequence.

- `radToDeg(rad)` / `degToRad(deg)`  
  Convert between radians and degrees (vectorized using NumPy).

- `eulerAngleRatesXYZ(attitude, omega_body)`  
  Computes Euler angle rates `[φ̇, θ̇, ψ̇]` from:
  - `attitude = [φ, θ, ψ]` (radians)
  - `omega_body = [p, q, r]` body angular rates (rad/s)  
  using the standard XYZ kinematic relationship.

- `eulerIntegration(X, Xdot, dt)`  
  Simple forward Euler integrator: `X_new = X + Xdot * dt`.

## Usage

1. Place the script in a `.py` file (for example, `attitude_simulation.py`).
2. Optionally edit `attitude` and `omega_body` to set your desired initial orientation and body rates.
3. Run it with Python:

   `python attitude_simulation.py`

4. The script will:
   - Simulate the attitude over 20 seconds.
   - Save a figure named `Attitude.png` showing roll, pitch, and yaw vs. time.
   - Optionally display the plot window if `plt.show()` is called in the script.

## Output

- **Plot**: `Attitude.png`  
  Contains three curves:
  - Roll (φ) in degrees
  - Pitch (θ) in degrees
  - Yaw (ψ) in degrees  
  all plotted against time in seconds.

## Notes

- All internal computations for rotations and rates use **radians**.
- `degToRad` / `radToDeg` are used only for user-facing values (initial conditions, plots).
- The transformation from body rates to Euler angle rates assumes the XYZ (roll–pitch–yaw) Euler angle convention and the standard aerospace right-hand rule.

## Possible Extensions

- Change `omega_body` to simulate different motion (for example, non-zero roll or pitch rates).
- Change the initial `attitude` to see how different starting orientations evolve.
- Use more advanced integration methods (for example, Runge–Kutta) for better numerical accuracy.
- Compare Euler angle integration with quaternion-based attitude integration.
