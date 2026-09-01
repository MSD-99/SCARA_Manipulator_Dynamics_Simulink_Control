# SCARA Robotic Manipulator: Kinematics, Dynamics & Simscape Multibody Control

<div align="center">

[![MATLAB](https://img.shields.io/badge/MATLAB-R2024b%2B-orange.svg?logo=mathworks)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-Simscape_Multibody-blue.svg?logo=mathworks)](https://www.mathworks.com/products/simulink.html)
[![SolidWorks](https://img.shields.io/badge/CAD-SolidWorks_3D-red.svg)](https://www.solidworks.com/)
[![Control Theory](https://img.shields.io/badge/Control-Computed_Torque_PID-emerald.svg)](https://en.wikipedia.org/wiki/Computed_torque_control)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

<br/>

<img src="docs/assets/scara_robot_3d.png" alt="SCARA Robot 3D Multibody Simulation" width="600"/>

<p align="center">
  <b>Physics-based multi-body modeling, Lagrangian dynamic formulation, trajectory planning, and computed torque control for a 4-DOF / 3-DOF SCARA industrial manipulator.</b>
</p>

</div>

---

## 📌 1. Project Overview

The **SCARA (Selective Compliance Assembly Robot Arm)** manipulator is widely utilized in high-speed pick-and-place, assembly, and precise trajectory tracking applications due to its rigid vertical axis and compliant horizontal planes.

This repository presents an end-to-end engineering implementation of a SCARA robotic manipulator:
1. **Mechanical Design & SolidWorks CAD Export:** Complete 3D multi-body assembly including base, links, prismatic joint, and end-effector.
2. **Kinematics Formulation:** Analytical Forward and Inverse Kinematics using Denavit-Hartenberg (D-H) parameterization.
3. **Euler-Lagrange Dynamic Modeling:** Exact closed-form formulation of the inertia matrix $M(q)$, Coriolis/centrifugal matrix $C(q, \dot{q})$, and gravitational vector $G(q)$.
4. **Trajectory Generation:** Real-time generation of smooth quintic and cubic spline joint/task-space reference trajectories.
5. **Computed Torque Control (CTC):** Nonlinear feedback linearization combined with proportional-integral-derivative (PID) tracking to eliminate dynamic coupled nonlinearities.
6. **Simscape Multibody Co-Simulation:** Full physical simulation in MATLAB/Simulink Simscape Multibody validating tracking precision and torque constraints.

---

## 📐 2. Kinematic & Dynamic Architecture

<div align="center">
  <img src="docs/assets/control_schematic_part2.png" alt="Computed Torque Control Block Diagram" width="800"/>
</div>

### D-H Parameterization & Kinematics
The forward kinematics transform relates the base frame to the end-effector position:
$$T_0^n = A_1(	heta_1) \cdot A_2(	heta_2) \cdot A_3(d_3) \cdot A_4(	heta_4)$$

- **Forward Kinematics:** Computes the Cartesian coordinates $(x, y, z, \phi)$ of the end-effector from joint positions $(	heta_1, 	heta_2, d_3, 	heta_4)$.
- **Analytical Inverse Kinematics:** Resolves joint variables analytically via geometric decoupling, ensuring exact solution tracking without numerical drift.

### Euler-Lagrange Dynamic Equations
The robot dynamic equations in joint space are formulated as:
$$M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) + F(\dot{q}) = 	au$$

Where:
- $M(q) \in \mathbb{R}^{n 	imes n}$: Symmetric positive-definite generalized mass/inertia matrix.
- $C(q, \dot{q}) \in \mathbb{R}^{n 	imes n}$: Coriolis and centrifugal torque matrix satisfying the skew-symmetry property $\dot{M}(q) - 2C(q, \dot{q})$.
- $G(q) \in \mathbb{R}^n$: Gravitational torque vector.
- $	au \in \mathbb{R}^n$: Generalized control torque/force input vector.

---

## 🎮 3. Control System Design

<div align="center">
  <img src="docs/assets/control_schematic_part3.png" alt="Advanced Trajectory Control Architecture" width="800"/>
</div>

### Computed Torque Control (Feedback Linearization)
To linearize the coupled nonlinear robotic dynamics, the control law is formulated as:
$$	au = M(q) u + C(q, \dot{q})\dot{q} + G(q)$$

Substituting into the system dynamics yields decoupled double-integrator error dynamics:
$$\ddot{e} + K_v \dot{e} + K_p e + K_i \int e \, dt = 0$$

Where:
- $e(t) = q_d(t) - q(t)$ is the joint tracking error.
- $K_p, K_v, K_i > 0$ are the proportional, derivative, and integral control gain matrices chosen to ensure global asymptotic stability.

---

## 📊 4. Simulation Results & Performance

The complete co-simulation was validated using MATLAB/Simulink and Simscape Multibody across continuous spatial trajectories.

| Metric | Measured Value | Unit |
| :--- | :---: | :---: |
| **Max Trajectory Tracking Error (X-Axis)** | $< 1.2 	imes 10^{-3}$ | $	ext{m}$ |
| **Max Trajectory Tracking Error (Y-Axis)** | $< 1.5 	imes 10^{-3}$ | $	ext{m}$ |
| **Z-Axis Settling Time** | $< 0.15$ | $	ext{s}$ |
| **Steady-State Error ($e_{ss}$)** | $pprox 0$ | $	ext{mm}$ |

### Quantitative Plots

<div align="center">
  <table>
    <tr>
      <td align="center"><b>3D Trajectory Tracking Accuracy</b></td>
      <td align="center"><b>Tracking Error Convergence Curve</b></td>
    </tr>
    <tr>
      <td><img src="docs/assets/tracking_accuracy.png" width="400"/></td>
      <td><img src="docs/assets/tracking_error_curve.png" width="400"/></td>
    </tr>
    <tr>
      <td align="center"><b>X-Z Plane Trajectory</b></td>
      <td align="center"><b>Multi-Axis Tracking Response</b></td>
    </tr>
    <tr>
      <td><img src="docs/assets/xz_trajectory_graph.png" width="400"/></td>
      <td><img src="docs/assets/all_simulation_results.png" width="400"/></td>
    </tr>
  </table>
</div>

---

## 💻 5. Repository Structure

```plaintext
SCARA_Manipulator_Dynamics_Simulink_Control/
├── models/
│   ├── scara_robot_model.slx                      # Base Simscape Multibody physical robot model
│   ├── scara_robot_computed_torque_control.slx    # Computed torque controller with PID outer loop
│   ├── scara_robot_trajectory_tracking.slx       # Full closed-loop 3D trajectory tracking model
│   ├── scara_model_parameters.m                  # Physical parameters, link mass, and inertia tensors
│   ├── scara_data_file1.m                        # Trajectory waypoint data & Simscape parameters
│   └── scara_data_file2.m                        # Joint constraints & simulation configuration
├── docs/
│   └── assets/                                   # High-resolution schematics, 3D CAD renders, & plots
├── .gitignore
└── README.md
```

---

## 🚀 6. Getting Started

### Prerequisites
- **MATLAB & Simulink** (R2022b or later recommended)
- **Simscape & Simscape Multibody** Toolbox
- **Control System Toolbox**

### Execution Steps
1. Clone this repository:
   ```bash
   git clone https://github.com/MSD-99/SCARA_Manipulator_Dynamics_Simulink_Control.git
   cd SCARA_Manipulator_Dynamics_Simulink_Control
   ```
2. Open MATLAB and navigate to the `models/` directory:
   ```matlab
   cd models
   ```
3. Load the robot physical parameters:
   ```matlab
   run('scara_model_parameters.m')
   ```
4. Open and run the trajectory tracking simulation:
   ```matlab
   open_system('scara_robot_trajectory_tracking.slx')
   sim('scara_robot_trajectory_tracking.slx')
   ```
5. View the 3D mechanics animation in the **Simscape Mechanics Explorer**.

---

## 👨‍💻 Author & Research Background

**Mehdi Sadeghian**  
- M.Sc. in Mechatronics Engineering, Tarbiat Modares University (TMU)
- Research Focus: Safe Autonomous Navigation, Deep Reinforcement Learning, Robotic Manipulator Control
- 🌐 Website: [msd-99.github.io](https://msd-99.github.io/)
- 💻 GitHub: [@MSD-99](https://github.com/MSD-99)
- 🔗 LinkedIn: [mehdi-sadeghian](https://www.linkedin.com/in/mehdi-sadeghian)

---

## 📄 License
This project is open-source and available under the [MIT License](LICENSE).
