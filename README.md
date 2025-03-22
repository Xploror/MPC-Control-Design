# MPC-Control-Design
The repository contains various MPC algorithm-based controller design used for solving Quadratic Programming optimization taking care of physical constraints.

`MPC_opt.slx` is the basic simulink model for the Model Predictive Control with a pre-set 5 predictive horizon windows. A fixed wing is simulated with functioning aileron, elevator and rudder control surfaces, however in the simulations, only the 2D static dynamics (x,y,x',y') were simulated (not the case in real-life scenario) with given charecteristic state space:

```math
      A = \begin{bmatrix}
                    -1.9311*10^{-2} & 8.8157 & -32.17 & -0.575\\
                    -2.5389*10^{-4} & -1.0189 & 0 & 0.9051\\
                    2.9465*10^{-12} & 0.8223 & 0 & -1.0774\\
                    0 & 0 & 1 & 0
      \end{bmatrix}\ \ \ \ \ \ \ \ B = \begin{bmatrix} 0.1737\\ -2.1499\\ -0.1756\\ 0 \end{bmatrix}
      \ \ \ \ \ \ \ \
      C = \begin{bmatrix} 0\\ 1\\ 0\\ 0 \end{bmatrix}\ \ \ \ \ \ \ \ \ D = 0
```

The sample reference signal was a mix of ramp and sinusoidal signals as a 1-D value. This reference value were to be tracked by the y-component of the position vector. 

## Sample Examples

<p align="center"> Flat Ramp with Low Frequency sinusoidal signal </p>
<p align="center">
  <img width=460 height=300 src="https://github.com/user-attachments/assets/8e63be16-3ca4-4aa3-ad05-793833733b43">
</p>

<p align="center"> Flat Ramp with High Frequency sinusoidal signal </p>
<p align="center">
  <img width=460 height=300 src="https://github.com/user-attachments/assets/6ce356e9-bfbb-430c-bafe-46d107555410">
</p>

<p align="center"> Ramp with sinusoidal signal </p>
<p align="center">
  <img width=460 height=300 src="https://github.com/user-attachments/assets/2c5a5db4-313e-422a-82aa-88ddf098a579">
</p>

The reference plot (yellow) and the aircraft's relevant position tracked via MPC optimization (blue) seems to have a good agreement for all the three cases. For the ramp with sinusoidal signal case, there is an observable steady state tracking error and this can be minimized by giving higher weights to the state and terminal weight matrix and more freedom to the elevator deflection angle constraint which is by default to 8 degrees.

> IMPORTANT - `MPC_proper_simulation.slx` contians an additional guidance block that can calculate the charecteristic matrices for each timestep and can have various logic to incorporate variable rates of tracking at different stages of the mission for example, cruise and terminal segments would require different performances from the controllers where for cruise, the controls could be comparatively relaxed for considerable room for tracking error however during terminal segment, the tracking is of the highest priority and so controls are allowed to even saturate. (Currently this slx file doesn't have a defined algorithm for guidance (in-progress))


