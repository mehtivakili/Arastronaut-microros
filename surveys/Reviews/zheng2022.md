- ![zheng2022.pdf](../assets/zheng2022_1737703329848_0.pdf)
- ### UWB Model
  
  **Equation**:  
$$
  d_{A^n T^m} = \| {}^W\mathbf{P}_{A^n} - ({}^W\mathbf{P}_B + {}^W\mathbf{R}_B {}^B\mathbf{P}_{T^m}) \| + N(0, \sigma)
  $$  
  **Description**:  
  This equation describes the distance measurement between a UWB anchor $A^n$ and a UWB tag $T^m$ . The distance $d_{A^n T^m}$ is calculated as the Euclidean norm of the difference between the anchor's position in the world frame:  
$$
  {}^W\mathbf{P}_{A^n}
  $$  
  and the estimated position of the tag:  
$$
  {}^W\mathbf{P}_B + {}^W\mathbf{R}_B {}^B\mathbf{P}_{T^m}.
  $$  
  The measurement includes Gaussian noise $N(0, \sigma)$ .  
  
---
- ### IMU Pre-Integration Model
- #### Rotation Error
  
  **Equation**:  
$$
  e_R = \text{Log}(\Delta {}^W\mathbf{R}_{i,j}^{-1} {}^W\mathbf{R}_{B}^i {}^B\mathbf{R}_{B}^j)
  $$  
  **Description**:  
  The rotation error $e_R$ represents the difference between the pre-integrated rotation measurement:  
$$
  \Delta {}^W\mathbf{R}_{i,j}
  $$  
  and the relative rotation computed from the estimated orientations of the body at timestamps $i$ and $j$ . The $\text{Log}$ operator maps the rotation matrix error to the Lie algebra:  
$$
  \text{Log}(\Delta {}^W\mathbf{R}_{i,j}^{-1} {}^W\mathbf{R}_{B}^i {}^B\mathbf{R}_{B}^j).
  $$  
---
- #### Velocity Error
  
  **Equation**:  
$$
  e_v = {}^W\mathbf{R}_B^i \left( {}^W\mathbf{v}_B^j - {}^W\mathbf{v}_B^i - \mathbf{g} \Delta t_{i,j} \right) - \Delta {}^B\mathbf{v}_{i,j}
  $$  
  **Description**:  
  The velocity error $e_v$ quantifies the discrepancy between the IMU pre-integrated velocity:  
$$
  \Delta {}^B\mathbf{v}_{i,j}
  $$  
  and the velocity computed from the body frame's velocity at timestamps $i$ and $j$ , adjusted for the effects of gravity $\mathbf{g}$ and the time interval $\Delta t_{i,j}$ :  
$$
  {}^W\mathbf{R}_B^i \left( {}^W\mathbf{v}_B^j - {}^W\mathbf{v}_B^i - \mathbf{g} \Delta t_{i,j} \right).
  $$  
---
- #### Position Error
  
  **Equation**:  
$$
  e_p = {}^W\mathbf{R}_B^i \left( {}^W\mathbf{P}_B^j - {}^W\mathbf{P}_B^i - {}^W\mathbf{v}_B^i \Delta t_{i,j} - \frac{1}{2} \mathbf{g} \Delta t_{i,j}^2 \right) - \Delta {}^B\mathbf{P}_{i,j}
  $$  
  **Description**:  
  The position error $e_p$ evaluates the mismatch between the pre-integrated position measurement:  
$$
  \Delta {}^B\mathbf{P}_{i,j}
  $$  
  and the position calculated from the body position at timestamps $i$ and $j$ , accounting for velocity and gravity effects over the time interval:  
$$
  {}^W\mathbf{R}_B^i \left( {}^W\mathbf{P}_B^j - {}^W\mathbf{P}_B^i - {}^W\mathbf{v}_B^i \Delta t_{i,j} - \frac{1}{2} \mathbf{g} \Delta t_{i,j}^2 \right).
  $$  
---
- ### UWB/IMU Fusion Model
- #### UWB Error Term
  
  **Equation**:  
$$
  E_{\text{UWB}} = \rho(e_{A T}^T \Sigma_U^{-1} e_{A T})
  $$  
  **Error Vector**:  
$$
  e_{A^n T^m} = \| {}^W\mathbf{P}_{A^n} - ({}^W\mathbf{P}_B + {}^W\mathbf{R}_B {}^B\mathbf{P}_{T^m}) \| - d_{A^n T^m}
  $$  
  **Description**:  
  The UWB error term $E_{\text{UWB}}$ represents the sum of squared residuals from UWB range measurements. The residual $e_{A^n T^m}$ is the difference between the measured distance $d_{A^n T^m}$ and the estimated distance:  
$$
  \| {}^W\mathbf{P}_{A^n} - ({}^W\mathbf{P}_B + {}^W\mathbf{R}_B {}^B\mathbf{P}_{T^m}) \| - d_{A^n T^m}.
  $$  
  The term is weighted by the UWB measurement information matrix $\Sigma_U^{-1}$ and robustified using a robust cost function $\rho$ .  
  
---
- #### IMU Error Term
  
  **Equation**:  
$$
  E_{\text{IMU}} = \rho([e_R, e_v, e_p] \Sigma_I [e_R, e_v, e_p]^T)
  $$  
  **Description**:  
  The IMU error term $E_{\text{IMU}}$ aggregates the errors in rotation, velocity, and position:  
$$
  [e_R, e_v, e_p].
  $$  
  These errors are weighted by the information matrix $\Sigma_I$ and robustified using the robust cost function $\rho$ .  
  
---
- #### Plane Motion Constraint (Optional)
  
  **Equation**:  
$$
  e_{\text{plane}} =
  \begin{bmatrix}
  {}^W\mathbf{P}_B^z - z_{\text{fixed}} \\
  {}^W\mathbf{R}_B [0, 0, 1]^T - [0, 0, 1]^T
  \end{bmatrix}
  $$  
  **Description**:  
  The plane motion constraint enforces that the body moves in a 2D plane. The $z$ -coordinate of the body:  
$$
  {}^W\mathbf{P}_B^z
  $$  
  is fixed at a known height $z_{\text{fixed}}$ , and the orientation is constrained to allow only yaw rotation:  
$$
  {}^W\mathbf{R}_B [0, 0, 1]^T - [0, 0, 1]^T.
  $$  
---
- ### Total Fusion Optimization Objective
  
  **Equation**:  
$$
  E_{\text{total}} = \sum (E_{\text{UWB}} + E_{\text{IMU}} + E_{\text{constraints}})
  $$  
  **Description**:  
  The total error function $E_{\text{total}}$ combines the UWB measurement error:  
$$
  E_{\text{UWB}},
  $$  
  the IMU pre-integration error:  
$$
  E_{\text{IMU}},
  $$  
  and optional constraints. The goal is to minimize this function using a graph-based optimization algorithm like Gauss-Newton or Levenberg-Marquardt, yielding the best estimates of the body’s position, velocity, and orientation.