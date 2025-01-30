# **Madgwick Filter: A Comprehensive Review**

## **1. Overview**
The **Madgwick Filter** is a sensor fusion algorithm designed to estimate **orientation** using data from a **gyroscope, accelerometer, and magnetometer**. It employs **gradient descent optimization** to iteratively refine the quaternion representation of orientation, aligning **measured sensor vectors** with their **expected directions** in the Earth frame.

---

## **2. Gyroscope Update (Orientation from Angular Rate)**

### **Gyroscope Measurement:**
A tri-axis gyroscope measures the angular rate about the x, y, and z axes of the sensor frame. These measurements are arranged into the angular velocity vector:

```math
{}^S \omega = \begin{bmatrix} \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}
```

### **Quaternion Derivative:**
The rate of change of the quaternion representing the orientation of the Earth frame relative to the sensor frame is computed using the angular velocity vector:

```math
{}^E_S\dot{q} = \frac{1}{2} {}^E_S\hat{q} \otimes {}^S \omega
```

where:
- ${}^E_S\hat{q}$ is the quaternion describing the orientation of the Earth frame relative to the sensor frame.
- ${}^S \omega$ is the angular velocity vector measured in the sensor frame.
- $\otimes$ denotes quaternion multiplication.

### **Quaternion Integration:**
To compute the orientation at time $t$, we integrate the quaternion derivative:

```math
{}^E_Sq_t = {}^E_Sq_{t-1} + {}^E_S\dot{q} \Delta t
```

where:
- ${}^E_Sq_{t-1}$ is the previous quaternion estimate.
- $\Delta t$ is the **sampling period**.

This update forms the basis for subsequent calculations using other sensors.

---

## **3. Accelerometer Update (Gravity Vector Alignment)**

### **Accelerometer Measurement:**
The accelerometer measures the gravity vector in the sensor frame, denoted as:

```math
{}^S a = \begin{bmatrix} a_x \\ a_y \\ a_z \end{bmatrix}
```

### **Reference Gravity Vector in the Earth Frame:**
The expected gravity vector in the Earth frame is:

```math
{}^E g = \begin{bmatrix} 0 \\ 0 \\ -1 \end{bmatrix}
```

This vector represents the gravity direction, assuming that gravity points downward in the Earth frame.

### **Objective Function for Accelerometer:**
The algorithm calculates the quaternion ${}^E_Sq$ that aligns the measured gravity vector ${}^S a$ with the expected gravity direction ${}^E g$:

```math
f({}^E_Sq, {}^E g, {}^S a) = {}^E_Sq^* \otimes {}^E g \otimes {}^E_Sq - {}^S a
```

Where:
- ${}^E_Sq^*$ is the quaternion conjugate of ${}^E_Sq$.
- The quaternion multiplication rotates the Earth-frame gravity vector into the sensor frame.
- The goal is to minimize the difference between the predicted and measured gravity vectors.

This alignment error is used to optimize the quaternion.

---

## **4. Magnetometer Update (Magnetic Field Alignment)**

### **Magnetic Field Measurement in the Sensor Frame:**
The magnetometer measures the Earth’s magnetic field in the sensor frame, denoted as:

```math
{}^S m = \begin{bmatrix} m_x \\ m_y \\ m_z \end{bmatrix}
```

### **Reference Magnetic Field in the Earth Frame:**
The Earth’s magnetic field in the Earth frame is modeled as:

```math
{}^E b = \begin{bmatrix} b_x \\ 0 \\ b_z \end{bmatrix}
```

Here, $b_x$ and $b_z$ represent the horizontal and vertical components of the magnetic field.

### **Objective Function for Magnetometer:**
The algorithm calculates the quaternion ${}^E_Sq$ that aligns the measured magnetic field vector ${}^S m$ with the expected direction of the Earth’s magnetic field ${}^E b$:

```math
f({}^E_Sq, {}^E b, {}^S m) = {}^E_Sq^* \otimes {}^E b \otimes {}^E_Sq - {}^S m
```

---

## **5. The Error Function and Quaternion Rotation**

The **rotation of the reference vector into the sensor frame** is computed by:

```math
{}^S v = {}^E_S\hat{q}_k^* \otimes {}^E d \otimes {}^E_S\hat{q}_k
```

where:
- ${}^E_S\hat{q}_k^*$ is the **conjugate of the quaternion** (which inverts the rotation).
- ${}^E d$ is the **expected reference direction in the Earth frame**.
- ${}^E_S\hat{q}_k$ is the **quaternion describing the sensor's orientation**.

This gives the **rotated frame of the corresponding sensor at timestamp \( t-1 \)**.  
The result ${}^S v$ represents how the reference vector should appear in the **sensor frame** if the quaternion is correct.  
The function is then **created by comparing this rotated reference vector to the measured sensor data**.

Thus, the **error function** is defined as:

```math
f({}^E_S\hat{q}_k, {}^E d, {}^S s) = {}^E_S\hat{q}_k^* \otimes {}^E d \otimes {}^E_S\hat{q}_k - {}^S s
```

Since this transformation is done for the **\( t-1 \) timestamp**, the quaternion used here represents the **previous time step's orientation estimate**.

---

## **6. Expansion of the Error Function**

Expanding the error function in terms of quaternion components gives:

```math
f({}^E_S\hat{q}_k, {}^E d, {}^S s) =
\begin{bmatrix}
2 d_x (0.5 - q_3^2 - q_4^2) + 2 d_y (q_1 q_4 + q_2 q_3) - s_x \\
2 d_x (q_2 q_3 - q_1 q_4) + 2 d_y (0.5 - q_2^2 - q_4^2) - s_y \\
2 d_x (q_1 q_3 + q_2 q_4) + 2 d_y (q_3 q_4 - q_1 q_2) - s_z
\end{bmatrix}
```

where:
- $q_1, q_2, q_3, q_4$ are the **quaternion components**.
- $d_x, d_y, d_z$ are the **components of the expected reference direction** ${}^E d$.
- $s_x, s_y, s_z$ are the **components of the measured sensor vector** ${}^S s$.

---
## **7. Computation of the Jacobian** $J$

The **Jacobian matrix** $J({}^E_S\hat{q}_k, {}^E d)$ contains the partial derivatives of the error function \( f \) with respect to the quaternion components:

```math
J({}^E_S\hat{q}_k, {}^E d) =
\begin{bmatrix}
2 d_y q_4 - 2 d_z q_3 & 2 d_y q_3 + 2 d_z q_4 & -2 d_x q_4 + 2 d_z q_2 & 2 d_x q_3 - 4 d_y q_2 + 2 d_z q_1 \\
2 d_x q_3 - 2 d_y q_2 & 2 d_x q_4 - 2 d_y q_1 - 4 d_z q_2 & -4 d_x q_3 + 2 d_y q_2 - 2 d_z q_1 & -4 d_x q_4 + 2 d_y q_1 + 2 d_z q_2 \\
2 d_x q_2 + 2 d_z q_4 & -2 d_x q_1 - 4 d_y q_4 + 2 d_z q_3 & 2 d_x q_1 + 2 d_y q_4 - 4 d_z q_3 & 2 d_x q_2 + 2 d_y q_3
\end{bmatrix}
```

This Jacobian matrix is used to compute the **gradient direction**, which informs the update step.

---

The **objective function** for the accelerometer is:

```math
f({}^E_S\hat{q}, {}^E g, {}^S a) = 
\begin{bmatrix}
2(q_2q_4 - q_1q_3) - a_x \\
2(q_1q_2 + q_3q_4) - a_y \\
2(0.5 - q_2^2 - q_3^2) - a_z
\end{bmatrix}

```

The **Jacobian matrix** for the accelerometer update is:

```math 
J_g({}^E_S\hat{q}) =
\begin{bmatrix}
-2q_3 & 2q_4 & -2q_2 & 2q_1 \\
2q_2 & 2q_1 & 2q_4 & 2q_3 \\
0 & -4q_2 & -4q_3 & 0
\end{bmatrix}

```

The **objective function** for the magnetometer is:

```math 
f({}^E_S\hat{q}, {}^E b, {}^S m) = 
\begin{bmatrix}
2b_x(0.5 - q_3^2 - q_4^2) + 2b_z(q_2q_4 - q_1q_3) - m_x \\
2b_x(q_2q_3 - q_1q_4) + 2b_z(q_1q_2 + q_3q_4) - m_y \\
2b_x(q_1q_3 + q_2q_4) + 2b_z(0.5 - q_2^2 - q_3^2) - m_z
\end{bmatrix}

```

The **Jacobian matrix** for the magnetometer update is:

```math
J_b({}^E_S\hat{q}, {}^E b) =
\begin{bmatrix}
-2b_z q_3 & 2b_z q_4 & -2b_x q_2 & 2b_x q_1 \\
-2b_z q_4 & -2b_z q_3 & 2b_x q_1 & 2b_x q_2 \\
-2b_x q_3 + 2b_z q_2 & -2b_x q_4 + 2b_z q_1 & 2b_x q_1 + 2b_z q_4 & -2b_x q_2 + 2b_z q_3
\end{bmatrix}

```

## **8. Computation of the Gradient** $\nabla f$

The gradient of the error function is computed as:

```math
\nabla f({}^E_S\hat{q}_k, {}^E d, {}^S s) = J^T({}^E_S\hat{q}_k, {}^E d) f({}^E_S\hat{q}_k, {}^E d, {}^S s)
```

where:
- $J({}^E_S\hat{q}_k, {}^E d)$ is the **Jacobian matrix**, which describes how the error function changes with small changes in the quaternion.
- $f({}^E_S\hat{q}_k, {}^E d, {}^S s)$ is the **error function**, which measures the misalignment between the transformed reference direction and the measured sensor vector.

The **Jacobian matrix** enables efficient gradient computation for quaternion updates.

---

## **9. Combined Orientation Update from Accelerometer and Magnetometer**

As discussed, the measurement of gravity or the Earth’s magnetic field alone will not provide a unique orientation of the sensor. To do so, the measurements and reference directions of both fields may be combined as described by equations (31) and (32). 

Whereas the solution surface created by the objective functions in equations (25) and (29) have a minimum defined by a line, the solution surface defined by equation (31) has a minimum defined by a single point, provided that \(b_z \neq 0\).

The **objective function** for the combined accelerometer and magnetometer update is:

```math
f_{g,b}({}^E_S\hat{q}, {}^S a, {}^S m) =
\begin{bmatrix}
f_g({}^E_S\hat{q}, {}^S a) \\
f_b({}^E_S\hat{q}, {}^E b, {}^S m)
\end{bmatrix}
```

### **Gradient Update Rule**

The **gradient** of the error function is computed as:

```math
\nabla f = 
\begin{cases} 
J_g^T({}^E_S\hat{q}_{\text{est},t-1}) f_g({}^E_S\hat{q}_{\text{est},t-1}, {}^S a_t) \\
J_b^T({}^E_S\hat{q}_{\text{est},t-1}, {}^E b) f_b({}^E_S\hat{q}_{\text{est},t-1}, {}^E b, {}^S m_t)
\end{cases}
```

### **Explanation**:
- The **gradient $nabla f$** is a combination of the **accelerometer** and **magnetometer** updates.
- The **Jacobian matrices $J_g$ and $J_b$ ** are transposed and then multiplied with the corresponding error functions $f_g$ and $f_b$ to obtain the overall gradient.
- $\mu_t$ is the step-size for the update, and it ensures the convergence of the quaternion estimate towards the correct orientation.

The gradient descent update for the quaternion at time $t$ is then given by:

```math
{}^E_Sq_{t} = {}^E_Sq_{\text{est},t-1} - \mu_t \frac{\nabla f}{\|\nabla f\|}
```

## **10. Quaternion Update Rule**

Using the computed **gradient** $ \nabla f $, the **quaternion update** is:

```math
{}^E_S\hat{q}_{k+1} = {}^E_S\hat{q}_k - \mu \frac{\nabla f({}^E_S\hat{q}_k, {}^E d, {}^S s)}{\|\nabla f({}^E_S\hat{q}_k, {}^E d, {}^S s)\|}
```




This iterative process ensures that the **quaternion estimate gradually aligns with the correct orientation**.

---

## **11. Summary**

✅ The **Madgwick Filter** estimates orientation using **gradient descent**.  
✅ It aligns **measured sensor vectors (accelerometer, magnetometer)** with their **expected Earth-frame directions**.  
✅ The quaternion is iteratively updated using:
   - The **error function** $f$.
   - The **Jacobian matrix** $J$.
   - **Gyroscope integration**.  
✅ The quaternion transformation is computed for the **\( t-1 \) timestamp** and applied to update the current estimate.

---

This updated version now follows a **logical procedural flow**, where each step defines the next formulation. It also includes **inline math expressions** for easy understanding on GitHub. Let me know if you need further revisions! 🚀
