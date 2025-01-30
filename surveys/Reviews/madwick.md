# Madgwick Filter: A Comprehensive Review

## 1. Overview
The **Madgwick Filter** is a sensor fusion algorithm designed to estimate **orientation** using data from a **gyroscope, accelerometer, and magnetometer**. It employs **gradient descent optimization** to iteratively refine the quaternion representation of orientation.

The filter works by aligning the **measured sensor vectors** (from the accelerometer and magnetometer) with their **expected directions** in the **Earth frame**.

---

## 2. Gradient Descent Update Rule
The **quaternion estimate** \( {^E_S}\hat{q} \) is updated iteratively to minimize the orientation error:

\[
{^E_S}\hat{q}_{k+1} = {^E_S}\hat{q}_k - \mu \frac{\nabla f({^E_S}\hat{q}_k, {^E}d, {^S}s)}{\|\nabla f({^E_S}\hat{q}_k, {^E}d, {^S}s)\|}
\]

where:
- \( {^E_S}\hat{q}_k \) is the **quaternion estimate at iteration \( k \)**.
- \( \mu \) is the **step-size parameter**, controlling how much the quaternion is updated.
- \( \nabla f \) is the **gradient of the error function**.
- \( {^E}d \) is the **expected reference vector in the Earth frame** (e.g., gravity or magnetic field).
- \( {^S}s \) is the **measured sensor vector in the sensor frame**.

The goal is to find the optimal quaternion \( {^E_S}\hat{q} \) that minimizes the misalignment between the expected and measured sensor vectors.

---

## 3. Computation of the Gradient \( \nabla f \)
The gradient of the error function is computed as:

\[
\nabla f({^E_S}\hat{q}_k, {^E}d, {^S}s) = J^T({^E_S}\hat{q}_k, {^E}d) f({^E_S}\hat{q}_k, {^E}d, {^S}s)
\]

where:
- \( J({^E_S}\hat{q}_k, {^E}d) \) is the **Jacobian matrix**, describing how the error function changes with small changes in the quaternion.
- \( f({^E_S}\hat{q}_k, {^E}d, {^S}s) \) is the **error function**, which measures the misalignment between the transformed reference direction and the measured sensor vector.

The **Jacobian matrix** enables efficient gradient computation for quaternion updates.

---

## 4. The Error Function and Quaternion Rotation
The **rotation of the reference vector into the sensor frame** is given by:

\[
{^S}v = {^E_S}\hat{q}_k^* \otimes {^E}d \otimes {^E_S}\hat{q}_k
\]

where:
- \( {^E_S}\hat{q}_k^* \) is the **conjugate of the quaternion** (which inverts the rotation).
- \( {^E}d \) is the **expected reference direction in the Earth frame**.
- \( {^E_S}\hat{q}_k \) is the **quaternion describing the sensor's orientation**.

🔹 **This is the rotated frame of the corresponding sensor at timestamp \( t-1 \).**  
🔹 The result \( {^S}v \) represents how the reference vector should appear in the **sensor frame** if the quaternion is correct.  
🔹 The function is then **created by comparing this rotated reference vector to the measured sensor data.**

Thus, the **error function** is defined as:

\[
f({^E_S}\hat{q}_k, {^E}d, {^S}s) = {^E_S}\hat{q}_k^* \otimes {^E}d \otimes {^E_S}\hat{q}_k - {^S}s
\]

Since this transformation is done for the **\( t-1 \) timestamp**, the quaternion used here represents the **previous time step's orientation estimate**.

---

## 5. Expansion of the Error Function
Expanding the error function in terms of quaternion components gives:

\[
f({^E_S}\hat{q}_k, {^E}d, {^S}s) =
\begin{bmatrix}
2 d_x (0.5 - q_3^2 - q_4^2) + 2 d_y (q_1 q_4 + q_2 q_3) - s_x \\
2 d_x (q_2 q_3 - q_1 q_4) + 2 d_y (0.5 - q_2^2 - q_4^2) - s_y \\
2 d_x (q_1 q_3 + q_2 q_4) + 2 d_y (q_3 q_4 - q_1 q_2) - s_z
\end{bmatrix}
\]

where:
- \( q_1, q_2, q_3, q_4 \) are the **quaternion components**.
- \( d_x, d_y, d_z \) are the **components of the expected reference direction** \( {^E}d \).
- \( s_x, s_y, s_z \) are the **components of the measured sensor vector** \( {^S}s \).

---

## 6. Computation of the Jacobian \( J \)
The **Jacobian matrix** consists of the partial derivatives of \( f \) with respect to the quaternion components:

\[
J({^E_S}\hat{q}_k, {^E}d) =
\begin{bmatrix}
2 d_y q_4 - 2 d_z q_3 & 2 d_y q_3 + 2 d_z q_4 & -2 d_x q_4 + 2 d_z q_2 & 2 d_x q_3 - 4 d_y q_2 + 2 d_z q_1 \\
2 d_x q_3 - 2 d_y q_2 & 2 d_x q_4 - 2 d_y q_1 - 4 d_z q_2 & -4 d_x q_3 + 2 d_y q_2 - 2 d_z q_1 & -4 d_x q_4 + 2 d_y q_1 + 2 d_z q_2 \\
2 d_x q_2 + 2 d_z q_4 & -2 d_x q_1 - 4 d_y q_4 + 2 d_z q_3 & 2 d_x q_1 + 2 d_y q_4 - 4 d_z q_3 & 2 d_x q_2 + 2 d_y q_3
\end{bmatrix}
\]

This matrix is used to **efficiently compute the gradient direction**.

---

## 7. Quaternion Update Rule
Using the computed **gradient** \( \nabla f \), the **quaternion update** is:

\[
{^E_S}\hat{q}_{k+1} = {^E_S}\hat{q}_k - \mu \frac{\nabla f({^E_S}\hat{q}_k, {^E}d, {^S}s)}{\|\nabla f({^E_S}\hat{q}_k, {^E}d, {^S}s)\|}
\]

This iterative process ensures that the **quaternion estimate gradually aligns with the correct orientation**.

---

## 8. Summary
✅ The **Madgwick Filter** estimates orientation using **gradient descent**.  
✅ It aligns **measured sensor vectors (accelerometer, magnetometer)** with their **expected Earth-frame directions**.  
✅ The quaternion is iteratively updated using:
   - The **error function \( f \)**.
   - The **Jacobian matrix \( J \)**.
   - **Gyroscope integration**.  
✅ The quaternion transformation is computed for the **\( t-1 \) timestamp** and applied to update the current estimate.
