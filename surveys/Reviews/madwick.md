# Madgwick Filter Implementation for Sensor Fusion

## 📌 Overview
This repository contains an implementation of the **Madgwick Filter**, an efficient sensor fusion algorithm used for **orientation estimation**. It combines **gyroscope, accelerometer, and magnetometer** data to compute the sensor's quaternion-based orientation in real-time.

This implementation is based on **Sebastian Madgwick's original paper** and has been analyzed in-depth to understand **how gravitational (\( g \)) and magnetic (\( b \)) field inputs are processed**.

## 🔍 Key Features
- Uses **quaternions** for orientation representation, avoiding singularities of Euler angles.
- **Gyroscope bias correction** to improve long-term accuracy.
- **Accelerometer & Magnetometer integration** for drift compensation.
- **Real-time computation efficiency**, suitable for embedded systems.
- Supports both **estimated and predefined magnetic field models**.

---

## 🛠️ How the Algorithm Works (Step by Step)
The Madgwick Filter estimates orientation using a **gradient descent optimization** approach:

1️⃣ **Normalize Sensor Readings**  
   - The accelerometer and magnetometer inputs are normalized to **unit vectors**.

2️⃣ **Compute Error Functions**  
   - The algorithm calculates the difference between:
     - The **measured gravitational vector** and the expected **Earth gravity direction**.
     - The **measured magnetic field vector** and the expected **Earth’s magnetic field**.

3️⃣ **Compute the Jacobian Matrix**  
   - This matrix contains **partial derivatives of the error function** with respect to the quaternion.

4️⃣ **Gradient Descent Update**  
   - The quaternion is updated using the **computed gradient and a learning rate \( \mu \)**.

5️⃣ **Gyroscope Integration**  
   - The gyroscope’s angular velocity is used to **predict the quaternion rate of change**.

6️⃣ **Quaternion Normalization**  
   - Ensures the quaternion remains a **unit quaternion**, avoiding drift.

---

## 📌 Discussion from Our Research
In our analysis, we explored two different ways to handle the **magnetic field input (\( b \))**:

1. **Using a Fixed Magnetic Field Reference (\( {^E}b \))**
   - This approach **pre-defines the Earth's magnetic field** at a location using **NOAA’s World Magnetic Model (WMM)**.
   - This ensures **geographically accurate** magnetic north tracking.

2. **Estimating the Magnetic Field Dynamically**
   - Instead of using a predefined magnetic field, the algorithm **estimates it from real-time magnetometer readings**.
   - This is the **default implementation** in Madgwick’s filter.

**Comparison**:
| Approach | Pros | Cons |
|----------|------|------|
| Fixed \( b
