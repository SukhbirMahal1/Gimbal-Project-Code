# Gimbal Project Code
Arduino code for a Two-Axis Payload Stabilization System using an MPU6050 as the inertial measurement unit and servos for actuation. This project includes additional features such as LED indicators for debugging and communication, and a mode-switching button. Developed in Visual Studio using PlatformIO.

## Project Overview
<p align="center">
  <img src="https://github.com/user-attachments/assets/898403e6-3759-439b-842f-9035a8aa3e9c" alt="Gimbal Side View" width="30%">
  <img src="https://github.com/user-attachments/assets/e15302f3-486e-4923-9990-e52842dd89af" alt="Gimbal Top View" width="30%">
  <img src="https://github.com/user-attachments/assets/dceb1235-da45-491e-aff6-0f7062a13bfa" alt="Electronics Arduino Shield" width="30%">
</p>
<p align="center">
  <b>Gimbal Side View</b> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <b>Gimbal Top View</b> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <b>Electronics Arduino Shield</b>
</p>

This system is part of a larger Fixed-Wing UAV University design project. It was broken down into two main tasks: (a) designing, building, and testing a wing for a lightweight UAV, and (b) developing a payload stabilization system that orients a bespoke antenna to always point normal to the Earth's surface.

The stabilisation system is controlled using a PID algorithm, found in Gimbal_PID_Control_Algorithm/src/main.cpp, which monitors the difference between the sensor orientation and the desired setpoint (0˚) at a fixed loop rate. This closed-loop control allows the gimbal to achieve stable responses that are resilient to disturbances. The actuators can move ±90˚ but are constrained to ±50˚ to prevent collisions between rotating elements in the structure. The design also includes user-friendly functions such as:

- Blue LED **ON** when initialising system setup
- Blue LED **FLASHING** when cables come loose/disconnect
- Green LED **ON** when the PID control algorithm is **active**
- A button to **activate** a _payload insert mode_
- Blue LED AND Green LED **ON** when the _payload insert mode_ is activaated

The system logic is illustrated below:

<p align="center">
  <img src="https://github.com/user-attachments/assets/b5aee6fb-7f46-46f5-8698-886205346b55" alt="System Logic" width="70%">
</p>

The IMU requires a high sampling rate (above 90˚/s) to ensure smooth orientation corrections. It initializes its accelerometer and gyroscope to ±2g and 250˚/s sensitivity, respectively, and communicates with the microcontroller via I2C at 400kHz. The gimbal uses the MPU6050's Digital Motion Processor (DMP) to obtain the antenna’s orientation data using quaternions, which offloads motion processing tasks from the microcontroller.

<p align="center">
  <img src="https://github.com/user-attachments/assets/55346361-172c-4cb6-ad64-e3af2ba082e4" alt="System Overview" width="45%">
  <img src="https://github.com/user-attachments/assets/0148ba3e-8693-4c15-926a-b3b851ecbdb3" alt="PID Control Process" width="45%">
</p>

The PID gain tuning was performed using a Gimbal Testing table during simulated flight conditions, including take-off, landing, and turbulence. The system response to pitch and roll perturbations exhibited a small steady-state error with a noticeable but non-disruptive overshoot, as shown above.
