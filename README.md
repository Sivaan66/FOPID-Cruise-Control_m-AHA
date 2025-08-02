
# Cruise Control System using FOPID with m-AHA Optimization

## 🚗 Project Title
**Designing FOPID Controlled Cruise Control System Using Modified Artificial Hummingbird Algorithm (m-AHA)**

---

## 📝 Project Description

This project presents a novel approach to designing an intelligent and efficient **Fractional Order PID (FOPID) Controller** for a vehicle's **Cruise Control System (CCS)**. The FOPID parameters are finely tuned using the **Modified Elite Opposition-Based Artificial Hummingbird Algorithm (m-AHA)**, which enhances system performance, stability, and fuel efficiency during dynamic driving tasks.

The system aims to maintain a desired vehicle speed while improving the **ride comfort**, **response time**, and **energy consumption** under varying driving conditions.

---

## 🎯 Objectives

- Design and implement a **FOPID controller** for cruise control.
- Optimize the controller parameters using **m-AHA**, an advanced nature-inspired metaheuristic algorithm.
- Achieve better performance metrics (rise time, settling time, overshoot, etc.) compared to classical PID and standard optimization methods.
- Simulate and validate the system using MATLAB/Simulink.

---

## ⚙️ Features

- Fractional calculus-based FOPID control for enhanced flexibility.
- Implementation of **Modified Artificial Hummingbird Algorithm (m-AHA)** with elite opposition-based learning.
- System modeled on a **linearized vehicle dynamics equation**.
- Step response analysis to evaluate **rise time**, **settling time**, and **steady-state error**.
- Performance comparison against traditional control techniques.

---

## 🧠 Technologies Used

- MATLAB / Simulink
- Control System Toolbox
- Optimization algorithms (custom-coded m-AHA) 
- LaTeX (for report documentation)

---

## 📊 System Specifications

- Plant modeled as:
  \[
  G(s) = \frac{C}{(s - p_1)(s - p_2)(s - p_3)}
  \]
- FOPID Controller:
  \[
  C(s) = K_p + \frac{K_i}{s^\lambda} + K_d s^\mu
  \]
- Objective function:
  \[
  F = (1 - e^\rho) \left(\frac{\%OS}{100} + E_{ss} \right) + e^\rho (T_s - T_r)
  \]

---

## 📈 Results (Sample)

- **Rise Time**: ~0.66 seconds  
- **Settling Time**: ~1.04 seconds  
- **Overshoot**: Minimal  
- **Performance**: Superior to traditional PID and other metaheuristics on benchmark tests.

---
# Cruise Control System using FOPID and m-AHA

## 🧠 System Architecture

Below is the system-level block diagram of the cruise control model controlled by a FOPID controller optimized using m-AHA:

- 🔗 [Check the results](./Model(Simulink)CruiseControlSystem.pdf)

## 📊 Simulation Result
The output, error of the system, overshoot, settling time, rise time can be verified from this diagram.

- 🔗 [Check the results](./step_response.png)

---

## 📌 Future Scope

- Real-time implementation with hardware-in-the-loop (HIL) simulation.
- Extension to **adaptive or nonlinear control** frameworks.
- Integration with full autonomous driving systems.
