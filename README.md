# STM32-Controlled 3-Axis CNC Drawing Machine  

A **3-axis CNC-style drawing machine** powered by an STM32 microcontroller and stepper motors. This project demonstrates how to combine **interrupt-driven input handling**, **stepper motor control**, and a **modular 3D-printed frame** to create a low-cost educational CNC platform. A video demo of the final product can be found on youtube using the following link https://youtube.com/shorts/fzyY_rNcf6s?si=H5-k7CZNy2wZm4F2 .

---

## ✨ Features  
- **Three-axis stepper motor control** (X, Y, and pen lift).  
- **Interrupt-driven rotary encoder** for precise manual jogging of an axis.  
- **Joystick input** for intuitive X–Y manual positioning.  
- **Push button input** for toggling motor direction / reset actions.  
- **Modular 3D-printed frame** for easy assembly and modification.  
- **Arduino-compatible C++ firmware** running on STM32.  

---

## 🛠️ Hardware  
- **Microcontroller**: STM32F429ZI Discovery (compatible with other STM32 boards).  
- **Motors**: 28BYJ-48 stepper motors (x3) with ULN2003 drivers.  
- **Inputs**:  
  - Rotary encoder (axis jog control, via interrupt)  
  - Joystick (manual XY motion)  
  - Push button (direction toggle / reset)  
- **Frame**: Custom CAD design, 3D printed.  

---
