# 🐭 Micromouse Maze Solver

This project implements an **intelligent maze-solving robot** based on the **Flood Fill algorithm**, designed for the **IEEE Victoris 2.0 Competition**, where it won the **Best Software Award**.  
The robot autonomously navigates a maze, learns its environment, and optimizes the shortest path to the goal using embedded C programming.

---

## 🚀 Features
- **Autonomous Navigation:** Uses the Flood Fill algorithm to explore and solve mazes efficiently.  
- **Dynamic Path Learning:** Updates and recalculates paths in real time as it discovers new maze walls.  
- **Embedded System Implementation:** Fully implemented in **Embedded C** for microcontroller-based systems.  
- **Sensor Integration:** Simulated IR or ultrasonic sensors for wall detection and feedback.  
- **Competition-Ready:** Developed and tested as part of the **IEEE Victoris 2.0 Micromouse Competition**.

---

## 🧩 Algorithm Overview
The **Flood Fill algorithm** works by:
1. Assigning numerical weights to each cell representing its distance from the goal.  
2. Continuously updating weights as the mouse explores.  
3. Selecting the neighbor cell with the lowest weight as the next move.  
4. Recomputing when obstacles are detected, ensuring optimal navigation.

This algorithm is widely used in **AI pathfinding** and **autonomous robotics**, serving as a foundational approach for intelligent motion planning.

---

## 🛠️ Technologies Used
- **Language:** Embedded C  
- **Platform:** Microcontroller (AVR)  
- **Tools:** Proteus / MPLAB / Arduino IDE  
- **Algorithm:** Flood Fill Pathfinding

---

## 🏆 Award
- 🥇 **Best Software Award** – IEEE Victoris 2.0 Competition (2023)  
  *Recognized for implementing an optimized version of the Flood Fill algorithm for efficient maze-solving robotics.*

---

## 💬 Acknowledgment
Developed as part of the **IEEE Victoris 2.0 Micromouse Competition**, Mansoura University, Egypt.
