# XPBD-Based Real-Time Coupling of Soft Tires and Deformable Terrain

## Project Overview

This repository contains the implementation of the master's thesis project conducted at  
**National Yang Ming Chiao Tung University, Institute of Computer Science and Engineering**.  
The project presents a real-time simulation framework based on **Extended Position-Based Dynamics (XPBD)** to model the two-way interaction between:

- Soft deformable tires  
- Deformable terrain  
- Terrain sinkage during wheel–ground contact  
- Lateral soil settlement (bump formation)  
- Dynamic rut formation  

The system runs in real time (30 FPS+) within the **Unity Engine**, supporting multiple terrain materials such as sand, clay, heavy clay, and snow, each exhibiting different deformation behaviors.

---

## Key Methodology

- Utilizes **XPBD (Extended Position-Based Dynamics)** to model:
  - Deformable soft tires (tetrahedral mesh + Neo-Hookean material)
  - Vehicle rigid body with suspension, steering, and wheel joints
- Estimates tire–terrain contact forces using **Lagrange multipliers** from XPBD collision constraints  
- Applies **Bekker’s pressure–sinkage model** to compute vertical sinkage
- Includes:
  - Slip sinkage modeling  
  - Directional bump formation weighted by tire velocity  
- Terrain represented as a **heightfield grid**

---

## Development Environment

- Unity Version: `Unity 2022.3.6f1`
- Programming Languages: C#, C++
- IDE: Visual Studio 2022
- Physics Models:
  - XPBD
  - Neo-Hookean soft body formulation
  - Bekker’s terramechanics model
- Rendering Backend: Unity Built-in 3D Renderer & Terrain System

---

## Project Structure

    .
    ├── ...
    ├── Assets 
    │   ├── ...  
    │   ├── Materials           # Materials used for models/grounds
    │   ├── Models              # 3D Vehicle and tire models
    │   ├── Plugins             # .dll files generated from ./cppdll/
    │   ├── Prefabs             # Prefabricated objects that integrate models and scripts
    │   ├── Scenes              # Scenes ready-to-use
    │   ├── Scripts             # C# scripts for simulation
    │   ├── Terrain             # Terrain textures
    │   ├── Textures            # Other textures
    │   └── ...                
    ├── cppdll                  # cpp plugin for importing tetrahedral mesh
    ├── ...
    └── README.md
   ---

## How to Use

1. Open `Assets/Scenes/Terrain.unity`  
2. Press **Play** to start the simulation  
3. Control the vehicle using the keyboard:

| Key | Action      |
|-----|--------------|
| `W` | Move Forward |
| `A` | Turn Left    |
| `S` | Move Backward |
| `D` | Turn Right    |

---

## Vehicle Parameters (TestCar → Car)

- **Acceleration** — Vehicle acceleration strength  
- **Braking Force** — Braking intensity  
- **Max Turn Angle** — Maximum steering angle of the front wheels  
- **Start Frame** — Start frame for automatic driving mode  
- **Duration Frame** — Duration (frames) for automatic driving  
- **Turn Start Frame** — Start frame of automated turning  
- **Turn Duration Frame** — Turning duration in automatic mode  
- **Is Auto Mode** — Toggle automatic driving mode  

---

## Terrain Parameters

- **Max Contact Time** — Maximum deformation response time  
- **Min Contact Time** — Minimum deformation response time  
- **Neighbor Search** — Radius for lateral soil displacement search  
- **Ground Material** — Material parameters for terrain  
  - **Poisson Ratio** `[0, 0.5]`  
  - **K_c, K_φ** — Elastic parameters  
  - **Cohesion** — Soil cohesion  
  - **Friction Angle** — Internal friction angle  
  - **Unit Weight** — Soil unit weight  

---

## Unity Asset Store Credits

Some models and assets used in this project are sourced from the Unity Asset Store and are used strictly for academic and non-commercial purposes.  
All copyrights remain with their respective asset creators.

Assets used in this project include:

- Outdoor Ground Textures — Author: A dog's life software
- Yughues Free Ground Materials — Author: Nobiax / Yughues
- Race Car Package — Author: Ysn Studio

This project is intended solely for academic research and thesis demonstration purposes and is not used for any commercial application.

---

## Author / Contact

**Po-Yuan Tsai (蔡柏垣)**  
Email: **<timothy.tsai123@gmail.com>**
