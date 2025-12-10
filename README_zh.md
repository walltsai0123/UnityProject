
# XPBD-Based Real-Time Coupling of Soft Tires and Deformable Terrain

（基於 XPBD 的非剛性輪胎與可變形地形之即時耦合）

## 專案簡介

本專案為國立陽明交通大學資訊科學與工程研究所碩士論文之對應實作系統，  
提出一套基於 **Extended Position-Based Dynamics (XPBD)** 的即時輪胎－地形雙向耦合模擬方法，用於模擬：

- 非剛性輪胎（Soft Tire）
- 可變形地形（Deformable Terrain）
- 輪胎行駛時的地形下陷（Sinkage）
- 側向堆積（Lateral Settlement / Bump）
- 輪跡形成（Rut Formation）

系統可於 **Unity 引擎中即時執行（30 FPS 以上）**，並能呈現不同地形材質（沙地、黏土、重黏土、雪地）下的物理行為差異。

---

## 研究方法重點

- 使用 **XPBD（Extended Position-Based Dynamics）** 建立：
  - 軟體輪胎（四面體網格 + Neo-Hookean 材料）
  - 剛性車體（含懸吊、轉向、輪軸）
- 由 XPBD 碰撞約束的 **拉格朗日乘數估算輪胎接觸力**
- 採用 **Bekker 壓力–下陷模型** 計算地形下陷（Sinkage）
- 加入：
  - Slip Sinkage（滑移下陷）
  - 依輪胎速度方向加權的側向堆積（Directional Bump）
- 地形表示方式為 **Height Field（高度圖）**

---

## 開發環境

- Unity 版本：`Unity 2022.3.6f1（依實際版本填寫）`
- 程式語言：C#, C++
- 開發工具：Visual Studio 2022
- 物理模型：
  - XPBD
  - Neo-Hookean Soft Body
  - Bekker's Terramechanics Model
- 渲染平台：Unity 內建 3D 渲染與 Terrain System

---

## 專案結構
  
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

## 使用方式

1. 開啟 `Assets/Scenes/Terrain.unity`  
2. 直接按下 `Play` 即可執行模擬  
3. 使用鍵盤控制車輛移動：  
   - `W`：前進  
   - `A`：左轉  
   - `S`：後退  
   - `D`：右轉  

---

## 車輛參數（TestCar → Car）

- **Acceleration**：車輛加速力度  
- **Braking Force**：車輛煞車力度  
- **Max Turn Angle**：前輪最大轉向角  
- **Start Frame**：自動模式開始幀數  
- **Duration Frame**：自動模式持續幀數  
- **Turn Start Frame**：自動模式開始轉向幀數  
- **Turn Duration Frame**：自動模式轉向持續幀數  
- **Is Auto Mode**：是否啟用自動模式  

---

## 地形參數（Terrain Parameters）

- **Max Contact Time**：地形變形最大反應時間  
- **Min Contact Time**：地形變形最小反應時間  
- **Neighbor Search**：周圍地形隆起的搜尋範圍  
- **Ground Material**：地形材質  
  - **Poisson Ratio**：材質壓縮率 `[0, 0.5]`  
  - **K_c, K_phi**：材質彈性參數  
  - **Cohesion**：材質凝聚力  
  - **Friction Angle**：材質摩擦角  
  - **Unit Weight**：材質單位重量  

---

## Unity Asset Store 使用資源聲明

本專案部分模型與素材來自 Unity Asset Store，僅用於學術研究與非商業用途，原始著作權皆屬於各資產作者所有。

已使用之資產包括：

- Outdoor Ground Textures — Author: A dog's life software
- Yughues Free Ground Materials — Author: Nobiax / Yughues
- Race Car Package — Author: Ysn Studio

本專案僅用於論文展示與學術研究用途，未用於任何商業行為。

---

## 作者/聯絡方式

**蔡柏垣 Tsai, Po-Yuan**  
Email: **<timothy.tsai123@gmail.com>**
