# 第 13 章：ros2_control 框架

## 章节大纲

### 13.1 引言与框架概述
- ros2_control 的设计理念
- 与 ROS1 control 的对比
- 硬件抽象的重要性

### 13.2 硬件抽象层设计
- Hardware Interface 架构
- Resource Manager 机制
- 硬件组件生命周期
- 自定义硬件接口开发

### 13.3 控制器链（Controller Chaining）
- 控制器管理器架构
- 级联控制器设计
- 控制器间数据流
- 动态加载与切换

### 13.4 实时控制循环
- 实时性保证机制
- 控制周期与更新率
- 线程模型与调度
- 延迟分析与优化

### 13.5 机械臂控制实例
- 7自由度机械臂建模
- 关节控制器实现
- 轨迹跟踪控制
- 力矩控制与重力补偿

### 13.6 产业案例研究：Universal Robots 力控与柔顺控制
- UR 机器人架构分析
- 力/力矩传感器集成
- 柔顺控制算法实现
- 安全性与碰撞检测

### 13.7 高级话题
- EtherCAT 总线集成
- 硬实时控制策略
- 多机器人同步控制
- 论文导读与开源项目

### 13.8 本章小结

### 13.9 练习题

### 13.10 常见陷阱与错误

### 13.11 最佳实践检查清单

---

## 13.1 引言与框架概述

ros2_control 是 ROS2 中用于机器人控制的核心框架，它提供了一个标准化、模块化的方式来管理机器人硬件接口和控制算法。本章将深入探讨 ros2_control 的架构设计、实时控制机制、以及在工业机器人中的实际应用。通过学习本章，您将掌握如何设计高性能的机器人控制系统，实现从简单的位置控制到复杂的力控和柔顺控制。

### ros2_control 的核心价值

ros2_control 解决了机器人控制系统开发中的几个关键挑战：

1. **硬件抽象**：通过统一的接口隔离硬件细节，使控制算法可以在不同硬件平台间复用
2. **实时性保证**：提供确定性的控制循环，满足工业级实时控制要求
3. **模块化设计**：控制器可以独立开发、测试和部署，支持热插拔
4. **标准化接口**：与 MoveIt2、Nav2 等高级框架无缝集成

### 架构演进：从 ROS1 到 ROS2

ROS1 的 ros_control 在工业界得到广泛应用，但存在一些限制：

- 单线程执行模型导致的性能瓶颈
- 缺乏生命周期管理
- 实时性依赖外部补丁

ROS2 control 的改进：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
ROS1 Control                    ROS2 Control
     │                               │
     ├─ Controller Manager           ├─ Controller Manager
     │   └─ Single Thread            │   ├─ Multi-threaded
     │                               │   └─ Lifecycle Management
     ├─ Hardware Interface           ├─ Hardware Components
     │   └─ Static Loading           │   ├─ Dynamic Loading
     │                               │   └─ State Interfaces
     └─ Transmission                └─ Chainable Controllers
         └─ Fixed Pipeline               └─ Flexible Routing
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

### 系统组件概览

ros2_control 框架由以下核心组件构成：

1. **Hardware Components**：封装实际硬件或仿真接口
2. **Resource Manager**：管理硬件资源的分配和访问
3. **Controller Manager**：负责控制器的生命周期和执行调度
4. **Controllers**：实现具体的控制算法
5. **Transmission System**：处理关节空间与执行器空间的映射

## 13.2 硬件抽象层设计

硬件抽象层是 ros2_control 的基础，它定义了控制系统与物理硬件之间的标准接口。这种抽象使得同一套控制算法可以无缝地在仿真环境和真实硬件之间切换。

### Hardware Interface 架构

Hardware Interface 采用组件化设计，每个硬件组件都继承自 `hardware_interface::SystemInterface`、`hardware_interface::ActuatorInterface` 或 `hardware_interface::SensorInterface`：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
SystemInterface (多关节系统，如机械臂)
    ├─ State Interfaces
    │   ├─ position
    │   ├─ velocity
    │   └─ effort
    └─ Command Interfaces
        ├─ position_command
        ├─ velocity_command
        └─ effort_command

ActuatorInterface (单个执行器)
    ├─ State: position, velocity, effort
    └─ Command: position/velocity/effort

SensorInterface (传感器)
    └─ State: force, torque, IMU data, etc.
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

### 状态与命令接口

ros2_control 使用状态接口（State Interface）和命令接口（Command Interface）来抽象硬件交互：

**状态接口**：从硬件读取当前状态
- 关节位置、速度、力矩
- 传感器读数（力传感器、IMU等）
- 硬件状态标志（温度、错误码等）

**命令接口**：向硬件发送控制命令
- 位置命令（用于位置控制）
- 速度命令（用于速度控制）
- 力矩/力命令（用于力控制）

### Resource Manager 机制

Resource Manager 是硬件资源的中央管理器，负责：

1. **资源注册与发现**
   ```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
   资源注册流程：
   Hardware Component → export_interfaces() → Resource Manager
                                                    │
                                                    ├─ State Interface Registry
                                                    └─ Command Interface Registry
   ```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

2. **资源分配与锁定**
   - 防止多个控制器同时访问同一命令接口
   - 允许多个控制器共享状态接口（只读）
   - 实现资源互斥和优先级管理

3. **生命周期管理**
   - Configure：加载硬件配置，建立通信
   - Activate：启动硬件，开始数据交换
   - Deactivate：停止硬件，保持连接
   - Cleanup：释放资源，断开连接

### 硬件组件生命周期

硬件组件遵循标准的生命周期状态机：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
    [Unconfigured] 
          │
      configure()
          ↓
     [Inactive]
       ↙    ↘
  activate() deactivate()
     ↓        ↑
    [Active]──┘
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

每个状态转换都有对应的回调函数：

- `on_configure()`: 读取参数，初始化硬件连接
- `on_activate()`: 启动硬件，开始控制循环
- `on_deactivate()`: 停止控制，保持安全状态
- `on_cleanup()`: 清理资源，关闭连接

### 自定义硬件接口开发

开发自定义硬件接口需要实现以下关键方法：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$cpp
class MyRobotSystem : public hardware_interface::SystemInterface {
public:
  // 导出状态和命令接口
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_count_; ++i) {
      state_interfaces.emplace_back(
        joint_names_[i], "position", &joint_positions_[i]);
      state_interfaces.emplace_back(
        joint_names_[i], "velocity", &joint_velocities_[i]);
    }
    return state_interfaces;
  }

  // 硬件读取
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override {
    // 从硬件读取当前状态
    // 更新 joint_positions_, joint_velocities_ 等
    return return_type::OK;
  }

  // 硬件写入
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override {
    // 将命令发送到硬件
    // 读取 joint_position_commands_ 并发送
    return return_type::OK;
  }
};
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

### 硬件接口配置

硬件接口通过 URDF/XACRO 文件配置：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$xml
<ros2_control name="MyRobotSystem" type="system">
  <hardware>
    <plugin>my_robot_hardware/MyRobotSystem</plugin>
    <param name="robot_ip">192.168.1.100</param>
    <param name="control_frequency">1000</param>
  </hardware>
  <joint name="joint1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

## 13.3 控制器链（Controller Chaining）

控制器链是 ros2_control 的创新特性，允许控制器之间形成级联关系，实现复杂的控制策略。这种设计使得控制系统可以像搭积木一样组合不同的控制模块。

### 控制器管理器架构

Controller Manager 是控制器的运行时环境，负责：

1. **控制器加载与配置**
2. **资源分配与冲突检测**
3. **执行顺序调度**
4. **实时循环管理**

控制器管理器的执行流程：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
Control Loop (1kHz)
    │
    ├─ read() → 从硬件读取状态
    │
    ├─ update() → 按依赖顺序更新控制器
    │   ├─ Sensor Controllers (第一层)
    │   ├─ State Controllers (第二层)
    │   └─ Command Controllers (第三层)
    │
    └─ write() → 向硬件写入命令
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

### 可链接控制器设计

可链接控制器（Chainable Controller）既可以作为其他控制器的输入源，也可以接收其他控制器的输出：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
传统控制器:
Hardware → Controller → Hardware

可链接控制器:
Hardware → Controller A → Controller B → Controller C → Hardware
              (滤波)        (PID)         (限幅)
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

实现可链接控制器的关键接口：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$cpp
class ChainableController : public controller_interface::ControllerInterface {
public:
  // 导出引用接口供下游控制器使用
  std::vector<hardware_interface::StateInterface> export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(
      get_node()->get_name(), "output", &output_value_);
    return state_interfaces;
  }
  
  // 声明需要的引用接口
  std::vector<hardware_interface::CommandInterface> export_reference_interfaces() {
    std::vector<hardware_interface::CommandInterface> reference_interfaces;
    reference_interfaces.emplace_back(
      get_node()->get_name(), "reference", &reference_value_);
    return reference_interfaces;
  }
};
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

### 控制器间数据流

控制器链中的数据流遵循严格的方向性：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
数据流示例：力控制链
┌─────────────┐     ┌──────────────┐     ┌────────────┐     ┌──────────┐
│Force Sensor │ --> │Gravity Comp. │ --> │Force PID   │ --> │Joint Cmd │
│Controller   │     │Controller    │     │Controller  │     │Interface │
└─────────────┘     └──────────────┘     └────────────┘     └──────────┘
     读取力              补偿重力           计算力矩          发送命令
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

### 动态加载与切换

控制器支持运行时动态加载和切换，这对于多模态控制至关重要：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$yaml
# 控制器配置示例
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz
    
    # 位置控制器
    position_controller:
      type: position_controllers/JointGroupPositionController
      joints:
        - joint1
        - joint2
      
    # 轨迹控制器  
    trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      joints:
        - joint1
        - joint2
      state_interfaces:
        - position
        - velocity
      command_interfaces:
        - position
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

控制器切换流程：

1. **停止当前控制器**
   ```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$bash
   ros2 control switch_controllers --stop position_controller
   ```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

2. **启动新控制器**
   ```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$bash
   ros2 control switch_controllers --start trajectory_controller
   ```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

3. **原子切换**（同时停止和启动）
   ```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$bash
   ros2 control switch_controllers \
     --stop position_controller \
     --start trajectory_controller \
     --strict  # 确保切换成功
   ```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

### 控制器优先级与冲突解决

当多个控制器请求同一资源时，需要优先级机制：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
优先级规则：
1. 安全控制器 (最高优先级)
   └─ 紧急停止、碰撞避免
2. 轨迹控制器
   └─ 路径跟踪、运动规划执行
3. 位置控制器
   └─ 点到点运动
4. 速度控制器 (最低优先级)
   └─ 手动控制、遥操作
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

### 控制器链实例：阻抗控制

阻抗控制是控制器链的典型应用：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
阻抗控制链：
┌────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────┐
│Position    │    │Impedance     │    │Torque       │    │Hardware  │
│Reference   │ -> │Controller    │ -> │Controller   │ -> │Interface │
└────────────┘    └──────────────┘    └─────────────┘    └──────────┘
                        │
                        ├─ K_p (刚度矩阵)
                        ├─ K_d (阻尼矩阵)  
                        └─ M (惯性矩阵)
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

阻抗控制器实现：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$cpp
class ImpedanceController : public ChainableController {
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) {
    
    // 计算位置误差
    Eigen::VectorXd pos_error = desired_position_ - current_position_;
    Eigen::VectorXd vel_error = desired_velocity_ - current_velocity_;
    
    // 阻抗控制律: F = K_p * Δx + K_d * Δẋ + M * ẍ_d
    Eigen::VectorXd force_cmd = 
      stiffness_matrix_ * pos_error +
      damping_matrix_ * vel_error +
      inertia_matrix_ * desired_acceleration_;
    
    // 输出到下游力矩控制器
    output_force_ = force_cmd;
    
    return controller_interface::return_type::OK;
  }
};
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

## 13.4 实时控制循环

实时控制是机器人系统的核心要求，ros2_control 通过精心设计的控制循环和调度策略，实现了微秒级的控制周期和确定性的响应时间。

### 实时性保证机制

ros2_control 的实时性建立在多个层次的保证机制之上：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
实时性层次结构：
┌─────────────────────────────┐
│     应用层 (控制算法)        │ ← 无动态内存分配
├─────────────────────────────┤
│     ros2_control 框架       │ ← 锁定内存页
├─────────────────────────────┤
│     DDS 中间件 (实时配置)    │ ← 零拷贝传输
├─────────────────────────────┤
│     实时内核 (RT_PREEMPT)   │ ← 抢占式调度
└─────────────────────────────┘
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

关键的实时保证技术：

1. **内存预分配**：避免运行时内存分配
2. **锁定内存页**：防止页面交换
3. **CPU 亲和性**：绑定到专用 CPU 核心
4. **实时调度策略**：SCHED_FIFO 或 SCHED_RR

### 控制周期与更新率

控制周期的选择直接影响系统性能：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
典型控制频率：
- 位置控制: 100-500 Hz
- 速度控制: 500-1000 Hz  
- 力/力矩控制: 1000-4000 Hz
- 电流控制: 10-40 kHz (通常在驱动器内部)
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

控制周期的数学关系：

$$T_s = \frac{1}{f_c}$$

其中 $T_s$ 是采样周期，$f_c$ 是控制频率。

奈奎斯特采样定理要求：
$$f_c > 2 \cdot f_{max}$$

其中 $f_{max}$ 是系统最高频率成分。

### 线程模型与调度

ros2_control 采用多线程架构，不同组件运行在不同优先级的线程中：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$cpp
// 控制循环线程配置
class ControlLoop {
  void configure_realtime() {
    // 设置线程优先级
    struct sched_param param;
    param.sched_priority = 95;  // 高优先级
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    
    // 锁定内存
    mlockall(MCL_CURRENT | MCL_FUTURE);
    
    // CPU 亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);  // 绑定到 CPU 3
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
  }
  
  void run() {
    rclcpp::Time last_time = node_->now();
    rclcpp::Duration period(0, 1000000);  // 1ms
    
    while (rclcpp::ok()) {
      auto current_time = node_->now();
      auto elapsed = current_time - last_time;
      
      // 硬件读取
      hardware_->read(current_time, elapsed);
      
      // 控制器更新
      controller_manager_->update(current_time, elapsed);
      
      // 硬件写入
      hardware_->write(current_time, elapsed);
      
      // 精确睡眠到下一个周期
      std::this_thread::sleep_until(last_time + period);
      last_time = current_time;
    }
  }
};
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

### 延迟分析与优化

控制循环的总延迟由多个部分组成：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
总延迟 = 传感器延迟 + 通信延迟 + 计算延迟 + 执行器延迟

典型延迟分解 (1kHz 控制循环):
├─ 传感器读取: 50-100 μs
├─ 控制计算:  100-200 μs  
├─ 通信传输:  50-100 μs
└─ 执行器响应: 200-500 μs
总计: 400-900 μs (< 1ms 周期)
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

延迟优化策略：

1. **传感器优化**
   - 使用硬件触发采样
   - DMA 直接内存访问
   - 并行读取多个传感器

2. **计算优化**
   - SIMD 向量化计算
   - 查表法替代复杂计算
   - 增量式算法

3. **通信优化**
   - 共享内存通信
   - 零拷贝消息传递
   - 批量数据传输

### 抖动（Jitter）控制

控制周期的抖动会严重影响控制性能：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
抖动来源：
1. 操作系统调度延迟
2. 中断处理
3. 缓存未命中
4. 总线竞争
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

抖动测量与监控：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$cpp
class JitterMonitor {
  void measure_jitter() {
    static rclcpp::Time last_time;
    auto current_time = node_->now();
    
    if (last_time.nanoseconds() > 0) {
      auto period = (current_time - last_time).nanoseconds();
      auto jitter = std::abs(period - expected_period_ns_);
      
      // 统计抖动
      max_jitter_ = std::max(max_jitter_, jitter);
      jitter_histogram_[jitter / 1000]++;  // 微秒级统计
      
      // 抖动告警
      if (jitter > jitter_threshold_) {
        RCLCPP_WARN(node_->get_logger(), 
          "High jitter detected: %ld ns", jitter);
      }
    }
    last_time = current_time;
  }
};
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

### 实时性能基准测试

评估实时性能的关键指标：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
性能指标：
- 平均延迟 (Average Latency)
- 最坏情况延迟 (WCET - Worst Case Execution Time)
- 抖动标准差 (Jitter Standard Deviation)
- 丢失周期率 (Missed Deadline Rate)
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$

基准测试工具：

```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$bash
# cyclictest 测试实时延迟
sudo cyclictest -p 95 -t1 -n -i 1000 -l 100000

# ros2_control 性能分析
ros2 run controller_manager ros2_control_benchmark \
  --iterations 10000 \
  --control-frequency 1000
```

## 13.5 机械臂控制实例

机械臂控制是 ros2_control 最典型的应用场景。本节将通过一个 7 自由度机械臂的完整实现，展示从底层硬件接口到高级控制算法的全栈开发流程。

### 7自由度机械臂建模

7-DOF 机械臂相比 6-DOF 具有冗余自由度，能够在保持末端位姿不变的情况下调整关节配置，这对于避障和优化关节运动至关重要。

```
7-DOF 机械臂运动学链：
Base → S0 → S1 → E0 → E1 → W0 → W1 → W2 → End-effector
       ↑    ↑    ↑    ↑    ↑    ↑    ↑
      肩部  肩部  肘部  肘部  腕部  腕部  腕部
      偏航  俯仰  偏航  俯仰  偏航  俯仰  滚转
```

DH 参数表（Denavit-Hartenberg）：

| Joint | a(mm) | α(rad) | d(mm) | θ |
|-------|-------|--------|-------|---|
| S0    | 0     | -π/2   | 317   | θ₁ |
| S1    | 81    | π/2    | 192.5 | θ₂ |
| E0    | 0     | -π/2   | 400   | θ₃ |
| E1    | 0     | π/2    | 168.5 | θ₄ |
| W0    | 0     | -π/2   | 400   | θ₅ |
| W1    | 0     | π/2    | 136.3 | θ₆ |
| W2    | 0     | 0      | 133.75| θ₇ |

正运动学变换矩阵：

$$T_{0}^{7} = \prod_{i=1}^{7} T_{i-1}^{i}(\theta_i)$$

其中每个关节变换矩阵：

$$T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 关节控制器实现

关节控制器是机械臂控制的基础，包括位置控制、速度控制和力矩控制三种基本模式。

**重力补偿计算**：

重力补偿是机械臂控制的关键，特别是在低速运动和静态保持时：

$$\tau_g = G(q) = \sum_{i=1}^{n} m_i g^T J_{v_i}^{c_i}(q)$$

其中：
- $m_i$ 是连杆 i 的质量
- $g$ 是重力向量
- $J_{v_i}^{c_i}$ 是连杆 i 质心的线速度雅可比矩阵

### 轨迹跟踪控制

轨迹控制器负责让机械臂沿着预定轨迹运动，需要处理位置、速度和加速度的协调控制。关键技术包括：

1. **三次样条插值**：保证位置和速度连续性
2. **前馈控制**：提高跟踪精度
3. **误差容限管理**：判断轨迹执行成功与否

轨迹规划的约束条件：
- 关节位置限制：$q_{min} \leq q \leq q_{max}$
- 关节速度限制：$|\dot{q}| \leq \dot{q}_{max}$
- 关节加速度限制：$|\ddot{q}| \leq \ddot{q}_{max}$
- 关节加加速度限制：$|\dddot{q}| \leq \dddot{q}_{max}$

### 力矩控制与动力学补偿

力矩控制提供最直接的控制方式，但需要精确的动力学模型：

动力学方程（拉格朗日形式）：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

其中：
- $M(q)$ 是惯性矩阵（对称正定）
- $C(q,\dot{q})$ 是科氏力和离心力矩阵
- $G(q)$ 是重力向量
- $\tau$ 是关节力矩

计算力矩控制律：

$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

其中 $e = q_d - q$ 是位置误差。

### 关节空间阻抗控制

阻抗控制允许机械臂表现出期望的动态特性，适用于与环境交互的任务：

阻抗模型：

$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

设计准则：
- 低刚度 $K_d$：柔顺接触，适合精密装配
- 高刚度 $K_d$：精确位置控制
- 适中阻尼 $D_d$：避免振荡，临界阻尼 $D_d = 2\sqrt{M_d K_d}$
## 13.6 产业案例研究：Universal Robots 力控与柔顺控制

Universal Robots (UR) 是协作机器人领域的领导者，其 e-Series 机器人（UR3e, UR5e, UR10e, UR16e）在力控和柔顺控制方面的实现为工业界树立了标杆。本节深入分析 UR 机器人如何通过 ros2_control 实现安全、高效的人机协作。

### UR 机器人架构分析

UR 机器人采用模块化设计，每个关节都是独立的智能单元：

```
UR 控制架构：
┌─────────────────────────────────────┐
│         主控制器 (Linux RT)           │
│  ┌─────────────┬──────────────────┐ │
│  │ Safety MCU  │  Control Thread   │ │
│  │  (双通道)    │   (125μs cycle)   │ │
│  └─────────────┴──────────────────┘ │
└─────────┬───────────────────────────┘
          │ EtherCAT (4kHz)
    ┌─────┴─────┬─────┬─────┬─────┬─────┐
    │Joint1│Joint2│Joint3│Joint4│Joint5│Joint6│
    └──────┴──────┴──────┴──────┴──────┴──────┘
      每个关节模块包含：
      - 伺服驱动器
      - 绝对编码器 (19-bit)
      - 力矩传感器
      - 温度传感器
      - 安全监控器
```

关键技术特性：

1. **双编码器系统**：
   - 主编码器：19 位分辨率，用于精确控制
   - 安全编码器：独立通道，用于安全监控

2. **分布式力矩感知**：
   - 每个关节内置力矩传感器
   - 采样率 4kHz，分辨率 0.01 Nm
   - 用于碰撞检测和力控制

3. **功能安全设计**：
   - 符合 ISO 13849-1 Cat. 3 PL d
   - 双通道安全架构
   - 安全速度和位置监控

### 力/力矩传感器集成

UR 机器人的力感知能力基于关节力矩传感器和动力学模型的融合：

**力矩估计算法**：

```
外力矩估计：
τ_ext = τ_measured - τ_model

其中：
τ_measured: 关节力矩传感器测量值
τ_model: 动力学模型计算值（包括重力、惯性、摩擦）
```

动力学模型辨识过程：

1. **质量参数辨识**：
   $$\tau_i = \sum_{j=i}^{n} (I_{j} \ddot{q}_j + m_j g^T J_{v,j})$$
   
   通过激励轨迹和最小二乘法辨识质量和质心位置

2. **摩擦模型辨识**：
   $$\tau_{friction} = F_c \cdot sign(\dot{q}) + F_v \cdot \dot{q} + F_s \cdot e^{-|\dot{q}|/\dot{q}_s}$$
   
   包含库仑摩擦、粘性摩擦和 Stribeck 效应

3. **温度补偿**：
   $$\tau_{comp} = \tau_{measured} \cdot (1 + \alpha \cdot \Delta T)$$
   
   补偿温度对传感器和机械结构的影响

**末端力估计**：

从关节力矩到末端力的映射：

$$F_{ee} = (J^T)^+ \tau_{ext}$$

其中 $(J^T)^+$ 是雅可比转置的伪逆。

为提高估计精度，UR 采用卡尔曼滤波器：

```
状态方程：
x_k = A x_{k-1} + B u_k + w_k
y_k = C x_k + v_k

其中：
x: [F_ext, F_ext_dot]  (外力及其导数)
u: τ_measured - τ_model (力矩残差)
w, v: 过程噪声和测量噪声
```

### 柔顺控制算法实现

UR 的柔顺控制采用混合力/位置控制架构：

**选择矩阵方法**：

$$\tau = J^T(S_f F_d + S_p K_p(x_d - x) + S_p K_d(\dot{x}_d - \dot{x}))$$

其中：
- $S_f$：力控制选择矩阵
- $S_p = I - S_f$：位置控制选择矩阵
- $F_d$：期望接触力
- $x_d, x$：期望和实际笛卡尔位置

**自适应阻抗控制**：

UR 实现了变刚度控制，根据任务需求动态调整阻抗参数：

```
阻抗参数调度：
         ┌─ 高刚度 (2000 N/m)：自由空间运动
K(t) = ──┼─ 中刚度 (500 N/m)：接近阶段
         └─ 低刚度 (100 N/m)：接触阶段

切换策略：
if |F_ext| < F_threshold_1:
    K = K_high  # 自由空间
elif |F_ext| < F_threshold_2:
    K = K_medium  # 过渡区
else:
    K = K_low  # 接触区
```

### 安全性与碰撞检测

UR 的安全系统是多层次的，确保在各种故障模式下都能保证人员安全：

**碰撞检测算法**：

1. **基于模型的检测**：
   ```
   碰撞指标：
   r = |τ_ext| - τ_threshold
   
   动态阈值：
   τ_threshold = τ_base + k_vel * |q_dot| + k_acc * |q_ddot|
   ```

2. **基于能量的检测**：
   ```
   碰撞能量：
   E_collision = ∫ F_ext · v dt
   
   触发条件：
   if E_collision > E_threshold:
       trigger_protective_stop()
   ```

3. **频域分析**：
   检测力矩信号的高频成分，识别碰撞特征

**安全响应策略**：

```
安全响应级别：
Level 0: 正常运行
Level 1: 降速运行 (速度限制到 250 mm/s)
Level 2: 保护停止 (零力矩，重力补偿)
Level 3: 紧急停止 (断电制动)

触发条件：
- Level 1: |F_ext| > 100N 或接近速度/位置限制
- Level 2: |F_ext| > 150N 或碰撞检测触发
- Level 3: 安全系统故障或急停按钮
```

**ISO/TS 15066 合规性**：

UR 机器人符合协作机器人标准，实现四种协作模式：

1. **安全监控停止**：人员进入时停止
2. **手动引导**：直接手动操作
3. **速度和距离监控**：根据距离调整速度
4. **功率和力限制**：限制接触力和压强

生物力学限值（ISO/TS 15066）：
| 身体部位 | 准静态力 (N) | 瞬态力 (N) |
|---------|-------------|-----------|
| 头部    | 130         | 175       |
| 胸部    | 140         | 210       |
| 手臂    | 150         | 190       |
| 手部    | 140         | 180       |

### 实际应用案例

**案例1：精密装配**

汽车零部件装配，公差 ±0.05mm：

```
控制策略：
1. 快速接近：位置控制，v=250mm/s
2. 搜索接触：速度控制，v=10mm/s，监测 Fz
3. 对准插入：混合力/位控制
   - X,Y: 位置控制，刚度 500 N/m
   - Z: 力控制，Fz = 10N
   - Rx,Ry: 柔顺，刚度 5 Nm/rad
4. 完全插入：力控制，Fz = 30N
```

成功率：99.5%，周期时间：3.2秒

**案例2：打磨抛光**

曲面打磨，恒定接触力 20N：

```
控制参数：
- 法向力控制：Fn = 20 ± 2N
- 切向速度：v = 100 mm/s
- 阻抗参数：
  - Kn = 100 N/m (法向低刚度)
  - Kt = 1000 N/m (切向高刚度)
```

表面粗糙度：Ra < 0.8μm，力控制精度：±1.5N

## 13.7 高级话题

ros2_control 的前沿研究和工业实践不断推动着机器人控制技术的发展。本节探讨 EtherCAT 总线集成、硬实时控制、多机器人同步等高级主题。

### EtherCAT 总线集成

EtherCAT (Ethernet for Control Automation Technology) 是工业机器人中最广泛使用的实时以太网协议，提供微秒级的控制周期和纳秒级的同步精度。

**EtherCAT 架构**：

```
EtherCAT 拓扑：
Master (IPC) ──┬── Slave 1 (Joint 1) 
               ├── Slave 2 (Joint 2)
               ├── Slave 3 (Joint 3)
               ├── Slave 4 (I/O Module)
               └── Slave 5 (Force Sensor)

通信特性：
- 周期时间：125 μs - 1 ms
- 抖动：< 1 μs
- 同步精度：< 100 ns (with DC)
- 最大节点数：65535
- 传输效率：> 90%
```

**SOEM 集成（Simple Open EtherCAT Master）**：

```cpp
class EtherCATHardware : public hardware_interface::SystemInterface {
private:
  // EtherCAT 上下文
  ecx_contextt* ec_context_;
  
  // 从站配置
  struct SlaveConfig {
    uint16_t alias;
    uint16_t position;
    uint32_t vendor_id;
    uint32_t product_code;
    ec_slave* slave;
  };
  std::vector<SlaveConfig> slaves_;
  
  // PDO 映射
  struct PDOMapping {
    uint8_t* inputs;
    uint8_t* outputs;
    size_t input_size;
    size_t output_size;
  };
  std::vector<PDOMapping> pdo_mappings_;
  
public:
  bool configure_ethercat() {
    // 初始化 SOEM
    if (ec_init(interface_name_.c_str()) <= 0) {
      return false;
    }
    
    // 配置从站
    int slave_count = ec_config_init(0);
    
    // 配置分布式时钟
    ec_configdc();
    
    // PDO 映射
    for (int i = 1; i <= slave_count; i++) {
      configure_slave_pdo(i);
    }
    
    // 切换到安全操作状态
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    
    // 启动周期性数据交换
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);
    
    return true;
  }
  
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // 发送过程数据
    ec_send_processdata();
    
    // 接收过程数据
    int wkc = ec_receive_processdata(EC_TIMEOUTRET);
    
    if (wkc >= ec_slave[0].expectedWKC) {
      // 解析输入数据
      for (size_t i = 0; i < slaves_.size(); i++) {
        parse_slave_inputs(i);
      }
    }
    
    return return_type::OK;
  }
  
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // 准备输出数据
    for (size_t i = 0; i < slaves_.size(); i++) {
      prepare_slave_outputs(i);
    }
    
    return return_type::OK;
  }
};
```

**分布式时钟同步**：

EtherCAT 的分布式时钟（DC）实现纳秒级同步：

```
DC 同步过程：
1. 主站广播 SYNC0 信号
2. 从站锁定本地时钟到 SYNC0
3. 补偿传播延迟
4. 实现全局时间同步

时间戳计算：
T_global = T_local + Offset + Drift * t
```

### 硬实时控制策略

硬实时控制要求系统在确定的时间内完成任务，任何延迟都可能导致系统失败。

**RT_PREEMPT 内核配置**：

```bash
# 内核配置选项
CONFIG_PREEMPT_RT=y
CONFIG_HIGH_RES_TIMERS=y
CONFIG_NO_HZ_FULL=y
CONFIG_HZ_1000=y

# CPU 隔离
isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3

# IRQ 亲和性
echo 2 > /proc/irq/24/smp_affinity  # 网卡中断绑定到 CPU2
```

**实时调度策略**：

```cpp
class RealtimeController {
  void configure_realtime() {
    // 设置调度策略
    struct sched_param param;
    param.sched_priority = 49;  // RT 优先级
    
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param)) {
      throw std::runtime_error("Failed to set realtime priority");
    }
    
    // 内存锁定
    if (mlockall(MCL_CURRENT | MCL_FUTURE)) {
      throw std::runtime_error("Failed to lock memory");
    }
    
    // 预分配堆栈
    stack_prefault();
    
    // CPU 亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);  // 绑定到隔离的 CPU3
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
  }
  
private:
  void stack_prefault() {
    unsigned char dummy[MAX_STACK_SIZE];
    memset(dummy, 0, MAX_STACK_SIZE);
  }
};
```

**WCET 分析（最坏执行时间）**：

```
控制循环 WCET 分解：
┌─────────────────────────────────┐
│ 硬件读取        │  50 μs │ 5%  │
│ 传感器融合      │ 100 μs │ 10% │
│ 轨迹插值        │  80 μs │ 8%  │
│ 逆运动学        │ 150 μs │ 15% │
│ 控制器计算      │ 200 μs │ 20% │
│ 动力学补偿      │ 180 μs │ 18% │
│ 安全检查        │  40 μs │ 4%  │
│ 硬件写入        │  50 μs │ 5%  │
│ 通信开销        │ 150 μs │ 15% │
├─────────────────────────────────┤
│ 总 WCET         │1000 μs │100% │
└─────────────────────────────────┘
```

### 多机器人同步控制

多机器人协作需要精确的时间同步和协调控制：

**时间同步协议**：

1. **PTP (IEEE 1588) 精确时间协议**：
   ```
   同步精度：< 1 μs
   
   PTP 消息序列：
   Master → Slave: Sync (t1)
   Master → Slave: Follow_Up (t1)
   Slave → Master: Delay_Req (t2)
   Master → Slave: Delay_Resp (t3, t4)
   
   时钟偏移计算：
   Offset = ((t2-t1) - (t4-t3)) / 2
   ```

2. **DDS 时间同步**：
   ```cpp
   class MultiRobotSync {
     void setup_dds_qos() {
       // 配置时间同步 QoS
       dds::core::policy::DestinationOrder order = 
         dds::core::policy::DestinationOrder::BySourceTimestamp();
       
       dds::core::policy::History history = 
         dds::core::policy::History::KeepLast(10);
       
       dds::core::policy::Reliability reliability = 
         dds::core::policy::Reliability::Reliable();
     }
   };
   ```

**协调控制架构**：

```
多机器人控制架构：
┌─────────────────────────────────────┐
│      协调控制器 (Coordinator)        │
│   ┌──────────┬──────────────────┐   │
│   │任务分配器│ 同步管理器        │   │
│   └──────────┴──────────────────┘   │
└───────┬──────────┬──────────┬───────┘
        │          │          │
   ┌────▼───┐ ┌───▼────┐ ┌───▼────┐
   │Robot 1 │ │Robot 2 │ │Robot 3 │
   └────────┘ └────────┘ └────────┘
```

**同步运动控制**：

```cpp
class SynchronizedMotionController {
private:
  // 同步屏障
  std::barrier sync_barrier_;
  
  // 共享轨迹时钟
  std::atomic<double> global_time_;
  
public:
  void synchronized_update() {
    // Phase 1: 本地计算
    compute_local_trajectory();
    
    // 同步点 1：等待所有机器人完成计算
    sync_barrier_.arrive_and_wait();
    
    // Phase 2: 交换状态信息
    broadcast_state();
    receive_neighbor_states();
    
    // 同步点 2：等待状态交换完成
    sync_barrier_.arrive_and_wait();
    
    // Phase 3: 协调控制
    compute_coordinated_control();
    
    // 同步点 3：同时执行
    sync_barrier_.arrive_and_wait();
    execute_control();
  }
};
```

### 论文导读与开源项目

**关键论文**：

1. **"Real-Time Control of Industrial Robots Using EtherCAT"** (2019)
   - 作者：Delgado et al.
   - 贡献：EtherCAT 在工业机器人中的最佳实践
   - 关键点：分布式时钟同步、确定性通信

2. **"Adaptive Impedance Control for Human-Robot Collaboration"** (2021)
   - 作者：Wang et al.
   - 贡献：基于学习的自适应阻抗参数调整
   - 关键点：在线参数优化、安全约束

3. **"Multi-Robot Coordination with ros2_control"** (2022)
   - 作者：Kim et al.
   - 贡献：分布式 ros2_control 架构
   - 关键点：时间同步、容错机制

**开源项目推荐**：

1. **ros2_control**
   - GitHub: ros-controls/ros2_control
   - 特点：官方控制框架

2. **EtherCAT for ROS2**
   - GitHub: ICube-Robotics/ethercat_driver_ros2
   - 特点：完整的 EtherCAT 集成

3. **RT_ROS2**
   - GitHub: ros-realtime/rt-kernel-docker-builder
   - 特点：实时内核构建工具

4. **Universal_Robots_ROS2_Driver**
   - GitHub: UniversalRobots/Universal_Robots_ROS2_Driver
   - 特点：UR 机器人官方驱动

### 性能极限优化技巧

**1. 零拷贝通信**：

```cpp
// 使用共享内存避免数据拷贝
class ZeroCopyPublisher {
  void publish_zero_copy() {
    auto loan = publisher_->borrow_loaned_message();
    // 直接写入借用的消息
    fill_message(*loan);
    publisher_->publish(std::move(loan));
  }
};
```

**2. SIMD 优化**：

```cpp
// 使用 AVX2 加速矩阵运算
void matrix_multiply_avx2(float* C, const float* A, const float* B, int n) {
  for (int i = 0; i < n; i += 8) {
    __m256 row = _mm256_load_ps(&A[i]);
    __m256 col = _mm256_load_ps(&B[i]);
    __m256 prod = _mm256_mul_ps(row, col);
    _mm256_store_ps(&C[i], prod);
  }
}
```

**3. 缓存优化**：

```cpp
// 数据布局优化，提高缓存命中率
struct alignas(64) JointData {  // 缓存行对齐
  double position;
  double velocity;
  double effort;
  double command;
  char padding[32];  // 避免伪共享
};
```## 13.8 本章小结

本章深入探讨了 ros2_control 框架的核心概念和实现细节。ros2_control 作为 ROS2 中机器人控制的基石，提供了从硬件抽象到高级控制算法的完整解决方案。

### 关键概念回顾

1. **硬件抽象层**
   - Hardware Interface 提供统一的硬件访问接口
   - Resource Manager 管理硬件资源的分配和访问权限
   - 生命周期管理确保硬件状态的可控转换

2. **控制器架构**
   - 控制器链支持模块化的控制策略组合
   - 实时执行保证确定性的控制性能
   - 动态加载和切换适应不同的控制需求

3. **实时控制**
   - 实时内核和调度策略保证微秒级控制周期
   - 内存预分配和锁定避免运行时延迟
   - WCET 分析确保时间约束满足

4. **高级控制算法**
   - 计算力矩控制实现精确的动力学补偿
   - 阻抗控制提供柔顺的环境交互能力
   - 力控制实现精确的接触力调节

### 核心公式总结

**动力学方程**：
$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

**计算力矩控制律**：
$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q,\dot{q})\dot{q} + G(q)$$

**阻抗控制模型**：
$$M_d(\ddot{x} - \ddot{x}_d) + D_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = F_{ext}$$

**外力估计**：
$$\tau_{ext} = \tau_{measured} - \tau_{model}$$
$$F_{ee} = (J^T)^+ \tau_{ext}$$

**混合力/位置控制**：
$$\tau = J^T(S_f F_d + S_p K_p(x_d - x) + S_p K_d(\dot{x}_d - \dot{x}))$$

### 设计模式与最佳实践

1. **分层设计**：硬件层、控制层、应用层清晰分离
2. **组件化架构**：控制器作为独立组件，支持热插拔
3. **实时优先**：关键路径上避免动态内存分配和系统调用
4. **安全第一**：多层次的安全机制，包括硬件、软件和算法层面
5. **标准化接口**：遵循 ros2_control 标准，确保组件互操作性

## 13.9 练习题

### 基础题

**练习 13.1**：硬件接口设计
设计一个 2-DOF 平面机械臂的硬件接口，要求支持位置和速度两种控制模式。列出需要导出的状态接口和命令接口。

*提示*：考虑每个关节需要的传感器数据和控制命令类型。

<details>
<summary>答案</summary>

硬件接口设计：

状态接口（State Interfaces）：
- joint1/position：关节1位置反馈
- joint1/velocity：关节1速度反馈  
- joint1/effort：关节1力矩反馈
- joint2/position：关节2位置反馈
- joint2/velocity：关节2速度反馈
- joint2/effort：关节2力矩反馈

命令接口（Command Interfaces）：
- joint1/position：关节1位置命令
- joint1/velocity：关节1速度命令
- joint2/position：关节2位置命令
- joint2/velocity：关节2速度命令

配置文件示例：
```xml
<ros2_control name="Planar2DOF" type="system">
  <joint name="joint1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <joint name="joint2">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

</details>

**练习 13.2**：控制周期计算
一个 6-DOF 机械臂的控制器需要执行以下操作：
- 硬件读取：80 μs
- 运动学计算：200 μs
- 控制算法：150 μs
- 硬件写入：70 μs

问：(a) 最小可行的控制周期是多少？(b) 为保证 20% 的安全裕度，应该设置多大的控制周期？

*提示*：考虑 WCET 和系统抖动。

<details>
<summary>答案</summary>

(a) 最小可行控制周期：
总执行时间 = 80 + 200 + 150 + 70 = 500 μs

理论最小周期 = 500 μs

(b) 带安全裕度的控制周期：
安全裕度 20% 意味着实际执行时间应占周期的 80%
控制周期 = 500 / 0.8 = 625 μs

实践中通常选择标准频率：
- 可选择 1000 Hz (1000 μs 周期)，提供 50% 裕度
- 或选择 2000 Hz (500 μs 周期)，需要优化代码

建议：选择 1000 Hz，提供充足的裕度应对抖动和意外延迟。

</details>

**练习 13.3**：重力补偿
一个 3-DOF 机械臂，连杆质量分别为 m₁=2kg, m₂=1.5kg, m₃=1kg，连杆长度 l₁=0.3m, l₂=0.25m, l₃=0.2m。当机械臂处于 q=[0°, 90°, 0°] 时，计算第二个关节的重力矩。假设重力 g=9.81 m/s²。

*提示*：使用递归牛顿-欧拉方法或拉格朗日方法。

<details>
<summary>答案</summary>

第二关节重力矩计算：

关节2需要支撑连杆2和连杆3的重量：

τ₂ = m₂ * g * (l₂/2) * cos(q₂) + m₃ * g * (l₂ * cos(q₂) + l₃/2 * cos(q₂+q₃))

代入数值：
- q₂ = 90° = π/2, cos(90°) = 0
- q₃ = 0°, q₂+q₃ = 90°, cos(90°) = 0

τ₂ = 1.5 * 9.81 * (0.25/2) * 0 + 1.0 * 9.81 * (0.25 * 0 + 0.2/2 * 0)
τ₂ = 0 Nm

当第二关节处于90°时，重力方向与关节轴平行，不产生力矩。

如果 q₂ = 45°：
τ₂ = 1.5 * 9.81 * 0.125 * 0.707 + 1.0 * 9.81 * (0.25 * 0.707 + 0.1 * 0.707)
τ₂ = 1.30 + 2.43 = 3.73 Nm

</details>

### 挑战题

**练习 13.4**：控制器链设计
设计一个控制器链，实现"柔顺插入"任务：机器人需要将一个轴插入孔中，孔的位置有 ±2mm 的不确定性。要求：
1. 列出控制器链中需要的控制器
2. 说明每个控制器的功能
3. 画出数据流图

*提示*：考虑搜索、对准、插入三个阶段。

<details>
<summary>答案</summary>

控制器链设计：

1. **力传感器控制器**（最底层）
   - 读取 F/T 传感器数据
   - 滤波和坐标变换
   - 输出：笛卡尔力/力矩

2. **接触检测控制器**
   - 输入：力传感器数据
   - 检测接触状态和接触点
   - 输出：接触标志和接触力

3. **螺旋搜索控制器**
   - 生成螺旋搜索轨迹
   - 输入：目标位置、搜索半径
   - 输出：位置/速度参考

4. **阻抗控制器**
   - 输入：力误差、位置误差
   - 实现可变刚度控制
   - 输出：速度修正量

5. **混合力/位控制器**
   - X,Y方向：位置控制（搜索）
   - Z方向：力控制（插入）
   - 输出：笛卡尔速度命令

6. **笛卡尔速度控制器**
   - 输入：笛卡尔速度命令
   - 执行逆运动学
   - 输出：关节速度命令

数据流：
```
F/T Sensor → Force Controller → Contact Detector ┐
                                                  ↓
Target Pose → Spiral Search → Impedance → Hybrid F/P → Cartesian → Joint Cmd
                                   ↑                      Velocity
                                   └──────────────────────┘
```

控制策略：
- 阶段1（搜索）：低Z力(2N)，XY螺旋运动
- 阶段2（对准）：检测到孔口，切换到柔顺模式
- 阶段3（插入）：Z力控(10N)，XY柔顺

</details>

**练习 13.5**：实时性能分析
某控制系统在 1kHz 控制频率下运行 1 小时，统计数据如下：
- 平均执行时间：750 μs
- 最大执行时间：980 μs
- 标准差：50 μs
- 超过 900 μs 的次数：15

评估系统的实时性能，并提出优化建议。

*提示*：计算抖动、成功率等指标。

<details>
<summary>答案</summary>

实时性能评估：

1. **基础指标**：
   - 控制周期：1000 μs
   - CPU 利用率：75% (750/1000)
   - 最坏情况利用率：98% (980/1000)
   - 安全裕度：仅 2%

2. **抖动分析**：
   - 标准差：50 μs (5% 的周期)
   - 抖动范围：约 ±150 μs (3σ)
   - 相对抖动：15%

3. **可靠性**：
   - 总执行次数：3,600,000 (1 hour × 1000 Hz)
   - 超时风险事件：15次
   - 成功率：99.9996%

4. **问题识别**：
   - 安全裕度太小（仅 20 μs）
   - 偶发的长延迟（可能是系统中断或 GC）
   - 抖动偏大，影响控制品质

5. **优化建议**：
   - **短期**：
     - 降低控制频率到 800 Hz，增加裕度
     - 优化关键路径代码，目标减少 50 μs
   - **中期**：
     - 使用 CPU 隔离，避免系统干扰
     - 实现分级控制，非关键任务降频
   - **长期**：
     - 迁移到实时内核（RT_PREEMPT）
     - 硬件加速关键算法（FPGA/GPU）

</details>

**练习 13.6**：多机器人同步
两个机器人需要协同搬运一个 2 米长的刚性杆。机器人 A 的控制频率是 1000 Hz，机器人 B 是 800 Hz。设计一个同步方案，确保两端的位置误差不超过 1mm。

*提示*：考虑最小公倍数和插值。

<details>
<summary>答案</summary>

同步方案设计：

1. **频率分析**：
   - Robot A: 1000 Hz (周期 1.0 ms)
   - Robot B: 800 Hz (周期 1.25 ms)
   - 最小公倍数：LCM(1000, 800) = 4000 Hz
   - 同步周期：5 ms (200 Hz)

2. **同步架构**：
```
每 5ms 同步点：
Robot A: 5 个控制周期
Robot B: 4 个控制周期

Timeline:
0ms    1ms    2ms    3ms    4ms    5ms
A: ●------●------●------●------●------● (sync)
B: ●--------●--------●--------●--------● (sync)
```

3. **轨迹同步策略**：
   - 使用全局时间戳
   - 每 5ms 交换状态信息
   - 预测-校正算法

4. **误差补偿**：
```python
# Robot A (1000 Hz)
def control_loop_A():
    t_global = get_global_time()
    if t_global % 5ms == 0:
        sync_with_B()
    
    # 局部轨迹插值
    pos_ref = trajectory.sample(t_global)
    
    # 预测 B 的位置
    pos_B_pred = predict_B_position(t_global)
    
    # 协调控制
    correction = compute_coordination_correction(pos_B_pred)
    pos_cmd = pos_ref + correction

# Robot B (800 Hz)
def control_loop_B():
    t_global = get_global_time()
    if t_global % 5ms == 0:
        sync_with_A()
    
    # 三次样条插值提高精度
    pos_ref = trajectory.spline_sample(t_global)
    
    # 类似的协调控制
```

5. **误差分析**：
   - 最大异步时间：2.5 ms
   - 最大速度 100 mm/s 时
   - 最大位置偏差：0.25 mm < 1 mm ✓

6. **容错机制**：
   - 通信延迟补偿
   - 丢包时使用预测值
   - 紧急停止同步

</details>

**练习 13.7**：EtherCAT 网络设计
设计一个 EtherCAT 网络，包含：8 个关节驱动器、2 个力传感器、1 个 IMU、16 个数字 I/O。要求实现 1ms 的控制周期，计算所需的网络带宽和延迟。

*提示*：考虑 PDO 大小和 EtherCAT 帧结构。

<details>
<summary>答案</summary>

EtherCAT 网络设计：

1. **设备清单和 PDO 大小**：
   - 8× 关节驱动器：每个 20 bytes (Tx) + 16 bytes (Rx) = 288 bytes
   - 2× 力传感器：每个 24 bytes (Tx only) = 48 bytes
   - 1× IMU：32 bytes (Tx only) = 32 bytes
   - 16× 数字 I/O：2 bytes (Tx) + 2 bytes (Rx) = 4 bytes
   - 总计：372 bytes

2. **EtherCAT 帧结构**：
   - Ethernet header：14 bytes
   - EtherCAT header：2 bytes
   - Datagram header：10 bytes × 4 = 40 bytes (假设4个数据报)
   - Working Counter：2 bytes × 4 = 8 bytes
   - CRC：4 bytes
   - 总开销：68 bytes

3. **带宽计算**：
   - 有效数据：372 bytes
   - 总帧大小：372 + 68 = 440 bytes
   - 每周期带宽：440 bytes × 1000 Hz = 440 KB/s
   - 比特率：3.52 Mbps
   - 100 Mbps 网络利用率：3.52%

4. **延迟分析**：
   - 传输延迟：440 bytes × 8 / 100 Mbps = 35.2 μs
   - 处理延迟（每从站）：~1 μs × 11 = 11 μs
   - 总线延迟：~46 μs
   - 往返时间：~92 μs

5. **拓扑优化**：
```
Master ─┬─ Joint1 ─ Joint2 ─ Joint3 ─ Joint4
        └─ Joint5 ─ Joint6 ─ Joint7 ─ Joint8
           └─ Force1 ─ Force2 ─ IMU ─ I/O

优化后（星型）：
        ┌─ Branch1: Joint1-4
Master ─┼─ Branch2: Joint5-8
        └─ Branch3: Sensors + I/O
```

6. **实时性保证**：
   - 周期时间：1000 μs
   - 网络往返：92 μs (9.2%)
   - 剩余时间：908 μs 用于控制计算
   - 结论：满足 1ms 周期要求，有充足裕度

</details>

## 13.10 常见陷阱与错误

### 1. 硬件接口陷阱

**问题**：硬件接口状态不一致
```cpp
// 错误：忘记更新所有状态
return_type read() {
  joint_positions_[0] = read_encoder(0);
  // 忘记更新 joint_velocities_!
  return return_type::OK;
}
```

**解决**：
```cpp
return_type read() {
  static rclcpp::Time last_time;
  auto current_time = clock_->now();
  
  for (size_t i = 0; i < joint_count_; ++i) {
    double new_position = read_encoder(i);
    if (last_time.nanoseconds() > 0) {
      joint_velocities_[i] = 
        (new_position - joint_positions_[i]) / 
        (current_time - last_time).seconds();
    }
    joint_positions_[i] = new_position;
  }
  last_time = current_time;
  return return_type::OK;
}
```

### 2. 实时性违规

**问题**：控制循环中的动态内存分配
```cpp
// 错误：运行时分配内存
update() {
  std::vector<double> temp_data(joint_count_);  // 分配！
  // ...
}
```

**解决**：
```cpp
class Controller {
  std::vector<double> temp_data_;  // 预分配
  
  on_configure() {
    temp_data_.resize(joint_count_);
  }
  
  update() {
    // 使用预分配的内存
    std::fill(temp_data_.begin(), temp_data_.end(), 0.0);
  }
};
```

### 3. 控制器冲突

**问题**：多个控制器访问同一命令接口
```yaml
# 错误配置：两个控制器控制同一关节
position_controller:
  joints: [joint1, joint2]
  
velocity_controller:
  joints: [joint1, joint3]  # joint1 冲突！
```

**解决**：使用控制器管理器的互斥检查，或实现控制器切换逻辑。

### 4. 时间戳问题

**问题**：使用系统时间而非 ROS 时间
```cpp
// 错误：使用系统时间
auto now = std::chrono::system_clock::now();
```

**解决**：
```cpp
// 正确：使用 ROS 时间
auto now = node_->now();
// 支持仿真时间和真实时间切换
```

### 5. 单位不一致

**问题**：角度和弧度混用
```cpp
// 错误：混用单位
double joint_limit_deg = 180;
if (joint_position_rad > joint_limit_deg) {  // 错误比较！
  // ...
}
```

**解决**：统一使用 SI 单位（弧度、米、秒）。

### 6. 数值稳定性

**问题**：矩阵求逆的数值问题
```cpp
// 错误：直接求逆
Eigen::MatrixXd M_inv = M.inverse();  // 可能奇异！
```

**解决**：
```cpp
// 使用分解方法
Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
if (ldlt.info() == Eigen::Success) {
  Eigen::VectorXd solution = ldlt.solve(b);
}
```

### 调试技巧

1. **实时性能监控**：
```bash
# 监控控制循环性能
ros2 run controller_manager controller_manager_diagnostics

# 查看实时统计
watch -n 0.1 'cat /proc/$(pidof controller_manager)/sched'
```

2. **控制器状态检查**：
```bash
# 列出所有控制器
ros2 control list_controllers

# 查看控制器状态
ros2 control list_controller_types
```

3. **硬件接口调试**：
```bash
# 列出硬件接口
ros2 control list_hardware_interfaces

# 查看硬件组件状态
ros2 control list_hardware_components
```

## 13.11 最佳实践检查清单

### 设计阶段

- [ ] **需求分析**
  - 确定控制频率要求
  - 识别实时性约束
  - 定义安全需求

- [ ] **架构设计**
  - 选择合适的硬件接口类型
  - 设计控制器链结构
  - 规划资源分配策略

- [ ] **接口定义**
  - 统一使用 SI 单位
  - 明确坐标系定义
  - 文档化所有接口

### 实现阶段

- [ ] **代码质量**
  - 避免动态内存分配
  - 使用 const correctness
  - 实现异常处理

- [ ] **实时保证**
  - 内存预分配和锁定
  - CPU 亲和性设置
  - 避免阻塞调用

- [ ] **数值稳定**
  - 使用稳定的数值算法
  - 检查除零和溢出
  - 验证矩阵条件数

### 测试阶段

- [ ] **单元测试**
  - 测试所有控制模式
  - 验证极限情况
  - 检查错误处理

- [ ] **集成测试**
  - 控制器切换测试
  - 通信延迟测试
  - 故障注入测试

- [ ] **性能测试**
  - WCET 分析
  - 抖动测量
  - 长时间稳定性测试

### 部署阶段

- [ ] **系统配置**
  - 实时内核配置
  - 网络优化
  - 日志级别设置

- [ ] **安全检查**
  - 紧急停止测试
  - 碰撞检测验证
  - 限位保护确认

- [ ] **文档完善**
  - 参数说明文档
  - 故障排除指南
  - 维护手册

### 维护阶段

- [ ] **监控指标**
  - 控制性能监控
  - 硬件健康检查
  - 日志分析

- [ ] **优化改进**
  - 性能瓶颈分析
  - 参数调优
  - 算法升级

- [ ] **知识管理**
  - 问题记录
  - 经验总结
  - 培训材料更新