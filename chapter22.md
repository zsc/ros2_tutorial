# 第 22 章：神经网络运动控制

## 章节大纲

### 1. 模仿学习与示教学习
- 1.1 行为克隆（Behavioral Cloning）基础
- 1.2 数据集收集与增强策略
- 1.3 分布偏移问题与解决方案
- 1.4 ROS2 中的示教数据管道

### 2. 扩散模型（Diffusion Policy）
- 2.1 扩散模型在机器人控制中的应用
- 2.2 条件生成与轨迹预测
- 2.3 实时推理优化
- 2.4 与传统规划器的对比

### 3. 神经网络 MPC
- 3.1 学习动力学模型
- 3.2 可微分MPC框架
- 3.3 约束处理与安全保证
- 3.4 在线学习与自适应

### 4. 力矩控制与柔顺控制
- 4.1 阻抗控制神经网络
- 4.2 接触丰富任务的学习
- 4.3 力/位混合控制
- 4.4 安全性与稳定性分析

### 5. 产业案例研究：Toyota Research 柔性装配
### 6. 高级话题：ACT 与端到端模仿学习
### 7. 本章小结
### 8. 练习题
### 9. 常见陷阱与错误
### 10. 最佳实践检查清单

---

## 开篇介绍

神经网络运动控制代表了机器人学习领域的前沿方向，它将深度学习的强大表征能力与机器人运动控制的实时性要求相结合。本章将深入探讨如何在 ROS2 环境中实现和部署基于神经网络的运动控制系统，涵盖从模仿学习到扩散模型的最新技术。

传统的机器人控制方法依赖于精确的数学模型和手工设计的控制器，而神经网络方法能够从数据中学习复杂的控制策略，处理高维状态空间，并适应未建模的动力学。这种范式转变为机器人系统带来了前所未有的灵活性和适应性，特别是在接触丰富的操作任务和复杂环境交互中。

本章的学习目标包括：
- 掌握模仿学习的核心技术和实施方法
- 理解扩散模型在机器人控制中的应用
- 实现神经网络增强的模型预测控制
- 设计安全可靠的力控制学习系统
- 部署端到端的神经网络控制器到实际机器人

## 1. 模仿学习与示教学习

### 1.1 行为克隆（Behavioral Cloning）基础

模仿学习是让机器人通过观察专家演示来学习控制策略的方法。在 ROS2 环境中，这涉及到数据收集、模型训练和实时部署的完整流程。

**核心概念**

行为克隆将控制问题转化为监督学习问题：

$$\pi_\theta^* = \arg\min_\theta \mathbb{E}_{(s,a) \sim \mathcal{D}_\text{expert}} \left[ \mathcal{L}(\pi_\theta(s), a) \right]$$

其中 $\pi_\theta$ 是参数化的策略网络，$\mathcal{D}_\text{expert}$ 是专家演示数据集，$\mathcal{L}$ 是损失函数（通常是 MSE 或交叉熵）。

**ROS2 实现架构**

```
┌─────────────────┐     ┌──────────────┐     ┌─────────────┐
│ Teleoperation   │────▶│ Data Logger  │────▶│ Training    │
│ Node            │     │ Node         │     │ Pipeline    │
└─────────────────┘     └──────────────┘     └─────────────┘
         │                      │                     │
         │                      ▼                     ▼
         │              ┌──────────────┐     ┌─────────────┐
         └─────────────▶│ Rosbag2      │     │ Neural      │
                        │ Recording    │     │ Policy Node │
                        └──────────────┘     └─────────────┘
```

### 1.2 数据集收集与增强策略

高质量的演示数据是成功的关键。ROS2 提供了强大的数据收集工具链：

**多模态数据同步**

在收集演示数据时，需要同步多种传感器输入：
- 关节状态 (sensor_msgs/JointState)
- 力/扭矩传感器 (geometry_msgs/WrenchStamped)  
- 视觉输入 (sensor_msgs/Image, sensor_msgs/PointCloud2)
- 触觉反馈 (自定义消息类型)

时间同步使用 ROS2 的 message_filters：

```
┌──────────┐  ┌──────────┐  ┌──────────┐
│ Camera   │  │ Force    │  │ Joint    │
│ 30 Hz    │  │ 1000 Hz  │  │ 500 Hz   │
└────┬─────┘  └────┬─────┘  └────┬─────┘
     │             │             │
     ▼             ▼             ▼
┌─────────────────────────────────────┐
│     TimeSynchronizer (ApproxTime)    │
│         Sync tolerance: 10ms         │
└─────────────────┬───────────────────┘
                  ▼
         ┌──────────────┐
         │ Synchronized │
         │ Data Buffer  │
         └──────────────┘
```

**数据增强技术**

为了提高泛化能力，采用以下增强策略：

1. **时间扰动**：对动作序列添加时间抖动
2. **空间增强**：对视觉输入进行几何变换
3. **噪声注入**：模拟传感器噪声
4. **域随机化**：变化环境参数

$$\tilde{a}_t = a_t + \epsilon_\text{time} + \mathcal{N}(0, \sigma^2)$$

### 1.3 分布偏移问题与解决方案

行为克隆的主要挑战是协变量偏移（covariate shift）问题。当机器人执行学习到的策略时，小的误差会累积，导致状态分布偏离训练分布。

**DAgger (Dataset Aggregation) 算法**

DAgger 通过迭代收集数据来解决这个问题：

1. 初始化：使用初始专家数据训练策略 $\pi_0$
2. 执行：运行当前策略 $\pi_i$ 收集状态
3. 标注：专家为这些状态提供正确动作
4. 聚合：将新数据加入数据集
5. 重训练：更新策略 $\pi_{i+1}$

**误差累积分析**

考虑 T 步的轨迹，误差累积为：

$$\text{Error}(T) \leq \epsilon_\text{single} \cdot \sum_{t=0}^{T-1} \beta^t$$

其中 $\epsilon_\text{single}$ 是单步预测误差，$\beta$ 是误差传播因子。

### 1.4 ROS2 中的示教数据管道

完整的数据管道包括收集、处理、训练和部署：

**数据收集节点设计**

```python
class DemonstrationCollector(Node):
    def __init__(self):
        super().__init__('demo_collector')
        
        # 订阅器
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', 
            self.joint_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image', 
            self.image_callback, 10)
            
        # 数据缓冲区
        self.buffer = DataBuffer(max_size=10000)
        
        # 同步器
        self.sync = ApproximateTimeSynchronizer(
            [self.joint_sub, self.image_sub],
            queue_size=10, 
            slop=0.01)  # 10ms 容差
```

**在线学习与适应**

实时适应是神经网络控制的关键优势：

```
┌──────────────┐     ┌─────────────┐     ┌──────────────┐
│ Experience   │────▶│ Replay      │────▶│ Online       │
│ Buffer       │     │ Buffer      │     │ Training     │
└──────────────┘     └─────────────┘     └──────────────┘
        ▲                                        │
        │                                        ▼
┌──────────────┐                        ┌──────────────┐
│ Robot        │◀───────────────────────│ Policy       │
│ Execution    │                        │ Network      │
└──────────────┘                        └──────────────┘
```

## 2. 扩散模型（Diffusion Policy）

### 2.1 扩散模型在机器人控制中的应用

扩散模型通过逐步去噪过程生成高质量的动作序列，特别适合处理多模态分布和长期规划任务。

**前向扩散过程**

给定轨迹 $\tau_0$，前向过程逐步添加噪声：

$$q(\tau_t | \tau_{t-1}) = \mathcal{N}(\tau_t; \sqrt{1-\beta_t}\tau_{t-1}, \beta_t I)$$

经过 T 步后，$\tau_T \sim \mathcal{N}(0, I)$

**反向去噪过程**

学习反向过程来生成轨迹：

$$p_\theta(\tau_{t-1} | \tau_t, s) = \mathcal{N}(\tau_{t-1}; \mu_\theta(\tau_t, t, s), \Sigma_\theta(\tau_t, t, s))$$

其中 $s$ 是当前观测状态，作为条件输入。

**优势分析**

1. **多模态建模**：自然处理多个可行解
2. **长期规划**：生成完整轨迹而非单步动作
3. **不确定性量化**：通过多次采样评估不确定性

### 2.2 条件生成与轨迹预测

**条件扩散架构**

```
State s ──────┐
              ▼
         ┌──────────┐
         │ Encoder  │
         └────┬─────┘
              │
Goal g ───────┼─────┐
              ▼     ▼
         ┌─────────────┐
         │ Cross       │
         │ Attention   │
         └──────┬──────┘
                │
Noisy τt ───────┼─────┐
                ▼     ▼
         ┌─────────────┐
         │ U-Net       │
         │ Denoiser    │
         └──────┬──────┘
                │
                ▼
           Predicted τ_{t-1}
```

**分类器引导（Classifier Guidance）**

通过添加梯度项增强条件生成：

$$\nabla_{\tau_t} \log p(\tau_{t-1}|\tau_t, s) = \nabla_{\tau_t} \log p(\tau_{t-1}|\tau_t) + \gamma \nabla_{\tau_t} \log p(s|\tau_t)$$

### 2.3 实时推理优化

扩散模型的主要挑战是推理速度。优化策略包括：

**DDIM 采样器**

确定性采样减少去噪步数：

$$\tau_{t-1} = \sqrt{\alpha_{t-1}} \left( \frac{\tau_t - \sqrt{1-\alpha_t} \epsilon_\theta(\tau_t, t)}{\sqrt{\alpha_t}} \right) + \sqrt{1-\alpha_{t-1}} \epsilon_\theta(\tau_t, t)$$

**模型蒸馏**

将多步去噪过程蒸馏为少步模型：

$$\mathcal{L}_\text{distill} = \mathbb{E}_{\tau_0, t} \left[ ||\tau_\text{teacher}^{(N)} - \tau_\text{student}^{(1)}||^2 \right]$$

**推理延迟分析**

| 方法 | 去噪步数 | 推理时间 | 控制频率 |
|------|---------|----------|----------|
| DDPM | 1000 | 2000ms | 0.5 Hz |
| DDIM | 50 | 100ms | 10 Hz |
| 蒸馏模型 | 1-4 | 5-20ms | 50-200 Hz |

### 2.4 与传统规划器的对比

**性能对比表**

| 特性 | 扩散模型 | RRT* | 轨迹优化 |
|------|---------|------|----------|
| 多模态处理 | ✓✓✓ | ✓ | ✗ |
| 计算复杂度 | O(T·N) | O(n log n) | O(H²) |
| 实时性 | 中等 | 低 | 高 |
| 泛化能力 | 高 | 低 | 中 |
| 约束处理 | 软约束 | 硬约束 | 硬约束 |

## 3. 神经网络 MPC

### 3.1 学习动力学模型

神经网络 MPC 使用学习的动力学模型进行预测控制：

**动力学模型架构**

$$s_{t+1} = f_\theta(s_t, a_t) + \epsilon$$

其中 $f_\theta$ 是神经网络，$\epsilon$ 是建模误差。

**集成模型（Ensemble）**

使用多个模型捕捉认知不确定性：

$$\bar{s}_{t+1} = \frac{1}{M} \sum_{i=1}^M f_{\theta_i}(s_t, a_t)$$

$$\text{Var}[s_{t+1}] = \frac{1}{M} \sum_{i=1}^M (f_{\theta_i}(s_t, a_t) - \bar{s}_{t+1})^2$$

### 3.2 可微分MPC框架

**优化问题表述**

$$\min_{a_{0:H-1}} \sum_{t=0}^{H-1} L(s_t, a_t) + L_f(s_H)$$

subject to:
$$s_{t+1} = f_\theta(s_t, a_t)$$
$$a_t \in \mathcal{A}, s_t \in \mathcal{S}$$

**梯度计算**

通过自动微分计算梯度：

$$\frac{\partial L}{\partial a_t} = \frac{\partial L}{\partial s_{t+1}} \cdot \frac{\partial f_\theta}{\partial a_t}$$
