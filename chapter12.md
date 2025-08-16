# 第 12 章：导航栈 Nav2

## 章节大纲

1. 开篇段落
2. 文字论述
   - 12.1 Nav2 架构概览
   - 12.2 行为树（Behavior Trees）架构
   - 12.3 路径规划器设计
   - 12.4 控制器与轨迹跟踪
   - 12.5 恢复行为与异常处理
   - 12.6 动态障碍物处理
3. 产业案例研究：Fetch Robotics 仓库导航解决方案
4. 高级话题：社会导航与人机共存
5. 本章小结
6. 练习题
7. 常见陷阱与错误
8. 最佳实践检查清单

---

## 开篇段落

Nav2（Navigation2）是 ROS2 中用于自主导航的完整软件栈，它继承并改进了 ROS1 中的 move_base 导航框架。本章将深入探讨 Nav2 的核心架构，重点介绍行为树（Behavior Trees）这一革命性的任务编排机制，以及各种路径规划器和控制器的设计原理。通过学习本章，您将掌握如何设计和调优复杂的机器人导航系统，处理动态环境中的各种挑战，并了解工业界在大规模部署中的最佳实践。

## 12.1 Nav2 架构概览

### 系统组成

Nav2 采用模块化设计，主要包含以下核心组件：

```
                    ┌─────────────────┐
                    │   Nav2 BT      │
                    │  Navigator     │
                    └────────┬────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
   ┌────▼────┐         ┌────▼────┐         ┌────▼────┐
   │ Planner │         │Controller│         │Recovery │
   │ Server  │         │  Server  │         │ Server  │
   └─────────┘         └──────────┘         └─────────┘
        │                    │                    │
   ┌────▼────┐         ┌────▼────┐         ┌────▼────┐
   │ Global  │         │  Local   │         │  Spin   │
   │ Costmap │         │ Costmap  │         │ Clear   │
   └─────────┘         └──────────┘         └─────────┘
```

### 生命周期管理

Nav2 的所有节点都实现了 ROS2 的生命周期接口，支持以下状态转换：

$$
\text{Unconfigured} \xrightarrow{\text{configure}} \text{Inactive} \xrightarrow{\text{activate}} \text{Active}
$$

这种设计允许系统在运行时动态重配置，无需重启整个导航栈。生命周期管理器（Lifecycle Manager）负责协调所有节点的状态转换：

```python
# 生命周期转换序列
lifecycle_sequence = [
    'map_server',
    'amcl',
    'controller_server',
    'planner_server',
    'recoveries_server',
    'bt_navigator'
]
```

### 坐标系约定

Nav2 遵循 REP-105 坐标系约定：

- **map**: 全局固定坐标系，用于长期路径规划
- **odom**: 里程计坐标系，短期内连续但长期会漂移
- **base_link**: 机器人本体坐标系
- **base_footprint**: 机器人在地面的投影

坐标变换关系：
$$
T_{map}^{base} = T_{map}^{odom} \cdot T_{odom}^{base}
$$

## 12.2 行为树（Behavior Trees）架构

### 行为树基础

行为树是一种用于描述任务执行流程的层次化结构，相比传统的有限状态机（FSM），具有更好的模块化和可重用性。

基本节点类型：

1. **控制节点（Control Nodes）**：
   - Sequence: 顺序执行子节点，任一失败则失败
   - Fallback: 依次尝试子节点，任一成功则成功  
   - Parallel: 并行执行多个子节点

2. **装饰节点（Decorator Nodes）**：
   - Inverter: 反转子节点结果
   - RetryUntilSuccessful: 重试直到成功
   - RateController: 限制执行频率

3. **动作节点（Action Nodes）**：
   - ComputePathToPose: 计算到目标的路径
   - FollowPath: 跟踪路径
   - Spin: 原地旋转

### Nav2 标准行为树

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1">
            <ComputePathToPose goal="{goal}" path="{path}"/>
            <ClearEntireCostmap service_name="global_costmap/clear_entirely"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1">
          <FollowPath path="{path}"/>
          <ClearEntireCostmap service_name="local_costmap/clear_entirely"/>
        </RecoveryNode>
      </PipelineSequence>
      <SequenceStar name="RecoveryActions">
        <ClearEntireCostmap service_name="local_costmap/clear_entirely"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely"/>
        <Spin spin_dist="1.57"/>
        <Wait wait_duration="5"/>
      </SequenceStar>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### 自定义行为树节点

创建自定义 BT 节点需要继承相应的基类：

```cpp
class CheckBattery : public BT::ConditionNode {
public:
  CheckBattery(const std::string& name,
                const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("min_battery") };
  }

  BT::NodeStatus tick() override {
    double min_battery;
    getInput("min_battery", min_battery);
    
    if (battery_level_ < min_battery) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};
```

## 12.3 路径规划器设计

### 全局规划器

Nav2 提供多种全局路径规划算法：

1. **NavFn Planner**（Dijkstra/A*）:
   
   A* 启发式函数：
   $$
   f(n) = g(n) + h(n)
   $$
   其中 $g(n)$ 是起点到节点 $n$ 的实际代价，$h(n)$ 是节点 $n$ 到目标的启发式估计。

2. **Smac Planner** (State Lattice/Hybrid-A*):
   
   Hybrid-A* 结合了连续空间和离散搜索：
   $$
   \text{State} = (x, y, \theta)
   $$
   
   运动原语基于 Dubins 或 Reeds-Shepp 曲线。

3. **Theta* Planner**（任意角度路径）:
   
   Line-of-sight 检查实现任意角度路径：
   $$
   \text{parent}(s) = \begin{cases}
   \text{parent}(s') & \text{if LOS}(\text{parent}(s'), s) \\
   s' & \text{otherwise}
   \end{cases}
   $$

### 路径平滑

原始路径通常需要平滑处理以提高可执行性：

1. **Simple Smoother**：基于梯度下降的平滑
   $$
   \min \sum_{i} \alpha \|p_i - p_i^{orig}\|^2 + \beta \|p_{i-1} - 2p_i + p_{i+1}\|^2
   $$

2. **Constrained Smoother**：保持障碍物约束的平滑
   
   约束优化问题：
   $$
   \begin{aligned}
   \min_{\{p_i\}} & \sum_{i} w_{smooth} \cdot \text{curvature}(p_i) \\
   \text{s.t.} & \quad \text{distance}(p_i, \text{obstacles}) > d_{min}
   \end{aligned}
   $$

### 代价地图（Costmap）

代价地图是路径规划的基础，将传感器数据转换为规划可用的代价值：

```
代价值范围：
- 0: 自由空间
- 1-252: 不同程度的代价
- 253: 膨胀半径内
- 254: 致命障碍物（碰撞）
- 255: 未知区域
```

膨胀函数：
$$
\text{cost}(d) = \begin{cases}
254 & d \leq r_{inscribed} \\
253 \cdot e^{-\alpha(d - r_{inscribed})} & r_{inscribed} < d \leq r_{inflation} \\
0 & d > r_{inflation}
\end{cases}
$$

## 12.4 控制器与轨迹跟踪

### DWB 控制器

Dynamic Window Based (DWB) 控制器基于动态窗口方法，在速度空间中搜索最优控制：

速度搜索空间：
$$
V_s = \{(v, \omega) | v \in [v_{min}, v_{max}], \omega \in [\omega_{min}, \omega_{max}]\}
$$

动态窗口约束：
$$
V_d = \{(v, \omega) | v \in [v_c - \dot{v} \cdot \Delta t, v_c + \dot{v} \cdot \Delta t]\}
$$

评价函数：
$$
G(v, \omega) = \sigma(\alpha \cdot \text{heading}(v, \omega) + \beta \cdot \text{dist}(v, \omega) + \gamma \cdot \text{vel}(v, \omega))
$$

### MPPI 控制器

Model Predictive Path Integral (MPPI) 控制器使用采样优化方法：

控制序列优化：
$$
u^* = \arg\min_{u} \mathbb{E}[\sum_{t=0}^{T} L(x_t, u_t) + \Phi(x_T)]
$$

权重更新：
$$
w_i = \frac{e^{-\frac{1}{\lambda}S_i}}{\sum_j e^{-\frac{1}{\lambda}S_j}}
$$

### 纯追踪控制器

Pure Pursuit 控制器通过跟踪前视点实现路径跟踪：

转向角计算：
$$
\delta = \arctan\left(\frac{2L\sin\alpha}{l_d}\right)
$$

其中 $L$ 是轴距，$\alpha$ 是前视点角度，$l_d$ 是前视距离。

自适应前视距离：
$$
l_d = k_v \cdot v + l_{d,min}
$$

## 12.5 恢复行为与异常处理

### 恢复行为设计

Nav2 提供了多层次的恢复机制来处理导航失败：

1. **局部恢复**：
   - Clear Costmap: 清除代价地图中的动态障碍物
   - Spin: 原地旋转以获取更多环境信息
   - Back Up: 后退一定距离

2. **全局恢复**：
   - Wait: 等待动态障碍物离开
   - Assisted Teleop: 请求人工干预

恢复决策树：
```
导航失败
├── 路径规划失败？
│   ├── 清除全局代价地图
│   └── 重新规划
├── 路径跟踪失败？
│   ├── 清除局部代价地图
│   ├── 原地旋转
│   └── 后退重试
└── 多次失败？
    └── 请求人工干预
```

### 异常检测与处理

实时监控导航状态，及时检测异常：

1. **振荡检测**：
   $$
   \text{oscillation} = \frac{1}{T} \int_0^T |\dot{\theta}(t)| dt > \theta_{threshold}
   $$

2. **卡死检测**：
   $$
   \text{stuck} = \|p_t - p_{t-\Delta t}\| < \epsilon \quad \forall t \in [t_0, t_0 + T_{stuck}]
   $$

3. **路径偏离检测**：
   $$
   d_{lateral} = \min_{p \in \text{path}} \|p_{robot} - p\| > d_{max}
   $$

### 多层安全保障

```cpp
class SafetyController {
  bool checkCollision(const Trajectory& traj) {
    // 层级1：几何碰撞检测
    if (geometricCollisionCheck(traj)) return true;
    
    // 层级2：动态障碍物预测
    if (predictiveCollisionCheck(traj)) return true;
    
    // 层级3：紧急制动距离
    double stop_dist = v * v / (2 * a_max);
    if (stop_dist > clearance) return true;
    
    return false;
  }
};
```

## 12.6 动态障碍物处理

### 障碍物检测与跟踪

使用多传感器融合进行障碍物检测：

```
传感器融合架构：
LaserScan ──┐
            ├── Obstacle ──> Tracking ──> Prediction
PointCloud ─┘    Layer
```

卡尔曼滤波器跟踪动态障碍物：

状态向量：
$$
x = [p_x, p_y, v_x, v_y]^T
$$

预测步骤：
$$
\begin{aligned}
x_{k|k-1} &= F x_{k-1|k-1} \\
P_{k|k-1} &= F P_{k-1|k-1} F^T + Q
\end{aligned}
$$

更新步骤：
$$
\begin{aligned}
K_k &= P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1} \\
x_{k|k} &= x_{k|k-1} + K_k(z_k - H x_{k|k-1})
\end{aligned}
$$

### 速度障碍物方法

Velocity Obstacles (VO) 用于动态避障：

速度障碍物集合：
$$
VO_{A|B} = \{v_A | \exists t \geq 0: p_A + tv_A \in B \oplus -A\}
$$

可行速度空间：
$$
V_{feasible} = V_{reachable} \setminus \bigcup_i VO_{A|O_i}
$$

### 社会力模型

在人群中导航时使用社会力模型：

$$
F_{total} = F_{goal} + \sum_i F_{obstacle}^i + \sum_j F_{social}^j
$$

目标吸引力：
$$
F_{goal} = k_{goal} \cdot \frac{p_{goal} - p_{robot}}{\|p_{goal} - p_{robot}\|}
$$

社会排斥力：
$$
F_{social} = A \cdot e^{(r_{ij} - d_{ij})/B} \cdot \vec{n}_{ij}
$$

### 时空规划

考虑时间维度的路径规划：

状态空间扩展：
$$
\text{State} = (x, y, t)
$$

时空代价函数：
$$
C(x, y, t) = C_{static}(x, y) + \sum_i C_{dynamic}^i(x, y, t)
$$

动态障碍物预测轨迹：
$$
p_i(t) = p_i(0) + v_i \cdot t + \frac{1}{2} a_i \cdot t^2
$$

## 产业案例研究：Fetch Robotics 仓库导航解决方案

### 项目背景

Fetch Robotics 为大型电商仓库提供自主移动机器人（AMR）解决方案，其 Freight 系列机器人需要在复杂的仓库环境中实现 24/7 不间断运行。主要挑战包括：

- 高密度动态环境（人员、叉车、其他机器人）
- 狭窄通道导航（过道宽度仅 1.2m）
- 实时任务调度（平均响应时间 < 30s）
- 多机器人协调（单仓库 50+ 机器人）

### 技术选型决策

1. **导航框架选择**：
   - ROS1 Navigation Stack → Nav2 迁移
   - 决策原因：生命周期管理、更好的实时性、行为树灵活性

2. **规划器配置**：
   ```yaml
   planner_server:
     ros__parameters:
       planner_plugins: ["GridBased", "Lattice"]
       GridBased:
         plugin: "nav2_navfn_planner/NavfnPlanner"
         use_astar: true
         allow_unknown: false
       Lattice:
         plugin: "nav2_smac_planner/SmacPlannerLattice"
         lattice_filepath: "warehouse_lattice.json"
         w_smooth: 0.3
         w_data: 0.2
   ```

3. **控制器选择**：
   - 直线段：Regulated Pure Pursuit（效率优先）
   - 转弯/对接：MPPI Controller（精度优先）

### 性能指标与优化

关键性能指标（KPIs）：

| 指标 | 目标值 | 实际达成 | 优化措施 |
|------|-------|---------|---------|
| 平均速度 | 1.2 m/s | 1.15 m/s | 动态速度调节 |
| 定位精度 | ±5 cm | ±3 cm | UWB 辅助定位 |
| 路径效率 | >85% | 89% | A* 启发式优化 |
| 碰撞率 | <0.01% | 0.008% | 多层安全检测 |
| 系统可用性 | >99% | 99.3% | 自动恢复机制 |

优化实践：

1. **代价地图分层**：
   ```cpp
   // 静态层：仓库布局
   static_layer:
     map_topic: "warehouse_map"
     subscribe_to_updates: false
   
   // 动态层：实时障碍物
   obstacle_layer:
     observation_sources: laser_scan point_cloud
     laser_scan:
       clearing: true
       marking: true
       max_obstacle_height: 1.8
   
   // 膨胀层：安全距离
   inflation_layer:
     inflation_radius: 0.55
     cost_scaling_factor: 3.0
   ```

2. **行为树优化**：
   ```xml
   <!-- 仓库特定行为树 -->
   <RecoveryNode number_of_retries="3">
     <Sequence>
       <!-- 检查货架占用 -->
       <CheckShelfOccupancy shelf_id="{target_shelf}"/>
       <!-- 动态重规划 -->
       <RateController hz="2.0">
         <ComputePathToPose goal="{goal}" path="{path}"/>
       </RateController>
       <!-- 精确对接 -->
       <PreciseDocking tolerance="0.02"/>
     </Sequence>
     <ForceSuccess>
       <RequestHumanAssistance/>
     </ForceSuccess>
   </RecoveryNode>
   ```

### 踩坑与解决方案

1. **问题：狭窄通道振荡**
   - 现象：机器人在 1.2m 宽通道中左右振荡
   - 原因：DWB 控制器评分函数不平衡
   - 解决：
   ```yaml
   DWBLocalPlanner:
     critics:
       - RotateToGoal: 
           scale: 32.0
           slowing_factor: 5.0
       - Oscillation:
           scale: 1.0
           oscillation_reset_dist: 0.05
   ```

2. **问题：多机器人死锁**
   - 现象：狭窄通道两机器人相向而行死锁
   - 解决：实现分布式协调协议
   ```python
   def resolve_deadlock(self, other_robot_id):
       priority = self.compute_priority()
       if priority < other_priority:
           # 后退到最近的避让点
           self.navigate_to_passing_zone()
       else:
           # 等待对方避让
           self.wait_for_clearance()
   ```

3. **问题：动态障碍物误检**
   - 现象：地面反光被误识别为障碍物
   - 解决：多传感器投票机制 + 高度过滤

### 大规模部署经验

1. **集中式地图管理**：
   - 所有机器人共享同一高精度地图
   - 增量更新机制（仅传输变化部分）
   - 版本控制确保一致性

2. **分布式路径协调**：
   ```python
   class FleetCoordinator:
       def coordinate_paths(self, robot_paths):
           # 时空预留表
           spacetime_map = SpaceTimeMap()
           
           for robot_id, path in robot_paths.items():
               # 检测冲突
               conflicts = spacetime_map.check_conflicts(path)
               
               if conflicts:
                   # 协商解决
                   resolved_path = self.negotiate_path(
                       robot_id, path, conflicts
                   )
                   spacetime_map.reserve(robot_id, resolved_path)
   ```

3. **性能监控系统**：
   - Prometheus + Grafana 实时监控
   - 关键指标：CPU 使用率、规划延迟、控制频率
   - 自动性能降级：高负载时降低规划频率

## 高级话题：社会导航与人机共存

### 前沿研究方向

#### 1. 社会感知导航（Socially-Aware Navigation）

传统导航将人类视为静态或动态障碍物，而社会导航考虑人类的社会规范和舒适度：

**社会代价函数**：
$$
C_{social} = w_1 \cdot C_{proximity} + w_2 \cdot C_{visibility} + w_3 \cdot C_{motion} + w_4 \cdot C_{personal\_space}
$$

其中：
- $C_{proximity}$：与人的距离代价
- $C_{visibility}$：进入人的视野代价  
- $C_{motion}$：相对运动模式代价
- $C_{personal\_space}$：个人空间侵犯代价

**Proxemics 理论应用**：

Hall's Proxemics 空间区域：
- 亲密距离（0-0.45m）：$C = \infty$（禁止进入）
- 个人距离（0.45-1.2m）：$C = K_1 \cdot e^{-d/\sigma_1}$
- 社交距离（1.2-3.6m）：$C = K_2 \cdot e^{-d/\sigma_2}$
- 公共距离（>3.6m）：$C = 0$

#### 2. 人类运动预测

**Social LSTM 模型**：

隐藏状态更新考虑社会池化：
$$
h_i^t = \text{LSTM}(h_i^{t-1}, e_i^t, S_i^t)
$$

社会池化层：
$$
S_i^t = \sum_{j \in \mathcal{N}_i} \phi(h_j^{t-1})
$$

**Trajectron++ 多模态预测**：

条件概率分布：
$$
p(\mathbf{y}^{t+1:t+T}_i | \mathbf{x}^{t-H:t}) = \sum_k \pi_k \mathcal{N}(\mu_k, \Sigma_k)
$$

#### 3. 意图识别与交互

**注视方向估计**：
$$
\vec{g} = R(\theta_{head}) \cdot R(\theta_{gaze}) \cdot \vec{e}_z
$$

**手势识别状态机**：
```
停止手势 → 减速 → 等待确认
挥手召唤 → 接近模式 → 服务交互
指向手势 → 方向理解 → 路径调整
```

### Paper Reading Guide

1. **"Socially Aware Motion Planning with Deep Reinforcement Learning"** (IROS 2017)
   - Chen, Chen, Liu et al.
   - 关键贡献：CADRL 算法，多智能体避碰
   - 实现要点：值函数分解、分布式训练

2. **"Social GAN: Socially Acceptable Trajectories with GANs"** (CVPR 2018)
   - Gupta, Johnson et al.
   - 关键贡献：对抗训练生成社会兼容轨迹
   - 实现要点：池化模块、变分判别器

3. **"Robot Navigation in Constrained Pedestrian Environments using RL"** (ICRA 2020)
   - Everett, Chen, How
   - 关键贡献：SA-CADRL，考虑运动学约束
   - 实现要点：注意力机制、课程学习

### 开源项目推荐

1. **SPENCER Robot Platform**
   - 机场社会导航完整方案
   - 人群跟踪、群体检测、社会力模型
   - GitHub: spencer-project/spencer_people_tracking

2. **Social Navigation Layers**
   - Nav2 社会层插件
   - 个人空间建模、通过区域标记
   - GitHub: SteveMacenski/social_navigation_layers

3. **CrowdNav**
   - 强化学习社会导航仿真
   - 多种 RL 算法实现对比
   - GitHub: vita-epfl/CrowdNav

### 性能极限优化技巧

#### 1. GPU 加速人群预测

```cpp
// CUDA 核函数：批量轨迹预测
__global__ void predictTrajectories(
    float* positions, float* velocities,
    float* predictions, int num_agents, int horizon
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < num_agents) {
        // 社会力计算
        float3 social_force = computeSocialForce(idx, positions);
        
        // 轨迹积分
        for (int t = 0; t < horizon; t++) {
            predictions[idx * horizon + t] = 
                integrateMotion(positions[idx], 
                               velocities[idx], 
                               social_force, t);
        }
    }
}
```

#### 2. 实时注意力机制

```python
class EfficientSocialAttention(nn.Module):
    def __init__(self, d_model, num_heads):
        super().__init__()
        self.attention = nn.MultiheadAttention(
            d_model, num_heads, batch_first=True
        )
        # 稀疏注意力模式
        self.sparsity_mask = self.create_sparse_mask()
    
    def forward(self, x, neighbors):
        # 仅关注最近的 K 个邻居
        top_k = torch.topk(distances, k=self.k, largest=False)
        sparse_attn = self.attention(
            x, neighbors[top_k.indices], 
            attn_mask=self.sparsity_mask
        )
        return sparse_attn
```

#### 3. 分层决策架构

```yaml
# 三层决策架构
high_level:
  frequency: 1 Hz
  tasks:
    - goal_selection
    - route_planning
    - social_context_analysis

mid_level:
  frequency: 10 Hz
  tasks:
    - local_planning
    - crowd_navigation
    - gesture_recognition

low_level:
  frequency: 50 Hz
  tasks:
    - collision_avoidance
    - trajectory_tracking
    - emergency_stop
```

### 实际应用案例

#### 服务机器人导航

```python
class SocialNavigator:
    def __init__(self):
        self.comfort_distance = 1.2  # 米
        self.social_force_model = SocialForceModel()
        self.gesture_recognizer = GestureRecognizer()
    
    def plan_social_path(self, start, goal, people_tracks):
        # 预测人群未来轨迹
        predictions = self.predict_crowd_motion(people_tracks)
        
        # 生成社会兼容路径
        path = self.astar_social(
            start, goal, 
            static_costmap=self.costmap,
            dynamic_costs=self.compute_social_costs(predictions)
        )
        
        # 速度调节
        velocity_profile = self.adapt_velocity_to_crowd(
            path, predictions
        )
        
        return path, velocity_profile
    
    def compute_social_costs(self, predictions):
        costs = np.zeros_like(self.costmap)
        for person_traj in predictions:
            # Gaussian 分布个人空间
            costs += self.gaussian_personal_space(
                person_traj, 
                sigma=self.comfort_distance
            )
        return costs
```

## 本章小结

本章深入探讨了 ROS2 导航栈 Nav2 的核心架构和关键技术。主要内容包括：

### 核心概念

1. **行为树架构**：相比传统状态机，行为树提供了更好的模块化、可重用性和可读性，成为 Nav2 任务编排的核心机制。

2. **路径规划算法**：
   - 全局规划：NavFn (A*/Dijkstra)、Smac (Hybrid-A*)、Theta*
   - 路径平滑：梯度下降平滑、约束优化平滑
   - 代价地图：分层架构、膨胀函数设计

3. **控制器设计**：
   - DWB：基于动态窗口的速度空间搜索
   - MPPI：模型预测路径积分优化
   - Pure Pursuit：几何路径跟踪

4. **恢复机制**：多层次恢复行为、异常检测、安全保障

5. **动态避障**：
   - 障碍物跟踪：卡尔曼滤波
   - 速度障碍物：VO/RVO 方法
   - 社会力模型：人群导航
   - 时空规划：考虑时间维度

### 关键公式

- A* 启发式：$f(n) = g(n) + h(n)$
- 膨胀函数：$\text{cost}(d) = 253 \cdot e^{-\alpha(d - r_{inscribed})}$
- DWB 评价：$G(v, \omega) = \alpha \cdot \text{heading} + \beta \cdot \text{dist} + \gamma \cdot \text{vel}$
- MPPI 优化：$u^* = \arg\min_{u} \mathbb{E}[\sum_{t} L(x_t, u_t)]$
- 社会力：$F_{total} = F_{goal} + \sum F_{obstacle} + \sum F_{social}$

### 工程实践要点

1. 根据环境特点选择合适的规划器和控制器组合
2. 合理配置代价地图层级和参数
3. 设计鲁棒的恢复策略处理异常
4. 在人机共存环境中考虑社会规范
5. 通过分层架构实现实时性能

## 练习题

### 基础题（理解概念）

**练习 12.1**：解释为什么 Nav2 选择行为树而不是有限状态机作为任务编排机制？至少给出三个技术优势。

<details>
<summary>提示</summary>
考虑模块化、并行执行、条件处理、可重用性等方面。
</details>

<details>
<summary>答案</summary>

行为树相比有限状态机的优势：

1. **模块化和可重用性**：行为树的节点是独立的模块，可以在不同的树中重复使用，而 FSM 的状态转换是紧耦合的。

2. **更好的并行处理**：Parallel 节点可以自然地表达并行任务，而 FSM 需要复杂的状态组合来实现并行。

3. **条件处理更清晰**：Fallback 节点提供了优雅的失败处理机制，避免了 FSM 中的状态爆炸问题。

4. **易于调试和可视化**：树形结构直观展示执行流程，每个节点返回明确的状态（成功/失败/运行中）。

5. **动态修改能力**：可以在运行时修改行为树结构，而 FSM 通常需要重新编译。

</details>

**练习 12.2**：在 DWB 控制器中，如果机器人最大速度为 1.0 m/s，最大加速度为 0.5 m/s²，时间步长为 0.1s，计算动态窗口的速度范围。

<details>
<summary>提示</summary>
动态窗口考虑一个时间步内可达的速度范围。
</details>

<details>
<summary>答案</summary>

给定：
- 当前速度：$v_c$（假设为 0.8 m/s）
- 最大加速度：$\dot{v} = 0.5$ m/s²
- 时间步长：$\Delta t = 0.1$ s

动态窗口速度范围：
$$v \in [v_c - \dot{v} \cdot \Delta t, v_c + \dot{v} \cdot \Delta t]$$
$$v \in [0.8 - 0.5 \times 0.1, 0.8 + 0.5 \times 0.1]$$
$$v \in [0.75, 0.85] \text{ m/s}$$

考虑物理限制：
$$v \in [\max(0, 0.75), \min(1.0, 0.85)] = [0.75, 0.85] \text{ m/s}$$

</details>

**练习 12.3**：给定机器人半径 0.3m，障碍物半径 0.2m，计算代价地图的内切圆半径（inscribed radius）和膨胀半径（inflation radius）。

<details>
<summary>提示</summary>
内切圆是机器人中心到边缘的距离，膨胀半径需要考虑安全裕度。
</details>

<details>
<summary>答案</summary>

1. **内切圆半径**：
   - 机器人半径：$r_{robot} = 0.3$ m
   - 内切圆半径：$r_{inscribed} = r_{robot} = 0.3$ m

2. **膨胀半径**：
   - 基础膨胀：$r_{base} = r_{robot} + r_{obstacle} = 0.3 + 0.2 = 0.5$ m
   - 安全裕度（通常 20%）：$r_{safety} = 0.2 \times r_{base} = 0.1$ m
   - 膨胀半径：$r_{inflation} = r_{base} + r_{safety} = 0.6$ m

3. **代价分配**：
   - $d \leq 0.3$ m：cost = 254（致命）
   - $0.3 < d \leq 0.6$ m：cost = $253 \cdot e^{-\alpha(d-0.3)}$（渐变）
   - $d > 0.6$ m：cost = 0（自由）

</details>

### 挑战题（深入思考）

**练习 12.4**：设计一个行为树来处理以下导航场景：机器人需要通过一个可能被占用的狭窄通道，如果通道被占用超过 30 秒，需要寻找替代路径。

<details>
<summary>提示</summary>
使用 Fallback、Timeout、RetryUntilSuccessful 等节点组合。
</details>

<details>
<summary>答案</summary>

```xml
<BehaviorTree ID="NarrowPassageNavigation">
  <Fallback name="MainStrategy">
    <!-- 策略1：尝试通过狭窄通道 -->
    <Sequence name="TryNarrowPassage">
      <CheckPassageClear passage_id="narrow_1"/>
      <Timeout msec="30000">
        <RetryUntilSuccessful num_attempts="6">
          <Sequence>
            <ComputePathThroughPassage path="{path}"/>
            <FollowPath path="{path}"/>
          </Sequence>
        </RetryUntilSuccessful>
      </Timeout>
    </Sequence>
    
    <!-- 策略2：等待并重试 -->
    <Sequence name="WaitAndRetry">
      <Wait wait_duration="5.0"/>
      <CheckPassageClear passage_id="narrow_1"/>
      <ComputePathThroughPassage path="{path}"/>
      <FollowPath path="{path}"/>
    </Sequence>
    
    <!-- 策略3：寻找替代路径 -->
    <Sequence name="AlternativeRoute">
      <SetBlackboard key="avoid_passage" value="narrow_1"/>
      <ComputePathToPose goal="{goal}" path="{alt_path}"
                         blacklist="{avoid_passage}"/>
      <FollowPath path="{alt_path}"/>
    </Sequence>
  </Fallback>
</BehaviorTree>
```

</details>

**练习 12.5**：实现一个简化的 MPPI 控制器，给定代价函数 $L(x,u) = \|x - x_{goal}\|^2 + 0.1\|u\|^2$，采样 100 条轨迹，计算最优控制。

<details>
<summary>提示</summary>
使用重要性采样和加权平均计算最优控制。
</details>

<details>
<summary>答案</summary>

```python
import numpy as np

class SimpleMPPI:
    def __init__(self, num_samples=100, horizon=10, lambda_=1.0):
        self.K = num_samples
        self.T = horizon
        self.lambda_ = lambda_
        
    def compute_control(self, x0, x_goal, dynamics):
        # 采样控制序列
        u_samples = np.random.randn(self.K, self.T, 2) * 0.5
        
        # 评估每条轨迹
        costs = np.zeros(self.K)
        trajectories = []
        
        for k in range(self.K):
            x = x0
            traj = [x]
            cost = 0
            
            for t in range(self.T):
                # 应用动力学
                x = dynamics(x, u_samples[k, t])
                traj.append(x)
                
                # 计算代价
                cost += np.linalg.norm(x - x_goal)**2
                cost += 0.1 * np.linalg.norm(u_samples[k, t])**2
            
            costs[k] = cost
            trajectories.append(traj)
        
        # 计算权重
        min_cost = np.min(costs)
        weights = np.exp(-(costs - min_cost) / self.lambda_)
        weights /= np.sum(weights)
        
        # 加权平均得到最优控制
        u_optimal = np.sum(weights[:, np.newaxis, np.newaxis] * 
                          u_samples, axis=0)
        
        return u_optimal[0]  # 返回第一个控制

# 使用示例
def simple_dynamics(x, u, dt=0.1):
    # 简单的双积分器模型
    return x + u * dt

mppi = SimpleMPPI()
x_current = np.array([0, 0])
x_goal = np.array([10, 10])
u_optimal = mppi.compute_control(x_current, x_goal, simple_dynamics)
```

</details>

**练习 12.6**：在社会导航场景中，机器人检测到前方 2m 处有一个人以 0.5 m/s 的速度横穿，设计一个基于速度障碍物（VO）的避让策略。

<details>
<summary>提示</summary>
计算 VO 锥形区域，选择锥外的可行速度。
</details>

<details>
<summary>答案</summary>

```python
import numpy as np

def compute_velocity_obstacle(robot_pos, human_pos, human_vel, 
                              robot_radius=0.3, human_radius=0.5):
    """
    计算速度障碍物区域
    """
    # 相对位置
    rel_pos = human_pos - robot_pos
    distance = np.linalg.norm(rel_pos)
    
    # Minkowski 和半径
    r_sum = robot_radius + human_radius
    
    # VO 锥的半角
    if distance <= r_sum:
        # 已经碰撞
        return None
    
    theta = np.arcsin(r_sum / distance)
    
    # VO 锥的中心方向
    center_dir = rel_pos / distance
    
    # VO 锥的边界方向
    rot_matrix_left = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    rot_matrix_right = np.array([
        [np.cos(-theta), -np.sin(-theta)],
        [np.sin(-theta), np.cos(-theta)]
    ])
    
    left_bound = rot_matrix_left @ center_dir
    right_bound = rot_matrix_right @ center_dir
    
    # 平移 VO 锥（考虑人的速度）
    vo_apex = human_vel
    
    return {
        'apex': vo_apex,
        'left_bound': left_bound,
        'right_bound': right_bound,
        'center': center_dir
    }

def select_collision_free_velocity(vo, v_desired, v_max=1.0):
    """
    选择避开 VO 的速度
    """
    # 检查期望速度是否在 VO 内
    if is_velocity_in_vo(v_desired, vo):
        # 寻找最近的可行速度
        candidates = []
        
        # 选项1：沿 VO 左边界
        v_left = vo['apex'] + vo['left_bound'] * v_max
        candidates.append(v_left)
        
        # 选项2：沿 VO 右边界  
        v_right = vo['apex'] + vo['right_bound'] * v_max
        candidates.append(v_right)
        
        # 选项3：减速
        v_slow = v_desired * 0.5
        if not is_velocity_in_vo(v_slow, vo):
            candidates.append(v_slow)
        
        # 选择与期望速度最接近的
        best_v = min(candidates, 
                    key=lambda v: np.linalg.norm(v - v_desired))
        return best_v
    else:
        return v_desired

# 应用示例
robot_pos = np.array([0, 0])
human_pos = np.array([2, 0])
human_vel = np.array([0, 0.5])  # 横穿
v_desired = np.array([1.0, 0])   # 机器人想直行

vo = compute_velocity_obstacle(robot_pos, human_pos, human_vel)
v_safe = select_collision_free_velocity(vo, v_desired)
print(f"安全速度: {v_safe}")
```

</details>

**练习 12.7**：设计一个多层恢复策略，处理机器人在电梯门口等待时的各种异常情况。

<details>
<summary>提示</summary>
考虑超时、人群拥挤、电梯故障等情况。
</details>

<details>
<summary>答案</summary>

```python
class ElevatorNavigationRecovery:
    def __init__(self):
        self.max_wait_time = 120  # 秒
        self.crowd_threshold = 5   # 人数
        
    def execute_recovery_strategy(self, context):
        """
        多层恢复策略
        """
        # 层级1：基础等待
        if context['wait_time'] < 30:
            return self.basic_wait(context)
        
        # 层级2：人群处理
        if context['crowd_size'] > self.crowd_threshold:
            return self.handle_crowded_elevator(context)
        
        # 层级3：超时处理
        if context['wait_time'] > self.max_wait_time:
            return self.handle_timeout(context)
        
        # 层级4：电梯故障
        if context['elevator_status'] == 'fault':
            return self.handle_elevator_fault(context)
        
        # 层级5：请求协助
        return self.request_assistance(context)
    
    def basic_wait(self, context):
        return {
            'action': 'wait',
            'position': 'elevator_waiting_zone',
            'duration': 10,
            'monitor': ['door_status', 'crowd_size']
        }
    
    def handle_crowded_elevator(self, context):
        return {
            'action': 'sequence',
            'steps': [
                {'action': 'announce', 
                 'message': 'Waiting for next elevator'},
                {'action': 'move_back', 'distance': 1.0},
                {'action': 'wait', 'duration': 30},
                {'action': 'retry'}
            ]
        }
    
    def handle_timeout(self, context):
        return {
            'action': 'fallback',
            'strategies': [
                {'action': 'find_alternative_route',
                 'avoid': 'elevators'},
                {'action': 'use_stairs',
                 'max_floors': 2},
                {'action': 'reschedule_task',
                 'delay': 300}
            ]
        }
    
    def handle_elevator_fault(self, context):
        return {
            'action': 'sequence',
            'steps': [
                {'action': 'report_fault',
                 'system': 'building_management'},
                {'action': 'update_map',
                 'mark_unavailable': context['elevator_id']},
                {'action': 'find_alternative_route'}
            ]
        }
```

</details>

**练习 12.8**：实现一个基于社会力模型的人群导航算法，考虑 5 个行人的相互作用。

<details>
<summary>提示</summary>
计算目标吸引力、障碍物排斥力和社会排斥力的矢量和。
</details>

<details>
<summary>答案</summary>

```python
import numpy as np

class SocialForceNavigator:
    def __init__(self):
        # 社会力参数
        self.A = 2.0      # 排斥力强度
        self.B = 0.8      # 排斥力范围
        self.k_goal = 1.0 # 目标吸引力系数
        self.tau = 0.5    # 松弛时间
        
    def compute_goal_force(self, pos, goal, v_desired=1.0):
        """目标吸引力"""
        direction = goal - pos
        distance = np.linalg.norm(direction)
        if distance > 0:
            desired_velocity = (direction / distance) * v_desired
            return (desired_velocity - self.velocity) / self.tau
        return np.zeros(2)
    
    def compute_social_force(self, pos, other_pos, other_vel):
        """社会排斥力"""
        diff = pos - other_pos
        distance = np.linalg.norm(diff)
        
        if distance > 0 and distance < 5.0:  # 5m 影响范围
            # 方向单位向量
            n_ij = diff / distance
            
            # 考虑相对速度的各向异性
            v_rel = self.velocity - other_vel
            # 时间到碰撞
            ttc = self.compute_time_to_collision(
                pos, self.velocity, other_pos, other_vel
            )
            
            # 基础排斥力
            f_magnitude = self.A * np.exp((0.5 - distance) / self.B)
            
            # 各向异性因子（前方的人影响更大）
            cos_phi = -np.dot(n_ij, self.velocity) / (
                np.linalg.norm(self.velocity) + 1e-6
            )
            anisotropy = (1 + cos_phi) / 2
            
            return f_magnitude * n_ij * anisotropy
        return np.zeros(2)
    
    def navigate_in_crowd(self, robot_pos, robot_vel, goal, 
                          pedestrians):
        """
        在人群中导航
        pedestrians: [(pos, vel), ...]
        """
        self.velocity = robot_vel
        
        # 计算总力
        force_total = np.zeros(2)
        
        # 目标吸引力
        force_total += self.compute_goal_force(robot_pos, goal)
        
        # 社会排斥力
        for ped_pos, ped_vel in pedestrians:
            force_total += self.compute_social_force(
                robot_pos, ped_pos, ped_vel
            )
        
        # 限制最大力（加速度）
        max_force = 2.0  # m/s²
        if np.linalg.norm(force_total) > max_force:
            force_total = force_total / np.linalg.norm(force_total)
            force_total *= max_force
        
        # 更新速度
        dt = 0.1
        new_velocity = robot_vel + force_total * dt
        
        # 限制最大速度
        max_speed = 1.2
        if np.linalg.norm(new_velocity) > max_speed:
            new_velocity = new_velocity / np.linalg.norm(new_velocity)
            new_velocity *= max_speed
        
        return new_velocity
    
    def compute_time_to_collision(self, p1, v1, p2, v2):
        """计算碰撞时间"""
        rel_pos = p2 - p1
        rel_vel = v2 - v1
        
        # 二次方程系数
        a = np.dot(rel_vel, rel_vel)
        b = 2 * np.dot(rel_pos, rel_vel)
        c = np.dot(rel_pos, rel_pos) - (0.8)**2  # 安全距离
        
        if a < 1e-6:  # 相对静止
            return float('inf')
        
        discriminant = b*b - 4*a*c
        if discriminant < 0:  # 不会碰撞
            return float('inf')
        
        t = (-b - np.sqrt(discriminant)) / (2*a)
        return max(t, 0)

# 使用示例
navigator = SocialForceNavigator()
robot_pos = np.array([0, 0])
robot_vel = np.array([0.5, 0])
goal = np.array([10, 0])

# 5个行人
pedestrians = [
    (np.array([3, 0.5]), np.array([-0.3, 0])),
    (np.array([2, -0.5]), np.array([0.2, 0.1])),
    (np.array([4, 1]), np.array([0, -0.5])),
    (np.array([5, -1]), np.array([-0.1, 0.3])),
    (np.array([3.5, 0]), np.array([0.4, 0]))
]

new_vel = navigator.navigate_in_crowd(
    robot_pos, robot_vel, goal, pedestrians
)
print(f"调整后速度: {new_vel}")
```

</details>

## 常见陷阱与错误

### 1. 代价地图配置错误

**问题**：机器人频繁碰撞或过度保守
```yaml
# 错误配置
inflation_layer:
  inflation_radius: 0.1  # 太小，容易碰撞
  cost_scaling_factor: 10.0  # 太大，过度保守
```

**正确配置**：
```yaml
inflation_layer:
  inflation_radius: 0.55  # 机器人半径 + 安全裕度
  cost_scaling_factor: 3.0  # 平滑代价渐变
```

### 2. 行为树死锁

**问题**：恢复行为相互触发导致死锁
```xml
<!-- 错误：可能无限循环 -->
<RecoveryNode number_of_retries="-1">
  <FollowPath path="{path}"/>
  <ClearCostmap/>
</RecoveryNode>
```

**解决**：设置重试次数限制和超时
```xml
<RecoveryNode number_of_retries="3">
  <Timeout msec="30000">
    <FollowPath path="{path}"/>
  </Timeout>
  <ClearCostmap/>
</RecoveryNode>
```

### 3. 控制器振荡

**症状**：机器人左右摇摆，无法稳定跟踪路径

**原因**：
- 控制增益过高
- 前视距离不当
- 评分函数权重失衡

**调试方法**：
```python
# 记录控制器输出用于分析
def debug_controller_oscillation(self):
    history = []
    for _ in range(100):
        cmd_vel = self.controller.compute_velocity()
        history.append({
            'linear': cmd_vel.linear.x,
            'angular': cmd_vel.angular.z,
            'error': self.path_error
        })
    
    # 检测振荡
    angular_changes = np.diff([h['angular'] for h in history])
    oscillation_score = np.sum(np.abs(angular_changes))
    
    if oscillation_score > threshold:
        self.reduce_controller_gains()
```

### 4. 动态障碍物处理不当

**问题**：对快速移动物体反应不及时

**解决**：提高传感器更新频率和预测范围
```yaml
obstacle_layer:
  observation_sources: laser
  laser:
    data_type: LaserScan
    clearing: true
    marking: true
    expected_update_rate: 0.0  # 不检查更新率
    observation_persistence: 0.0  # 立即清除旧数据
```

### 5. 多机器人协调失败

**问题**：多机器人在狭窄空间死锁

**预防措施**：
```python
class MultiRobotCoordinator:
    def prevent_deadlock(self):
        # 1. 优先级分配
        self.assign_dynamic_priorities()
        
        # 2. 预留时空槽
        self.reserve_spacetime_slots()
        
        # 3. 死锁检测
        if self.detect_circular_wait():
            self.break_deadlock_by_backoff()
```

## 最佳实践检查清单

### 系统设计审查

- [ ] **导航需求分析**
  - 环境类型（结构化/非结构化）
  - 动态障碍物密度
  - 精度要求 vs 速度要求
  - 安全等级要求

- [ ] **算法选择**
  - 全局规划器适配环境特点
  - 控制器匹配机器人运动学
  - 恢复策略覆盖所有失败模式

- [ ] **参数调优**
  - 代价地图分辨率与计算能力平衡
  - 控制频率满足实时要求
  - 安全距离考虑最坏情况

### 实施检查

- [ ] **行为树设计**
  - 无死锁和无限循环
  - 恢复行为有重试限制
  - 关键动作有超时保护

- [ ] **性能优化**
  - CPU 使用率 < 80%
  - 规划延迟 < 100ms
  - 控制频率 ≥ 20Hz

- [ ] **安全机制**
  - 紧急停止始终可用
  - 多层碰撞检测
  - 速度限制动态调整

### 测试验证

- [ ] **单元测试**
  - 各规划器独立测试
  - 控制器极限情况测试
  - 恢复行为触发测试

- [ ] **集成测试**
  - 长时间运行稳定性
  - 多机器人协调测试
  - 异常情况处理测试

- [ ] **现场测试**
  - 真实环境性能验证
  - 人机交互安全性
  - 边缘情况处理能力

### 部署运维

- [ ] **监控指标**
  - 导航成功率
  - 平均完成时间
  - 碰撞/近失事件统计

- [ ] **日志记录**
  - 关键决策点记录
  - 异常情况详细日志
  - 性能指标时序数据

- [ ] **更新维护**
  - 参数版本控制
  - A/B 测试框架
  - 回滚机制准备