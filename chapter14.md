# 第 14 章：MoveIt2 运动规划

MoveIt2 是 ROS2 生态系统中最重要的运动规划框架，为机械臂、移动操作机器人和其他复杂运动系统提供了完整的运动规划、碰撞检测、运动学求解和抓取规划解决方案。本章将深入探讨 MoveIt2 的核心架构、各种求解器的选择策略、场景表示方法以及高级规划技术。我们将通过 ABB YuMi 双臂机器人的实际装配案例，展示如何在工业场景中应用这些技术，并探讨基于优化的轨迹规划等前沿话题。

## 14.1 MoveIt2 架构概览

MoveIt2 采用模块化设计，将运动规划问题分解为多个独立但相互协作的组件。相比 MoveIt1，MoveIt2 在架构上进行了重大改进，充分利用 ROS2 的实时性特性和 DDS 通信机制，实现了更低的延迟和更高的可靠性。整个系统围绕 Move Group 节点构建，通过标准化的接口与各个功能模块交互，支持插件化扩展。

```
┌─────────────────────────────────────────────────────┐
│                    User Interface                    │
│            (RViz2 Plugin / MoveIt API)              │
└─────────────────┬───────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────┐
│               Move Group Interface                   │
│         (Action Server / Service Server)            │
└─────────────────┬───────────────────────────────────┘
                  │
        ┌─────────┴─────────┬─────────────┬──────────┐
        │                   │             │          │
┌───────▼──────┐  ┌────────▼────────┐  ┌─▼──────────▼─┐
│   Planning   │  │    Kinematic    │  │  Collision   │
│   Pipeline   │  │     Solvers     │  │  Detection   │
│              │  │                 │  │              │
│ OMPL/CHOMP  │  │ KDL/IKFast/etc │  │   FCL/Bullet │
└──────────────┘  └─────────────────┘  └──────────────┘
```

### 14.1.1 核心组件职责

**Move Group** 是 MoveIt2 的中心协调器，负责：
- 接收运动规划请求（通过 Action 或 Service）
- 协调各个组件完成规划流程
- 管理机器人状态（关节位置、速度、加速度）
- 执行轨迹跟踪与监控
- 处理规划场景更新
- 管理运动规划插件生命周期

Move Group 采用插件架构，允许用户根据需求选择不同的规划器、运动学求解器和轨迹处理算法。这种设计提供了极大的灵活性，使得 MoveIt2 能够适应从简单的工业机械臂到复杂的人形机器人等各种应用场景。

**Planning Scene** 维护环境的完整表示：
- 机器人当前状态（包括关节状态、连杆变换、末端执行器状态）
- 环境中的障碍物（静态和动态物体的几何表示）
- 附着对象（机器人抓取的物体，成为机器人模型的一部分）
- 允许碰撞矩阵（ACM，定义哪些物体对之间的碰撞可以忽略）
- 世界几何表示（包括网格、基本形状、点云等）
- 传感器数据集成（深度图像、点云的实时更新）

Planning Scene 通过差分更新机制高效管理场景变化，只传输和处理发生变化的部分，大大减少了通信开销和计算负担。场景监视器（Planning Scene Monitor）持续跟踪环境变化，通过 tf2 获取机器人状态，通过传感器接口更新障碍物信息。

**Motion Planning Pipeline** 处理规划流程：
- 预处理阶段
  - 场景验证（检查初始状态合法性）
  - 目标可达性初步检查
  - 规划请求适配器链（Planning Request Adapters）
  - 添加默认约束和参数
- 规划器调用阶段
  - 选择合适的规划算法（OMPL、CHOMP、Pilz等）
  - 设置规划上下文（时间限制、优化目标）
  - 并行规划尝试（多线程加速）
  - 解的验证和选择
- 后处理阶段
  - 时间参数化（添加速度、加速度信息）
  - 轨迹平滑（减少不必要的振荡）
  - 响应适配器链（Planning Response Adapters）
  - 最终碰撞检查和安全验证

整个管道支持自定义扩展，用户可以插入自己的预处理和后处理步骤，实现特定的优化目标或约束条件。

## 14.2 运动学求解器集成

### 14.2.1 求解器类型与选择策略

MoveIt2 支持多种运动学求解器，每种都有其适用场景。选择合适的求解器对于实现高效可靠的运动规划至关重要。求解器的选择需要考虑机器人的运动学结构、实时性要求、精度需求以及计算资源等多个因素：

**1. KDL (Kinematics and Dynamics Library)**
- 基于数值迭代的通用求解器（Newton-Raphson 方法）
- 适用于任意运动链结构
- 收敛速度慢，可能陷入局部最优
- 对奇异点敏感，需要阻尼处理

```
求解时间复杂度: O(n·m)
其中 n 为迭代次数，m 为自由度
典型求解时间: 5-50ms
成功率: 60-80%（取决于初始猜测）
内存占用: O(m²)
适用场景: 原型开发、非实时应用、通用机器人
```

KDL 使用伪逆雅可比矩阵进行迭代：
$$\Delta q = J^+ \cdot \Delta x + (I - J^+ J) \cdot \nabla H$$
其中 $J^+$ 为雅可比伪逆，$\nabla H$ 为零空间优化项。

**2. IKFast**
- 解析求解器生成器（基于符号计算）
- 为特定机器人生成闭式解
- 速度极快，但需要预先生成
- 生成过程可能耗时数小时

```
求解时间复杂度: O(1)
典型求解时间: <1ms (通常 0.01-0.1ms)
成功率: 100%（在工作空间内）
限制: 仅支持6自由度或特定结构
内存占用: 取决于生成的代码大小（通常几MB）
适用场景: 工业机器人、高频控制、实时系统
```

IKFast 生成流程：
1. 分析机器人 URDF/SDF 模型
2. 符号化 DH 参数或关节链
3. 生成解析解方程组
4. 编译为 C++ 插件

**3. TRAC-IK**
- 结合 KDL 和 SQP（Sequential Quadratic Programming）优化
- 并行运行多个求解器实例
- 提高成功率和速度
- 支持关节限位和任务空间约束

```
算法伪代码:
parallel_for solver in [KDL_Chain, SQP_Solver]:
    solution = solver.solve(target_pose, timeout)
    if solution.valid:
        return first_valid_solution
```

**4. BioIK**
- 基于进化算法（遗传算法 + 粒子群优化）
- 支持多目标优化
- 优雅处理冗余自由度
- 可添加自定义优化目标

目标函数：
$$J = w_p \cdot ||p_{target} - p_{current}||^2 + w_o \cdot ||q_{target} \ominus q_{current}||^2 + \sum_{i} w_i \cdot g_i$$

其中 $g_i$ 为自定义目标（如关节限位、避障等）

### 14.2.2 求解器配置与优化

求解器的性能很大程度上取决于正确的配置。不同的任务和机器人需要不同的参数调优策略：

```yaml
# kinematics.yaml 配置示例
manipulator:
  kinematics_solver: bio_ik/BioIKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
  
  # BioIK 特定参数
  mode: bio2_memetic
  population_size: 100
  elite_count: 10
  
  # 目标权重
  position_weight: 100.0
  orientation_weight: 50.0
  joint_centering_weight: 1.0
  minimal_displacement_weight: 10.0
```

### 14.2.3 混合求解策略

对于复杂任务，可以组合多个求解器形成级联求解策略，利用各求解器的优势互补。这种方法在实际应用中被证明能显著提高求解成功率和效率：

```python
class HybridIKSolver:
    def solve(self, target_pose, timeout=0.1):
        # 第一阶段：快速解析解
        if self.has_analytic_solver:
            solution = self.ikfast_solve(target_pose, timeout=0.001)
            if solution:
                return solution
        
        # 第二阶段：数值优化
        initial_guess = self.get_nearest_configuration()
        solution = self.trac_ik_solve(
            target_pose, 
            initial_guess,
            timeout=timeout * 0.5
        )
        if solution:
            return solution
            
        # 第三阶段：全局搜索
        return self.bio_ik_solve(
            target_pose,
            timeout=timeout * 0.5,
            population_size=200
        )
```

## 14.3 场景规划与碰撞检测

### 14.3.1 场景表示

MoveIt2 使用 Planning Scene 来表示环境：

```
Planning Scene 结构:
├── World Geometry (环境模型)
│   ├── Collision Objects (障碍物)
│   └── Octomap (3D 占据栅格)
├── Robot State (机器人状态)
│   ├── Joint Values
│   ├── Link Transforms
│   └── Attached Objects
└── Allowed Collision Matrix (ACM)
    └── Collision Pairs Enable/Disable
```

### 14.3.2 碰撞检测算法

**FCL (Flexible Collision Library)**
- 基于包围盒层次结构（BVH）
- 支持连续碰撞检测（CCD）

碰撞检测流程：
1. 宽相位（Broad Phase）：AABB 包围盒快速剔除
2. 窄相位（Narrow Phase）：精确几何检测

```
碰撞检测时间复杂度:
宽相位: O(n log n) 使用空间分割
窄相位: O(m) 其中 m 为潜在碰撞对数量
```

**Bullet Physics**
- 物理仿真引擎的碰撞检测
- 支持凸包优化
- 更好的 mesh-mesh 碰撞

### 14.3.3 距离计算与安全边界

安全距离计算对于碰撞避免至关重要：

$$d_{safe} = \min_{i,j} (d(L_i, O_j) - margin_{ij})$$

其中 $L_i$ 为机器人连杆，$O_j$ 为障碍物，$margin_{ij}$ 为安全边界。

```python
class SafetyDistanceMonitor:
    def compute_safety_field(self, robot_state, obstacles):
        distances = {}
        for link in robot_state.links:
            min_distance = float('inf')
            closest_point = None
            
            for obstacle in obstacles:
                dist, point = self.fcl.distance(
                    link.collision_geometry,
                    obstacle.geometry
                )
                if dist < min_distance:
                    min_distance = dist
                    closest_point = point
                    
            distances[link.name] = {
                'distance': min_distance,
                'gradient': self.compute_distance_gradient(
                    link, closest_point
                ),
                'safety_margin': self.margins[link.name]
            }
        
        return distances
```

### 14.3.4 Octomap 集成

Octomap 提供了高效的 3D 环境表示：

```
Octomap 参数优化:
├── Resolution: 0.02m (精度与性能平衡)
├── Max Range: 5.0m (传感器范围)
├── Probability Hit: 0.7 (占据概率更新)
├── Probability Miss: 0.4
└── Clamping Thresholds: [0.12, 0.97]
```

更新方程：
$$P(n|z_{1:t}) = \left[1 + \frac{1-P(n|z_t)}{P(n|z_t)} \cdot \frac{1-P(n|z_{1:t-1})}{P(n|z_{1:t-1})} \cdot \frac{P(n)}{1-P(n)}\right]^{-1}$$

## 14.4 抓取规划

### 14.4.1 抓取生成策略

MoveIt2 的抓取规划包含三个核心步骤：

1. **抓取姿态生成**
2. **可达性分析**
3. **抓取质量评估**

```python
class GraspGenerator:
    def generate_grasps(self, object_mesh, gripper_model):
        grasps = []
        
        # 基于几何的抓取生成
        for point in object_mesh.surface_points:
            for approach_angle in np.linspace(0, 2*np.pi, 16):
                grasp_pose = self.compute_grasp_pose(
                    point, 
                    approach_angle,
                    object_mesh.surface_normal(point)
                )
                
                # 抓取前后姿态
                pre_grasp = self.offset_pose(grasp_pose, -0.1)
                post_grasp = self.offset_pose(grasp_pose, 0.05)
                
                grasps.append({
                    'pose': grasp_pose,
                    'pre_grasp': pre_grasp,
                    'post_grasp': post_grasp,
                    'quality': self.evaluate_grasp(grasp_pose, object_mesh)
                })
        
        return sorted(grasps, key=lambda g: g['quality'], reverse=True)
```

### 14.4.2 抓取质量度量

**力闭合分析（Force Closure）**

抓取质量 ε-metric：
$$\epsilon = \min_{||w||=1} \max_i |w^T \cdot g_i|$$

其中 $g_i$ 为接触点的单位扳手（wrench），$w$ 为外部扰动。

**可操作性椭球（Manipulability Ellipsoid）**

$$\mu = \sqrt{\det(J \cdot J^T)}$$

其中 $J$ 为雅可比矩阵，$\mu$ 越大表示可操作性越好。

### 14.4.3 抓取规划管道

```
抓取执行流程:
1. 目标识别与定位
   ├── 点云分割
   └── 6D 姿态估计
   
2. 抓取候选生成
   ├── GPD (Grasp Pose Detection)
   ├── 基于模板匹配
   └── 深度学习方法
   
3. 可达性过滤
   ├── 逆运动学检查
   └── 碰撞检测
   
4. 轨迹规划
   ├── Approach (接近)
   ├── Grasp (抓取)
   └── Retreat (撤退)
   
5. 执行与监控
   ├── 力/力矩反馈
   └── 视觉伺服
```

## 14.5 任务级规划

### 14.5.1 任务与运动规划集成（TAMP）

任务级规划将符号规划与几何规划结合：

```python
class TaskAndMotionPlanner:
    def plan(self, initial_state, goal_state):
        # 符号层规划
        symbolic_plan = self.task_planner.plan(
            initial_state.symbolic,
            goal_state.symbolic
        )
        
        # 几何可行性检查
        for action in symbolic_plan:
            motion_constraints = self.extract_constraints(action)
            
            # 运动规划
            trajectory = self.motion_planner.plan(
                start=self.get_robot_state(),
                goal=motion_constraints,
                scene=self.planning_scene
            )
            
            if not trajectory:
                # 回溯并重新规划
                return self.backtrack_and_replan(
                    symbolic_plan, 
                    failed_action=action
                )
        
        return self.compile_full_plan(symbolic_plan)
```

### 14.5.2 约束规划

MoveIt2 支持多种约束类型：

**路径约束（Path Constraints）**
- 位置约束：保持末端在特定区域
- 姿态约束：保持工具方向
- 关节约束：限制关节范围
- 可见性约束：保持目标在视野内

约束满足的优化问题：
$$\min_{q} \sum_{i} w_i \cdot ||f_i(q) - t_i||^2$$

受约束于：
$$g_j(q) \leq 0, \quad j = 1...m$$

其中 $f_i$ 为任务空间目标，$g_j$ 为不等式约束。

### 14.5.3 双臂协调

双臂协调需要考虑：

1. **时间同步**
```python
def synchronize_dual_arm_trajectories(traj_left, traj_right):
    # 时间归一化
    t_max = max(traj_left.duration, traj_right.duration)
    
    # 重新参数化
    traj_left_sync = time_parameterize(traj_left, t_max)
    traj_right_sync = time_parameterize(traj_right, t_max)
    
    # 碰撞检查
    for t in np.linspace(0, t_max, 100):
        if check_self_collision(
            traj_left_sync.sample(t),
            traj_right_sync.sample(t)
        ):
            return None
    
    return traj_left_sync, traj_right_sync
```

2. **相对约束维持**
$$C_{relative} = ||T_{left}^{-1} \cdot T_{right} - T_{desired}||_F$$

其中 $T$ 为变换矩阵，$||\cdot||_F$ 为 Frobenius 范数。

## 14.6 轨迹优化与平滑

### 14.6.1 时间最优轨迹参数化（TOTP）

给定几何路径，计算时间最优的速度剖面：

$$\min_{s(t)} \int_0^T dt$$

受约束于：
- 速度约束：$|\dot{q}_i| \leq v_{max,i}$
- 加速度约束：$|\ddot{q}_i| \leq a_{max,i}$
- 加加速度约束：$|\dddot{q}_i| \leq j_{max,i}$

相空间表示：
$$\ddot{s} = u(s, \dot{s})$$

其中 $s$ 为路径参数，边界条件由动力学约束决定。

### 14.6.2 STOMP 平滑算法

STOMP（Stochastic Trajectory Optimization for Motion Planning）通过随机采样优化轨迹：

```python
def stomp_optimize(initial_trajectory, cost_function, iterations=100):
    trajectory = initial_trajectory
    
    for _ in range(iterations):
        # 生成噪声轨迹
        noisy_trajectories = []
        for _ in range(num_samples):
            noise = generate_smooth_noise(trajectory)
            noisy_trajectories.append(trajectory + noise)
        
        # 计算成本
        costs = [cost_function(traj) for traj in noisy_trajectories]
        
        # 计算概率权重
        weights = compute_probability_weights(costs)
        
        # 更新轨迹
        delta = sum(w * (traj - trajectory) 
                   for w, traj in zip(weights, noisy_trajectories))
        trajectory += learning_rate * delta
    
    return trajectory
```

成本函数典型包含：
$$J = w_1 J_{obstacle} + w_2 J_{smoothness} + w_3 J_{dynamics}$$

## 14.7 产业案例研究：ABB YuMi 双臂协作装配

### 14.7.1 项目背景

ABB YuMi（You and Me）是专为人机协作设计的双臂机器人，在消费电子产品装配线上广泛应用。本案例研究基于某电子制造商使用 YuMi 进行精密电子装配的实际项目。

**系统规格：**
- 双 7 自由度机械臂
- 负载：每臂 0.5kg
- 重复精度：±0.02mm
- 工作空间：559mm 半径
- 集成视觉系统：2D/3D 相机
- 力传感器：腕部 6 轴力/力矩

**任务需求：**
- PCB 板精密装配
- 小型连接器插装
- 柔性线缆布线
- 质量检测与分拣

### 14.7.2 技术架构

```
YuMi ROS2 系统架构:
┌──────────────────────────────────────────┐
│          Task Coordinator Node            │
│    (基于状态机的任务调度与监控)            │
└────────────┬──────────────────────────────┘
             │
    ┌────────┴────────┬────────────┐
    │                 │            │
┌───▼──────┐ ┌───────▼──────┐ ┌──▼──────┐
│ Vision   │ │  MoveIt2     │ │  Force  │
│ Pipeline │ │  Dual Arm    │ │ Control │
│          │ │  Planning    │ │         │
└──────────┘ └──────────────┘ └─────────┘
    │                 │            │
┌───▼──────────────────────────────▼──────┐
│        ABB Robot Driver (RWS/EGM)       │
│         实时以太网通信 (1kHz)             │
└──────────────────────────────────────────┘
```

### 14.7.3 双臂协调策略

**1. 主从模式（Master-Slave）**

左臂作为主臂执行主要操作，右臂辅助：

```python
class MasterSlaveCoordinator:
    def execute_assembly(self, part_pose):
        # 主臂抓取并定位
        master_traj = self.plan_master_grasp(part_pose)
        
        # 从臂计算辅助位置
        slave_goal = self.compute_support_pose(
            master_traj.goal_pose,
            offset=[0.05, 0.0, 0.02]  # 相对偏移
        )
        
        # 同步执行
        self.execute_synchronized([
            (self.left_arm, master_traj),
            (self.right_arm, self.plan_to_pose(slave_goal))
        ])
```

**2. 对称协作模式**

双臂对称操作，用于处理较大物体：

```python
def plan_symmetric_grasp(object_center, object_width):
    # 计算对称抓取点
    grasp_offset = object_width / 2 + gripper_clearance
    
    left_grasp = Transform(
        position=[object_center.x - grasp_offset, 
                 object_center.y, 
                 object_center.z],
        orientation=quaternion_from_euler(0, np.pi/2, 0)
    )
    
    right_grasp = Transform(
        position=[object_center.x + grasp_offset,
                 object_center.y,
                 object_center.z],
        orientation=quaternion_from_euler(0, np.pi/2, np.pi)
    )
    
    return left_grasp, right_grasp
```

### 14.7.4 性能优化措施

**1. 轨迹缓存与预计算**

```python
class TrajectoryCache:
    def __init__(self, max_cache_size=1000):
        self.cache = LRUCache(max_cache_size)
        
    def get_trajectory(self, start, goal, constraints):
        cache_key = self.compute_hash(start, goal, constraints)
        
        if cache_key in self.cache:
            # 验证缓存轨迹仍然有效
            cached_traj = self.cache[cache_key]
            if self.validate_trajectory(cached_traj):
                return cached_traj
        
        # 计算新轨迹
        trajectory = self.planner.plan(start, goal, constraints)
        self.cache[cache_key] = trajectory
        return trajectory
```

**2. 并行规划优化**

```python
async def parallel_dual_arm_planning(left_goal, right_goal):
    # 并发规划
    left_future = asyncio.create_task(
        plan_arm_async(left_arm, left_goal)
    )
    right_future = asyncio.create_task(
        plan_arm_async(right_arm, right_goal)
    )
    
    # 等待两臂规划完成
    left_traj, right_traj = await asyncio.gather(
        left_future, right_future
    )
    
    # 碰撞后处理
    return resolve_collisions(left_traj, right_traj)
```

### 14.7.5 实际性能指标

**装配任务性能：**
- 平均周期时间：8.5 秒/件
- 装配成功率：99.2%
- 位置精度：±0.03mm
- 力控精度：±0.5N

**规划性能：**
- IK 求解时间：< 2ms (IKFast)
- 双臂规划时间：50-200ms
- 轨迹执行偏差：< 1mm
- 碰撞检测开销：15% CPU

### 14.7.6 关键挑战与解决方案

**挑战 1：线缆处理**

柔性线缆的形变建模困难：

```python
class CableManipulationPlanner:
    def plan_cable_routing(self, cable_model, fixtures):
        # 基于最小能量的线缆形状预测
        cable_shape = self.predict_cable_deformation(
            cable_model,
            grasp_points,
            gravity_vector
        )
        
        # 生成中间路径点
        waypoints = self.generate_routing_waypoints(
            cable_shape,
            fixtures,
            clearance=0.02
        )
        
        # 双臂协调移动
        return self.plan_bimanual_trajectory(waypoints)
```

**挑战 2：精密插装**

高精度要求的连接器插装：

```python
def precision_insertion_with_force_feedback():
    # 视觉粗定位
    target_pose = vision_localize_connector()
    
    # 接近阶段
    approach_pose = offset_pose(target_pose, [0, 0, -0.05])
    move_to_pose(approach_pose)
    
    # 力引导精细对准
    while not aligned:
        force, torque = read_force_sensor()
        
        # 计算修正量
        correction = compute_alignment_correction(
            force, torque,
            stiffness_matrix
        )
        
        # 柔顺控制
        apply_cartesian_correction(correction)
        
        aligned = check_alignment_criteria(force, torque)
    
    # 插入执行
    execute_insertion_with_force_limit(
        target_pose,
        force_threshold=10.0  # N
    )
```

### 14.7.7 经验教训

1. **工作空间优化**：YuMi 的工作空间有限，需要精心设计工装布局，确保双臂可达性。

2. **实时性要求**：装配任务对实时性要求高，使用 RT-PREEMPT 内核将控制延迟降至 1ms 以下。

3. **传感器融合**：结合视觉和力反馈，视觉用于粗定位，力传感器用于精细操作。

4. **异常处理**：建立完善的错误恢复机制，包括抓取失败重试、碰撞检测停止等。

5. **标定精度**：手眼标定和工具标定的精度直接影响装配质量，采用主动标定策略定期更新。