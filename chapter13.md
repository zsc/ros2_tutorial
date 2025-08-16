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
   资源注册流程：
   Hardware Component → export_interfaces() → Resource Manager
                                                    │
                                                    ├─ State Interface Registry
                                                    └─ Command Interface Registry
   ```

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

每个状态转换都有对应的回调函数：

- `on_configure()`: 读取参数，初始化硬件连接
- `on_activate()`: 启动硬件，开始控制循环
- `on_deactivate()`: 停止控制，保持安全状态
- `on_cleanup()`: 清理资源，关闭连接

### 自定义硬件接口开发

开发自定义硬件接口需要实现以下关键方法：

```cpp
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

### 硬件接口配置

硬件接口通过 URDF/XACRO 文件配置：

```xml
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

### 可链接控制器设计

可链接控制器（Chainable Controller）既可以作为其他控制器的输入源，也可以接收其他控制器的输出：

```
传统控制器:
Hardware → Controller → Hardware

可链接控制器:
Hardware → Controller A → Controller B → Controller C → Hardware
              (滤波)        (PID)         (限幅)
```

实现可链接控制器的关键接口：

```cpp
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

### 控制器间数据流

控制器链中的数据流遵循严格的方向性：

```
数据流示例：力控制链
┌─────────────┐     ┌──────────────┐     ┌────────────┐     ┌──────────┐
│Force Sensor │ --> │Gravity Comp. │ --> │Force PID   │ --> │Joint Cmd │
│Controller   │     │Controller    │     │Controller  │     │Interface │
└─────────────┘     └──────────────┘     └────────────┘     └──────────┘
     读取力              补偿重力           计算力矩          发送命令
```

### 动态加载与切换

控制器支持运行时动态加载和切换，这对于多模态控制至关重要：

```yaml
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

控制器切换流程：

1. **停止当前控制器**
   ```bash
   ros2 control switch_controllers --stop position_controller
   ```

2. **启动新控制器**
   ```bash
   ros2 control switch_controllers --start trajectory_controller
   ```

3. **原子切换**（同时停止和启动）
   ```bash
   ros2 control switch_controllers \
     --stop position_controller \
     --start trajectory_controller \
     --strict  # 确保切换成功
   ```

### 控制器优先级与冲突解决

当多个控制器请求同一资源时，需要优先级机制：

```
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

### 控制器链实例：阻抗控制

阻抗控制是控制器链的典型应用：

```
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

阻抗控制器实现：

```cpp
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

## 13.4 实时控制循环

实时控制是机器人系统的核心要求，ros2_control 通过精心设计的控制循环和调度策略，实现了微秒级的控制周期和确定性的响应时间。

### 实时性保证机制

ros2_control 的实时性建立在多个层次的保证机制之上：

```
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

关键的实时保证技术：

1. **内存预分配**：避免运行时内存分配
2. **锁定内存页**：防止页面交换
3. **CPU 亲和性**：绑定到专用 CPU 核心
4. **实时调度策略**：SCHED_FIFO 或 SCHED_RR

### 控制周期与更新率

控制周期的选择直接影响系统性能：

```
典型控制频率：
- 位置控制: 100-500 Hz
- 速度控制: 500-1000 Hz  
- 力/力矩控制: 1000-4000 Hz
- 电流控制: 10-40 kHz (通常在驱动器内部)
```

控制周期的数学关系：

$$T_s = \frac{1}{f_c}$$

其中 $T_s$ 是采样周期，$f_c$ 是控制频率。

奈奎斯特采样定理要求：
$$f_c > 2 \cdot f_{max}$$

其中 $f_{max}$ 是系统最高频率成分。

### 线程模型与调度

ros2_control 采用多线程架构，不同组件运行在不同优先级的线程中：

```cpp
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

### 延迟分析与优化

控制循环的总延迟由多个部分组成：

```
总延迟 = 传感器延迟 + 通信延迟 + 计算延迟 + 执行器延迟

典型延迟分解 (1kHz 控制循环):
├─ 传感器读取: 50-100 μs
├─ 控制计算:  100-200 μs  
├─ 通信传输:  50-100 μs
└─ 执行器响应: 200-500 μs
总计: 400-900 μs (< 1ms 周期)
```

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
抖动来源：
1. 操作系统调度延迟
2. 中断处理
3. 缓存未命中
4. 总线竞争
```

抖动测量与监控：

```cpp
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

### 实时性能基准测试

评估实时性能的关键指标：

```
性能指标：
- 平均延迟 (Average Latency)
- 最坏情况延迟 (WCET - Worst Case Execution Time)
- 抖动标准差 (Jitter Standard Deviation)
- 丢失周期率 (Missed Deadline Rate)
```

基准测试工具：

```bash
# cyclictest 测试实时延迟
sudo cyclictest -p 95 -t1 -n -i 1000 -l 100000

# ros2_control 性能分析
ros2 run controller_manager ros2_control_benchmark \
  --iterations 10000 \
  --control-frequency 1000
```
