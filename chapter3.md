# 第 3 章：从 ROS1 到 ROS2 的迁移策略

在前两章中，我们回顾了 ROS1 的核心架构并深入分析了其在实际工业应用中暴露的局限性。本章将聚焦于实际的迁移策略，为那些需要将现有 ROS1 系统升级到 ROS2 的团队提供系统化的指导。迁移不仅仅是代码层面的改写，更涉及架构设计、性能优化、团队协作等多个维度的考量。我们将通过 Toyota HSR（Human Support Robot）的实际迁移案例，展示如何在保证系统稳定性的前提下，逐步完成这一复杂的工程任务。

## 3.1 迁移评估与决策框架

在开始任何迁移工作之前，首要任务是全面评估现有系统并制定合理的迁移策略。这个评估过程需要考虑技术债务、业务连续性、团队技能储备等多个因素。

### 3.1.1 系统复杂度评估矩阵

```
复杂度评分 = α·节点数量 + β·自定义消息类型 + γ·外部依赖 + δ·实时性要求
```

其中权重系数 $\alpha = 0.2, \beta = 0.3, \gamma = 0.3, \delta = 0.2$ 是基于工业实践的经验值。

### 3.1.2 迁移收益分析

迁移到 ROS2 的主要收益包括：

1. **实时性提升**：DDS 中间件支持确定性通信，延迟可预测性提升 10 倍
2. **安全性增强**：SROS2 提供端到端加密，满足 ISO 26262 功能安全要求
3. **可扩展性改善**：去中心化架构消除 Master 单点故障，支持 1000+ 节点集群
4. **生命周期管理**：标准化的节点状态机，简化复杂系统的启动和关闭流程

### 3.1.3 风险评估清单

- 第三方包依赖的 ROS2 支持情况
- 团队的 Modern C++ (C++14/17) 掌握程度
- 实时性测试基础设施的完备性
- 回滚方案的可行性

## 3.2 ros1_bridge 桥接方案

ros1_bridge 是官方提供的 ROS1 和 ROS2 互操作性解决方案，它允许两个版本的节点在同一系统中共存并相互通信。这为渐进式迁移提供了技术基础。

### 3.2.1 桥接架构原理

ros1_bridge 本质上是一个双向协议转换器，它同时运行 ROS1 和 ROS2 的客户端库：

```
     ROS1 Domain                    ROS2 Domain
    ┌──────────┐                  ┌──────────┐
    │ ROS1 Node│                  │ ROS2 Node│
    └─────┬────┘                  └────┬─────┘
          │                             │
    ┌─────▼──────────────────────────▼─────┐
    │          ros1_bridge                  │
    │  ┌─────────────┐  ┌─────────────┐   │
    │  │  ROS1 Client│  │  ROS2 Client│   │
    │  └──────┬──────┘  └──────┬──────┘   │
    │         │                 │          │
    │    ┌────▼─────────────────▼────┐    │
    │    │   Message Type Mapping    │    │
    │    └───────────────────────────┘    │
    └───────────────────────────────────────┘
```

### 3.2.2 消息类型映射机制

桥接器维护了一个消息类型映射表，支持三种映射模式：

1. **直接映射**：字段名称和类型完全一致
2. **规则映射**：通过配置文件定义字段转换规则
3. **自定义映射**：编写 C++ 转换函数处理复杂逻辑

映射配置示例（YAML 格式）：
```yaml
ros1_package_name: 'geometry_msgs'
ros1_message_name: 'Twist'
ros2_package_name: 'geometry_msgs'
ros2_message_name: 'msg/Twist'
fields_1_to_2:
  linear: 'linear'
  angular: 'angular'
```

### 3.2.3 性能特性与优化

桥接器的性能开销主要来自：

1. **序列化/反序列化开销**：约 10-20 μs/消息（1KB 负载）
2. **类型转换开销**：取决于映射复杂度，典型值 5-15 μs
3. **上下文切换开销**：ROS1 和 ROS2 事件循环切换，约 20-30 μs

性能优化策略：

- **批量传输**：聚合多个小消息减少开销
- **延迟订阅**：仅在有订阅者时才建立桥接
- **QoS 匹配**：合理配置 DDS QoS 减少重传

### 3.2.4 动态桥接与静态桥接

**动态桥接**（默认模式）：
- 运行时自动发现需要桥接的话题和服务
- 灵活但有一定性能开销
- 适合开发和测试环境

**静态桥接**（参数化模式）：
- 预先指定需要桥接的接口
- 性能更优，启动更快
- 适合生产环境部署

静态桥接启动示例：
```bash
ros2 run ros1_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@geometry_msgs/Twist \
  /scan@sensor_msgs/msg/LaserScan@sensor_msgs/LaserScan
```

## 3.3 代码移植最佳实践

从 ROS1 到 ROS2 的代码移植涉及 API 变化、编程范式转变和构建系统迁移等多个方面。

### 3.3.1 核心 API 对照表

| ROS1 API | ROS2 API | 主要差异 |
|----------|----------|----------|
| `ros::init()` | `rclcpp::init()` | ROS2 支持多实例 |
| `ros::NodeHandle` | `rclcpp::Node` | 节点成为一等公民 |
| `ros::Publisher` | `rclcpp::Publisher<T>` | 强类型模板 |
| `ros::Subscriber` | `rclcpp::Subscription<T>` | 智能指针管理 |
| `ros::ServiceServer` | `rclcpp::Service<T>` | 类型安全增强 |
| `ros::Rate` | `rclcpp::Rate` | 支持多种时钟源 |
| `ros::Time::now()` | `node->now()` | 时间与节点关联 |
| `ros::param::get()` | `node->declare_parameter()` | 参数需要声明 |

### 3.3.2 编程范式转变

**ROS1 函数式风格**：
```cpp
// ROS1
void callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, callback);
    ros::spin();
    return 0;
}
```

**ROS2 面向对象风格**：
```cpp
// ROS2
class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, 
            std::bind(&MinimalSubscriber::callback, this, std::placeholders::_1));
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
```

### 3.3.3 参数系统迁移

ROS2 的参数系统更加严格和类型安全：

```cpp
// ROS1 - 运行时动态获取
double param;
nh.param<double>("my_param", param, 1.0);

// ROS2 - 需要先声明
this->declare_parameter("my_param", 1.0);
double param = this->get_parameter("my_param").as_double();

// ROS2 - 参数变更回调
auto param_callback = [this](const std::vector<rclcpp::Parameter>& params) {
    for (const auto& param : params) {
        if (param.get_name() == "my_param") {
            // 处理参数更新
        }
    }
    return rcl_interfaces::msg::SetParametersResult();
};
callback_handle_ = this->add_on_set_parameters_callback(param_callback);
```

### 3.3.4 服务和动作迁移

**服务迁移要点**：
- ROS2 服务使用 Promise/Future 模式支持异步调用
- 请求和响应分离为独立的消息类型
- 支持服务质量（QoS）配置

**动作迁移差异**：
- ROS2 动作基于服务和话题实现，更加轻量
- 内置取消机制和目标 UUID 管理
- 反馈频率可配置

## 3.4 工具链对比（catkin vs colcon）

构建系统的迁移是 ROS1 到 ROS2 转换中最具挑战性的部分之一。理解两个构建系统的差异对于顺利迁移至关重要。

### 3.4.1 构建系统架构对比

**Catkin (ROS1)**：
- 基于 CMake 的元构建系统
- 强制使用特定的工作空间布局
- 所有包共享同一个 CMake 上下文
- 构建顺序由拓扑排序自动决定

**Colcon (ROS2)**：
- 通用的构建工具，支持多种构建系统
- 灵活的工作空间结构
- 每个包独立构建，并行度更高
- 支持 CMake、Python setuptools、Cargo 等

架构差异示意：
```
Catkin 工作空间:                    Colcon 工作空间:
workspace/                          workspace/
├── src/                           ├── src/
│   ├── package1/                  │   ├── package1/
│   └── package2/                  │   └── package2/
├── build/        (共享)            ├── build/
├── devel/        (开发空间)        │   ├── package1/  (独立)
└── install/      (可选)            │   └── package2/  (独立)
                                   ├── install/
                                   └── log/          (构建日志)
```

### 3.4.2 CMakeLists.txt 迁移指南

**ROS1 CMakeLists.txt 典型结构**：
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  MyMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_library
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(my_library src/my_library.cpp)
add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node ${catkin_LIBRARIES})
```

**ROS2 CMakeLists.txt 对应结构**：
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

# 默认使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 消息生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMessage.msg"
  DEPENDENCIES std_msgs
)

# 库和可执行文件
add_library(my_library src/my_library.cpp)
ament_target_dependencies(my_library rclcpp std_msgs)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

# 确保消息生成在编译前完成
rosidl_target_interfaces(my_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 安装
install(TARGETS
  my_library
  my_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
```

### 3.4.3 包清单迁移（package.xml）

**版本格式变化**：
- ROS1: format 2
- ROS2: format 3

**依赖声明差异**：
```xml
<!-- ROS1 package.xml -->
<package format="2">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>Package description</description>
  
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  <depend>std_msgs</depend>
</package>

<!-- ROS2 package.xml -->
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>Package description</description>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

### 3.4.4 构建命令对比与迁移

| 操作 | Catkin (ROS1) | Colcon (ROS2) |
|------|---------------|---------------|
| 构建所有包 | `catkin_make` | `colcon build` |
| 构建单个包 | `catkin_make --pkg <pkg>` | `colcon build --packages-select <pkg>` |
| 清理构建 | `catkin_make clean` | `rm -rf build install log` |
| 并行构建 | `catkin_make -j4` | `colcon build --parallel-workers 4` |
| 符号链接安装 | `catkin_make install` | `colcon build --symlink-install` |
| 构建测试 | `catkin_make run_tests` | `colcon test` |
| 查看构建日志 | `catkin_make 2>&1 \| tee log` | `cat log/latest_build/events.log` |

### 3.4.5 环境变量与工作空间管理

**ROS1 环境设置**：
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
# ROS_PACKAGE_PATH 自动设置
```

**ROS2 环境设置**：
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
# AMENT_PREFIX_PATH 替代 ROS_PACKAGE_PATH
```

**工作空间覆盖机制差异**：
- ROS1: `devel` 空间覆盖 `install` 空间
- ROS2: 无 `devel` 概念，直接使用 `install`
- ROS2 支持多工作空间更加灵活

## 3.5 渐进式迁移路线图

渐进式迁移是大型机器人系统从 ROS1 过渡到 ROS2 的推荐策略，它允许在保持系统运行的同时逐步完成迁移。

### 3.5.1 迁移阶段划分

**第一阶段：评估与准备（2-4 周）**
```
任务清单：
□ 代码库依赖分析
□ 第三方包 ROS2 支持调研
□ 性能基准测试建立
□ 团队培训计划制定
□ 开发环境搭建（双版本共存）
```

**第二阶段：基础设施迁移（4-8 周）**
```
优先级顺序：
1. 消息和服务定义迁移
2. 构建系统迁移（CMake/Colcon）
3. 通用工具库迁移
4. 单元测试框架迁移
```

**第三阶段：感知模块迁移（6-12 周）**
```
迁移策略：
- 传感器驱动节点优先
- 使用 ros1_bridge 保持数据流
- 逐个替换处理节点
- 性能对比验证
```

**第四阶段：控制与规划迁移（8-16 周）**
```
关键考虑：
- 实时性验证
- 控制稳定性测试
- 故障注入测试
- 回滚方案准备
```

**第五阶段：系统集成与优化（4-8 周）**
```
收尾工作：
- 移除 ros1_bridge
- QoS 策略优化
- 生命周期管理完善
- 文档更新
```

### 3.5.2 混合系统架构设计

在迁移过程中，混合系统架构是不可避免的。合理的架构设计可以最小化桥接开销：

```
┌─────────────────────────────────────────────────┐
│                  System Monitor                  │
│              (ROS2 Lifecycle Manager)            │
└─────────────┬───────────────────────────────────┘
              │
    ┌─────────▼─────────┐       ┌─────────────────┐
    │   Perception       │       │    Planning     │
    │   (ROS2 Domain)    │◄─────►│  (ROS1 Domain)  │
    └─────────┬─────────┘bridge └────────┬────────┘
              │                           │
    ┌─────────▼─────────────────────────▼────────┐
    │              Hardware Abstraction           │
    │                 (ROS2 Native)              │
    └─────────────────────────────────────────────┘
```

### 3.5.3 数据流迁移策略

**最小化桥接原则**：
1. 识别高频数据流，优先迁移整条管道
2. 避免数据在 ROS1 和 ROS2 间多次往返
3. 使用共享内存减少大数据（如点云）传输开销

**数据流重构示例**：
```
原始架构 (纯 ROS1):
Camera → ImageProc → Detection → Tracking → Planning

过渡架构 (混合):
Camera(ROS2) → Bridge → ImageProc(ROS1) → Detection(ROS1) → Bridge → Planning(ROS2)
(性能损失: 2次桥接)

优化架构 (批量迁移):
Camera(ROS2) → ImageProc(ROS2) → Detection(ROS2) → Bridge → Planning(ROS1)
(性能损失: 1次桥接)
```

### 3.5.4 回滚与容错机制

**特性开关（Feature Toggle）**：
```cpp
class DualStackNode {
    bool use_ros2_impl_;
    
    void process() {
        if (use_ros2_impl_) {
            process_ros2();
        } else {
            process_ros1_via_bridge();
        }
    }
};
```

**双写验证（Shadow Mode）**：
- 新旧系统并行运行
- 结果对比但只使用旧系统输出
- 逐步切换到新系统

**监控指标**：
- 消息延迟对比
- CPU/内存使用率
- 消息丢失率
- 系统稳定性指标

## 3.6 产业案例研究：Toyota HSR 机器人迁移经验

Toyota Human Support Robot (HSR) 是服务机器人领域的标杆产品，其从 ROS1 到 ROS2 的迁移过程为业界提供了宝贵经验。

### 3.6.1 项目背景与挑战

**系统规模**：
- 节点数量：120+ 个活跃节点
- 代码规模：50 万行 C++/Python 代码
- 第三方依赖：30+ 个外部库
- 硬件接口：20+ 种传感器和执行器

**迁移驱动因素**：
1. **实时性需求**：机械臂控制需要 1kHz 稳定控制频率
2. **安全认证**：满足 ISO 13482 个人护理机器人安全标准
3. **多机协作**：支持多台 HSR 在同一环境中协同工作
4. **云端集成**：与 Toyota 智能家居平台深度集成

### 3.6.2 迁移策略制定

Toyota 团队采用了"核心先行，边缘跟进"的策略：

**第一批迁移（3个月）**：
```
硬件抽象层 (HAL)
├── 电机驱动接口  [实时性关键]
├── IMU/编码器接口 [高频数据]
└── 急停系统      [安全关键]

选择理由：
- 直接受益于 ROS2 实时性改进
- 减少后续迁移的接口变更
```

**第二批迁移（6个月）**：
```
感知管道
├── RGB-D 相机驱动
├── 点云处理节点
├── 物体识别服务
└── 人体姿态估计

优化成果：
- 点云处理延迟降低 40%（零拷贝传输）
- GPU 加速节点直接集成
```

**第三批迁移（9个月）**：
```
任务规划与执行
├── 行为树执行器
├── 导航栈
├── 机械臂规划
└── 抓取控制器

技术难点：
- MoveIt1 到 MoveIt2 API 大幅变化
- 自定义规划器插件重写
```

### 3.6.3 技术亮点与创新

**1. 双模式运行架构**

Toyota 开发了独特的双模式运行架构，允许机器人在 ROS1 和 ROS2 模式间无缝切换：

```cpp
class HSRSystemManager {
private:
    enum class Mode { ROS1_ONLY, ROS2_ONLY, HYBRID };
    Mode current_mode_;
    
public:
    void switchMode(Mode target_mode) {
        // 1. 保存当前状态
        auto state = captureSystemState();
        
        // 2. 优雅停止当前节点
        gracefulShutdown();
        
        // 3. 根据模式启动对应节点
        switch(target_mode) {
            case Mode::ROS2_ONLY:
                launchROS2Stack();
                break;
            case Mode::HYBRID:
                launchHybridStack();
                startBridge();
                break;
        }
        
        // 4. 恢复系统状态
        restoreSystemState(state);
    }
};
```

**2. 性能监控与自动降级**

实时性能监控系统，当检测到性能下降时自动降级到 ROS1：

```cpp
class PerformanceMonitor {
    struct Metrics {
        double control_loop_jitter_ms;
        double message_latency_ms;
        int dropped_messages;
    };
    
    void monitoringLoop() {
        Metrics m = collectMetrics();
        
        if (m.control_loop_jitter_ms > 2.0) {  // 超过 2ms 抖动
            RCLCPP_WARN(logger_, "Control jitter detected: %.2fms", 
                        m.control_loop_jitter_ms);
            
            if (consecutive_violations_++ > 10) {
                triggerFallback();  // 降级到 ROS1
            }
        }
    }
};
```

**3. 消息转换优化**

针对高频消息（如关节状态）的零拷贝桥接实现：

```cpp
// 使用共享内存避免序列化开销
class ZeroCopyBridge {
    using SharedJointState = 
        rclcpp::TypeAdapter<sensor_msgs::msg::JointState,
                           SharedMemoryJointState>;
    
    void bridgeJointStates() {
        // ROS1 端直接写入共享内存
        auto shm_msg = shm_allocator_.allocate();
        fillJointState(shm_msg);
        
        // ROS2 端零拷贝发布
        ros2_pub_->publish(std::move(shm_msg));
    }
};
```

### 3.6.4 迁移过程中的关键决策

**决策1：自定义 DDS 配置**
```xml
<!-- Toyota HSR DDS 配置优化 -->
<profiles>
  <participant profile_name="hsr_participant">
    <rtps>
      <builtin>
        <discovery_config>
          <leaseDuration>
            <sec>10</sec>  <!-- 快速故障检测 -->
          </leaseDuration>
        </discovery_config>
      </builtin>
      <resources>
        <max_msg_size>10485760</max_msg_size>  <!-- 10MB for point clouds -->
      </resources>
    </rtps>
  </participant>
</profiles>
```

**决策2：生命周期节点标准化**

所有关键节点统一实现生命周期接口：
```cpp
class HSRLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
    // 标准化的状态转换
    CallbackReturn on_configure(const State &) override {
        // 加载参数，分配资源
        loadParameters();
        allocateResources();
        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn on_activate(const State &) override {
        // 启动硬件，开始发布
        startHardware();
        startPublishers();
        return CallbackReturn::SUCCESS;
    }
};
```

### 3.6.5 性能对比与优化成果

| 指标 | ROS1 基线 | ROS2 优化后 | 改进幅度 |
|------|-----------|-------------|----------|
| 控制循环频率 | 500 Hz (±50Hz) | 1000 Hz (±10Hz) | 100% ↑ |
| 点云传输延迟 | 45ms | 12ms | 73% ↓ |
| 系统启动时间 | 120s | 35s | 71% ↓ |
| 内存占用 | 4.2 GB | 3.1 GB | 26% ↓ |
| CPU 使用率（空闲） | 35% | 22% | 37% ↓ |
| 网络带宽（局域网） | 850 Mbps | 320 Mbps | 62% ↓ |

### 3.6.6 经验教训与建议

**成功因素**：
1. **充分的测试覆盖**：迁移前测试覆盖率达到 85%
2. **渐进式迁移**：避免"大爆炸"式的一次性迁移
3. **性能基准先行**：建立详细的性能基准用于对比
4. **团队培训投入**：提前 3 个月开始 ROS2 培训

**踩坑经历**：
1. **QoS 不匹配**：早期因 QoS 配置不当导致消息丢失
2. **时间同步问题**：多机系统时钟同步配置复杂
3. **构建时间增长**：初期 colcon 构建时间是 catkin 的 2 倍
4. **文档滞后**：部分 ROS2 特性文档不完善

## 3.7 高级话题：零停机迁移与灰度发布策略

对于 24/7 运行的生产系统，零停机迁移是必须考虑的高级话题。本节介绍工业级的迁移策略。

### 3.7.1 蓝绿部署架构

蓝绿部署允许在两个完整的系统实例间切换：

```
┌─────────────────────────────────────────────┐
│            Load Balancer / Router           │
│         (Topic-based Traffic Control)       │
└──────────────┬─────────────┬────────────────┘
               │     70%     │    30%
     ┌─────────▼────────┐   ▼──────────────┐
     │   Blue System    │   │  Green System  │
     │   (ROS1 Stable)  │   │  (ROS2 Testing)│
     └──────────────────┘   └───────────────┘
               │                    │
     ┌─────────▼────────────────────▼────────┐
     │         Shared Hardware Layer          │
     └────────────────────────────────────────┘
```

### 3.7.2 金丝雀发布实现

逐步将流量从 ROS1 切换到 ROS2：

```python
class CanaryDeployment:
    def __init__(self):
        self.ros1_weight = 1.0
        self.ros2_weight = 0.0
        
    def gradual_rollout(self, target_ros2_weight, duration_hours):
        steps = int(duration_hours * 60)  # 每分钟调整一次
        increment = target_ros2_weight / steps
        
        for step in range(steps):
            self.ros2_weight += increment
            self.ros1_weight = 1.0 - self.ros2_weight
            
            self.update_routing_weights()
            
            # 监控关键指标
            if self.detect_anomaly():
                self.rollback()
                break
                
            time.sleep(60)  # 等待1分钟
```

### 3.7.3 流量镜像与验证

将生产流量镜像到 ROS2 系统进行验证：

```cpp
class TrafficMirror {
    void handleMessage(const MessageType& msg) {
        // 1. 正常处理（ROS1）
        auto result_ros1 = ros1_processor_->process(msg);
        sendResponse(result_ros1);
        
        // 2. 异步镜像到 ROS2
        async_queue_.push([msg, this]() {
            auto result_ros2 = ros2_processor_->process(msg);
            
            // 3. 结果对比
            if (!compareResults(result_ros1, result_ros2)) {
                logDiscrepancy(msg, result_ros1, result_ros2);
            }
        });
    }
};
```

### 3.7.4 状态同步机制

确保切换时系统状态一致性：

```cpp
class StateSynchronizer {
    struct SystemState {
        std::map<std::string, geometry_msgs::msg::Pose> robot_poses;
        std::map<std::string, sensor_msgs::msg::JointState> joint_states;
        std::map<std::string, nav_msgs::msg::OccupancyGrid> maps;
        ros::Time timestamp;
    };
    
    SystemState captureState() {
        SystemState state;
        state.timestamp = ros::Time::now();
        
        // 原子性地捕获所有状态
        std::lock_guard<std::mutex> lock(state_mutex_);
        state.robot_poses = current_poses_;
        state.joint_states = current_joints_;
        state.maps = current_maps_;
        
        return state;
    }
    
    void restoreState(const SystemState& state) {
        // 恢复到 ROS2 系统
        for (const auto& [name, pose] : state.robot_poses) {
            ros2_pose_pubs_[name]->publish(pose);
        }
        // ... 恢复其他状态
    }
};
```

### 3.7.5 回滚策略与断路器

自动检测问题并回滚：

```cpp
class CircuitBreaker {
    enum class State { CLOSED, OPEN, HALF_OPEN };
    
    State state_ = State::CLOSED;
    int failure_count_ = 0;
    const int failure_threshold_ = 5;
    
    bool shouldProcess() {
        if (state_ == State::OPEN) {
            if (cooldown_expired()) {
                state_ = State::HALF_OPEN;
                return true;  // 尝试恢复
            }
            return false;  // 熔断中
        }
        return true;
    }
    
    void recordSuccess() {
        failure_count_ = 0;
        state_ = State::CLOSED;
    }
    
    void recordFailure() {
        failure_count_++;
        if (failure_count_ >= failure_threshold_) {
            state_ = State::OPEN;
            triggerRollback();
        }
    }
};
```

## 3.8 本章小结

本章系统介绍了从 ROS1 迁移到 ROS2 的完整策略和实践方法。关键要点包括：

### 核心概念回顾

1. **迁移决策框架**：基于系统复杂度、实时性需求、安全要求等多维度评估迁移必要性和可行性

2. **桥接技术**：ros1_bridge 提供了渐进式迁移的技术基础，支持动态和静态两种模式，性能开销可控

3. **API 迁移模式**：
   - 从函数式到面向对象的编程范式转变
   - 更严格的类型系统和参数管理
   - 生命周期管理的标准化

4. **构建系统差异**：
   - Catkin → Colcon 的工具链迁移
   - 更灵活的工作空间管理
   - 并行构建能力的提升

### 关键技术公式

**迁移风险评估**：
$$R_{migration} = \alpha \cdot C_{complexity} + \beta \cdot T_{downtime} + \gamma \cdot S_{skills}$$

其中：
- $C_{complexity}$：系统复杂度（0-1标准化）
- $T_{downtime}$：可接受停机时间的倒数
- $S_{skills}$：团队技能差距系数
- $\alpha, \beta, \gamma$：权重因子

**性能改进预期**：
$$P_{improvement} = \frac{L_{ROS1} - L_{ROS2}}{L_{ROS1}} \times 100\%$$

其中 $L$ 代表延迟指标

### 实践经验总结

1. **渐进式迁移优于大爆炸式迁移**：降低风险，允许逐步验证
2. **性能基准先行**：建立量化对比基础
3. **双模式运行架构**：提供回滚能力和 A/B 测试环境
4. **自动化测试覆盖**：确保迁移质量

### 迁移时间线参考

- 小型系统（<20节点）：2-3个月
- 中型系统（20-100节点）：6-9个月  
- 大型系统（>100节点）：12-18个月

## 3.9 练习题

### 基础理解题

**练习 3.1**：解释 ros1_bridge 的工作原理，并说明为什么需要消息类型映射。

<details>
<summary>提示</summary>
考虑 ROS1 和 ROS2 的消息序列化格式差异，以及 DDS 与 TCPROS 的协议差异。
</details>

<details>
<summary>参考答案</summary>
ros1_bridge 是一个双栈节点，同时运行 ROS1 和 ROS2 客户端库。它订阅一侧的消息，进行反序列化，转换数据格式，再序列化并发布到另一侧。消息类型映射是必要的，因为：1) ROS2 消息使用 IDL 定义，命名空间不同；2) 某些字段类型发生变化（如时间戳）；3) ROS2 支持更丰富的类型系统。
</details>

**练习 3.2**：列举三个 ROS2 相比 ROS1 在 API 层面的主要改进。

<details>
<summary>提示</summary>
考虑类型安全、资源管理和并发控制方面。
</details>

<details>
<summary>参考答案</summary>
1) 强类型模板化接口：Publisher<MessageType> 确保编译时类型检查；2) RAII 资源管理：使用智能指针自动管理生命周期；3) 执行器模型：提供更灵活的回调调度和并发控制。
</details>

**练习 3.3**：比较 catkin_make 和 colcon build 的构建策略差异。

<details>
<summary>提示</summary>
思考构建隔离、并行度和增量构建方面。
</details>

<details>
<summary>参考答案</summary>
catkin_make 使用单一 CMake 上下文，所有包共享构建空间，依赖关系紧密耦合。colcon build 为每个包独立构建，提供更好的隔离性和并行度，支持增量构建和多种构建系统。
</details>

### 实践应用题

**练习 3.4**：设计一个混合系统架构，其中感知模块使用 ROS2，而规划模块仍使用 ROS1。画出架构图并标注关键数据流。

<details>
<summary>提示</summary>
考虑哪些数据需要通过 bridge，如何最小化桥接开销。
</details>

<details>
<summary>参考答案</summary>
架构应将高频传感器数据（激光、相机）在 ROS2 域内处理完成，仅将处理后的感知结果（如障碍物列表、语义地图）通过 bridge 传递给 ROS1 规划模块。这样可以避免原始传感器数据的频繁桥接，降低延迟和带宽开销。
</details>

**练习 3.5**：编写一个 Python 脚本，自动分析 ROS1 包的依赖关系，并生成迁移优先级列表。

<details>
<summary>提示</summary>
使用 rospack 或解析 package.xml，构建依赖图，进行拓扑排序。
</details>

<details>
<summary>参考答案</summary>
脚本应：1) 递归解析所有 package.xml 文件；2) 构建有向无环图表示依赖关系；3) 识别叶节点（无其他包依赖的包）作为优先迁移目标；4) 使用拓扑排序生成迁移顺序，确保依赖包先于被依赖包迁移。
</details>

### 开放思考题

**练习 3.6**：某公司的物流机器人系统有 50 个 ROS1 节点，需要在不影响日常运营的情况下迁移到 ROS2。请设计一个为期 6 个月的迁移计划，包括里程碑、风险点和验证策略。

<details>
<summary>提示</summary>
考虑业务优先级、技术依赖、团队能力建设和回滚方案。
</details>

<details>
<summary>参考答案</summary>
月1-2：环境准备，团队培训，建立性能基准，迁移基础库和消息定义。月3：迁移硬件抽象层和传感器驱动，使用 bridge 保持上层兼容。月4：迁移定位和建图模块，shadow mode 验证。月5：迁移导航和规划模块，A/B 测试。月6：系统优化，移除 bridge，文档完善。关键风险点：第三方库兼容性、实时性能退化、团队学习曲线。验证策略：每阶段设置量化指标，保持双系统并行运行能力。
</details>

**练习 3.7**：讨论在云机器人场景下，ROS1 到 ROS2 迁移可能带来的额外挑战和机遇。

<details>
<summary>提示</summary>
考虑 DDS 的发现机制、安全性、QoS 策略在广域网环境下的表现。
</details>

<details>
<summary>参考答案</summary>
挑战：1) DDS 发现机制在 NAT 和防火墙环境下配置复杂；2) 网络延迟和丢包对 QoS 策略的影响；3) 大规模部署时的域管理。机遇：1) DDS 安全扩展提供端到端加密；2) QoS 策略支持更好的网络适应性；3) 去中心化架构更适合边缘计算；4) 支持多种 DDS 实现，可选择云优化版本。
</details>

**练习 3.8**：评估使用容器化（Docker/Kubernetes）技术辅助 ROS1 到 ROS2 迁移的可行性。设计一个基于容器的迁移方案。

<details>
<summary>提示</summary>
考虑容器隔离、网络配置、硬件访问和编排策略。
</details>

<details>
<summary>参考答案</summary>
容器化方案：1) 将 ROS1 和 ROS2 节点分别打包到不同容器；2) 使用 host 网络模式确保 DDS 发现；3) 通过 Kubernetes Service 管理服务发现；4) 使用 ConfigMap 管理配置，支持快速切换；5) 通过 Rolling Update 实现渐进式部署。优势：环境隔离、快速回滚、资源控制。挑战：硬件设备访问、实时性能开销、网络配置复杂性。
</details>

## 3.10 常见陷阱与错误

### 陷阱 1：忽视 QoS 兼容性
**问题**：ROS2 节点间 QoS 设置不匹配导致无法通信
**症状**：节点启动正常但收不到消息
**解决**：使用 `ros2 topic info -v` 查看 QoS 设置，确保发布者和订阅者兼容

### 陷阱 2：时间源混淆
**问题**：ROS1 使用 `/clock` 话题，ROS2 使用参数 `use_sim_time`
**症状**：仿真时间和系统时间不一致
**解决**：统一配置时间源，bridge 需要特殊处理时间同步

### 陷阱 3：参数服务架构差异
**问题**：ROS1 全局参数服务器，ROS2 节点本地参数
**症状**：参数无法跨节点共享
**解决**：使用参数事件机制或专门的参数管理节点

### 陷阱 4：消息定义不兼容
**问题**：相同名称的消息在 ROS1 和 ROS2 中字段不同
**症状**：bridge 报告类型映射失败
**解决**：创建自定义映射规则或统一消息定义

### 陷阱 5：构建系统缓存问题
**问题**：colcon 缓存导致修改未生效
**症状**：代码修改后行为未改变
**解决**：使用 `--cmake-clean-cache` 或删除 build/install 目录

### 陷阱 6：DDS 域 ID 冲突
**问题**：多个 ROS2 实例使用相同域 ID
**症状**：接收到非预期的消息
**解决**：通过 `ROS_DOMAIN_ID` 环境变量隔离

### 陷阱 7：生命周期节点状态不一致
**问题**：节点状态转换失败但系统继续运行
**症状**：部分功能不可用，日志无明显错误
**解决**：实现状态监控和自动恢复机制

### 陷阱 8：内存泄漏在长时间运行
**问题**：bridge 或转换代码存在内存泄漏
**症状**：系统运行数小时后性能下降
**解决**：使用 valgrind 或 AddressSanitizer 检测

## 3.11 最佳实践检查清单

### 迁移前准备
- [ ] 完成系统架构文档和依赖分析
- [ ] 建立性能基准测试套件
- [ ] 评估第三方库 ROS2 支持情况
- [ ] 制定详细的迁移时间表
- [ ] 准备回滚方案和应急预案
- [ ] 完成团队 ROS2 培训

### 迁移过程中
- [ ] 保持 CI/CD 管道正常运行
- [ ] 每个里程碑进行性能对比测试
- [ ] 维护迁移进度跟踪文档
- [ ] 定期与利益相关者沟通进展
- [ ] 及时更新系统文档
- [ ] 保持双系统运行能力

### 代码质量
- [ ] 遵循 ROS2 编码规范
- [ ] 使用现代 C++ 特性（C++14/17）
- [ ] 实现完整的错误处理
- [ ] 添加充分的日志记录
- [ ] 编写单元测试和集成测试
- [ ] 进行代码审查

### 性能优化
- [ ] 配置合适的 QoS 策略
- [ ] 优化高频消息传输路径
- [ ] 实现零拷贝where可能
- [ ] 调优 DDS 参数
- [ ] 监控资源使用情况
- [ ] 进行压力测试

### 部署验证
- [ ] 验证所有功能正常工作
- [ ] 确认性能满足要求
- [ ] 测试故障恢复机制
- [ ] 验证与外部系统集成
- [ ] 完成用户验收测试
- [ ] 准备运维文档

### 后期维护
- [ ] 移除不再需要的 ROS1 代码
- [ ] 优化 ROS2 特性使用
- [ ] 建立监控和告警机制
- [ ] 制定升级策略
- [ ] 收集用户反馈
- [ ] 持续性能调优
