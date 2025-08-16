# 第 4 章：ROS2 架构与设计理念

本章深入探讨 ROS2 的核心架构设计，重点解析其如何通过 DDS 中间件实现分布式通信、如何满足实时系统需求，以及如何通过 QoS 策略提供灵活的通信保证。通过对比 ROS1，我们将理解 ROS2 在架构层面的根本性改进，这些改进使其能够满足工业级机器人系统的严苛要求。

## 4.1 DDS 中间件架构

### 4.1.1 DDS 标准概述

数据分发服务（Data Distribution Service, DDS）是对象管理组织（OMG）制定的中间件协议和 API 标准，专为分布式实时系统设计。ROS2 采用 DDS 作为其通信基础设施，从根本上改变了机器人系统的通信架构。

DDS 的核心概念模型：

```
┌─────────────────────────────────────────────────────┐
│                   DDS Domain                        │
│  ┌──────────────┐              ┌──────────────┐    │
│  │ Domain       │              │ Domain       │    │
│  │ Participant  │◄────────────►│ Participant  │    │
│  │              │     RTPS      │              │    │
│  │ ┌──────────┐ │              │ ┌──────────┐ │    │
│  │ │Publisher │ │              │ │Subscriber│ │    │
│  │ │          │ │              │ │          │ │    │
│  │ │┌────────┐│ │              │ │┌────────┐│ │    │
│  │ ││DataWriter│─────Topic────►│ ││DataReader│ │    │
│  │ │└────────┘│ │              │ │└────────┘│ │    │
│  │ └──────────┘ │              │ └──────────┘ │    │
│  └──────────────┘              └──────────────┘    │
└─────────────────────────────────────────────────────┘
```

DDS 采用以数据为中心的发布-订阅（DCPS）模型，具有以下关键特性：

1. **自动发现机制**：节点可以动态加入和离开网络，无需中心化的服务发现
2. **类型安全**：强类型系统确保数据结构一致性
3. **QoS 策略**：细粒度的服务质量控制
4. **可扩展性**：支持从嵌入式系统到大规模分布式系统

### 4.1.2 RTPS 协议详解

实时发布订阅协议（Real-Time Publish-Subscribe, RTPS）是 DDS 的有线协议，定义了数据在网络上的传输格式。RTPS 2.2 规范包含以下核心组件：

**RTPS 实体模型**：
- **Participant**：代表通信域中的一个应用实例
- **Endpoint**：通信端点，包括 Writer 和 Reader
- **GUID**：全局唯一标识符，格式为 `{prefix, entityId}`

**发现协议（Discovery Protocol）**：

RTPS 使用两阶段发现机制：

1. **简单参与者发现协议（SPDP）**：
   - 使用多播定期广播 Participant 信息
   - 默认多播地址：`239.255.0.1:7400` (IPv4)
   - 发现消息包含：GUID、位置器（Locator）、QoS 等

2. **简单端点发现协议（SEDP）**：
   - 在已发现的 Participant 间交换 Endpoint 信息
   - 使用可靠单播通信
   - 交换 Topic 名称、类型信息、QoS 配置

**可靠性机制**：

RTPS 提供多种可靠性保证：

```
心跳机制（Heartbeat）：
Writer ──HB(seq_min, seq_max)──► Reader
      ◄──────ACK/NACK───────────
      ────────DATA──────────────►

其中：
- HB: 心跳消息，包含序列号范围
- ACK: 确认接收
- NACK: 请求重传
```

### 4.1.3 DDS 实现对比

ROS2 支持多种 DDS 实现，各有特点：

| DDS 实现 | 开发商 | 许可证 | 特点 | 适用场景 |
|---------|--------|--------|------|----------|
| **Fast DDS** | eProsima | Apache 2.0 | ROS2 默认，功能全面 | 通用开发 |
| **Cyclone DDS** | Eclipse | EPL/EDL | 轻量级，低延迟 | 嵌入式系统 |
| **RTI Connext** | RTI | 商业 | 工业级，高可靠性 | 关键任务系统 |
| **CoreDX** | Twin Oaks | 商业 | 极小内存占用 | 资源受限设备 |
| **GurumDDS** | Gurum Networks | 商业 | 韩国航天认证 | 航空航天 |

**性能基准测试**（基于 1KB 消息，1000Hz 发布频率）：

```
延迟对比（microseconds）：
Cyclone DDS:  ████ 45μs
Fast DDS:     ███████ 78μs  
RTI Connext:  █████ 52μs
CoreDX:       ███ 38μs

吞吐量对比（messages/sec）：
Fast DDS:     ████████████ 120K
Cyclone DDS:  ██████████ 100K
RTI Connext:  ███████████████ 150K
CoreDX:       █████████ 90K
```

### 4.1.4 ROS2 中的 DDS 抽象层

ROS2 通过 RMW（ROS MiddleWare）接口抽象不同的 DDS 实现：

```cpp
// RMW 接口示例
typedef struct rmw_node_t {
  const char * name;
  const char * namespace_;
  rmw_context_t * context;
  rmw_node_impl_t * impl;
} rmw_node_t;

// 创建节点的统一接口
rmw_node_t * rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_);
```

RMW 层次结构：

```
┌─────────────┐
│  ROS2 API   │  <- rclcpp/rclpy
├─────────────┤
│     RCL     │  <- ROS Client Library
├─────────────┤
│     RMW     │  <- 抽象接口层
├─────────────┤
│  DDS Impl   │  <- Fast DDS/Cyclone等
├─────────────┤
│   Network   │  <- UDP/共享内存
└─────────────┘
```

## 4.2 实时性与确定性设计

### 4.2.1 实时系统基础

实时系统的核心要求是 **确定性（Determinism）** 和 **时间约束（Timing Constraints）**。ROS2 从设计之初就考虑了实时性需求：

**实时性分类**：
- **硬实时（Hard Real-Time）**：错过截止时间会导致系统失败
- **固实时（Firm Real-Time）**：偶尔错过截止时间可接受，但无价值
- **软实时（Soft Real-Time）**：性能随错过次数下降

ROS2 实时性保证的数学模型：

设系统响应时间为 $R$，包含：
- $T_{sensor}$：传感器采样时间
- $T_{comm}$：通信延迟
- $T_{proc}$：处理时间
- $T_{actuator}$：执行器响应时间

则：$R = T_{sensor} + T_{comm} + T_{proc} + T_{actuator}$

对于硬实时系统，必须满足：$R \leq D_{deadline}$

### 4.2.2 内存管理策略

实时系统必须避免动态内存分配导致的不确定性。ROS2 提供多种内存管理策略：

**1. 预分配内存池**：

```cpp
// 使用自定义分配器避免运行时分配
template<typename T>
class PoolAllocator {
  private:
    std::array<T, POOL_SIZE> pool_;
    std::bitset<POOL_SIZE> used_;
    
  public:
    T* allocate(size_t n) {
        // O(1) 时间复杂度的分配
        for(size_t i = 0; i < POOL_SIZE; ++i) {
            if(!used_[i]) {
                used_[i] = true;
                return &pool_[i];
            }
        }
        throw std::bad_alloc();
    }
};
```

**2. 零拷贝通信**：

ROS2 支持通过共享内存实现零拷贝：

```
进程 A                     共享内存                    进程 B
┌──────┐                 ┌─────────┐                ┌──────┐
│Writer│───write ptr────►│ Message │◄───read ptr────│Reader│
└──────┘                 └─────────┘                └──────┘
         无需复制，直接访问同一内存区域
```

### 4.2.3 调度策略与优先级

ROS2 执行器支持多种调度策略：

**1. 单线程执行器（SingleThreadedExecutor）**：
- 顺序执行所有回调
- 无并发问题
- 适合简单实时任务

**2. 多线程执行器（MultiThreadedExecutor）**：
- 并行执行回调
- 需要同步机制
- 提高吞吐量

**3. 静态单线程执行器（StaticSingleThreadedExecutor）**：
- 编译时确定执行顺序
- 最小运行时开销
- 最佳实时性能

实时调度示例：

```cpp
// 设置实时优先级
struct sched_param param;
param.sched_priority = 95;  // RT优先级 1-99
pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

// CPU 亲和性设置
cpu_set_t cpuset;
CPU_ZERO(&cpuset);
CPU_SET(2, &cpuset);  // 绑定到 CPU 2
pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
```

### 4.2.4 时间抖动分析

时间抖动（Jitter）是实时系统的关键指标：

```
理想周期执行：
t0───┬───t1───┬───t2───┬───t3
     │100ms  │100ms  │100ms

实际执行（含抖动）：
t0───┬────t1'───┬──t2'────┬───t3'
     │102ms    │98ms     │101ms
     
抖动 = max(|ti' - ti|) = 2ms
```

ROS2 提供工具测量和优化抖动：

```bash
# 使用 ros2 trace 分析时间特性
ros2 trace --session-name realtime_analysis
ros2 run my_pkg realtime_node
# 分析结果
babeltrace ~/tracing/realtime_analysis | grep callback_duration
```

## 4.3 与 ROS1 的本质区别

### 4.3.1 架构范式转变

ROS1 和 ROS2 在架构上存在根本性差异：

**ROS1 架构（中心化）**：
```
        ┌─────────┐
        │ Master  │  <- 单点故障
        └────┬────┘
    ┌────────┼────────┐
    ▼        ▼        ▼
┌──────┐ ┌──────┐ ┌──────┐
│Node A│ │Node B│ │Node C│
└──────┘ └──────┘ └──────┘
    └────────┬────────┘
         P2P 通信
```

**ROS2 架构（去中心化）**：
```
┌──────┐     ┌──────┐     ┌──────┐
│Node A│◄───►│Node B│◄───►│Node C│
└──────┘     └──────┘     └──────┘
    ▲            ▲            ▲
    └────────────┼────────────┘
           DDS Discovery
         （自动发现）
```

### 4.3.2 通信机制演进

| 特性 | ROS1 | ROS2 |
|------|------|------|
| **协议** | TCPROS/UDPROS | DDS-RTPS |
| **发现机制** | Master 注册 | 自动发现 |
| **QoS** | 基础（队列大小） | 完整 QoS 策略 |
| **安全性** | 无内置支持 | SROS2 安全框架 |
| **传输层** | TCP/UDP | UDP/共享内存 |
| **类型协商** | MD5 哈希 | 类型描述 |

### 4.3.3 生命周期管理

ROS2 引入了标准化的节点生命周期：

```
        ┌─────────────┐
        │Unconfigured │
        └──────┬──────┘
               │ configure()
        ┌──────▼──────┐
        │  Inactive   │
        └──────┬──────┘
               │ activate()
        ┌──────▼──────┐
        │   Active    │
        └──────┬──────┘
               │ deactivate()
        ┌──────▼──────┐
        │  Inactive   │
        └──────┬──────┘
               │ cleanup()
        ┌──────▼──────┐
        │Unconfigured │
        └─────────────┘
```

生命周期节点实现：

```cpp
class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
  public:
    CallbackReturn on_configure(const State &) {
        // 分配资源，读取参数
        publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn on_activate(const State &) {
        // 开始处理
        publisher_->on_activate();
        return CallbackReturn::SUCCESS;
    }
};
```

### 4.3.4 构建系统对比

| 方面 | ROS1 (catkin) | ROS2 (colcon/ament) |
|------|---------------|---------------------|
| **构建工具** | catkin_make | colcon build |
| **包格式** | package.xml v1 | package.xml v3 |
| **CMake** | catkin macros | ament_cmake |
| **Python** | setup.py | setup.py + setup.cfg |
| **并行构建** | 有限支持 | 原生支持 |
| **增量构建** | 需要 catkin tools | 默认支持 |