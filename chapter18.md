# 第 18 章：多机器人系统

多机器人系统（Multi-Robot Systems, MRS）代表了机器人技术的前沿方向，通过协调多个机器人共同完成复杂任务，实现单机器人无法达到的效率、鲁棒性和覆盖范围。本章深入探讨 ROS2 中多机器人系统的设计与实现，从底层通信架构到高级群体智能算法，从理论模型到工业部署实践，全面覆盖多机器人系统的核心技术栈。

## 18.1 命名空间与域 ID 管理

### 18.1.1 ROS2 多机器人架构基础

ROS2 从设计之初就考虑了多机器人系统的需求，通过 DDS（Data Distribution Service）提供了强大的分布式通信能力。理解其架构对于构建高效的多机器人系统至关重要。

**DDS 域与分区概念：**

```
DDS 通信域架构：
┌─────────────────────────────────────────┐
│              DDS Domain 0                │
│  ┌──────────────┬──────────────┐        │
│  │  Partition A │  Partition B  │        │
│  │  (Robot1)    │  (Robot2)     │        │
│  └──────────────┴──────────────┘        │
│         Shared Topics/Services           │
└─────────────────────────────────────────┘

┌─────────────────────────────────────────┐
│              DDS Domain 1                │
│  ┌──────────────┬──────────────┐        │
│  │  Partition C │  Partition D  │        │
│  │  (Robot3)    │  (Robot4)     │        │
│  └──────────────┴──────────────┘        │
│         Isolated Communication           │
└─────────────────────────────────────────┘
```

**域 ID 分配策略：**

```cpp
class DomainManager {
private:
    struct DomainAllocation {
        int domainId;
        std::string purpose;
        std::set<std::string> robots;
        QoSProfile qosProfile;
    };
    
    std::map<int, DomainAllocation> domainMap;
    
public:
    // 智能域分配算法
    int allocateDomain(const std::string& robotGroup, 
                       const std::string& taskType) {
        // 基于任务类型和网络拓扑分配域
        if (taskType == "swarm_coordination") {
            // 群体协调任务使用共享域
            return 0;  
        } else if (taskType == "isolated_operation") {
            // 独立操作使用隔离域
            return generateUniqueDomain();
        } else if (taskType == "hierarchical_control") {
            // 分层控制使用多域架构
            return getHierarchicalDomain(robotGroup);
        }
        
        return 0;  // 默认域
    }
    
    // 动态域迁移
    void migrateToDomain(const std::string& robotId, 
                        int fromDomain, 
                        int toDomain) {
        // 优雅关闭当前域连接
        shutdownDomainParticipant(fromDomain, robotId);
        
        // 等待清理完成
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 加入新域
        createDomainParticipant(toDomain, robotId);
        
        // 重新发现邻居
        performDiscovery(toDomain);
    }
    
private:
    int generateUniqueDomain() {
        // 基于网络段和时间戳生成唯一域 ID
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
            now.time_since_epoch()).count();
        
        // 域 ID 范围：0-232（避免与系统保留域冲突）
        return (timestamp % 200) + 33;
    }
};
```

### 18.1.2 命名空间设计模式

命名空间是 ROS2 中组织多机器人系统的核心机制，良好的命名空间设计可以避免冲突、简化管理并提高系统可扩展性。

**层次化命名空间架构：**

```python
class NamespaceHierarchy:
    """多机器人系统命名空间管理器"""
    
    def __init__(self):
        self.namespace_tree = {
            'fleet': {
                'warehouse': {
                    'zone_a': ['robot_001', 'robot_002'],
                    'zone_b': ['robot_003', 'robot_004']
                },
                'delivery': {
                    'urban': ['robot_005', 'robot_006'],
                    'suburban': ['robot_007', 'robot_008']
                }
            }
        }
        
    def generate_namespace(self, robot_id: str, task_context: dict) -> str:
        """基于上下文生成命名空间"""
        # 基础命名空间
        base_ns = f"/{robot_id}"
        
        # 任务命名空间
        if task_context.get('collaborative'):
            # 协作任务使用共享命名空间
            task_ns = f"/collaborative/{task_context['task_id']}"
            return f"{task_ns}/{robot_id}"
        
        # 位置命名空间
        if task_context.get('location'):
            location = task_context['location']
            return f"/{location}/{robot_id}"
        
        # 功能命名空间
        if task_context.get('role'):
            role = task_context['role']
            return f"/{role}/{robot_id}"
        
        return base_ns
    
    def resolve_topic(self, namespace: str, topic_name: str) -> str:
        """解析完整话题名称"""
        # 处理相对和绝对话题名
        if topic_name.startswith('/'):
            return topic_name  # 绝对话题
        elif topic_name.startswith('~'):
            # 私有话题
            return f"{namespace}/{topic_name[1:]}"
        else:
            # 相对话题
            return f"{namespace}/{topic_name}"
```

**动态命名空间重映射：**

```cpp
class DynamicRemapper : public rclcpp::Node {
private:
    struct RemapRule {
        std::string source;
        std::string target;
        std::function<bool(const std::string&)> condition;
    };
    
    std::vector<RemapRule> remapRules;
    std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers;
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscribers;
    
public:
    DynamicRemapper() : Node("dynamic_remapper") {
        // 配置重映射规则
        setupRemapRules();
        
        // 定时器检查并更新映射
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DynamicRemapper::updateRemappings, this)
        );
    }
    
    void addRemapRule(const std::string& source, 
                      const std::string& target,
                      std::function<bool(const std::string&)> condition) {
        remapRules.push_back({source, target, condition});
    }
    
    void updateRemappings() {
        for (const auto& rule : remapRules) {
            if (rule.condition(rule.source)) {
                // 创建桥接
                createBridge(rule.source, rule.target);
            } else {
                // 移除桥接
                removeBridge(rule.source, rule.target);
            }
        }
    }
    
private:
    void createBridge(const std::string& source, const std::string& target) {
        // 检查是否已存在
        if (publishers.find(target) != publishers.end()) {
            return;
        }
        
        // 创建订阅者
        auto sub = create_subscription<std_msgs::msg::String>(
            source, 10,
            [this, target](const std_msgs::msg::String::SharedPtr msg) {
                // 转发消息到目标话题
                if (publishers.find(target) != publishers.end()) {
                    publishers[target]->publish(*msg);
                }
            }
        );
        
        // 创建发布者
        auto pub = create_publisher<std_msgs::msg::String>(target, 10);
        
        subscribers[source] = sub;
        publishers[target] = pub;
    }
};
```

### 18.1.3 发现机制与网络配置

ROS2 的自动发现机制在多机器人系统中既是优势也是挑战。正确配置发现机制对系统性能和安全性至关重要。

**自定义发现策略：**

```cpp
class DiscoveryManager {
private:
    struct PeerInfo {
        std::string robotId;
        std::string ipAddress;
        int domainId;
        std::chrono::steady_clock::time_point lastSeen;
        double signalStrength;  // 用于无线网络
        double latency;
    };
    
    std::map<std::string, PeerInfo> discoveredPeers;
    std::set<std::string> trustedPeers;
    
public:
    // 配置 DDS 发现参数
    void configureDDSDiscovery() {
        // 设置初始发现列表（避免广播）
        setenv("ROS_DISCOVERY_SERVER", "192.168.1.100:11811", 1);
        
        // 配置发现协议
        auto qos = rmw_qos_profile_default;
        qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
        qos.liveliness_lease_duration.sec = 1;
        qos.liveliness_lease_duration.nsec = 0;
        
        // 限制发现范围
        configureMulticast("239.255.0.1", 7400);
    }
    
    // 自适应发现策略
    void adaptiveDiscovery() {
        // 基于网络质量调整发现频率
        for (auto& [id, peer] : discoveredPeers) {
            if (peer.latency > 100.0) {  // 高延迟
                // 降低发现频率
                setDiscoveryPeriod(id, std::chrono::seconds(10));
            } else if (peer.signalStrength < -70) {  // 弱信号
                // 增加重试次数
                setDiscoveryRetries(id, 5);
            }
        }
    }
    
    // 安全发现过滤
    bool filterPeer(const std::string& peerId, const std::string& peerInfo) {
        // 白名单检查
        if (trustedPeers.find(peerId) == trustedPeers.end()) {
            RCLCPP_WARN(logger_, "Untrusted peer rejected: %s", peerId.c_str());
            return false;
        }
        
        // 验证证书（如果启用 SROS2）
        if (!verifyCertificate(peerId)) {
            return false;
        }
        
        return true;
    }
    
private:
    void configureMulticast(const std::string& address, int port) {
        // 配置多播地址和端口
        std::string config = R"(
            <dds>
                <transport_descriptors>
                    <transport_descriptor>
                        <transport_id>CustomMulticast</transport_id>
                        <type>UDPv4</type>
                        <interfaceWhiteList>
                            <address>192.168.1.0/24</address>
                        </interfaceWhiteList>
                    </transport_descriptor>
                </transport_descriptors>
            </dds>
        )";
        
        // 应用配置
        applyDDSConfig(config);
    }
};
```

**网络分区与隔离：**

```python
class NetworkPartitioner:
    """网络分区管理器for多机器人系统"""
    
    def __init__(self):
        self.partitions = {}
        self.routing_table = {}
        
    def create_partition(self, partition_id: str, 
                        robots: List[str],
                        isolation_level: str = 'soft'):
        """创建网络分区"""
        partition = {
            'id': partition_id,
            'robots': robots,
            'isolation': isolation_level,
            'vlan_id': self._allocate_vlan() if isolation_level == 'hard' else None,
            'multicast_group': self._allocate_multicast_group(),
            'qos_profile': self._determine_qos(len(robots))
        }
        
        self.partitions[partition_id] = partition
        
        # 配置路由
        self._configure_routing(partition)
        
        return partition
    
    def _configure_routing(self, partition):
        """配置分区路由规则"""
        if partition['isolation'] == 'hard':
            # 硬隔离：配置 VLAN
            for robot in partition['robots']:
                self._set_vlan(robot, partition['vlan_id'])
        elif partition['isolation'] == 'soft':
            # 软隔离：配置防火墙规则
            rules = self._generate_firewall_rules(partition)
            self._apply_firewall_rules(rules)
        
    def merge_partitions(self, partition_ids: List[str]):
        """合并多个分区"""
        merged_robots = []
        for pid in partition_ids:
            if pid in self.partitions:
                merged_robots.extend(self.partitions[pid]['robots'])
                del self.partitions[pid]
        
        # 创建新的合并分区
        new_partition_id = f"merged_{'_'.join(partition_ids)}"
        return self.create_partition(new_partition_id, merged_robots, 'soft')
```

## 18.2 分布式通信架构

### 18.2.1 通信拓扑设计

多机器人系统的通信拓扑直接影响系统的可扩展性、鲁棒性和性能。选择合适的拓扑结构是系统设计的关键决策。

**常见通信拓扑对比：**

```
1. 星型拓扑（Star Topology）：
     ┌──R1──┐
     │      │
   R2┤  C   ├R4
     │      │
     └──R3──┘
   优点：简单、易管理
   缺点：单点故障、扩展性差

2. 网状拓扑（Mesh Topology）：
   R1 ─── R2
   │ ╲   ╱ │
   │   ╳   │
   │ ╱   ╲ │
   R3 ─── R4
   优点：高冗余、鲁棒性强
   缺点：复杂度高、开销大

3. 分层拓扑（Hierarchical Topology）：
       M
      ╱ ╲
    L1   L2
   ╱ ╲  ╱ ╲
  R1 R2 R3 R4
   优点：可扩展、分级控制
   缺点：层间瓶颈、延迟增加
```

**自适应拓扑管理器：**

```cpp
class AdaptiveTopologyManager {
private:
    enum class TopologyType {
        STAR,
        MESH,
        HIERARCHICAL,
        HYBRID
    };
    
    struct NetworkMetrics {
        double averageLatency;
        double packetLossRate;
        double bandwidth;
        int nodeCount;
        double networkDiameter;  // 最大跳数
    };
    
    TopologyType currentTopology;
    NetworkMetrics metrics;
    
public:
    // 动态拓扑选择
    TopologyType selectOptimalTopology() {
        updateNetworkMetrics();
        
        if (metrics.nodeCount < 5) {
            // 小规模：全连接网状
            return TopologyType::MESH;
        } else if (metrics.nodeCount < 20) {
            // 中等规模：分层结构
            return TopologyType::HIERARCHICAL;
        } else if (metrics.packetLossRate > 0.05) {
            // 高丢包率：增加冗余的混合拓扑
            return TopologyType::HYBRID;
        } else {
            // 大规模稳定网络：星型with备份
            return TopologyType::STAR;
        }
    }
    
    // 拓扑重构
    void reconfigureTopology(TopologyType newTopology) {
        // 保存当前状态
        auto currentState = captureSystemState();
        
        // 通知所有节点准备重构
        broadcastReconfigurationNotice(newTopology);
        
        // 执行分阶段重构
        switch (newTopology) {
            case TopologyType::STAR:
                reconfigureToStar();
                break;
            case TopologyType::MESH:
                reconfigureToMesh();
                break;
            case TopologyType::HIERARCHICAL:
                reconfigureToHierarchical();
                break;
            case TopologyType::HYBRID:
                reconfigureToHybrid();
                break;
        }
        
        // 验证新拓扑
        if (!validateTopology()) {
            // 回滚到之前状态
            rollbackTopology(currentState);
        }
    }
    
private:
    void reconfigureToHierarchical() {
        // 选举层级领导者
        auto leaders = electLeaders();
        
        // 分配节点到集群
        auto clusters = partitionNodes(leaders);
        
        // 建立层内连接
        for (const auto& cluster : clusters) {
            establishIntraClusterConnections(cluster);
        }
        
        // 建立层间连接
        establishInterClusterConnections(leaders);
    }
};
```

### 18.2.2 消息路由与负载均衡

在多机器人系统中，高效的消息路由和负载均衡对于系统性能至关重要。

**智能消息路由器实现：**

```cpp
class MessageRouter {
private:
    struct Route {
        std::string source;
        std::string destination;
        double cost;  // 路由成本（延迟、跳数等）
        std::chrono::steady_clock::time_point lastUpdate;
    };
    
    struct MessageQueue {
        std::deque<std::any> messages;
        std::atomic<size_t> size{0};
        std::mutex mutex;
        std::condition_variable cv;
    };
    
    // 路由表
    std::map<std::pair<std::string, std::string>, Route> routingTable;
    // 消息队列
    std::map<std::string, MessageQueue> queues;
    
public:
    // 基于内容的路由
    void routeMessage(const std::string& topic, 
                     const std::any& message,
                     const std::map<std::string, std::any>& metadata) {
        // 提取路由键
        auto routingKey = extractRoutingKey(topic, metadata);
        
        // 查找最佳路由
        auto routes = findRoutes(routingKey);
        
        // 负载均衡策略
        auto selectedRoute = loadBalance(routes);
        
        // 发送消息
        sendViaRoute(message, selectedRoute);
    }
    
    // 动态负载均衡算法
    Route loadBalance(const std::vector<Route>& routes) {
        if (routes.empty()) {
            throw std::runtime_error("No available routes");
        }
        
        // 加权轮询with动态权重调整
        static std::map<std::string, int> weights;
        static std::map<std::string, int> currentWeight;
        
        Route selectedRoute;
        int maxWeight = -1;
        
        for (const auto& route : routes) {
            // 动态计算权重based on延迟和队列长度
            int weight = calculateDynamicWeight(route);
            weights[route.destination] = weight;
            
            // 加权轮询算法
            currentWeight[route.destination] += weight;
            
            if (currentWeight[route.destination] > maxWeight) {
                maxWeight = currentWeight[route.destination];
                selectedRoute = route;
            }
        }
        
        // 更新当前权重
        currentWeight[selectedRoute.destination] -= 
            std::accumulate(weights.begin(), weights.end(), 0,
                          [](int sum, const auto& p) { return sum + p.second; });
        
        return selectedRoute;
    }
    
private:
    int calculateDynamicWeight(const Route& route) {
        // 基础权重
        int baseWeight = 100;
        
        // 延迟因子（延迟越低，权重越高）
        double latencyFactor = 1.0 / (1.0 + route.cost);
        
        // 队列长度因子
        size_t queueLength = queues[route.destination].size.load();
        double queueFactor = 1.0 / (1.0 + queueLength / 100.0);
        
        // CPU 负载因子（需要监控系统）
        double cpuLoad = getCPULoad(route.destination);
        double cpuFactor = 1.0 - cpuLoad;
        
        return static_cast<int>(baseWeight * latencyFactor * queueFactor * cpuFactor);
    }
};
```
