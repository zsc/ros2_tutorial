# 第 11 章：SLAM 与定位系统

本章深入探讨 ROS2 中的同时定位与地图构建（SLAM）技术和定位系统。我们将从经典的自适应蒙特卡洛定位（AMCL）开始，逐步深入到现代图优化 SLAM 方法，最后讨论多传感器融合定位策略。通过学习本章，读者将掌握在复杂环境中实现机器人精确定位的核心技术，理解不同定位方法的适用场景，并能够根据实际需求选择和优化定位方案。

## 11.1 AMCL 定位原理

自适应蒙特卡洛定位（Adaptive Monte Carlo Localization, AMCL）是 ROS2 中最常用的概率定位算法，基于粒子滤波实现全局定位和位姿跟踪。

### 11.1.1 粒子滤波基础

AMCL 使用粒子集合表示机器人位姿的概率分布：

$$
\text{Bel}(x_t) \approx \{x_t^{[i]}, w_t^{[i]}\}_{i=1}^N
$$

其中 $x_t^{[i]} = (x, y, \theta)$ 表示第 $i$ 个粒子的位姿，$w_t^{[i]}$ 是对应权重。

粒子滤波的核心步骤：

```
1. 预测步骤（运动更新）
   对每个粒子 i:
   x_t^[i] = sample_motion_model(u_t, x_{t-1}^[i])
   
2. 更新步骤（观测更新）
   对每个粒子 i:
   w_t^[i] = measurement_model(z_t, x_t^[i], m)
   
3. 重采样
   如果有效粒子数 N_eff < 阈值:
   执行重采样，生成新粒子集
```

### 11.1.2 运动模型

AMCL 支持三种运动模型：

**1. 差分驱动模型（Differential Model）**

适用于差分驱动机器人，考虑平移和旋转噪声：

$$
\begin{aligned}
\delta_{rot1} &= \text{atan2}(\Delta y, \Delta x) - \theta_{t-1} \\
\delta_{trans} &= \sqrt{\Delta x^2 + \Delta y^2} \\
\delta_{rot2} &= \theta_t - \theta_{t-1} - \delta_{rot1}
\end{aligned}
$$

噪声参数：
- `alpha1`: 旋转引起的旋转噪声
- `alpha2`: 平移引起的旋转噪声  
- `alpha3`: 平移引起的平移噪声
- `alpha4`: 旋转引起的平移噪声

**2. 全向模型（Omni Model）**

适用于全向移动机器人，独立处理 x、y、θ 方向：

$$
\begin{aligned}
x_t &= x_{t-1} + \Delta x + \epsilon_x \\
y_t &= y_{t-1} + \Delta y + \epsilon_y \\
\theta_t &= \theta_{t-1} + \Delta \theta + \epsilon_\theta
\end{aligned}
$$

**3. 静态模型（Stationary Model）**

用于机器人静止时的定位维持，仅添加少量噪声防止粒子退化。

### 11.1.3 观测模型

AMCL 的激光雷达观测模型混合四种概率分布：

$$
p(z_t|x_t, m) = z_{hit} \cdot p_{hit} + z_{short} \cdot p_{short} + z_{max} \cdot p_{max} + z_{rand} \cdot p_{rand}
$$

各分量含义：
- **$p_{hit}$**: 高斯分布，建模测量噪声
- **$p_{short}$**: 指数分布，建模动态障碍物
- **$p_{max}$**: 点质量分布，建模最大测距
- **$p_{rand}$**: 均匀分布，建模随机噪声

似然场模型（Likelihood Field Model）优化：

```
对于激光点 (x_k, y_k):
1. 计算到最近障碍物的距离 d
2. 似然值 = exp(-d²/(2σ²))
3. 总似然 = ∏ 各点似然值
```

### 11.1.4 自适应粒子数

AMCL 通过 KLD 采样动态调整粒子数：

$$
N = \frac{k-1}{2\epsilon} \left\{1 - \frac{2}{9(k-1)} + \sqrt{\frac{2}{9(k-1)}}z_{1-\delta}\right\}^3
$$

其中：
- $k$: 占用的直方图单元数
- $\epsilon$: KLD 误差上界
- $\delta$: 置信度参数

### 11.1.5 参数调优策略

关键参数及调优建议：

```yaml
# 粒子数配置
min_particles: 100      # 最少粒子数，增加提高精度
max_particles: 5000     # 最大粒子数，限制计算量

# 运动模型噪声（差分驱动）
alpha1: 0.2   # 减小使粒子更集中
alpha2: 0.2   # 增大应对打滑
alpha3: 0.2   # 根据里程计精度调整
alpha4: 0.2   # 通常较小

# 激光模型参数
laser_z_hit: 0.5        # 正确测量权重
laser_z_short: 0.05     # 动态障碍物权重
laser_z_max: 0.05       # 最大测距权重
laser_z_rand: 0.4       # 随机测量权重
laser_sigma_hit: 0.2    # 测量标准差

# 更新频率
update_min_d: 0.25      # 最小平移更新距离(m)
update_min_a: 0.2       # 最小旋转更新角度(rad)

# 重采样阈值
resample_interval: 1    # 重采样间隔
recovery_alpha_slow: 0.001  # 慢速平均，用于检测定位失败
recovery_alpha_fast: 0.1    # 快速平均
```

调优流程：
1. 从默认参数开始
2. 根据里程计质量调整运动噪声
3. 根据激光雷达特性调整观测模型
4. 在计算资源允许下增加粒子数
5. 使用 rviz2 可视化粒子云评估收敛性

## 11.2 SLAM 工具链（slam_toolbox）

slam_toolbox 是 ROS2 推荐的 2D SLAM 解决方案，基于图优化提供在线和离线建图能力。

### 11.2.1 图优化 SLAM 原理

slam_toolbox 使用位姿图（Pose Graph）表示 SLAM 问题：

$$
x^* = \arg\min_x \sum_{ij} e_{ij}^T \Omega_{ij} e_{ij}
$$

其中：
- $x = \{x_1, ..., x_n\}$: 机器人位姿序列
- $e_{ij} = z_{ij} - h(x_i, x_j)$: 测量残差
- $\Omega_{ij}$: 信息矩阵（协方差逆）

核心组件：

```
图结构：
- 节点(Vertex): 机器人位姿 x_i = (x, y, θ)
- 边(Edge): 位姿间约束
  - 里程计边: 连续位姿间
  - 回环边: 检测到回环时添加
  - 激光匹配边: scan-to-map 匹配

优化器：
- 使用 Sparse Pose Adjustment (SPA)
- Levenberg-Marquardt 迭代求解
- 增量式优化减少计算量
```

### 11.2.2 扫描匹配算法

slam_toolbox 使用相关性扫描匹配（Correlative Scan Matching）：

**1. 粗匹配（Branch-and-Bound）**

```
搜索空间离散化:
- 位置: Δx, Δy ∈ [-search_size, search_size]
- 角度: Δθ ∈ [-search_angle, search_angle]

分支定界加速:
1. 计算上界 upper_bound(region)
2. 如果 upper_bound < current_best:
   剪枝该分支
3. 否则继续细分
```

**2. 精匹配（Hill Climbing）**

$$
\text{score} = \sum_{i} M(T(p_i))
$$

其中 $M$ 是占用栅格地图，$T$ 是变换，$p_i$ 是激光点。

使用梯度上升优化：
```
while not converged:
  计算当前位姿得分
  尝试6个方向(±Δx, ±Δy, ±Δθ)
  选择得分最高的方向
  更新步长（自适应）
```

### 11.2.3 在线与离线建图

**在线建图（Online SLAM）**

实时构建和优化地图：

```yaml
# 在线模式配置
mode: mapping

# 扫描缓冲
scan_buffer_size: 10
scan_buffer_maximum_scan_distance: 10.0

# 匹配参数
distance_variance_penalty: 0.5
angle_variance_penalty: 1.0
fine_search_angle_offset: 0.00349

# 回环检测
loop_search_space_dimension: 8.0
loop_search_space_resolution: 0.05
loop_search_space_smear_deviation: 0.03

# 优化触发
minimum_travel_distance: 0.5
minimum_travel_heading: 0.5
```

**离线建图（Offline SLAM）**

批处理优化已录制数据：

```yaml
# 离线模式配置
mode: localization_and_mapping

# 全局优化
do_loop_closing: true
loop_match_minimum_chain_size: 10
loop_match_maximum_variance_coarse: 3.0
loop_match_minimum_response_coarse: 0.35

# 批处理优化
optimization_frequency: 20.0
global_optimization_frequency: 1.0
```

离线处理流程：
1. 加载 rosbag 文件
2. 顺序处理激光扫描
3. 定期执行全局优化
4. 保存优化后的地图

### 11.2.4 回环检测与闭合

回环检测策略：

**1. 候选检测**

```python
def find_loop_candidates(current_pose, pose_graph):
    candidates = []
    for node in pose_graph:
        # 空间接近但时间远离
        spatial_dist = euclidean_distance(current_pose, node.pose)
        temporal_dist = current_time - node.time
        
        if spatial_dist < loop_search_radius and \
           temporal_dist > minimum_time_interval:
            candidates.append(node)
    
    return candidates
```

**2. 验证与优化**

```
对每个候选:
1. 执行扫描匹配
2. 计算匹配得分和协方差
3. 如果得分 > 阈值:
   - 添加回环约束边
   - 触发图优化
   - 更新地图
```

### 11.2.5 地图序列化与管理

slam_toolbox 的地图格式：

```yaml
# .posegraph 文件格式
header:
  version: 1.0.0
  robot_id: "robot_0"
  
nodes:
  - id: 0
    pose: [x, y, theta]
    scan: [ranges, angles]
    time: 1234567890.123
    
edges:
  - type: "odometry"
    from: 0
    to: 1
    transform: [dx, dy, dtheta]
    information: [[3x3 matrix]]
    
map_data:
  resolution: 0.05
  origin: [0.0, 0.0, 0.0]
  data: [occupancy_grid]
```

地图操作 API：

```python
# 保存地图
slam_toolbox.save_map(filename="warehouse_map")

# 加载地图继续建图
slam_toolbox.deserialize_map(filename="warehouse_map.posegraph")

# 合并多个地图
slam_toolbox.merge_maps(
    maps=["floor1.posegraph", "floor2.posegraph"],
    output="building.posegraph"
)
```

## 11.3 地图服务器架构

ROS2 的地图服务器负责地图的加载、发布和管理，支持多种地图格式和动态更新。

### 11.3.1 地图格式与表示

**占用栅格地图（OccupancyGrid）**

标准 2D 地图表示：

```yaml
# map.yaml 配置文件
image: warehouse.pgm
resolution: 0.05  # 米/像素
origin: [-10.0, -10.0, 0.0]  # [x, y, yaw]
negate: 0  # 0=白色自由,1=黑色自由
occupied_thresh: 0.65  # >此值为占用
free_thresh: 0.196     # <此值为自由

# 栅格值含义
# -1: 未知区域
#  0: 自由空间
#  100: 占用（障碍物）
# 1-99: 占用概率
```

地图消息结构：

```python
# nav_msgs/msg/OccupancyGrid
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: "map"
  
info:
  map_load_time: {sec: 0, nanosec: 0}
  resolution: 0.05
  width: 2048
  height: 2048
  origin:
    position: {x: -51.2, y: -51.2, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    
data: [int8 array]  # row-major order
```

**成本地图（Costmap）**

用于导航的膨胀地图：

```
成本值定义:
0: 自由空间
1-252: 成本递增
253: 膨胀半径内
254: 致命障碍物膨胀区
255: 致命障碍物

膨胀函数:
cost = 253 * exp(-α * distance)
其中 α = -ln(0.01) / inflation_radius
```

### 11.3.2 map_server 设计

ROS2 map_server 节点架构：

```
map_server 组件:
├── MapServer (加载和发布静态地图)
│   ├── 服务: /map_server/load_map
│   ├── 话题: /map (OccupancyGrid)
│   └── 参数: yaml_filename
│
├── MapSaver (保存地图到文件)
│   ├── 服务: /map_server/save_map
│   ├── 订阅: /map
│   └── 参数: save_map_timeout
│
└── CostmapServer (成本地图管理)
    ├── 话题: /global_costmap/costmap
    ├── 服务: /global_costmap/clear_entirely
    └── 插件: static_layer, inflation_layer
```

生命周期管理：

```python
class MapServer(Node):
    def __init__(self):
        super().__init__('map_server')
        
        # 声明参数
        self.declare_parameter('yaml_filename', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic_name', 'map')
        
        # 创建发布者
        self.map_publisher = self.create_publisher(
            OccupancyGrid, 
            self.get_parameter('topic_name').value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        
        # 创建服务
        self.load_map_service = self.create_service(
            LoadMap, 
            'load_map',
            self.load_map_callback
        )
        
    def load_map(self, yaml_file):
        # 加载 YAML 配置
        with open(yaml_file, 'r') as f:
            map_config = yaml.safe_load(f)
            
        # 加载图像文件
        map_image = cv2.imread(map_config['image'], -1)
        
        # 转换为 OccupancyGrid
        map_msg = self.image_to_occupancy_grid(
            map_image, 
            map_config
        )
        
        # 发布地图
        self.map_publisher.publish(map_msg)
```

### 11.3.3 多层地图管理

分层地图架构支持不同抽象级别：

```yaml
# multi_layer_map.yaml
layers:
  - name: "static_map"
    type: "occupancy_grid"
    file: "building_structure.pgm"
    
  - name: "semantic_layer"
    type: "semantic_map"
    regions:
      - {name: "office_101", polygon: [[x1,y1], ...]}
      - {name: "corridor_A", polygon: [[x2,y2], ...]}
      
  - name: "topology_layer"
    type: "topology_map"
    nodes:
      - {id: 1, name: "entrance", pose: [x, y, theta]}
      - {id: 2, name: "elevator", pose: [x, y, theta]}
    edges:
      - {from: 1, to: 2, distance: 15.0}
```

层间同步机制：

```python
class MultiLayerMapServer:
    def __init__(self):
        self.layers = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        
    def add_layer(self, name, layer_type, data):
        self.layers[name] = {
            'type': layer_type,
            'data': data,
            'publisher': self.create_publisher(...)
        }
        
    def query_location(self, x, y):
        """查询位置的多层信息"""
        result = {}
        
        # 占用信息
        result['occupied'] = self.layers['static_map'].is_occupied(x, y)
        
        # 语义信息
        result['semantic'] = self.layers['semantic_layer'].get_region(x, y)
        
        # 拓扑信息
        result['nearest_node'] = self.layers['topology_layer'].nearest(x, y)
        
        return result
```

### 11.3.4 动态地图更新

实现动态障碍物和地图修改：

**临时障碍物层**

```python
class DynamicObstacleLayer:
    def __init__(self):
        self.obstacles = []
        self.decay_rate = 0.1  # 障碍物消失速率
        
    def add_obstacle(self, x, y, radius, duration):
        """添加临时障碍物"""
        obstacle = {
            'center': (x, y),
            'radius': radius,
            'timestamp': self.get_clock().now(),
            'duration': duration
        }
        self.obstacles.append(obstacle)
        
    def update_costmap(self, costmap):
        """更新成本地图"""
        current_time = self.get_clock().now()
        
        for obs in self.obstacles[:]:
            age = (current_time - obs['timestamp']).nanoseconds / 1e9
            
            if age > obs['duration']:
                self.obstacles.remove(obs)
            else:
                # 渐变消失效果
                alpha = 1.0 - (age / obs['duration'])
                self.apply_obstacle_to_costmap(
                    costmap, obs, alpha
                )
```

**增量地图更新**

```python
class IncrementalMapUpdate:
    def __init__(self):
        self.base_map = None
        self.updates = []
        
    def apply_laser_update(self, scan, pose):
        """使用激光扫描更新地图"""
        # 射线追踪清除动态障碍物
        for i, range_val in enumerate(scan.ranges):
            if range_val < scan.range_max:
                # 清除射线路径
                self.clear_ray(
                    pose, 
                    pose + range_val * angle_to_vector(scan.angle_min + i * scan.angle_increment)
                )
                # 标记终点为占用
                self.mark_occupied(endpoint)
                
    def merge_updates(self):
        """合并更新到基础地图"""
        merged = self.base_map.copy()
        
        for update in self.updates:
            # 贝叶斯更新
            merged = self.bayesian_update(merged, update)
            
        return merged
```

### 11.3.5 地图服务接口

标准地图服务：

```python
# 加载地图服务
class LoadMapService:
    def handle_load_map(self, request, response):
        try:
            # 加载指定地图
            map_data = self.load_map_from_file(request.map_url)
            
            # 发布到话题
            self.publish_map(map_data)
            
            # 返回结果
            response.result = LoadMap.Response.RESULT_SUCCESS
            response.map = map_data
            
        except Exception as e:
            response.result = LoadMap.Response.RESULT_INVALID_MAP
            
        return response

# 保存地图服务
class SaveMapService:
    def handle_save_map(self, request, response):
        # 获取当前地图
        current_map = self.get_current_map()
        
        # 保存到文件
        self.save_to_file(
            request.filename,
            current_map,
            request.resolution,
            request.free_thresh,
            request.occupied_thresh
        )
        
        response.result = True
        return response
```

## 11.4 多传感器融合定位

现代机器人系统通常集成多种传感器实现鲁棒定位，包括 IMU、里程计、激光雷达、视觉和 GPS 等。

### 11.4.1 传感器特性分析

不同传感器的互补特性：

| 传感器 | 更新频率 | 精度特点 | 误差类型 | 适用场景 |
|--------|----------|----------|----------|----------|
| 轮式里程计 | 10-100 Hz | 短期精确 | 累积漂移 | 平滑地面 |
| IMU | 100-1000 Hz | 高频响应 | 偏置/漂移 | 动态运动 |
| 激光雷达 | 10-40 Hz | 高精度 | 环境依赖 | 结构化环境 |
| 视觉 | 10-60 Hz | 特征丰富 | 光照敏感 | 纹理环境 |
| GPS/RTK | 1-10 Hz | 全局定位 | 多径/遮挡 | 室外开阔 |

### 11.4.2 扩展卡尔曼滤波（EKF）

robot_localization 包的 EKF 实现：

**状态向量定义**

15 维状态向量：
$$
\mathbf{x} = [x, y, z, \phi, \theta, \psi, \dot{x}, \dot{y}, \dot{z}, \dot{\phi}, \dot{\theta}, \dot{\psi}, \ddot{x}, \ddot{y}, \ddot{z}]^T
$$

**预测步骤**

运动模型：
$$
\mathbf{x}_k = f(\mathbf{x}_{k-1}, \mathbf{u}_k) + \mathbf{w}_k
$$

其中 $f$ 是非线性状态转移函数，$\mathbf{w}_k$ 是过程噪声。

协方差预测：
$$
\mathbf{P}_k^- = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^T + \mathbf{Q}_k
$$

**更新步骤**

观测模型：
$$
\mathbf{z}_k = h(\mathbf{x}_k) + \mathbf{v}_k
$$

卡尔曼增益：
$$
\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + \mathbf{R}_k)^{-1}
$$

状态更新：
$$
\mathbf{x}_k = \mathbf{x}_k^- + \mathbf{K}_k(\mathbf{z}_k - h(\mathbf{x}_k^-))
$$

### 11.4.3 robot_localization 配置

**EKF 节点配置**

```yaml
# ekf_localization.yaml
ekf_filter_node:
  ros__parameters:
    # 基础配置
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    
    # 坐标系
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    
    # 里程计输入
    odom0: /wheel_odometry
    odom0_config: [false, false, false,   # x, y, z
                   false, false, false,   # roll, pitch, yaw
                   true,  true,  false,   # vx, vy, vz
                   false, false, true,    # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
    odom0_differential: false
    odom0_relative: false
    
    # IMU 输入
    imu0: /imu/data
    imu0_config: [false, false, false,    # x, y, z
                  true,  true,  true,     # roll, pitch, yaw
                  false, false, false,    # vx, vy, vz
                  true,  true,  true,     # vroll, vpitch, vyaw
                  true,  true,  true]     # ax, ay, az
    imu0_differential: false
    imu0_relative: true
    imu0_remove_gravitational_acceleration: true
    
    # 激光定位输入
    pose0: /amcl_pose
    pose0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
    pose0_differential: false
    pose0_relative: false
    
    # 过程噪声协方差
    process_noise_covariance: [
      0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0.04, 0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.02, 0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.015
    ]
```

### 11.4.4 无迹卡尔曼滤波（UKF）

UKF 通过确定性采样处理非线性：

**Sigma 点生成**

```python
def generate_sigma_points(x, P, alpha=0.001, beta=2, kappa=0):
    n = len(x)
    lambda_ = alpha**2 * (n + kappa) - n
    
    # 权重计算
    Wm = np.zeros(2*n + 1)
    Wc = np.zeros(2*n + 1)
    Wm[0] = lambda_ / (n + lambda_)
    Wc[0] = Wm[0] + (1 - alpha**2 + beta)
    Wm[1:] = Wc[1:] = 0.5 / (n + lambda_)
    
    # Sigma 点
    sigma_points = np.zeros((2*n + 1, n))
    sigma_points[0] = x
    
    sqrt_matrix = np.linalg.cholesky((n + lambda_) * P)
    for i in range(n):
        sigma_points[i+1] = x + sqrt_matrix[i]
        sigma_points[n+i+1] = x - sqrt_matrix[i]
    
    return sigma_points, Wm, Wc
```

**UKF 更新**

```python
class UKF:
    def predict(self, dt):
        # 生成 sigma 点
        sigma_points, Wm, Wc = generate_sigma_points(
            self.x, self.P
        )
        
        # 通过运动模型传播
        sigma_points_pred = np.array([
            self.motion_model(sp, dt) for sp in sigma_points
        ])
        
        # 计算预测均值和协方差
        self.x = np.sum(Wm[:, None] * sigma_points_pred, axis=0)
        self.P = self.Q.copy()
        for i in range(len(sigma_points)):
            diff = sigma_points_pred[i] - self.x
            self.P += Wc[i] * np.outer(diff, diff)
    
    def update(self, z, sensor_model):
        # 生成 sigma 点
        sigma_points, Wm, Wc = generate_sigma_points(
            self.x, self.P
        )
        
        # 通过观测模型传播
        z_sigma = np.array([
            sensor_model(sp) for sp in sigma_points
        ])
        
        # 计算观测预测
        z_pred = np.sum(Wm[:, None] * z_sigma, axis=0)
        
        # 计算协方差
        Pzz = self.R.copy()
        Pxz = np.zeros((len(self.x), len(z)))
        
        for i in range(len(sigma_points)):
            dz = z_sigma[i] - z_pred
            dx = sigma_points[i] - self.x
            Pzz += Wc[i] * np.outer(dz, dz)
            Pxz += Wc[i] * np.outer(dx, dz)
        
        # 卡尔曼增益
        K = Pxz @ np.linalg.inv(Pzz)
        
        # 状态更新
        self.x += K @ (z - z_pred)
        self.P -= K @ Pzz @ K.T
```

### 11.4.5 传感器标定

**IMU-里程计标定**

```python
def calibrate_imu_odometry(imu_data, odom_data):
    """标定 IMU 和里程计之间的外参"""
    # 时间同步
    imu_sync, odom_sync = time_synchronize(
        imu_data, odom_data
    )
    
    # 提取角速度
    imu_angular_vel = imu_sync['angular_velocity']
    odom_angular_vel = compute_angular_velocity(odom_sync)
    
    # 最小二乘标定
    scale = np.linalg.lstsq(
        imu_angular_vel, odom_angular_vel, rcond=None
    )[0]
    
    # 计算时间偏移
    correlation = np.correlate(
        imu_angular_vel, odom_angular_vel, mode='full'
    )
    time_offset = np.argmax(correlation) - len(imu_angular_vel)
    
    return scale, time_offset
```

**激光-IMU 标定**

```yaml
# 外参标定结果
lidar_imu_extrinsics:
  translation: [0.15, 0.0, 0.08]  # [x, y, z]
  rotation: [0.0, 0.0, 0.0]       # [roll, pitch, yaw]
  time_offset: 0.05                # 秒
```

### 11.4.6 故障检测与恢复

传感器故障检测：

```python
class SensorFaultDetector:
    def __init__(self):
        self.chi2_threshold = 9.21  # 95% 置信度，3自由度
        self.innovation_window = 10
        
    def check_innovation(self, innovation, S):
        """马氏距离检验"""
        chi2 = innovation.T @ np.linalg.inv(S) @ innovation
        return chi2 < self.chi2_threshold
    
    def check_sensor_health(self, sensor_name, data):
        """传感器健康检查"""
        checks = {
            'data_rate': self.check_data_rate(data),
            'range_valid': self.check_range(data),
            'noise_level': self.check_noise(data),
            'consistency': self.check_consistency(data)
        }
        
        if not all(checks.values()):
            self.handle_sensor_failure(sensor_name, checks)
            
    def handle_sensor_failure(self, sensor_name, failures):
        """故障处理策略"""
        if sensor_name == 'gps' and not failures['data_rate']:
            # GPS 信号丢失，切换到纯里程计/IMU
            self.switch_to_dead_reckoning()
            
        elif sensor_name == 'imu' and not failures['noise_level']:
            # IMU 噪声过大，降低权重
            self.reduce_sensor_weight('imu', factor=0.1)
```

## 产业案例研究：Amazon 仓库机器人大规模定位

Amazon Robotics（前 Kiva Systems）在全球数百个配送中心部署了超过 50 万台自主移动机器人（AMR），实现了仓库物流的革命性变革。

### 系统架构

**分层定位系统**

```
全局层（Fleet Manager）
├── 集中式地图管理
├── 全局路径规划
└── 任务分配

区域层（Zone Controller）
├── 局部地图更新
├── 区域内协调
└── 拥塞管理

机器人层（Robot）
├── 板载定位（AMCL+EKF）
├── 局部避障
└── 执行控制
```

### 技术方案

**1. 二维码辅助定位**

```python
class FiducialLocalization:
    def __init__(self):
        self.fiducial_map = {}  # ID -> (x, y, theta)
        self.last_fiducial = None
        self.dead_reckoning_distance = 0
        
    def update(self, fiducial_detection, odometry):
        if fiducial_detection:
            # 绝对定位校正
            fid_pose = self.fiducial_map[fiducial_detection.id]
            self.correct_pose(fid_pose, fiducial_detection.relative_pose)
            self.last_fiducial = fiducial_detection.id
            self.dead_reckoning_distance = 0
        else:
            # 里程计推算
            self.dead_reckoning_update(odometry)
            self.dead_reckoning_distance += odometry.distance
            
        # 不确定性管理
        if self.dead_reckoning_distance > 5.0:  # 5米
            self.request_fiducial_scan()
```

**2. 多机器人协作定位**

```yaml
# 协作配置
cooperative_localization:
  enable: true
  max_range: 10.0  # 米
  min_observations: 3
  
  # 相对观测
  robot_detection:
    method: "uwb_ranging"  # 或 "visual_markers"
    frequency: 5.0
    
  # 信息融合
  fusion:
    method: "distributed_ekf"
    communication: "wifi_mesh"
```

### 性能指标

| 指标 | 目标值 | 实际达成 |
|------|--------|----------|
| 定位精度 | < 5cm | 2-3cm |
| 更新频率 | > 10Hz | 20Hz |
| 初始化时间 | < 10s | 5-8s |
| 系统可用性 | > 99.9% | 99.95% |
| 单仓库机器人数 | 1000+ | 3000+ |

### 关键优化

**1. 地图压缩与分发**

```python
class MapDistribution:
    def __init__(self):
        self.global_map = None  # 完整地图
        self.zone_maps = {}     # 分区地图
        
    def generate_robot_map(self, robot_id, task):
        """生成任务相关的精简地图"""
        # 确定活动区域
        active_zones = self.get_task_zones(task)
        
        # 提取相关地图
        robot_map = OccupancyGrid()
        for zone in active_zones:
            robot_map.merge(self.zone_maps[zone])
            
        # 压缩传输
        compressed = self.compress_map(robot_map)
        return compressed
```

**2. 定位失败恢复**

```python
class LocalizationRecovery:
    def __init__(self):
        self.recovery_behaviors = [
            self.rotate_in_place,
            self.move_to_open_space,
            self.request_assistance
        ]
        
    def handle_lost(self):
        """定位丢失处理流程"""
        # 1. 停止移动
        self.stop_robot()
        
        # 2. 尝试恢复
        for behavior in self.recovery_behaviors:
            if behavior():
                return True
                
        # 3. 请求人工介入
        self.alert_operator()
        return False
```

### 踩坑与解决

**问题 1：WiFi 干扰导致通信延迟**
- 症状：高密度部署区域定位跳变
- 解决：5GHz 频段 + 定向天线 + 时间槽分配

**问题 2：地面标记磨损**
- 症状：二维码识别率下降
- 解决：冗余标记 + 预测性维护 + 激光 SLAM 备份

**问题 3：动态障碍物（人员）**
- 症状：频繁重定位
- 解决：人员检测 + 预测轨迹 + 安全区域

## 高级话题：GraphSLAM 与因子图优化

### GraphSLAM 原理

GraphSLAM 将 SLAM 问题表示为因子图，通过最大后验估计（MAP）求解：

$$
X^*, L^* = \arg\max_{X,L} P(X, L | Z, U)
$$

因子图表示：
- 变量节点：机器人位姿 $x_i$，地标位置 $l_j$
- 因子节点：运动约束 $f_u$，观测约束 $f_z$

### 增量式平滑与建图（iSAM2）

```python
class iSAM2:
    def __init__(self):
        self.bayes_tree = BayesTree()
        self.theta = {}  # 当前估计
        
    def add_factor(self, factor):
        """增量添加因子"""
        # 识别受影响的变量
        affected = self.find_affected_cliques(factor)
        
        # 重新线性化
        for clique in affected:
            self.relinearize(clique)
            
        # 部分消元
        self.partial_elimination(affected)
        
        # 反向替换更新
        self.back_substitution()
```

### g2o 优化框架集成

```cpp
// g2o 图优化示例
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

class GraphSLAMOptimizer {
    g2o::SparseOptimizer optimizer;
    
public:
    void optimize() {
        // 配置求解器
        auto linearSolver = std::make_unique<
            g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
        auto blockSolver = std::make_unique<
            g2o::BlockSolver_6_3>(std::move(linearSolver));
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            std::move(blockSolver));
            
        optimizer.setAlgorithm(solver);
        
        // 添加顶点（位姿）
        for (const auto& pose : poses) {
            auto vertex = new g2o::VertexSE2();
            vertex->setId(pose.id);
            vertex->setEstimate(pose.to_se2());
            optimizer.addVertex(vertex);
        }
        
        // 添加边（约束）
        for (const auto& constraint : constraints) {
            auto edge = new g2o::EdgeSE2();
            edge->setVertex(0, optimizer.vertex(constraint.from));
            edge->setVertex(1, optimizer.vertex(constraint.to));
            edge->setMeasurement(constraint.transform);
            edge->setInformation(constraint.information);
            optimizer.addEdge(edge);
        }
        
        // 执行优化
        optimizer.initializeOptimization();
        optimizer.optimize(100);
    }
};
```

### 分布式 SLAM

多机器人协作 SLAM 算法：

```python
class DistributedSLAM:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.local_graph = FactorGraph()
        self.shared_variables = {}
        
    def consensus_optimization(self):
        """ADMM 共识优化"""
        for iteration in range(max_iterations):
            # 本地优化
            self.local_update()
            
            # 交换共享变量
            self.exchange_shared_variables()
            
            # 拉格朗日乘子更新
            self.update_dual_variables()
            
            # 检查收敛
            if self.check_convergence():
                break
```

### 语义 SLAM

结合语义信息的 SLAM：

```python
class SemanticSLAM:
    def __init__(self):
        self.geometric_map = OccupancyGrid()
        self.semantic_map = {}
        self.object_detector = YOLOv8()
        
    def process_frame(self, image, depth, pose):
        # 目标检测
        detections = self.object_detector(image)
        
        # 3D 重建
        for det in detections:
            object_3d = self.backproject(det, depth, pose)
            
            # 数据关联
            matched = self.data_association(object_3d)
            
            if matched:
                # 更新已有对象
                self.update_object(matched, object_3d)
            else:
                # 添加新对象
                self.add_object(object_3d)
                
        # 语义约束优化
        self.optimize_with_semantics()
```

### 推荐论文

1. **Grisetti et al., 2010** - "A Tutorial on Graph-Based SLAM"
   - GraphSLAM 理论基础

2. **Kaess et al., 2012** - "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree"
   - 增量式优化方法

3. **Rosinol et al., 2020** - "Kimera: Real-Time Metric-Semantic SLAM"
   - 语义 SLAM 最新进展

### 开源项目

- **GTSAM**: Georgia Tech Smoothing and Mapping library
- **g2o**: General Graph Optimization
- **Cartographer**: Google 的实时 2D/3D SLAM
- **ORB-SLAM3**: 视觉/视觉-惯性 SLAM

## 本章小结

本章系统介绍了 ROS2 中的 SLAM 与定位技术栈：

**核心概念**
- AMCL 粒子滤波定位：运动模型、观测模型、自适应粒子数
- slam_toolbox 图优化：扫描匹配、回环检测、地图序列化
- 地图服务器：多层地图、动态更新、服务接口
- 多传感器融合：EKF/UKF、robot_localization、传感器标定

**关键公式**
- 粒子滤波后验：$\text{Bel}(x_t) \approx \{x_t^{[i]}, w_t^{[i]}\}_{i=1}^N$
- 图优化目标：$x^* = \arg\min_x \sum_{ij} e_{ij}^T \Omega_{ij} e_{ij}$
- EKF 更新：$\mathbf{x}_k = \mathbf{x}_k^- + \mathbf{K}_k(\mathbf{z}_k - h(\mathbf{x}_k^-))$

**实践要点**
- 根据环境特征选择合适的定位方法
- 多传感器融合提高鲁棒性
- 合理配置参数平衡精度与计算量
- 实施故障检测与恢复机制

## 练习题

### 基础题

**1. AMCL 参数调优**
```
场景：仓库环境，面积 5000m²，机器人最大速度 2m/s
要求：配置 AMCL 参数实现 <10cm 定位精度
```
<details>
<summary>提示</summary>
考虑粒子数量、更新频率、运动噪声参数的平衡
</details>

<details>
<summary>参考答案</summary>

关键参数配置：
- min_particles: 500（保证精度）
- max_particles: 2000（限制计算）
- update_min_d: 0.1（频繁更新）
- update_min_a: 0.1
- alpha1-4: 0.1, 0.1, 0.15, 0.1（根据里程计精度）
- laser_sigma_hit: 0.1（激光精度高）
- resample_interval: 1
</details>

**2. 回环检测实现**
```
任务：设计回环检测算法的判断条件
输入：当前位姿、历史轨迹、激光扫描
输出：是否检测到回环
```
<details>
<summary>提示</summary>
综合考虑空间距离、时间间隔、扫描匹配得分
</details>

<details>
<summary>参考答案</summary>

回环判断条件：
1. 空间距离 < 2m
2. 时间间隔 > 30s
3. 扫描匹配得分 > 0.8
4. 匹配协方差特征值 < 阈值
5. 连续 N 帧确认
</details>

**3. 地图格式转换**
```
需求：将 PGM 格式地图转换为 nav_msgs/OccupancyGrid
输入：map.pgm (2048x2048), map.yaml
输出：OccupancyGrid 消息
```
<details>
<summary>提示</summary>
注意坐标系转换和占用值映射
</details>

<details>
<summary>参考答案</summary>

转换步骤：
1. 读取 PGM 图像和 YAML 配置
2. 根据 negate 参数反转颜色
3. 映射像素值到占用概率：
   - 白色(255) → 0 (free)
   - 黑色(0) → 100 (occupied)
   - 灰色 → -1 (unknown)
4. 设置 origin 和 resolution
5. 填充 OccupancyGrid 消息
</details>

**4. EKF 传感器配置**
```
传感器：IMU(100Hz)、轮式里程计(20Hz)、GPS(1Hz)
目标：室外导航，精度要求 0.5m
```
<details>
<summary>提示</summary>
考虑传感器更新频率和测量维度
</details>

<details>
<summary>参考答案</summary>

配置策略：
- IMU: 使用角度、角速度、加速度
- 里程计: 使用线速度、角速度
- GPS: 使用绝对位置 x, y
- EKF 频率: 30Hz
- two_d_mode: true（平面运动）
- 过程噪声: 根据地形调整
</details>

### 挑战题

**5. 多机器人协作定位**
```
场景：3 个机器人在同一环境中运行
要求：设计相对观测和信息融合方案
约束：通信带宽 < 1MB/s
```
<details>
<summary>提示</summary>
考虑分布式架构、数据压缩、时间同步
</details>

<details>
<summary>参考答案</summary>

设计方案：
1. 相对观测：
   - UWB 测距（10Hz）
   - 视觉标记识别
   - 激光特征匹配

2. 信息融合：
   - 分布式 EKF
   - 共享地图特征点
   - 协方差交集算法

3. 通信优化：
   - 增量地图更新
   - 特征点压缩
   - 事件触发通信
</details>

**6. 动态环境 SLAM**
```
问题：人员密集环境中的建图
挑战：50% 激光点击中动态物体
设计：鲁棒的 SLAM 算法
```
<details>
<summary>提示</summary>
动静态点分离、概率占用更新、长期地图维护
</details>

<details>
<summary>参考答案</summary>

解决方案：
1. 动态点过滤：
   - 基于占用一致性
   - 速度阈值判断
   - 机器学习分类

2. 地图更新策略：
   - 短期/长期地图分离
   - 贝叶斯占用更新
   - 衰减因子

3. 定位补偿：
   - 增加静态特征权重
   - 使用天花板特征
   - IMU 辅助
</details>

**7. 定位失败诊断**
```
症状：机器人位置突变，粒子云发散
任务：设计自动诊断和恢复系统
```
<details>
<summary>提示</summary>
监控多个指标，分级恢复策略
</details>

<details>
<summary>参考答案</summary>

诊断系统：
1. 监控指标：
   - 粒子云协方差
   - 激光匹配得分
   - 里程计-激光一致性
   - 有效粒子数

2. 故障分类：
   - 绑架问题
   - 传感器故障
   - 地图不匹配

3. 恢复策略：
   - Level 1: 增加粒子数
   - Level 2: 全局重定位
   - Level 3: 切换备用传感器
   - Level 4: 安全停止
</details>

**8. 性能优化**
```
当前：SLAM 占用 80% CPU，建图延迟 2s
目标：降至 40% CPU，实时建图
硬件：ARM Cortex-A72 四核
```
<details>
<summary>提示</summary>
算法优化、并行计算、参数调整
</details>

<details>
<summary>参考答案</summary>

优化方案：
1. 算法层：
   - 降采样激光点
   - 减少搜索范围
   - 自适应分辨率

2. 实现层：
   - SIMD 指令优化
   - 多线程扫描匹配
   - GPU 加速（如可用）

3. 系统层：
   - 降低地图更新频率
   - 延迟非关键计算
   - 内存池复用
</details>

## 常见陷阱与错误

### 1. 定位系统

**陷阱：初始位姿设置错误**
```bash
# 错误：未设置初始位姿就启动导航
ros2 run nav2_bringup navigation_launch.py

# 正确：先设置初始位姿
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped ...
# 或在 rviz2 中使用 2D Pose Estimate
```

**陷阱：坐标系配置混乱**
```yaml
# 错误：world_frame 设置不当
world_frame: map  # EKF 应该是 odom

# 正确配置
ekf_odom:
  world_frame: odom
ekf_map:
  world_frame: map
```

### 2. SLAM 建图

**陷阱：回环误匹配**
```python
# 问题：阈值过低导致错误回环
loop_match_minimum_response_coarse: 0.1  # 太低

# 解决：提高阈值，增加验证
loop_match_minimum_response_coarse: 0.35
loop_match_minimum_chain_size: 10
```

**陷阱：地图漂移累积**
```yaml
# 问题：纯里程计建图
slam_toolbox:
  use_scan_matching: false  # 错误

# 解决：启用扫描匹配
  use_scan_matching: true
  scan_matcher_variance: 5.0
```

### 3. 传感器融合

**陷阱：时间戳不同步**
```python
# 问题：直接使用原始时间戳
imu_msg.header.stamp = sensor_time

# 解决：时间同步
imu_msg.header.stamp = self.get_clock().now().to_msg()
# 或使用 message_filters 同步
```

**陷阱：协方差设置不当**
```yaml
# 问题：过度信任某传感器
odom0_config: [true, true, true, ...]  # 使用所有维度

# 解决：选择可靠维度
odom0_config: [false, false, false,  # 位置不可靠
               false, false, false,  # 姿态不可靠  
               true,  true,  false,  # 线速度可靠
               false, false, true]   # 角速度可靠
```

### 4. 性能问题

**陷阱：粒子数过多**
```yaml
# 问题：盲目增加粒子数
max_particles: 10000  # CPU 负载过高

# 解决：自适应粒子数
min_particles: 100
max_particles: 2000
kld_err: 0.05
kld_z: 0.99
```

## 最佳实践检查清单

### 系统设计审查

- [ ] **传感器选择**
  - [ ] 传感器组合满足精度要求
  - [ ] 考虑环境限制（室内/室外）
  - [ ] 冗余设计应对故障

- [ ] **算法选择**
  - [ ] AMCL vs SLAM 场景适配
  - [ ] EKF vs UKF 非线性程度
  - [ ] 2D vs 3D 需求匹配

- [ ] **地图管理**
  - [ ] 地图格式标准化
  - [ ] 更新策略明确
  - [ ] 存储和传输优化

### 参数调优

- [ ] **AMCL 调优**
  - [ ] 粒子数量与CPU平衡
  - [ ] 运动模型匹配机器人特性
  - [ ] 观测模型适应传感器特点

- [ ] **SLAM 调优**
  - [ ] 扫描匹配参数验证
  - [ ] 回环检测阈值测试
  - [ ] 优化频率设置合理

- [ ] **融合调优**
  - [ ] 传感器权重合理分配
  - [ ] 过程噪声反映实际情况
  - [ ] 异常值检测启用

### 测试验证

- [ ] **功能测试**
  - [ ] 静态定位精度 < 要求值
  - [ ] 动态跟踪稳定
  - [ ] 建图完整性验证

- [ ] **鲁棒性测试**
  - [ ] 传感器故障处理
  - [ ] 绑架问题恢复
  - [ ] 动态环境适应

- [ ] **性能测试**
  - [ ] CPU 使用率 < 限制
  - [ ] 内存占用稳定
  - [ ] 更新频率达标

### 部署检查

- [ ] **日志监控**
  - [ ] 关键指标记录
  - [ ] 异常事件告警
  - [ ] 性能统计分析

- [ ] **维护方案**
  - [ ] 地图更新流程
  - [ ] 参数调整机制
  - [ ] 故障恢复预案

- [ ] **文档完备**
  - [ ] 参数说明文档
  - [ ] 调试指南
  - [ ] 常见问题FAQ