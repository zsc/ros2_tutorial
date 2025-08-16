# 第 7 章：Launch 系统与配置管理

## 开篇段落

ROS2 的 Launch 系统是整个机器人系统的指挥中心，负责协调启动、配置和管理数十甚至上百个节点。与 ROS1 的 XML-only launch 文件相比，ROS2 提供了基于 Python 的动态启动系统，支持条件逻辑、事件处理和复杂的参数管理。本章将深入探讨 Launch 系统的架构设计、高级特性以及在大规模机器人系统中的最佳实践。

## 学习目标

- 掌握 ROS2 Launch 系统的核心架构和设计理念
- 熟练使用 Python、XML、YAML 三种格式编写 Launch 文件
- 理解条件启动、事件处理和参数替换机制
- 学会设计可复用、可扩展的 Launch 模块
- 掌握分布式多机器人系统的部署策略

## 7.1 Launch 文件架构

### 7.1.1 架构演进：从 ROS1 到 ROS2

ROS1 的 launch 系统基于静态 XML 文件，虽然简单直观，但在处理复杂逻辑时显得力不从心。主要局限包括：

1. **静态配置**：缺乏条件逻辑和循环结构
2. **参数管理简陋**：参数服务器集中式架构，难以实现模块化
3. **扩展性差**：自定义功能需要修改 roslaunch 源码
4. **调试困难**：XML 解析错误信息不够友好

ROS2 引入了全新的 Launch 架构，核心改进包括：

```
ROS1 Launch                    ROS2 Launch
     │                              │
     ├── XML Only                   ├── Python (Primary)
     ├── Static Config              ├── XML/YAML (Frontend)
     ├── Parameter Server           ├── Distributed Parameters
     └── Limited Logic              └── Event-Based System
```

关键设计理念：
- **程序化配置**：Python API 提供完整的编程能力
- **事件驱动**：基于事件的异步架构
- **模块化设计**：支持 Launch 文件组合和复用
- **声明式前端**：XML/YAML 作为简化接口

### 7.1.2 Python Launch 系统设计

Python Launch 系统采用了描述-执行分离的架构：

```
LaunchDescription
     │
     ├── Actions (节点、包含文件、设置参数等)
     ├── Substitutions (参数替换、环境变量等)
     ├── Conditions (条件表达式)
     └── Event Handlers (事件处理器)
           │
           v
    LaunchService (执行引擎)
           │
           ├── Event Loop
           ├── Action Executor
           └── Context Manager
```

核心组件解析：

**LaunchDescription**：声明式的启动描述，包含所有要执行的动作序列。

**Actions**：可执行的原子操作，主要类型包括：
- `Node`：启动 ROS2 节点
- `ExecuteProcess`：执行系统进程
- `IncludeLaunchDescription`：包含其他 launch 文件
- `SetParameter`：设置参数
- `GroupAction`：组合多个动作

**Substitutions**：延迟求值的表达式系统：
- `LaunchConfiguration`：启动配置变量
- `PathJoinSubstitution`：路径拼接
- `FindPackageShare`：查找包的 share 目录
- `Command`：执行 shell 命令并获取输出

**LaunchService**：执行引擎，负责：
1. 解析 LaunchDescription
2. 管理事件循环
3. 执行 Actions
4. 处理生命周期事件

### 7.1.3 XML 和 YAML 格式支持

虽然 Python 是主要接口，ROS2 也提供了 XML 和 YAML 前端，便于简单场景的快速配置。

**格式对比**：

| 特性 | Python | XML | YAML |
|------|--------|-----|------|
| 条件逻辑 | 完整支持 | 基础支持 | 基础支持 |
| 循环结构 | 支持 | 不支持 | 不支持 |
| 自定义函数 | 支持 | 不支持 | 不支持 |
| 可读性 | 中等 | 高 | 最高 |
| 学习曲线 | 陡峭 | 平缓 | 平缓 |
| 适用场景 | 复杂系统 | 中等复杂度 | 简单配置 |

**转换机制**：

```
XML/YAML → Frontend Parser → Python AST → LaunchDescription
                                  │
                                  v
                           LaunchService
```

XML/YAML 实际上是通过解析器转换为等价的 Python 代码，因此功能是 Python API 的子集。

### 7.1.4 Launch 描述架构

Launch 描述采用了分层架构设计：

```
Application Layer (应用层)
    │
    ├── Launch Files (用户编写的启动文件)
    └── Launch Configurations (配置参数)
         │
Description Layer (描述层)
    │
    ├── Actions (动作定义)
    ├── Substitutions (替换表达式)
    └── Conditions (条件逻辑)
         │
Execution Layer (执行层)
    │
    ├── LaunchService (执行服务)
    ├── Event System (事件系统)
    └── Context (执行上下文)
         │
System Layer (系统层)
    │
    ├── Process Management (进程管理)
    ├── Signal Handling (信号处理)
    └── I/O Redirection (输入输出重定向)
```

**执行流程**：

1. **解析阶段**：
   - 加载 launch 文件
   - 构建 LaunchDescription
   - 注册事件处理器

2. **准备阶段**：
   - 初始化 LaunchContext
   - 解析命令行参数
   - 设置环境变量

3. **执行阶段**：
   - 遍历 Actions 列表
   - 执行每个 Action
   - 处理产生的事件

4. **清理阶段**：
   - 等待进程结束
   - 清理资源
   - 返回退出码

**上下文管理**：

LaunchContext 维护了执行时的全局状态：

```python
class LaunchContext:
    launch_configurations: Dict[str, str]  # 配置参数
    environment_variables: Dict[str, str]  # 环境变量
    global_parameters: List[Parameter]     # 全局参数
    asyncio_loop: AbstractEventLoop       # 事件循环
```

这种设计允许在运行时动态修改配置，实现灵活的系统行为。

## 7.2 复杂系统组合与参数传递

### 7.2.1 参数文件与覆盖机制

ROS2 的参数系统支持多层次的配置覆盖，优先级从低到高为：

```
默认值 → YAML文件 → Launch参数 → 命令行覆盖
```

**参数文件格式**：

```yaml
# config/robot_params.yaml
/robot_controller:
  ros__parameters:
    wheel_radius: 0.05
    wheel_base: 0.3
    max_velocity: 2.0
    control_frequency: 100.0
    
/sensor_processor:
  ros__parameters:
    lidar:
      range_min: 0.15
      range_max: 12.0
      angle_increment: 0.00436  # 0.25 degree
    camera:
      fps: 30
      resolution: [640, 480]
```

**参数覆盖机制**：

```python
# 优先级示例
Node(
    package='robot_control',
    executable='controller',
    parameters=[
        # 1. 从文件加载基础配置
        {'config_file': 'base_config.yaml'},
        
        # 2. 加载特定环境配置（覆盖基础配置）
        {'config_file': 'env_specific.yaml'},
        
        # 3. Launch 文件中的直接参数（最高优先级）
        {'max_velocity': 1.5},
        
        # 4. 允许命令行覆盖
        {'use_sim_time': LaunchConfiguration('use_sim_time')}
    ]
)
```

**参数命名空间隔离**：

```python
# 多机器人参数隔离
for i in range(num_robots):
    robot_ns = f'robot_{i}'
    nodes.append(
        Node(
            namespace=robot_ns,
            package='nav2_controller',
            executable='controller_server',
            parameters=[
                {'robot_id': i},
                {'base_frame': f'{robot_ns}/base_link'},
                # 每个机器人独立的参数文件
                f'config/{robot_ns}_params.yaml'
            ]
        )
    )
```

**动态参数更新**：

Launch 系统支持在运行时更新参数：

```python
# 参数更新事件处理
RegisterEventHandler(
    OnProcessStart(
        target_action=controller_node,
        on_start=[
            # 节点启动后动态设置参数
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', 
                     '/controller', 'gain', '1.5']
            )
        ]
    )
)
```

### 7.2.2 命名空间与重映射

命名空间和重映射是实现模块化和复用的关键机制。

**命名空间层次结构**：

```
/                          # 全局命名空间
├── /robot1                # 机器人1命名空间
│   ├── /sensors
│   │   ├── /lidar
│   │   └── /camera
│   └── /actuators
│       └── /wheels
└── /robot2                # 机器人2命名空间
    └── ...
```

**话题重映射**：

```python
Node(
    package='sensor_fusion',
    executable='fusion_node',
    remappings=[
        # 输入重映射
        ('scan', '/robot1/lidar/scan'),
        ('image', '/robot1/camera/image_raw'),
        
        # 输出重映射
        ('fused_data', '/perception/obstacles'),
        
        # 服务重映射
        ('calibrate', '/calibration/trigger')
    ]
)
```

**相对与绝对命名**：

```python
# 相对命名（相对于节点的命名空间）
remappings=[
    ('cmd_vel', 'mobile_base/cmd_vel'),  # 相对路径
]

# 绝对命名（从根命名空间开始）
remappings=[
    ('cmd_vel', '/robot1/mobile_base/cmd_vel'),  # 绝对路径
]

# 私有命名（节点私有命名空间）
remappings=[
    ('~diagnostics', 'system/health'),  # ~ 表示私有
]
```

### 7.2.3 组合 Launch 文件

模块化设计是管理大型系统的关键。ROS2 支持多种 Launch 文件组合模式。

**基础包含模式**：

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 包含其他包的 launch 文件
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        # 传递参数到被包含的 launch 文件
        launch_arguments={
            'use_sim_time': 'true',
            'map': '/path/to/map.yaml',
            'params_file': FindPackageShare('my_robot').find('config/nav2_params.yaml')
        }.items()
    )
    
    return LaunchDescription([nav2_launch])
```

**条件包含**：

```python
# 根据条件包含不同的 launch 文件
simulation_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([...]),
    condition=IfCondition(LaunchConfiguration('use_sim'))
)

hardware_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([...]),
    condition=UnlessCondition(LaunchConfiguration('use_sim'))
)
```

**组合模式最佳实践**：

```python
# 分层组合架构
def generate_launch_description():
    ld = LaunchDescription()
    
    # 1. 硬件抽象层
    ld.add_action(
        IncludeLaunchDescription(
            'hardware_interface_launch.py',
            launch_arguments={'robot_type': 'differential_drive'}
        )
    )
    
    # 2. 传感器层
    ld.add_action(
        GroupAction([
            IncludeLaunchDescription('lidar_launch.py'),
            IncludeLaunchDescription('camera_launch.py'),
            IncludeLaunchDescription('imu_launch.py'),
        ], 
        scoped=True,  # 作用域隔离
        forwarding=True  # 转发启动参数
        )
    )
    
    # 3. 感知层
    ld.add_action(
        IncludeLaunchDescription(
            'perception_launch.py',
            condition=LaunchConfigurationEquals('enable_perception', 'true')
        )
    )
    
    # 4. 导航层
    ld.add_action(
        IncludeLaunchDescription('navigation_launch.py')
    )
    
    return ld
```

### 7.2.4 参数替换与环境变量

ROS2 的替换机制允许在运行时动态解析参数值。

**常用替换类型**：

```python
from launch.substitutions import (
    Command, LaunchConfiguration, 
    EnvironmentVariable, FindExecutable,
    PathJoinSubstitution, TextSubstitution
)

# 1. 命令替换 - 执行命令获取输出
robot_description = Command([
    FindExecutable(name='xacro'), ' ',
    PathJoinSubstitution([
        FindPackageShare('my_robot'),
        'urdf',
        'robot.urdf.xacro'
    ])
])

# 2. 环境变量替换
ros_domain_id = EnvironmentVariable('ROS_DOMAIN_ID', default_value='0')

# 3. 启动配置替换
use_sim_time = LaunchConfiguration('use_sim_time', default='false')

# 4. 条件替换
robot_name = PythonExpression([
    '"simulation_robot" if "',
    LaunchConfiguration('use_sim'),
    '" == "true" else "real_robot"'
])
```

**复杂替换表达式**：

```python
# 组合多个替换构建复杂表达式
frame_prefix = PythonExpression([
    '"" if "',
    LaunchConfiguration('namespace'),
    '" == "" else "',
    LaunchConfiguration('namespace'),
    '/"'
])

# 在节点参数中使用
Node(
    parameters=[{
        'frame_prefix': frame_prefix,
        'base_frame': [frame_prefix, 'base_link'],
        'odom_frame': [frame_prefix, 'odom']
    }]
)
```

**环境变量管理**：

```python
# 设置环境变量
SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', 
                      '[{severity}] [{time}] [{name}]: {message}')

# 条件设置环境变量
PushEnvironmentVariable(
    'LD_LIBRARY_PATH',
    '/opt/custom/lib',
    condition=IfCondition(LaunchConfiguration('use_custom_libs'))
)

# 从文件加载环境变量
SetEnvironmentVariable(
    'GAZEBO_MODEL_PATH',
    Command(['cat', FindPackageShare('my_robot').find('config/gazebo_paths.txt')])
)
```

## 7.3 条件启动与事件处理

### 7.3.1 条件表达式系统

ROS2 Launch 提供了强大的条件系统，支持复杂的启动逻辑。

**基础条件类型**：

```python
from launch.conditions import (
    IfCondition, UnlessCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
    EnvironmentEquals
)

# 基础布尔条件
Node(
    package='rviz2',
    executable='rviz2',
    condition=IfCondition(LaunchConfiguration('gui'))
)

# 反向条件
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    condition=UnlessCondition(LaunchConfiguration('use_joint_state_publisher_gui'))
)

# 值比较条件
Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    condition=LaunchConfigurationEquals('simulator', 'gazebo')
)
```

**复合条件表达式**：

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression, AndSubstitution, OrSubstitution

# 使用 Python 表达式构建复杂条件
complex_condition = IfCondition(
    PythonExpression([
        '"', LaunchConfiguration('robot_type'), '" == "mobile" and "',
        LaunchConfiguration('environment'), '" == "outdoor"'
    ])
)

# 逻辑组合
condition = IfCondition(
    AndSubstitution(
        LaunchConfiguration('use_slam'),
        NotSubstitution(LaunchConfiguration('use_localization'))
    )
)

# 多条件分支
def choose_planner():
    return [
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[{'planner_plugin': 'NavfnPlanner'}],
            condition=LaunchConfigurationEquals('planner', 'navfn')
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[{'planner_plugin': 'SmacPlanner2D'}],
            condition=LaunchConfigurationEquals('planner', 'smac')
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[{'planner_plugin': 'ThetaStarPlanner'}],
            condition=LaunchConfigurationEquals('planner', 'theta_star')
        )
    ]
```

**条件包含组**：

```python
# 条件组 - 所有动作共享相同条件
from launch.actions import GroupAction

simulation_group = GroupAction(
    condition=IfCondition(LaunchConfiguration('use_sim')),
    actions=[
        # Gazebo 仿真器
        IncludeLaunchDescription(...),
        # 生成机器人模型
        Node(package='gazebo_ros', executable='spawn_entity.py'),
        # 仿真时钟
        Node(package='gazebo_ros', executable='sim_time_publisher')
    ]
)
```

### 7.3.2 事件处理器架构

事件系统是 Launch 动态行为的核心。

**事件类型层次**：

```
Event (基类)
├── ProcessEvent
│   ├── ProcessStarted
│   ├── ProcessExited
│   └── ProcessStdout/Stderr
├── ExecutionEvent  
│   ├── ExecutionComplete
│   └── Shutdown
└── LifecycleEvent
    ├── StateTransition
    └── ChangeState
```

**常用事件处理器**：

```python
from launch.event_handlers import (
    OnProcessStart, OnProcessExit,
    OnProcessIO, OnExecutionComplete,
    OnShutdown
)

# 进程启动事件
RegisterEventHandler(
    OnProcessStart(
        target_action=robot_state_publisher,
        on_start=[
            LogInfo(msg='Robot state publisher started'),
            # 启动依赖节点
            Node(package='tf2_ros', executable='static_transform_publisher')
        ]
    )
)

# 进程退出事件
RegisterEventHandler(
    OnProcessExit(
        target_action=navigation_node,
        on_exit=[
            LogInfo(msg='Navigation node exited'),
            # 根据退出码采取不同行动
            EmitEvent(
                event=Shutdown(reason='Navigation failed')
            ) if LaunchConfiguration('critical_nav') else LogInfo(msg='Continuing...')
        ]
    )
)
```

**自定义事件处理**：

```python
class CustomEventHandler(EventHandler):
    def __init__(self, *, target_action=None, **kwargs):
        super().__init__(**kwargs)
        self._target_action = target_action
        
    def handle(self, event, context):
        # 自定义事件处理逻辑
        if isinstance(event, ProcessExited):
            if event.returncode != 0:
                # 错误处理
                return [
                    LogError(msg=f'Process failed with code {event.returncode}'),
                    # 尝试重启
                    TimerAction(
                        period=5.0,
                        actions=[self._target_action]
                    )
                ]
        return None
```

**事件链式处理**：

```python
# 创建事件处理链
def create_restart_handler(node, max_restarts=3):
    restart_count = {'count': 0}
    
    def on_exit(event, context):
        restart_count['count'] += 1
        if restart_count['count'] <= max_restarts:
            return [
                LogInfo(msg=f'Restarting node (attempt {restart_count["count"]})'),
                TimerAction(period=2.0, actions=[node])
            ]
        else:
            return [
                LogError(msg='Max restart attempts reached'),
                EmitEvent(event=Shutdown())
            ]
    
    return OnProcessExit(
        target_action=node,
        on_exit=on_exit
    )
```

### 7.3.3 生命周期节点管理

生命周期节点提供了标准化的状态管理接口。

**生命周期状态机**：

```
Unconfigured → Configuring → Inactive
     ↑              ↓            ↓
     ←─────── CleaningUp    Activating
                    ↑            ↓
                    ←────── Active
                    ↑            ↓
                    ←─── Deactivating
                    ↑            ↓
                    ←──── ShuttingDown
                             ↓
                          Finalized
```

**生命周期节点启动**：

```python
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

lifecycle_node = LifecycleNode(
    package='nav2_controller',
    executable='controller_server',
    name='controller_server',
    namespace='',
    parameters=[controller_params_file],
    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
)

# 自动配置和激活
configure_event = EmitEvent(
    event=ChangeState(
        lifecycle_node_matcher=matches_action(lifecycle_node),
        transition_id=Transition.TRANSITION_CONFIGURE
    )
)

activate_event = EmitEvent(
    event=ChangeState(
        lifecycle_node_matcher=matches_action(lifecycle_node),
        transition_id=Transition.TRANSITION_ACTIVATE
    )
)
```

**生命周期管理器**：

```python
# 批量管理生命周期节点
Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    parameters=[{
        'autostart': True,
        'node_names': [
            'controller_server',
            'planner_server',
            'recoveries_server',
            'bt_navigator'
        ],
        'bond_timeout': 4.0,
        'attempt_respawn_reconnection': True
    }]
)
```

### 7.3.4 动态系统重配置

Launch 系统支持运行时的动态配置更新。

**动态加载节点**：

```python
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    # 运行时决定启动哪些节点
    robot_count = int(LaunchConfiguration('robot_count').perform(context))
    
    nodes = []
    for i in range(robot_count):
        nodes.append(
            Node(
                package='robot_controller',
                executable='controller',
                namespace=f'robot_{i}',
                parameters=[{
                    'robot_id': i,
                    'total_robots': robot_count
                }]
            )
        )
    
    return nodes

# 在 LaunchDescription 中使用
LaunchDescription([
    DeclareLaunchArgument('robot_count', default_value='3'),
    OpaqueFunction(function=launch_setup)
])
```

**运行时参数更新**：

```python
# 参数更新服务
class ParameterUpdateHandler:
    def __init__(self):
        self.param_client = None
        
    def update_parameters(self, node_name, params):
        # 创建参数客户端
        self.param_client = Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name=f'{node_name}_param_client'
        )
        
        # 构建参数更新请求
        update_actions = []
        for key, value in params.items():
            update_actions.append(
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', node_name, key, str(value)]
                )
            )
        
        return update_actions

# 条件参数更新
RegisterEventHandler(
    OnProcessStart(
        target_action=sensor_node,
        on_start=[
            # 根据环境动态调整传感器参数
            OpaqueFunction(
                function=lambda context: [
                    ExecuteProcess(
                        cmd=['ros2', 'param', 'set', '/sensor',
                             'range_max', '5.0' if context.launch_configurations.get('indoor') == 'true' else '50.0']
                    )
                ]
            )
        ]
    )
)
```

**热重载配置**：

```python
# 监听配置文件变化并重载
import inotify.adapters

class ConfigWatcher:
    def __init__(self, config_file, node_action):
        self.config_file = config_file
        self.node_action = node_action
        
    def create_watcher(self):
        return RegisterEventHandler(
            OnProcessStart(
                target_action=self.node_action,
                on_start=[
                    ExecuteProcess(
                        cmd=['python3', '-c', f'''
import inotify.adapters
import subprocess

i = inotify.adapters.Inotify()
i.add_watch("{self.config_file}")

for event in i.event_gen():
    if event and "IN_MODIFY" in event[1]:
        subprocess.run(["ros2", "param", "load", 
                       "{self.node_action.name}", "{self.config_file}"])
                       '''],
                        shell=False
                    )
                ]
            )
        )
```

## 7.4 分布式系统部署

### 7.4.1 多机部署架构

### 7.4.2 网络配置与发现

### 7.4.3 远程启动与监控

### 7.4.4 容器化部署策略

## 7.5 产业案例研究：Boston Dynamics Spot 多节点系统启动

## 7.6 高级话题

### 7.6.1 动态节点加载与热重启

### 7.6.2 Launch 系统性能优化

### 7.6.3 前沿研究与论文导读

## 7.7 本章小结

## 7.8 练习题

## 7.9 常见陷阱与错误

## 7.10 最佳实践检查清单