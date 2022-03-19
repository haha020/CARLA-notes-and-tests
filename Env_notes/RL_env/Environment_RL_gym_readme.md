carla_env_v1.py， 来自**[RL-frenet-trajectory-planning-in-CARLA](https://github.com/MajidMoghadam2006/RL-frenet-trajectory-planning-in-CARLA)**

记录一些环境封装学习笔记和获取信息的方式，拆一下代码



# Class CarlaGymEnv(gym.Env):

## \_\_init\_\_(self):

### parameters

| 参数                                  | 说明                                                       |
| ------------------------------------- | ---------------------------------------------------------- |
| **\# simulation**                     | **模拟器用控制**                                           |
| \_\_version\_\_                       | 版本号                                                     |
| verbosity                             | 额外输出信息控制                                           |
| auto_render                           | 环境自动渲染（bool）                                       |
| n_step                                | step记录                                                   |
| global_route                          | track waypoints (center lane of the second lane from left) |
| **\# constraints**                    | **限制约束**                                               |
| targetSpeed                           | 目标速度                                                   |
| maxSpeed                              | 速度上限                                                   |
| maxAcc                                | 加速度上限                                                 |
| LANE_WIDTH                            | 车道宽度                                                   |
| N_SPAWN_CARS                          | 车辆生成数                                                 |
| **\# frenet**                         | **frenet 坐标用（frenet坐标是(s, d)）**                    |
| f_idx                                 | frenet的index                                              |
| init_s                                | 初始的s                                                    |
| max_s                                 | 最长s                                                      |
| track_length                          | track长度                                                  |
| look_back                             |                                                            |
| time_step                             |                                                            |
| loop_break                            |                                                            |
| effective_distance_from_vehicle_ahead |                                                            |
| lanechange                            | 是否变道（bool）                                           |
| is_first_path                         | 是否第一条路线（bool）                                     |
| **\# RL**                             | **强化学习相关**                                           |
| w_speed                               |                                                            |
| w_r_speed                             |                                                            |
| min_speed_gain                        |                                                            |
| min_speed_loss                        |                                                            |
| lane_change_reward                    | 换道奖励                                                   |
| lane_change_penalty                   | 换道惩罚                                                   |
| off_the_road_penalty                  | 离开道路惩罚                                               |
| collision_penalty                     | 碰撞惩罚                                                   |
| low_state                             |                                                            |
| high_state                            |                                                            |
| action_low                            |                                                            |
| action_high                           |                                                            |
| state                                 |                                                            |
| **\# instances**                      | **实体信息**                                               |
| ego                                   | 自己的车车                                                 |
| ego_los_sensor                        | LineOfSightSensor                                          |
| module_manager                        |                                                            |
| world_module                          |                                                            |
| traffic_module                        |                                                            |
| hud_module                            |                                                            |
| input_module                          |                                                            |
| control_module                        |                                                            |
| init_transform                        | 自车的初始transform                                        |
| acceleration_                         |                                                            |
| eps_rew                               |                                                            |
| actor_enumerated_dict                 | 总的dict，每个网格的信息都在这里，可以给lstm提取历史       |
| actor_enumeration                     |                                                            |
| side_window                           |                                                            |
| **motionPlanner**                     |                                                            |
| vehicleController                     |                                                            |
| dt                                    | fixed delta_time同步固定时长                               |

## begin_modules(self, args):

对各种东西的初始化赋值，run在make环境的时候会调用，可能是为了不重复赋值搞得优化而没有写在reset里

| 参数                  | 初始化值                                                     |
| --------------------- | ------------------------------------------------------------ |
| verbosity             | args.verbosity                                               |
| **module_manager**    | **ModuleManager()**                                          |
| **world_module**      | **ModuleWorld**(MODULE_WORLD, args, timeout=10.0, module_manager=self.module_manager,width=width, height=height) |
| **traffic_module**    | **TrafficManager**(MODULE_TRAFFIC, module_manager=self.module_manager) |
| **hud_module**        | **ModuleHUD**(MODULE_HUD, width, height, module_manager=self.module_manager) |
| **input_module**      | **ModuleInput**(MODULE_INPUT, module_manager=self.module_manager) |
| ***motionPlanner***   | ***MotionPlanner()***                                        |
| **vehicleController** | **VehiclePIDController**(self.ego, args_lateral={'K_P': 1.5, 'K_D': 0.0, 'K_I': 0.0}) |
| **IDM**               | **IntelligentDriverModel**(self.ego)                         |

- 上述加粗均是模块，分散在其他文件里，后续会挑选重要的节选记录。已有记录如下
  - **MotionPlanner**：用来规划路径的，和三次样条等东西有关
  - **ModuleWorld**：
- 接下来启动每个module，然后进行ego赋值，tick一次（所有module更新一次）

## reset(self):

基本都是.reset()调用自己的方法，没有重新赋值，都在begin_modules里写了



## step(self, action=None):

- 初始化init

  - n_step+=1
  - ego dict设置

- Motion Planner

  - 得到speed速度，acc加速度，psi转角，location位置给到ego_state

  - `ego_state = [self.ego.get_location().x, self.ego.get_location().y, speed, acc, psi, temp, self.max_s]`

    - temp是ego的速度和加速度矢量（3D vector）
    - self.max_s是最长的s设置

  - 然后这些给到MotionPlanner的**run_step_single_path**（这个见下方的MotionPlanner里的函数）

    `fpath, self.lanechange, off_the_road = self.motionPlanner.run_step_single_path(ego_state, self.f_idx, df_n=action, Tf=5, Vf_n=-1)`

    得到生成frenet路径，车道变化信息和出道信息

- Controller

  - 控制层的东西，直接给了自己的vehicleController信息生成命令，命令给到carla模拟器就好

- Draw Waypoints

  - play_mode下的控制绘画

- Update Carla

  - `self.module_manager.tick()  # Update carla world`
  - render
  - 计算碰撞和自身信息

- RL Observation

  - 这个是next_obs的生成
  - 除了print以外，就是这个**`self.fix_representation()`**

- RL Reward Function

  - 奖励函数设置

-  Episode Termination

  - 终止条件设置



## fix_representation(self):

> ```python
> """
> Given the traffic actors fill the desired tensor with appropriate values and time_steps
> """
> ```

这个是生成网格周边车辆并填充空白为-1的方法

- 首先是计算周围车辆：枚举，调用方法**`self.enumerate_actors()`**在后续函数部分
- 然后是对look_back的长度取历史，截取拼接形成lstm_obs，返回这个组合向量



## enumerate_actors(self):

> ```python
> """
> Given the traffic actors and ego_state this fucntion enumerate actors, calculates their relative positions with
> to ego and assign them to actor_enumerated_dict.
> Keys to be updated: ['LEADING', 'FOLLOWING', 'LEFT', 'LEFT_UP', 'LEFT_DOWN', 'LLEFT', 'LLEFT_UP',
> 'LLEFT_DOWN', 'RIGHT', 'RIGHT_UP', 'RIGHT_DOWN', 'RRIGHT', 'RRIGHT_UP', 'RRIGHT_DOWN']
> """
> ```

- 每个道路都做一个check检测，然后筛选车辆放入网格图
  - 如果没车就是-1，如果出车道就算-2
  - 车道的划分是按照d计算的，**所以这里的所有计算其实是按d的值离散到车道那样子，然后控制会去走定车道中线**
  - 返回填充的网格信息





# MotionPlanner

- 在env1里用的是**FrenetPlanner**

## FrenetPlanner

### \_\_init\_\_(self):

#### parameters

| 参数                | 说明                                                         |
| ------------------- | ------------------------------------------------------------ |
| dt                  | fixed_delta_time同步固定时长                                 |
| MAX_SPEED           | 最大速度 m/s                                                 |
| MAX_ACCEL           | 最大加速度 m/ss                                              |
| MAX_CURVATURE       | 最大曲率 1/m                                                 |
| LANE_WIDTH          | 车道宽                                                       |
| MAXT                | 最大预测时间(m)                                              |
| MINT                | 最小预测时间(m)                                              |
| D_T                 | 预测（时间）timestep的长度                                   |
| D_T_S               | target speed sampling length [m/s] 目标速度采样长            |
| N_S_SAMPLE          | 目标速度采样数量                                             |
| ROBOT_RADIUS        | robot半径                                                    |
| MAX_DIST_ERR        | max distance error to update frenet states based on ego states |
| **path**            | **当前frenet路径**                                           |
| **ob**              | **n个障碍[[x1, y1, z1], [x2, y2, z2], ... ,[xn, yn, zn]]**   |
| **csp**             | **cubic spline for global rout 全局路径的三次样条**          |
| **steps**           | **planner steps**                                            |
| targetSpeed         | 目标速度                                                     |
| speed_center        | 速度中心                                                     |
| speed_radius        | 速度半径                                                     |
| **\# cost weights** | **权重**                                                     |
| KJ                  | 0.1                                                          |
| KT                  | 0.1                                                          |
| KD                  | 1.0                                                          |
| KLAT                | 1.0                                                          |
| KLON                | 1.0                                                          |

关于后续计算可能出现的s,s_d,s_dd,s_ddd,d,d_d,d_dd,d_ddd：

![image-20220319105712669](/home/yihang/snap/typora/57/.config/Typora/typora-user-images/image-20220319105712669.png)

### start(self, route):

steps置零，然后update_global_route(route)在下方



### reset(self, s, d, df_n=0, Tf=4, Vf_n=0, optimal_path=True):

reset，然后找optimal_path或者**generate_single_frenet_path**



### generate_single_frenet_path(self, f_state, df=0, Tf=4, Vf=30 / 3.6):

> ```python
> """
> generate a single frenet path based on the current and terminal frenet state values
> input: ego's current frenet state and terminal frenet values (lateral displacement, time of arrival, and speed)
> output: single frenet path
> """
> ```





### calc_global_paths(self, fplist):

> ```python
> """
> transform paths from frenet frame to inertial frame
> input: path list
> output: path list
> """
> ```





### update_global_route(self, global_route):

> fit an spline to the updated global route in inertial frame
>
> 在惯性坐标系中，用样条曲线对更新的全局路径进行拟合

global_route信息取出然后放入**cubic_spline_planner.Spline3D**计算csp

>  self.csp = None  \# cubic spline for global rout

**cubic_spline_planner.Spline3D**看上去是一个专门计算三次样条曲线的库，和数学很关联就先不细看了



### run_step_single_path(self, ego_state, idx, df_n=0, Tf=4, Vf_n=0):

> ```python
> """
> input: ego states, current frenet path's waypoint index, actions
> output: frenet path
> actions: final values for frenet lateral displacement (d), time, and speed
> """
> ```

- 首先step+=1
- 然后转到**estimate_frenet_state**估计frenet的状态
- 接下来计算信息和路径，返回







### estimate_frenet_state(self, ego_state, idx):

> ```python
> """
> estimate the frenet state based on ego state and the current frenet state
> procedure: - initialize the estimation with the last frenet state
>            - check the error btw ego true position and the frenet's estimation of global position # btw = between
>            - if error larger than threshold, update the frenet state
> """
> ```











# modules.py

## ModuleWorld

### \_\_init\_\_(self, name, args, timeout, module_manager, width, height):

- 有个play_mode的显示，使用pygame进行的渲染



### _get_data_from_carla(self):

- 主要的连接和获取信息，client连接server，更换地图（town04），还有获取traffic manager的port，设置天气，获取map



### update_global_route_csp(self, global_route_csp):

- 传递一下来自motion planning的csp信息（就是个赋值）

```python
self.global_csp = global_route_csp
```









## TrafficManager

### \_\_init\_\_(self, name, module_manager):





### update_global_route_csp(self, global_route_csp):

- 传递一下来自motion planning的csp信息（就是个赋值）

```python
self.global_csp = global_route_csp
```









## ModuleManager

- 一个简单的总控，代码都贴下边了，比较短

### __init__(self):

- 只有一个list`self.modules = []`初始化一句话

### register_module(self, module):

- 注册module:`self.modules.append(module)`

### clear_modules(self):

- 清空`del self.modules[:]`

### tick(self):

- 总控每个module进行一个tick运行update

```python
# Update all the modules
for module in self.modules:
	module.tick()
```

### render(self, display):

- 控制能render的模块都render一下

```python
display.fill(COLOR_ALUMINIUM_4)
for module in self.modules:
	if hasattr(module, 'render'):
		module.render(display)
pygame.display.flip()
```

### get_module(self, name):

- 获取指定name的module

```python
for module in self.modules:
	if module.name == name:
		return module
```

### start_modules(self):

- 每一个module都start一下

```python
for module in self.modules:
	module.start()
```

### exit_game(self):

- 退出！

```python
self.clear_modules()
pygame.quit()
sys.exit()
```







## TrafficLightSurfaces















# Useful methods

```python
# 生成waypoints记录
# generate and save global route if it does not exist in the road_maps folder
if self.global_route is None:
	self.global_route = np.empty((0, 3))
    distance = 1
    for i in range(1520):
        wp = self.world_module.town_map.get_waypoint(carla.Location(x=406, y=-100, z=0.1),
				project_to_road=True).next(distance=distance)[0]
        distance += 2
        self.global_route = np.append(self.global_route,[[wp.transform.location.x, 																				wp.transform.location.y,wp.transform.location.z]], axis=0)
     	# To visualize point clouds
     	self.world_module.points_to_draw['wp {}'.format(wp.id)] = [wp.transform.location, 																	'COLOR_CHAMELEON_0']
     np.save('road_maps/global_route_town04', self.global_route)
```

