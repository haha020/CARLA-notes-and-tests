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
| f_idx                                 |                                                            |
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
| ego_los_sensor                        |                                                            |
| module_manager                        |                                                            |
| world_module                          |                                                            |
| traffic_module                        |                                                            |
| hud_module                            |                                                            |
| input_module                          |                                                            |
| control_module                        |                                                            |
| init_transform                        | 自车的初始transform                                        |
| acceleration_                         |                                                            |
| eps_rew                               |                                                            |
| actor_enumerated_dict                 |                                                            |
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



### start(self, route):

steps置零，然后update_global_route(route)在下方



### update_global_route(self, global_route):

> fit an spline to the updated global route in inertial frame
>
> 在惯性坐标系中，用样条曲线对更新的全局路径进行拟合

global_route信息取出然后放入**cubic_spline_planner.Spline3D**计算csp

>  self.csp = None  \# cubic spline for global rout

**cubic_spline_planner.Spline3D**看上去是一个专门计算三次样条曲线的库，和数学很关联就先不细看了







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

