这里的代码大部分来自 [gym_carla](https://github.com/cjy1992/gym-carla) ， 只是做了少量改动

环境是 Carla v0.9.10 + UE4

> Copyright (c) 2019: Jianyu Chen (jianyuchen@berkeley.edu)
>
> This work is licensed under the terms of the MIT license.
>
> For a copy, see <https://opensource.org/licenses/MIT>.

- ***TODO：add try...catch...***





# Class: CarlaGymEnvTest

## function '_\_init\_\_'

### parameters

| 参数（self.ParamsName）    | 说明                                               |
| -------------------------- | -------------------------------------------------- |
| display_size               | 客户端框size                                       |
| **max_past_step**          | **最大历史信息步数**                               |
| number_of_vehicles         | 机动车数量                                         |
| number_of_walkers          | 行人数量                                           |
| dt                         | fixed_deltatime，同步模式的step固定时间            |
| task_mode                  |                                                    |
| max_time_episode           | 最大episode数                                      |
| max_waypt                  | 最大waypoints                                      |
| obs_range                  | LIDAR观测范围                                      |
| lidar_bin                  |                                                    |
| d_behind                   |                                                    |
| obs_size                   | rgb相机的观测图片大小                              |
| out_lane_thres             | 出车道的                                           |
| desired_speed              | 期望车速                                           |
| max_ego_spawn_times        | 自车最大尝试重生成次数（超过则reset）              |
| display_route              | 显示路径                                           |
| pixor                      | 网格化（？                                         |
| pixor_size                 | 网格大小（？                                       |
| dests                      | 如果是环岛有四个destination                        |
| **discrete**               | **是否是离散动作空间（Bool）**                     |
| discrete_act               | 离散动作空间总值array                              |
| n_acc                      | 加速度离散选择数量                                 |
| n_steer                    | 转向离散选择数量                                   |
| **self.action_space**      | **动作空间，Discrete或Box**                        |
| **self.observation_space** | **观测空间**                                       |
| **self.world**             | **carla的world**                                   |
| vehicle_spawn_points       | 车生成点（api调取）                                |
| walker_spawn_points        | 人生成点（地图随机选取）                           |
| ego_bp                     | 自己车辆的蓝图                                     |
| **collision_hist**         | **碰撞检测历史（list）**                           |
| collision_hist_l           | 碰撞检测历史的长度（length）                       |
| collision_bp               | 碰撞检测传感器蓝图                                 |
| **lidar_data**             | **LIDAR数据**                                      |
| lidar_height               | LIDAR的z轴位置（高）                               |
| lidar_trans                | LIDAR对应变换                                      |
| lidar_bp                   | LIDAR蓝图                                          |
| **camera_img**             | **相机图片数据**                                   |
| camera_trans               | 相机对应变换                                       |
| camera_bp                  | 相机蓝图                                           |
| **self.settings**          | **模拟器的设置**                                   |
| reset_step                 | 记录步数                                           |
| total_step                 | 记录总步数                                         |
| pixel_grid                 | 网格点（[0,0],[1,0],[2,0],...,[0,1],[1,1],......） |

### new add parameters

| 参数                    | 说明                                    |
| ----------------------- | --------------------------------------- |
| blueprint_library_cache | world蓝图存储，用来减少每次调world的get |
|                         |                                         |
|                         |                                         |

- 主要是一些基本的设定和设置



## function 'reset'

- 有一个取消同步

```python
# Disable sync mode
self._set_synchronous_mode(False)
```

- 后边自己打开的，*我们编写的时候给改统一函数调用了*

```python
# Enable sync mode
self.settings.synchronous_mode = True
self.world.apply_settings(self.settings)
```



### Parameters

| 参数             | 说明                 |
| ---------------- | -------------------- |
| vehicle_polygons | 车辆的polygon list表 |
| walker_polygons  | 行人的polygon list表 |
|                  |                      |

- 最后 return 一次obs信息



## function ‘step’

- 目测step中的离散action用了一个编码，acc * self.n_steer + steer = action
- return (self.\_get\_obs(), self.\_get\_reward(), self.\_terminal(), copy.deepcopy(info))
- **也就是返回next_obs, reward, done, info**，每个去对应的函数里调整
- info比较特殊，是在这里设置的，源代码就这两个：

```python
info = {
	'waypoints': self.waypoints,
	'vehicle_front': self.vehicle_front
}
```



## 辅助用函数

### _try_spawn_random_walker_at(self, transform)

- 用来在给定的位置生成个行人，随机选择终点，速度1+random
- return 是否成功生成行人（True/False）



### _set_synchronous_mode(self, synchronous = True)

- 就是设置同步用的，这里不写参数默认设置是True
- **注意reset里给关了同步，然后生成东西结束之后又给开开了，但不是用这个函数开的是setting的**
- 没有return



### _clear_all_actors(self, actor_filters)

- 清空所有在actor_filters的满足actor对象
- *修改了只获取一次actors的list，而不是调用len(actor_filters)次get_actors*
- 没有return



### _create_vehicle_blueprint(self,  actor_filter, color=None, number_of_wheels=[4])

- 是生成车辆蓝图的，从blueprint里选符合actor_filter和number_of_wheels的车，给他们上color的颜色
- *修改了每次调用这个函数都调用一次get_blueprint_library()*
- return 生成的蓝图 bp



### _try_spawn_random_vehicle_at(self, transform, number_of_wheels=[4])

- 用来尝试生成随机（指类型）车辆，获取蓝图（调用_create_vehicle_blueprint）后在transform处spawn
- 如果生成成功会直接设置autopilot并开启
- return 是否成功生成车辆（True/False）



### _get_actor_polygons(self, filt)

- 获取actor的多变形bounding box，filt是指定选择器
- *修改了只获取一次actor_list*
- return 一个dict，actor.id和其全局坐标系下的bounding box polygon



### _try_spawn_ego_vehicle_at(self, transform)

- 尝试生成ego车，如果生成位置和其他车辆有碰撞交集（bounding box检测）则失败，否则成功生成
- 检测依靠俩个车的欧式距离是否大于8决定，overlap就退出
- return 是否成功（True/False）



### _init_renderer(self)

- 用pygame写的鸟瞰图视角渲染，主要看那个**BirdeyeRender**
- 无return



### 下一个状态 _get_obs(self)

- 获取observation，包括渲染waypoint，route，roadmap，actors，ego vehicle，LIDAR点云图，rgb相机图这些东西

- return很多东西：

```python
'camera':camera.astype(np.uint8),	# 相机render obs
'lidar':lidar.astype(np.uint8),		# lidar render obs
'birdeye':birdeye.astype(np.uint8),	# 鸟瞰图 render obs
'state': state,						# 自车状态 \
    								# lateral_dis, -delta_yaw, speed, self.vehicle_front

# if self.pixor:
'roadmap':roadmap.astype(np.uint8),							# 道路 render
'vh_clas':np.expand_dims(vh_clas, -1).astype(np.float32),	# Vehicle classification 
'vh_regr':vh_regr.astype(np.float32),						# Vehicle regression
'pixor_state': pixor_state,									# 自车的pixor状态 \
# ego_x, ego_y, np.cos(ego_yaw), np.sin(ego_yaw), speed
```

- **也就是说这属于是下一个state**



### 奖励函数 _get_reward(self)

- 获取reward，**就是Reward设计的地方了**
- return 奖励



### 终止设置 _terminal(self):

- 设置终止条件





# misc.py

#### get_speed(vehicle):

- 获取vehicle的速度，**单位km/h**， return 速度float值

#### get_pos(vehicle):

- 获取vehicle的位置，return x, y 

#### get_info(vehicle):

- 获取vehicle的信息
  - x,y 位置信息
  - yaw 转角信息（角度制）
  - l,w bounding box信息（length,width的**一半**）
- return 上述信息

#### get_local_pose(global_pose, ego_pose):

- 坐标系转换，从global到ego（这个应该是自身周围的其他vehicle转到自身坐标）
- return 转换后的local_pose

#### get_pixel_info(local_info, d_behind, obs_range, image_size):

- 把local_info转换为pixel_info

>Transform local vehicle info to pixel info, with ego placed at lower center of image.
>
>Here the ego local coordinate is left-handed, the pixel coordinate is also left-handed,
>
>with its origin at the left bottom.
>
>:param local_info: local vehicle info in ego coordinate
>
>:param d_behind: distance from ego to bottom of FOV
>
>:param obs_range: length of edge of FOV
>
>:param image_size: size of edge of image
>
>:return: tuple of pixel level info, including (x, y, yaw, l, w) all in pixels

#### get_poly_from_info(info):

- 转化info(x,y,yaw,l,w)到多边形info那个长方形的四个角的点坐标上
- return 4*2的矩阵，分别是四个角点坐标

#### get_pixels_inside_vehicle(pixel_info, pixel_grid):

- 获取在vehicle中的pixels

> Get pixels inside a vehicle, given its pixel level info (x, y, yaw, l, w)
>
> :param pixel_info: pixel level info of the vehicle
>
> :param pixel_grid: pixel_grid of the image, a tall numpy array pf x, y pixels
>
> :return: the pixels that are inside the vehicle  

#### get_lane_dis(waypoints, x, y):

- 计算坐标(x, y)距离waypoints的距离，waypoints是一个list表示车道lane
- return 距离和最近的waypoint方向

#### get_preview_lane_dis(waypoints, x, y, idx=2):

- 类似上一个函数，这次指定了waypoints的index值
- return 对这个waypoint的距离和方向

#### is_within_distance_ahead(target_location, current_location, orientation, max_distance):

- 检测target是否在current的orientation的max_distance以内
- return True/False

#### compute_magnitude_angle(target_location, current_location, orientation):

- 计算target和current相差的角度（current的orientation方向）和距离
- return (距离，角度)

#### distance_vehicle(waypoint, vehicle_transform):

- 简简单单算个距离
- return 欧式距离（waypoint到vehicle_transform）

#### set_carla_transform(pose)

- 给pose（x,y,yaw），转换成carla的transform
- return 对应的carla transform

#### display_to_rgb(display, obs_size):

- 把从pygame里捕获的image转换成rgb
- return rgb矩阵（uint8）

#### rgb_to_display_surface(rgb, display_size):

- rgb转换到pygame的surface
- return surface
