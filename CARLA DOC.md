# CARLA DOC

- 记录与引用来自 `carla.readthedocs.io`

## A. World and client

### 连接 connect

- 连接server
  - 默认是2000端口，timeout设置最长响应时长

```python
client = carla.Client('localhost', 2000)
client.set_timeout(10.0) # seconds
```

- world 获取

```python
world = client.get_world()
```

- CARLA 地图/更换地图

>  The client can also get a list of available maps to change the current  one. This will destroy the current world and create a new one.

```python
print(client.get_available_maps())
...
world = client.load_world('Town01')
# client.reload_world() creates a new instance of the world with the same map. 
```



### 客户端命令 client commands 

用commands可以简化操作，对batch使用。

> However, using the methods Client.apply_batch() or Client.apply_batch_sync(), a list of commands can be applied in one single simulation step.
>
> The following example uses a batch to destroy a list of vehicles all at once.  

```python
client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
```



### 世界 world

- 模拟器里主要的规则设置等，一些常规设置：

  >- Actors in the simulation and the spectator.   Actor
  >- Blueprint library.    蓝图库
  >- Map.   地图
  >- Simulation settings.    模拟器选项
  >- Snapshots.    快照
  >- Weather and light manager.    天气和光线设置

#### Actor

> - Spawn actors (but not destroy them). 
> - Get every actor on scene, or find one in particular.  
> - Access the blueprint library.  
> - Access the spectator actor, the simulation's point of view.  
> - Retrieve a random location that is fitting to spawn an actor.  





#### Weather

- 一个天气控制例

```python
weather = carla.WeatherParameters(
    cloudiness=80.0,
    precipitation=30.0,
    sun_altitude_angle=70.0) # 先做一个参数设置

world.set_weather(weather) # 应用参数

print(world.get_weather()) # 一个打印当前天气信息的api
```

- ***更多天气参数控制项在[这个页面](https://carla.readthedocs.io/en/latest/python_api/#carla.WeatherParameters)***

- 官方提供的`environment.py`和`dynamic_weather.py`也可以用来调试（因为client可以多个加入）

> Changes in the weather do not affect physics. They are only visuals that can be captured by the camera sensors. 
>
> 改变天气并不会影响物理，他们只是对相机传感器的捕获输入起到视觉上的变化





#### Lights (待补充)

#### Debugging (待补充)

#### World snapshots

- 记录了一帧下所有Actor的状态

```python
# Retrieve a snapshot of the world at current frame.
world_snapshot = world.get_snapshot()

timestamp = world_snapshot.timestamp # Get the time reference 

for actor_snapshot in world_snapshot: # Get the actor and the snapshot information
    actual_actor = world.get_actor(actor_snapshot.id)
    actor_snapshot.get_transform()
    actor_snapshot.get_velocity()
    actor_snapshot.get_angular_velocity()
    actor_snapshot.get_acceleration()  

actor_snapshot = world_snapshot.find(actual_actor.id) # Get an actor's snapshot
```



#### World settings

- 详细参数 https://carla.readthedocs.io/en/latest/python_api/#carla.WorldSettings

- 比较重要的是**synchronous_mode 同步模式**，**fixed_delta_seconds 固定的更新delta time**



## B. Actors and blueprints

### 蓝图 Blueprints

- Blueprints library : https://carla.readthedocs.io/en/latest/bp_library/

- 使用

  ```python
  blueprint_library = world.get_blueprint_library()
  
  # Find a specific blueprint.
  collision_sensor_bp = blueprint_library.find('sensor.other.collision')
  # Choose a vehicle blueprint at random.
  vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
  ```

  参数get与set（有些参数不能set，只能get）

  ```python
  is_bike = [vehicle.get_attribute('number_of_wheels') == 2]
  if(is_bike)
      vehicle.set_attribute('color', '255,0,0')
  ```

  推荐参数...

  ```python
  for attr in blueprint:
      if attr.is_modifiable:
          blueprint.set_attribute(attr.id, random.choice(attr.recommended_values))
  ```



### 生命周期 Actor life cycle

#### Spawning

- 两种生成方式：

  - [`spawn_actor()`](https://carla.readthedocs.io/en/latest/python_api/#carla.World.spawn_actor) raises an exception if the spawning fails.

  - [`try_spawn_actor()`](https://carla.readthedocs.io/en/latest/python_api/#carla.World.try_spawn_actor) returns `None` if the spawning fails.

```python
transform = Transform(Location(x=230, y=195, z=40), Rotation(yaw=180))
actor = world.spawn_actor(blueprint, transform)
```

- 如果生成位置会碰撞（collision）则生成失败，所以建议用这个

> `map.get_spawn_points()` **for vehicles**. Returns a list of recommended spawning points. 
>
> `world.get_random_location()` **for walkers**. Returns a random point on a sidewalk. This same method is used to set a goal location for walkers.  

```python
spawn_points = world.get_map().get_spawn_points()

spawn_point = carla.Transform()
spawn_point.location = world.get_random_location_from_navigation()
```

- Attach 到 parent 物件（比如相机绑上去）

```python
camera = world.spawn_actor(camera_bp, relative_transform, attach_to=my_vehicle, carla.AttachmentType.Rigid)
```

- 生成后的find寻找

```python
actor_list = world.get_actors()
# Find an actor by id.
actor = actor_list.find(id)
# Print the location of all the speed limit signs in the world.
for speed_sign in actor_list.filter('traffic.speed_limit.*'):
    print(speed_sign.get_location())
```



#### Handling

对其get和set

```python
print(actor.get_acceleration())
print(actor.get_velocity())

location = actor.get_location()
location.z += 10.0
actor.set_location(location)
```

冻结physics

```python
actor.set_simulate_physics(False)
```



#### Destruction

```python
destroyed_sucessfully = actor.destroy() # Returns True if successful
```

> *** Destroying an actor blocks the simulator until the process finishes.  ***
>
> 销毁Actor会在销毁完成前一直阻塞模拟器的运行



### Actor类别 Types of actors

#### Sensor

- 简单使用例

```python
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, relative_transform, attach_to=my_vehicle)
camera.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame))
```



#### Spectator

- 改变观测视角（这里是找指定vehicle）

```python
spectator = world.get_spectator()
transform = vehicle.get_transform()
spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
carla.Rotation(pitch=-90)))
```



#### Traffic signs and traffic lights

- 它们不能在blueprints library找到并生成

- 通过[carla.BoundingBox](https://carla.readthedocs.io/en/latest/python_api/#carla.BoundingBox)来影响其中的vehicles

```python
#Get the traffic light affecting a vehicle
if vehicle_actor.is_at_traffic_light():
    traffic_light = vehicle_actor.get_traffic_light()
```

- 控制改变

```python
#Change a red traffic light to green
if traffic_light.get_state() == carla.TrafficLightState.Red:
    traffic_light.set_state(carla.TrafficLightState.Green)
    traffic_light.set_set_green_time(4.0)
```

> Vehicles will only be aware of a traffic light if the light is red.
>
> 车辆只会关注红灯指示...



#### Vehicles

- **[carla.VehicleControl](https://carla.readthedocs.io/en/latest/python_api/#carla.VehicleControl)** 

  - 控制底层，像是油门、转角、刹车......

    ```python
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1.0))
    ```

- **[carla.VehiclePhysicsControl](https://carla.readthedocs.io/en/latest/python_api/#carla.VehiclePhysicsControl)**

  - 更精细的控制...还有齿轮与轮子控制...

  - [carla.GearPhysicsControl](https://carla.readthedocs.io/en/latest/python_api/#carla.GearPhysicsControl) 和 [carla.WheelPhysicsControl](https://carla.readthedocs.io/en/latest/python_api/#carla.WheelPhysicsControl)

    ```python
    vehicle.apply_physics_control(carla.VehiclePhysicsControl(max_rpm = 5000.0, center_of_mass = carla.Vector3D(0.0, 0.0, 0.0), torque_curve=[[0,400],[5000,400]]))
    ```

  - ***[carla.BoundingBox](https://carla.readthedocs.io/en/latest/python_api/#carla.BoundingBox)***

    - 每一个vehicle都有一个boundingbox封框，**碰撞检测**和**物理系统**依靠boundingbox

    ```python
        box = vehicle.bounding_box
        print(box.location)         # Location relative to the vehicle.
        print(box.extent)           # XYZ half-box extents in meters.
    ```

- **Autopilot mode**

  将vehicle订阅到一个Traffic manager上

  ```python
  vehicle.set_autopilot(True)
  ```

  Traffic Manager是控制策略，没有基于Machine learning

- **Vehicle lights**

   [**carla.VehicleLightState**](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.VehicleLightState)

  > The lights of a vehicle can be retrieved and updated anytime using the methods [carla.Vehicle.get_light_state](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.Vehicle.get_light_state) and [carla.Vehicle.set_light_state](https://carla.readthedocs.io/en/0.9.10/core_actors/#python_api.md#carla.Vehicle.set_light_state). These use binary operations to customize the light setting.  

  ```python
  # Turn on position lights
  current_lights = carla.VehicleLightState.NONE
  current_lights |= carla.VehicleLightState.Position
  vehicle.set_light_state(current_lights)
  ```



#### Walkers

和vehicles差不多

> - [**carla.WalkerControl**](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.WalkerControl) moves the pedestrian around with a certain direction and speed. It also allows them to jump. 
> - [**carla.WalkerBoneControl**](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.WalkerBoneControl) provides control over the 3D skeleton. [This tutorial](https://carla.readthedocs.io/en/0.9.10/tuto_G_control_walker_skeletons/) explains how to control it. 

[**carla.WalkerAIController**](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.WalkerAIController)

```python
walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
world.SpawnActor(walker_controller_bp, carla.Transform(), parent_walker)
```

行人只有AI，没有Autopilot。AI需要给出初始位置，目标点，（可选参数：速度）

当行人走到目标点之后会随机下一个位置，如果目标点走不到他们会走去距离自己当前最近那个点

> **To destroy AI pedestrians**, stop the AI controller and destroy both, the actor, and the controller. 
>
> **销毁AI行人**，需要同时销毁actor和控制器controller



## C. Maps and navigation

- 比较重要的部分

### 地图 The map

*OpenDRIVE 文件描述了道路信息， [OpenDRIVE standard 1.4](http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.4H.pdf)*

#### 改变地图 Changing the map

- 改换一个地图，world也会被改变（在UE4内部所有东西会被从头重建），两种方法

  - `reload_world()` creates a new instance of the world with the same map.  

  - `load_world()` changes the current map and creates a new world.  

    ```python
    world = client.load_world('Town01')
    ```

  - 可用地图的list

    ```python
    print(client.get_available_maps())
    ```



#### 地标 Landmarks

- 从OpenDRIVE里定义的Traffic signs信息会被转换为CARLA的landmark
- **[carla.Landmark](https://carla.readthedocs.io/en/latest/python_api/#carla.Landmark)** 表示OpenDRIVE signals，这个object记录了参数，方法和有效位置
  - [**carla.LandmarkOrientation**](https://carla.readthedocs.io/en/latest/python_api/#carla.LandmarkOrientation) ，方向
  - [**carla.LandmarkType**](https://carla.readthedocs.io/en/latest/python_api/#carla.LandmarkType) ，类型
  - 上边这俩点进去看看就比较明白点了
- **[carla.Waypoint](https://carla.readthedocs.io/en/latest/python_api/#carla.Waypoint)** 可以得到前方指定一段距离的（指定）landmark
- **[carla.Map](https://carla.readthedocs.io/en/latest/python_api/#carla.Map)** 得到landmark的集合（set），可以返回map中的所有landmark或ID/类型/组别相似的landmark
- **[carla.World](https://carla.readthedocs.io/en/latest/python_api/#carla.World)** 就像 *carla.TrafficSign* 和 *carla.TrafficLight* 与landmarks的中介，让他们能在模拟器里具象化

```python
my_waypoint.get_landmarks(200.0,True)
```



#### 车道(线) Lanes

在CARLA中转换为 [**carla.LaneType**](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.LaneType)

- [**carla.LaneMarking**](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.LaneMarking)

  > - [**carla.LaneMarkingType**](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.LaneMarkingType) are enum values according to OpenDRIVE standards. 
  > - [**carla.LaneMarkingColor**](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.LaneMarkingColor) are enum values to determine the color of the marking. 
  > - **width** to state thickness of the marking. 
  > - [**carla.LaneChange**](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.LaneChange) to state permissions to perform lane changes.  

```python
# Get the lane type where the waypoint is. 
lane_type = waypoint.lane_type

# Get the type of lane marking on the left. 
left_lanemarking_type = waypoint.left_lane_marking.type()

# Get available lane changes for this waypoint.
lane_change = waypoint.lane_change
```



#### 交叉 Junctions

转换为 [carla.Junction](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.Junction) 用bounding box来分辨lanes和vehicles在不在junction里

>  The most remarkable method of this class returns a pair of waypoints per lane inside the junction. Each pair is located at the starting and  ending point of the junction boundaries.  

```python
waypoints_junc = my_junction.get_waypoints()
```



#### 路标点 Waypoints

- [**carla.Waypoint**](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.Waypoint) ，3D的指示点，所有与waypoint有关的事件都发生在**client端**。每个waypoint里有一个[carla.Transform](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.Transform)，表明了其位置和所在路段方向。四个变量可以`road_id`,`section_id`,`lane_id`和`s`可以转换到OpenDRIVE road —— waypoint id**（2cm 以内的同road上的waypoint共享一个`id`）**
- waypoint还包括lane的信息，特别是左右车道的lane markings，还有一个boolean表示是否在junction里

```python
# Examples of a waypoint accessing to lane information
inside_junction = waypoint.is_junction()
width = waypoint.lane_width
right_lm_color = waypoint.right_lane_marking.color
```



### CARLA中的导航 Navigation in CARLA

*——Navigation in CARLA is managed via the waypoint API*

#### 通过路标点导航 Navigating through waypoints

> - `next(d)` creates a list of waypoints at an approximate distance `d` **in the direction of the lane**. The list contains one waypoint for each deviation possible. 
>
>   `next(d)` 创建一个距离在约`d`的 **顺着lane方向** 的waypoint的list，这个list对每个偏差可能都有一个waypoint
>
> - `previous(d)` creates a list of waypoints waypoint at an approximate distance `d` **on the opposite direction of the lane**. The list contains one waypoint for each deviation possible.  
>
>   `previous(d)`创建一个距离在约`d`的 **反着lane方向** 的waypoint的list，这个list对每个偏差可能都有一个waypoint
>
> - `next_until_lane_end(d)` and `previous_until_lane_start(d)` returns a list of waypoints a distance `d` apart. The list goes from the current waypoint to the end and start of its lane, respectively.  
>
>   `next_until_lane_end(d)` 和`previous_until_lane_start(d)` 返回距离为`d`的waypoint的list，这个list从当前点到lane的end或start
>
> - `get_right_lane()` and `get_left_lane()`  return the equivalent waypoint in an adjacent lane, if any. A lane  change maneuver can be made by finding the next waypoint to the one on  its right/left lane, and moving to it.
>
>   `get_right_lane()` 和 `get_left_lane()` 返回对旁边车道等量的waypoints，换道策略可以通过设置目标为移动到左侧或右侧的车道实现

```python
# Disable physics, in this example the vehicle is teleported.
vehicle.set_simulate_physics(False)
while True:
    # Find next waypoint 2 meters ahead.
    waypoint = random.choice(waypoint.next(2.0))
    # Teleport the vehicle.
    vehicle.set_transform(waypoint.transform)
```



#### 生成地图导航 Generating a map navigation

- **Get recommended spawn points for vehicles** 
- **Get the closest waypoint**
- **Generate a collection of waypoints**
- **Generate road topology**
- **Convert simulation point to geographical coordinates**
- **Save road information**





### CARLA maps

熟悉的地图（Town01,02,03,04,05,06,07,10），最复杂的地图是Town 03

![img](https://carla.readthedocs.io/en/0.9.10/img/Town03.jpg)



## D. Sensors and data

-  目前不怎么用，简单看看记录一些，[carla.Sensor](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.Sensor)

### 一步步构造传感器 Sensors step-by-step

- Data格式：全部继承自[carla.SensorData](https://carla.readthedocs.io/en/0.9.10/python_api/#carla.SensorData). 具体取决于传感器类型
- 每种传感器均依靠listen()方法监听获取数据，在模拟器一个step或特定事件注册发生采集信息。具体取决于传感器
- 除去各自的不同，每种传感器使用的方法是相近的



#### Setting

从蓝图创建，这决定了传感器的最终输出结果（格式），Sensor参考：[sensors reference](https://carla.readthedocs.io/en/0.9.10/ref_sensors/)

```python
# Find the blueprint of the sensor.
blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
# Modify the attributes of the blueprint to set image resolution and field of view.
blueprint.set_attribute('image_size_x', '1920')
blueprint.set_attribute('image_size_y', '1080')
blueprint.set_attribute('fov', '110')
# Set the time in seconds between sensor captures
blueprint.set_attribute('sensor_tick', '1.0')
```



#### Spawning

- Sensor需要关联（attachment）在parent actor上（通常是vehicle）
- 关联的类型不同会决定Sensor关于vehicle位置更新
  - Rigid attachment：严格按照parent的移动而移动
  - SpringArm attachment：丝滑的跳动，一般推荐用来记录模拟的视频录制使用

```python
transform = carla.Transform(carla.Location(x=0.8, z=1.7))
sensor = world.spawn_actor(blueprint, transform, attach_to=my_vehicle)
```



#### Listening

- sensor获取数据时都会调用一次listen()
- listen()的参数是callback函数，是一个lambda function，它告诉了sensor应该怎么处理收集的数据

```python
# do_something() will be called each time a new image is generated by the camera.
sensor.listen(lambda data: do_something(data))

...

# This collision sensor would print everytime a collision is detected. 
def callback(event):
    for actor_id in event:
        vehicle = world_ref().get_actor(actor_id)
        print('Vehicle too close: %s' % vehicle.type_id)

sensor02.listen(callback)
```



#### Data

每个sensor的data一般不同，参考[sensors reference](https://carla.readthedocs.io/en/0.9.10/ref_sensors/)，常规均有的基础信息如下

| Sensor data attribute | Type                                                         | Description                                                  |
| --------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| `frame`               | int                                                          | Frame number when the measurement took place.                |
| `timestamp`           | double                                                       | Timestamp of the measurement in simulation seconds since the beginning of the episode. |
| `transform`           | [carla.Transform](https://carla.readthedocs.io/en/0.9.10/python_api#carlatransform) | World reference of the sensor at the time of the measurement. |

> `is_listening` is a **sensor attribute** that enables/disables data listening at will.
>  `sensor_tick` is a **blueprint attribute** that sets the simulation time between data received.  



### 传感器类型 Types of sensors

#### 相机类 Cameras

- 相机类，拍摄图片

| Sensor                                | Output                                                       | Overview                                                     |
| ------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| Depth（深度相机）                     | [carla.Image](https://carla.readthedocs.io/en/0.9.10/python_api#carlaimage) | Renders the depth of the elements in the field of view in a gray-scale map. |
| RGB（彩色相机）                       | [carla.Image](https://carla.readthedocs.io/en/0.9.10/python_api#carlaimage) | Provides clear vision of the surroundings. Looks like a normal photo of the scene. |
| Semantic segmentation（语义分割相机） | [carla.Image](https://carla.readthedocs.io/en/0.9.10/python_api#carlaimage) | Renders elements in the field of view with a specific color according to their tags. |



#### 探测类 Detectors

- 探测类，**一般当特定注册事件触发时（trigger）会记录数据**

> **Retrieve data** every simulation step.  

| Sensor                    | Output                                                       | Overview                                                  |
| ------------------------- | ------------------------------------------------------------ | --------------------------------------------------------- |
| Collision（碰撞检测）     | [carla.CollisionEvent](https://carla.readthedocs.io/en/0.9.10/python_api#carlacollisionevent) | Retrieves collisions between its parent and other actors. |
| Lane invasion（车道跨越） | [carla.LaneInvasionEvent](https://carla.readthedocs.io/en/0.9.10/python_api#carlalaneinvasionevent) | Registers when its parent crosses a lane marking.         |
| Obstacle（障碍检测）      | [carla.ObstacleDetectionEvent](https://carla.readthedocs.io/en/0.9.10/python_api#carlaobstacledetectionevent) | Detects possible obstacles ahead of its parent.           |



#### 其他 Other

- 其他类，很杂，导航/物理测量/2D或3D点云等，**一般在一个step记录数据**

> **Retrieve data** every simulation step.  

| Sensor                      | Output                                                       | Overview                                                     |
| --------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| GNSS（地理位置）            | [carla.GNSSMeasurement](https://carla.readthedocs.io/en/0.9.10/python_api#carlagnssmeasurement) | Retrieves the geolocation of the sensor.                     |
| IMU（惯性测量单元）         | [carla.IMUMeasurement](https://carla.readthedocs.io/en/0.9.10/python_api#carlaimumeasurement) | Comprises an accelerometer, a gyroscope, and a compass.      |
| LIDAR（激光雷达）           | [carla.LidarMeasurement](https://carla.readthedocs.io/en/0.9.10/python_api#carlalidarmeasurement) | A rotating LIDAR. Generates a 4D point cloud with coordinates and intensity per point to model the surroundings. |
| Radar（雷达）               | [carla.RadarMeasurement](https://carla.readthedocs.io/en/0.9.10/python_api#carlaradarmeasurement) | 2D point map modelling elements in sight and their movement regarding the sensor. |
| RSS（**安全检测相关**）     | [carla.RssResponse](https://carla.readthedocs.io/en/0.9.10/python_api#carlarssresponse) | Modifies the controller applied to a vehicle according to safety  checks. This sensor works in a different manner than the rest, and there is specific [RSS documentation](https://carla.readthedocs.io/en/0.9.10/adv_rss) for it. |
| Semantic LIDAR（语义LIDAR） | [carla.SemanticLidarMeasurement](https://carla.readthedocs.io/en/0.9.10/python_api#carlasemanticlidarmeasurement) | A rotating LIDAR. Generates a 3D point cloud with extra information regarding instance and semantic segmentation. |

