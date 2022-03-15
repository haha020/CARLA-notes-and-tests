# 环境封装

~~这封装就和写u3d或着game一样.......~~

很多信息来源于这个地方 https://zhuanlan.zhihu.com/p/434687295

- **创建环境**：`gym.make()`

```python
env = gym.make('EnvName')
```



- **重置环境**：`env.reset()`

等于重来！

销毁所有sprite，然后创建新的sprite（突然sprite

sprite记得写到sprite_group里（意思就是这里actor写到actor_list中方便销毁啥的）

 

- **执行单步动作**：`env.step()` 

比较重要的地方

```python
next_obs, reward, done, info = env.step(action)
# 下一个环境，奖励，done标记，信息
```

***对于CARLA：***

1. action对应到carla控制可以用VehicleControl

```python
vehicle_control = carla.VehicleControl(
            throttle=float(throttle),
            steer=float(steer),
            brake=float(brake),
            hand_brake=False,
            reverse=False,
            manual_gear_shift=False
        )
self.ego.apply_control(vehicle_control)
```



2. 获取新一步环境信息

获取车辆自身的状态信息：get_transform()（actor的方法，包括location和rotation）

```python
		transform = self.ego.get_transform()
        location = transform.location
        rotation = transform.rotation
        x_pos = location.x
        y_pos = location.y
        z_pos = location.z

        if deg2rad:
            # if True, convert the degree to radians
            # such as 90 degree means pi/2=3.14/2=1.57
            pitch = np.radians(rotation.pitch)
            yaw = np.radians(rotation.yaw)
            roll = np.radians(rotation.roll)
        else:
            pitch = rotation.pitch
            yaw = rotation.yaw
            roll = rotation.roll
        
        def vector_to_scalar(vector):
           scalar = np.around(np.sqrt(vector.x ** 2 +
                                      vector.y ** 2 +
                                      vector.z ** 2), 2)
           return scalar

        acceleration = vector_to_scalar(self.ego.get_acceleration())
        angular_velocity = vector_to_scalar(self.ego.get_angular_velocity())
        velocity = vector_to_scalar(self.ego.get_velocity())
```

传感器是数据流格式，需要变换一下，比如那个语义

```python
		image.convert(cc.CityScapesPalette)
        bgra = np.array(image.raw_data).reshape(84, 84, 4)
        bgr = bgra[:, :, :3]
        rgb = np.flip(bgr, axis=2)      
```



3. reward和done

例子是一个碰撞传感器

```python
def _get_collision_reward(self):
	if not self.collision:
		return False, 0 # 返回done=False，reward=0这样子
	else:
		return True, -1 # 返回done=True，reward=-1这样子
```



- 关闭环境：`env.close()` 

- 渲染环境：`env.render()` 



# 一些Trick

SkipFrame跳帧：代码来自 https://pytorch.org/tutorials/intermediate/mario_rl_tutorial.html

```python
class SkipFrame(gym.Wrapper):
    def __init__(self, env, skip):
        """Return only every `skip`-th frame"""
        super().__init__(env)
        self._skip = skip

    def step(self, action):
        """Repeat action, and sum reward"""
        total_reward = 0.0
        done = False
        for i in range(self._skip):
            # Accumulate reward and repeat the same action
            obs, reward, done, info = self.env.step(action)
            total_reward += reward
            if done:
                break
        return obs, total_reward, done, info
    
env = SkipFrame(env, skip=4)
```



还是通过这个tutorial，大概感觉是env这个gym环境可以外部套很多Wrapper，等于是给env上几层特殊trick一样

```python
# Apply Wrappers to environment
env = SkipFrame(env, skip=4) # 套一层
env = GrayScaleObservation(env) # 再来套一层
env = ResizeObservation(env, shape=84) # 继续套一层
env = FrameStack(env, num_stack=4) # 再套一层
```

