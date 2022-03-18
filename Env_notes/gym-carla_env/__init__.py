from gym.envs.registration import register
from importlib_metadata import entry_points

register(
    id='CarlaGymEnvTest',
    entry_point='CarlaGymEnv.CarlaGymEnvTest:CarlaGymEnvTest'
)