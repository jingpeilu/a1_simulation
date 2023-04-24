"""Simple script for executing random actions on A1 robot."""

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from tqdm import tqdm
import numpy as np
from motion_imitation.envs import env_builder
from motion_imitation.robots import a1
from motion_imitation.robots import laikago
from motion_imitation.robots import robot_config

robot_type = "A1"
_motor_control_mode = "Position"
on_rack = True


ROBOT_CLASS_MAP = {'A1': a1.A1, 'Laikago': laikago.Laikago}

MOTOR_CONTROL_MODE_MAP = {
    'Torque': robot_config.MotorControlMode.TORQUE,
    'Position': robot_config.MotorControlMode.POSITION,
    'Hybrid': robot_config.MotorControlMode.HYBRID
}


def main():
  robot = ROBOT_CLASS_MAP[robot_type]
  motor_control_mode = MOTOR_CONTROL_MODE_MAP[_motor_control_mode]
  env = env_builder.build_regular_env(robot,
                                      motor_control_mode=motor_control_mode,
                                      enable_rendering=True,
                                      on_rack=on_rack)

  env.reset()
  for _ in tqdm(range(100)):
    #print(env.action_space.sample())
    a = np.zeros(12)
    observation_dict, reward, done, misc = env.step(a)
    # observation_dict : BaseDisplacementSensor (3), IMUSensor (4), MotorAngleSensor(12)
    print(observation_dict[7:])



if __name__ == "__main__":
  main()
