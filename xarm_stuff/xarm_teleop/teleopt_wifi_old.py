import numpy as np
from retry import retry

from bus_servo_http import BusServoHttp

class BusServoRemoteTelopt(BusServoHttp):

    def __init__(self, *args, **kwargs):
        super(BusServoRemoteTelopt, self).__init__(*args, **kwargs)
        self.last_position = [-10,-10,-10,-10,-10,-10]

    def disable_torque(self):
        servo_ids  = [1, 2, 3, 4, 5, 6]
        for id in servo_ids:
            self.unload(id)

    def enable_torque(self):
        servo_ids = [1, 2, 3, 4, 5, 6]
        for id in servo_ids:
            self.load(id)

    @retry( tries=3, delay=0.05)
    def read_position(self):
        try:
            goal_positions = self.get_positions()
            self.last_position = goal_positions
            return np.array(goal_positions)
        except Exception as e:
            print(f'got exception {e}')
            raise e

    def set_goal_pos(self, goal_positions, servo_runtime=250, verbose=True):
        print(goal_positions)
        for n, p in enumerate(goal_positions):
            if abs(self.last_position[n] - goal_positions[n]) > 3:
                if verbose:
                    print(f'setting position old {self.last_position[n]}  NEW {goal_positions[n]} ')
                self.run(n + 1, p, servo_runtime)
        self.last_position = goal_positions

    # mock method
    def read_velocity(self):
        return np.zeros(6)

if __name__ == '__main__':
    print('this')
    # follower = BusServoRemoteTelopt('192.168.1.119')
    # leader = BusServoRemoteTelopt('192.168.1.122')
    # leader.disable_torque()

    # follower.set_goal_pos(leader.read_position(), servo_runtime=300)

    # while True:
    #     print('run_loop')
    #     follower.set_goal_pos(leader.read_position())