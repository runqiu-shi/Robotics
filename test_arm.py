from ik import Ik
import math
from pyCreate2.vrep import vrep

class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.time = factory.create_time_helper()
        self.arm = factory.create_kuka_lbr4p()
        self.create = factory.create_create()
        self.ik = Ik(self.arm, self.time)

    def run(self):
        self.pick_up()
        self.ik.inverse_kinematics(0, 1)
        self.place_on_shelf()


    def pick_up(self):
        self.arm.go_to(5, -1)
        self.ik.inverse_kinematics(-0.62, 0.2)
        self.time.sleep(2)
        self.arm.close_gripper()
        self.arm.close_gripper()
        self.time.sleep(2)

    def place_on_shelf(self):
        self.arm.go_to(0, 1)
        self.arm.go_to(4, -1)
        self.arm.go_to(5, -1.5)
        self.time.sleep(3)
        self.ik.inverse_kinematics(0.2, 0.9)
        # self.arm.go_to(6, math.pi)
        self.time.sleep(3)
        self.ik.inverse_kinematics(0.43, 0.8)
        self.arm.open_gripper()
        self.arm.open_gripper()
        self.time.sleep(3)