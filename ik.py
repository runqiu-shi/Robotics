import math


class Ik:

    def __init__(self, arm, time):
        self.arm = arm
        self.time = time
        self.prev_theta1 = 0
        self.prev_theta2 = 0

    def inverse_kinematics(self, x_i, z_i):
        L1 = 0.4  # estimated using V-REP (joint2 - joint4)
        L2 = 0.39  # estimated using V-REP (joint4 - joint6)
        # Corrections for our coordinate system
        z = z_i - 0.3105
        x = -x_i
        # compute inverse kinematics
        r = math.sqrt(x * x + z * z)
        alpha = math.acos((L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2))
        theta2 = math.pi - alpha

        beta = math.acos((r * r + L1 * L1 - L2 * L2) / (2 * L1 * r))
        theta1 = math.atan2(x, z) - beta
        if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            theta2 = math.pi + alpha
            theta1 = math.atan2(x, z) + beta
        if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            print("Not possible")
            return

        delta_theta1 = (theta1 - self.prev_theta1)/20
        delta_theta2 = (theta2 - self.prev_theta2)/20
        for i in range(1, 21):
            self.arm.go_to(1, self.prev_theta1 + delta_theta1*i)
            self.arm.go_to(3, self.prev_theta2 + delta_theta2*i)
            self.time.sleep(2)
        self.prev_theta1 = theta1
        self.prev_theta2 = theta2

        print("Go to [{},{}], IK: [{} deg, {} deg]".format(x_i, z_i, math.degrees(theta1), math.degrees(theta2)))