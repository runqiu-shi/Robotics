from pyCreate2 import create2
import math
import odometry
import pid_controller
import lab8_map
import lab10_map as rrt_map
import particle_filter
import rrt
import ik
import numpy as np


class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.arm = factory.create_kuka_lbr4p()
        self.virtual_create = factory.create_virtual_create()
        # self.virtual_create = factory.create_virtual_create("192.168.1.XXX")
        self.odometry = odometry.Odometry()
        self.mapJ = lab8_map.Map("lab8_map.json")
        self.map = rrt_map.Map("configuration_space.png")
        self.rrt = rrt.RRT(self.map)
        self.ik = ik.Ik(self.arm, self.time)

        # pid params
        self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-200, 200], is_angle=True)
        self.pdWF = pid_controller.PIDController(200, 50, 0, [0, 0], [-100, 100])
        self.pf = None

        self.joint_angles = np.zeros(7)

        # Roomba params
        self.offset_x = 1.0
        self.offset_y = 0.5
        self.odometry.x = self.offset_x
        self.odometry.y = self.offset_y
        self.base_speed = 200
        self.at_goal = False
        self.sonar_readings = []
        self.sonar_avg = 10
        self.mode = 1  # 1 for go-to-goal, 2 for wall following, 3 for skipping waypoints
        self.wall_follow_direction = 'left'
        self.goal_point = (1.5, 2.5)  # TODO: enter the goal point,
        self.start_point = None       # starting point is automatically detected by calling sim functions

        # pf params
        self.pf_last_x = self.odometry.x
        self.pf_last_y = self.odometry.y
        self.pf_last_theta = 0

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, goal_theta):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
                math.sin(goal_theta - self.odometry.theta),
                math.cos(goal_theta - self.odometry.theta))) > 0.05:
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        # self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
        while True:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(base_speed + output_theta), int(base_speed - output_theta))

            # stop if close enough to goal
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            if distance < 0.05:
                self.create.drive_direct(0, 0)
                break
            self.sleep(0.01)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def visualize(self):
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)
        return x, y, theta

    def run(self):
        """main function"""

        '''initialization'''
        self.create.start()
        self.create.safe()
        self.create.sim_get_position()

        self.create.drive_direct(0, 0)

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        self.sleep(0.5)

        self.pf = particle_filter.ParticleFilter(self.mapJ, 500, 0.05, 0.25, 0.1, self.create.sim_get_position())
        self.start_point = self.create.sim_get_position()
        self.visualize()
        self.virtual_create.enable_buttons()
        self.visualize()
        self.odometry.x = self.start_point[0]
        self.odometry.y = self.start_point[1]

        ''' navigate robot to goal '''
        # RRT path generation
        start_x = self.start_point[0] * 100
        start_y = (3.0 - self.start_point[1]) * 100
        self.rrt.build((start_x, start_y), 800, 30)
        goal_x = self.goal_point[0] * 100
        goal_y = (3.0 - self.goal_point[1]) * 100
        point_goal = self.rrt.nearest_neighbor((goal_x, goal_y))
        path = self.rrt.shortest_path(point_goal)

        # drawing
        for v in self.rrt.T:
            for u in v.neighbors:
                self.map.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0, 0, 0))
        for idx in range(0, len(path) - 1):
            self.map.draw_line((path[idx].state[0], path[idx].state[1]),
                               (path[idx + 1].state[0], path[idx + 1].state[1]), (0, 255, 0))
        self.map.save("project_rrt.png")

        # reformat path to a list of coordinates
        waypoints = []
        for p in path:
            goal_x = p.state[0] / 100.0
            goal_y = 3 - p.state[1] / 100.0
            waypoints.append((goal_x, goal_y))
        self.goal_point = waypoints[-1]

        # waypoints = [(1.3, 0.6), (1.5, 0.7), (1.6, 0.6), (1.4, 0.7), (1.65, 0.8), (1.4, 1.2), (1.5, 1.5), (1.2, 1.7),
        #              (1.55, 2.1), (1.55, 2.8)]

        # follow each waypoint, use obstacle avoidance when necessary
        for idx, waypoint in enumerate(waypoints):
            # check if the current waypoint is the last one,
            # if so, wall following gets disabled (robot gets very close to the wall eventually)
            if idx == len(waypoints) - 1:
                curr_waypoint_is_goal = True
            else:
                curr_waypoint_is_goal = False

            # loop between the two behaviors until waypoint is reached
            last_check = self.time.time()
            goto_check = self.time.time()
            while True:
                wp_x, wp_y = waypoint[0], waypoint[1]

                # go to goal
                if self.mode == 1:
                    if not self.at_goal:
                        # update sonar
                        self.update_sonar_array()

                        # enable wall following if distance is too close
                        if self.sonar_avg < 0.3 and not curr_waypoint_is_goal:
                            self.mode = 2
                            if self.odometry.theta < math.pi / 2:  # follow right
                                self.servo.go_to(-75)
                                self.go_to_angle(self.odometry.theta + math.pi / 2)
                                self.wall_follow_direction = 'right'
                            else:
                                self.servo.go_to(75)  # follow left
                                self.go_to_angle(self.odometry.theta - math.pi / 2)
                                self.wall_follow_direction = 'left'
                    last_check = self.time.time()

                    self.go_to_goal(wp_x, wp_y)

                    if self.time.time() > goto_check + 5:
                        self.update_pf()
                        break;

                # wall following
                if self.mode == 2:
                    self.wall_follow(last_check)
                    self.mode = 3

                # skipping after wf
                if self.mode == 3:
                    # if current position is beyond the current waypoint, skip it
                    distance_to_goal = math.sqrt(math.pow(self.goal_point[0] - self.odometry.x, 2) +
                                                 math.pow(self.goal_point[1] - self.odometry.y, 2))
                    wp_to_goal = math.sqrt(math.pow(self.goal_point[0] - wp_x, 2) +
                                           math.pow(self.goal_point[1] - wp_y, 2))
                    if distance_to_goal < wp_to_goal:
                        print('skipping...')
                        break
                    else:
                        self.update_pf()
                        self.mode = 1

                # calculate distance to current waypoint
                distance_to_wp = math.sqrt(math.pow(wp_x - self.odometry.x, 2) + math.pow(wp_y - self.odometry.y, 2))
                if not curr_waypoint_is_goal:
                    if distance_to_wp < 0.2:
                        self.mode = 1
                        self.update_pf()
                        break
                else:
                    if distance_to_wp < 0.1:
                        self.mode = 1
                        self.update_pf()
                        break

                # manual sensing is retained for testing
                b = self.virtual_create.get_last_button()
                if b == self.virtual_create.Button.Sense:
                    self.update_pf()

        # all waypoints have been reached
        self.at_goal = 1
        self.create.drive_direct(0, 0)
        self.go_to_angle(0.5*math.pi)
        self.create.drive_direct(0, 0)
        self.update_pf()
        print("robot reached goal position")
        # print("actual position: ", self.create.sim_get_position())

        # move arm
        delta_x = 1.5 - self.odometry.x
        delta_y = 2.5 - self.odometry.y
        self.pick_up(delta_x, delta_y)
        self.ik.inverse_kinematics(0, 1)
        self.time.sleep(2)
        self.place_on_shelf()
        while True:
            self.sleep(5)

        # # button detection
        # while True:
        #     b = self.virtual_create.get_last_button()
        #     if b == self.virtual_create.Button.MoveForward:
        #         self.forward()
        #         self.visualize()
        #     elif b == self.virtual_create.Button.TurnLeft:
        #         self.go_to_angle(self.odometry.theta + math.pi / 2)
        #         self.visualize()
        #     elif b == self.virtual_create.Button.TurnRight:
        #         self.go_to_angle(self.odometry.theta - math.pi / 2)
        #         self.visualize()
        #     elif b == self.virtual_create.Button.Sense:
        #         distance = self.sonar.get_distance()
        #         # print(distance)
        #         self.pf.measure(distance, 0)
        #         self.visualize()
        #
        #     # posC = self.create.sim_get_position()
        #     # print(posC)
        #     # self.arm.go_to(4, math.radians(-90))
        #     # self.arm.go_to(5, math.radians(90))
        #     # self.time.sleep(100)
        #
        #     self.time.sleep(0.01)

    '''roomba functions'''
    def update_sonar_array(self):
        # read from sonar and average
        self.sonar_readings = np.append(self.sonar_readings, self.sonar.get_distance())
        if len(self.sonar_readings) > 3:
            self.sonar_readings = np.delete(self.sonar_readings, 0)
        self.sleep(0.01)
        self.sonar_avg = np.average(self.sonar_readings)
        print("sonar:{:.3f}".format(self.sonar_avg))

    def go_to_goal(self, goal_x, goal_y):
        state = self.create.update()
        if state is not None and self.mode == 1:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(self.base_speed + output_theta), int(self.base_speed - output_theta))
            print("[{:.3f},{:.3f},{:.3f}] in go_to_goal, goal({:.3f},{:.3f})".format(self.odometry.x, self.odometry.y,
                                                                             math.degrees(self.odometry.theta), goal_x,
                                                                             goal_y), end=' ')

    def wall_follow(self, last_check):
        while (self.time.time() - last_check) < 2:
            if self.wall_follow_direction == 'left':
                self.servo.go_to(75)
                self.update_sonar_array()
                state = self.create.update()
                if state is not None:
                    output = self.pdWF.update(self.sonar_avg, 0.4, self.time.time())
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    self.create.drive_direct(int(self.base_speed - output), int(self.base_speed + output))
                    print("[{:.3f},{:.3f},{:.3f}] in wall_follow_left".format(self.odometry.x, self.odometry.y,
                                                                              math.degrees(self.odometry.theta)))
            if self.wall_follow_direction == 'right':
                self.servo.go_to(-75)
                self.update_sonar_array()
                state = self.create.update()
                if state is not None:
                    output = self.pdWF.update(self.sonar_avg, 0.4, self.time.time())
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    self.create.drive_direct(int(self.base_speed + output), int(self.base_speed - output))
                    print("[{:.3f},{:.3f},{:.3f}] in wall_follow_right".format(self.odometry.x, self.odometry.y,
                                                                               math.degrees(self.odometry.theta)))
        self.servo.go_to(0)
        self.mode = 1
        self.create.drive_direct(0, 0)
        self.sleep(1)
        self.update_pf()

    def update_pf(self):
        # move particles
        delta_x = self.odometry.x - self.pf_last_x
        delta_y = self.odometry.y - self.pf_last_y
        delta_theta = self.odometry.theta - self.pf_last_theta
        self.pf.move_by(delta_x, delta_y, delta_theta)

        # get measurements and the estimate
        distance = self.sonar.get_distance()
        self.pf.measure(distance, 0)
        self.pf_last_x, self.pf_last_y, self.pf_last_theta = self.visualize()

        # update odometry values using particle filter estimate (take the average)
        odometry_percentage = 0.9
        pf_percentage = 0.1
        self.odometry.x, self.odometry.y, self.odometry.theta = ((odometry_percentage * self.odometry.x + pf_percentage * self.pf_last_x),
                                                                 (odometry_percentage * self.odometry.y + pf_percentage * self.pf_last_y),
                                                                 (odometry_percentage * self.odometry.theta + pf_percentage * self.pf_last_theta))

    def pick_up(self, delta_x, delta_y):
        self.arm.go_to(5, -0.8)
        self.arm.go_to(0, -0.5*delta_x-0.1)
        self.arm.open_gripper()
        self.arm.open_gripper()
        self.ik.inverse_kinematics(-0.6 - 0.5 * delta_y, 0.15)
        self.time.sleep(5)
        self.arm.close_gripper()
        self.arm.close_gripper()
        self.time.sleep(5)

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
