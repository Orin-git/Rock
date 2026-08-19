! /usr/bin/env python
# -*-coding:utf-8-*-
import time
import rospy
from sensor_msgs.msg import LaserScan, Imu
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Int8, Bool, Float32, Int16
from threading import Thread
import numpy as np
import math
import tf_conversions
#from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseWithCovarianceStamped
from YLLog import yl_logger
import traceback

TIMEOUT = 90

distance_func = lambda theta1, rho1, theta2, rho2: (rho1 ** 2 + rho2 ** 2 - 2 * rho1 * rho2 * math.cos(theta1 - theta2)) ** 0.5

class Reflaction:
    theta1 = 0.0
    rho1 = 0.0
    theta2 = 0.0
    rho2 = 0.0

    def __init__(self, theta1=0, rho1=0, theta2=0, rho2=0):
        self.theta1 = theta1
        self.rho1 = rho1
        self.theta2 = theta2
        self.rho2 = rho2

    def __repr__(self):
        return 'Reflaction{}'.format((self.theta1, self.rho1, self.theta2, self.rho2))

    def push_forward(self,pose, distance):
        # 创建一个新的 PoseStamped
        new_pose = PoseStamped()
        new_pose.header = pose.header

        # 提取当前 pose 的位置和方向
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        qx = pose.pose.orientation.x
        qy = pose.pose.orientation.y
        qz = pose.pose.orientation.z
        qw = pose.pose.orientation.w

        # 转换四元数为欧拉角，提取航向角（yaw）
        r = R.from_quat([qx, qy, qz, qw])
        _, _, yaw = r.as_euler('xyz', degrees=False)

        # 计算新的位置
        new_x = x + distance * math.cos(yaw)
        new_y = y + distance * math.sin(yaw)

        # 更新新的位置
        new_pose.pose.position.x = new_x
        new_pose.pose.position.y = new_y
        new_pose.pose.position.z = z  # 保持高度不变

        # 保持方向不变
        new_pose.pose.orientation = pose.pose.orientation

        return new_pose

    def get_pose(self):
        x1 = self.rho1 * math.cos(self.theta1)
        y1 = self.rho1 * math.sin(self.theta1)
        x2 = self.rho2 * math.cos(self.theta2)
        y2 = self.rho2 * math.sin(self.theta2)

        y_offset = 0 #0.05
        yaw_offset = 0# -5 * math.pi / 180
        #yaw = math.atan2(y2 - y1, x2 - x1) + math.pi / 2
        yaw = math.atan2(y2 - y1, x2 - x1) + math.pi / 2 - yaw_offset
        pose = PoseStamped()
        pose.header.frame_id = 'base_laser_link'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = (x1 + x2) / 2
        pose.pose.position.y = (y1 + y2) / 2 + y_offset
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        #pose=self.push_forward(pose,0.4)
        return pose

class ReflactionDetector:
    _encode = None
    _err = None
    @classmethod
    def init(cls):
        cls._encode = [0.06, 0.025, 0.08, 0.025, 0.06]
        cls._err = 0.02

    @classmethod
    def find_reflaction(cls, msg):
        angle = msg.angle_min
        res = []
        reflaction = []
        last_v = None
        for i, v in enumerate(msg.ranges):
            angle += msg.angle_increment
            intensity = msg.intensities[i]
            if intensity > 200:
                last_v = v
                reflaction.append(i)
            else:
                if len(reflaction) > 1:
                    theta1 = msg.angle_min + msg.angle_increment * reflaction[0]
                    rho1 = msg.ranges[reflaction[0]]
                    theta2 = msg.angle_min + msg.angle_increment * reflaction[-1]
                    rho2 = msg.ranges[reflaction[-1]]
                    value = Reflaction(theta1, rho1, theta2, rho2)
                    res.append(value)
                reflaction = []
                last_v = None
        return res

    @classmethod
    def find_charger(cls, msg):
        reflactions = cls.find_reflaction(msg)
        for i, r in enumerate(reflactions):
            dis = distance_func(r.theta1, r.rho1, r.theta2, r.rho2)
            #rospy.loginfo("find_reflaction: {}".format((i, dis, r)))
        reflactions_num = (len(cls._encode) + 1) / 2

        if len(reflactions) < reflactions_num:
            return None
        #rospy.loginfo("get enough reflactions")
        start = 0
        end = 0
        last_r = None
        match_i = 0
        for i in range(len(reflactions)):
            x1 = reflactions[i].rho1 * math.cos(reflactions[i].theta1)
            y1 = reflactions[i].rho1 * math.sin(reflactions[i].theta1)
            x2 = reflactions[i].rho2 * math.cos(reflactions[i].theta2)
            y2 = reflactions[i].rho2 * math.sin(reflactions[i].theta2)
            r_x = (x1 + x2) / 2
            r_y = (y1 + y2) / 2
            #rospy.loginfo('x y {}'.format((r_x, r_y)))
            if (r_x < 0.3) or (r_x > 1.5) or (r_y < -0.8) or (r_y > 0.4):
                continue
            if last_r is None:
                dis = distance_func(reflactions[i].theta1, reflactions[i].rho1, reflactions[i].theta2, reflactions[i].rho2)
                #rospy.loginfo('dis107 {}'.format(dis))
                if math.fabs(dis - cls._encode[match_i]) < cls._err:
                    last_r = reflactions[i]
                    match_i += 1
                    start = i
                continue

            # match space
            dis = distance_func(reflactions[i].theta1, reflactions[i].rho1, reflactions[i - 1].theta2, reflactions[i - 1].rho2)
            #rospy.loginfo('dis116 {}'.format(dis))
            if math.fabs(dis - cls._encode[match_i]) < cls._err:
                last_r = reflactions[i]
                match_i += 1
            else:
                last_r = None
                match_i = 0
                continue
            # match reflact
            dis = distance_func(reflactions[i].theta1, reflactions[i].rho1, reflactions[i].theta2, reflactions[i].rho2)
            #rospy.loginfo('dis126 {}'.format(dis))
            if math.fabs(dis - cls._encode[match_i]) < cls._err:
                last_r = reflactions[i]
                match_i += 1
            else:
                last_r = None
                match_i = 0
            #rospy.loginfo('i start end last_r match_i {}'.format((i, start, end, last_r, match_i)))
            if match_i == len(cls._encode):
                end = i
                break
        #rospy.loginfo('start end last_r match_i reflactions_num {}'.format((start, end, last_r, match_i, reflactions_num)))
        if end - start + 1 == reflactions_num:
            #rospy.loginfo('get suceess!!')
            return Reflaction(reflactions[start].theta1, reflactions[start].rho1, reflactions[end].theta2, reflactions[end].rho2)
        return None

class ChargeStatus(object):
    # 清乐自动充电参数
    # 1.充电接触距离
    charge_contact_distance = 0.40
    # 2.接近充电桩 开始减速的距离
    charge_slow_distance = 0.3
    # 3.外壳与baselink距离
    shell_wheel_distanse = 0.03

    # 掉头距离
    turn_head_distance = 0.45

    charge_cmd = None


cm = ChargeStatus

class Controller(object):
    _speed_pub = None
    _tf_buffer = None
    _docking_flag_pub = None
    _leaving_flag_pub = None
    _auto_charge_flag_sub = None
    auto_charge_flag = False
    _auto_charge_control_pub=None
    _localInitialPose_pub=None
    _reset_raw_encoder_pub=None
    battery_current=0.0

    @classmethod
    def init(cls):
        cls._speed_pub = rospy.Publisher('/first_vel', Twist, queue_size=1)
        cls._docking_flag_pub = rospy.Publisher('/charger_get_flag', Bool, queue_size=1)
        cls._leaving_flag_pub = rospy.Publisher('/charger_leave_flag', Bool, queue_size=1)
        #cls._save_bag_pub=rospy.Publisher('/save_bag',Bool,queue_size=1)
        cls._auto_charge_flag_sub = rospy.Subscriber('/charger_status', Int8, cls.auto_charge_flag_callback)
        cls._battery_current_sub = rospy.Subscriber('/battery_current', Float32, cls.battery_current_callback)
        cls._auto_charge_control_pub=rospy.Publisher('/charge_control_new', Int8, queue_size=1)
        cls._reset_raw_encoder_pub=rospy.Publisher('/reset_raw_encoder',Int8,queue_size=1)
        cls._localInitialPose_pub=rospy.Publisher('/localInitialPose',PoseWithCovarianceStamped,queue_size=1)
        cls._tf_buffer = tf2_ros.Buffer()

    @classmethod
    def send_speed(cls, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        msg.linear.z = 101
        cls._speed_pub.publish(msg)

    @classmethod
    def battery_current_callback(cls, msg):
        data=msg.data
        cls.battery_current=data


    @classmethod
    def pub_docking_flag(cls, data):
        msg = Bool()
        msg.data = data
        cls._docking_flag_pub.publish(msg)

    @classmethod
    def pub_leaving_flag(cls, data):
        msg = Bool()
        msg.data = data
        cls._leaving_flag_pub.publish(msg)

    @classmethod
    def auto_charge_flag_callback(cls, msg):
        data = msg.data
        cls.auto_charge_flag = data

    # @classmethod
    # def pub_save_bag_flag(cls, data):
    #     msg = Bool()
    #     msg.data = data
    #     cls._save_bag_pub.publish(msg)

class ChargerPose(PoseStamped):

    def __init__(self, x=0, y=0, yaw=0, frame_id='charger'):
        super(self.__class__, self).__init__()
        self.header.frame_id = frame_id
        self.header.stamp = rospy.Time.now()
        self.pose.position.x = x
        self.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

    @classmethod
    def copy(cls, posestmap):
        res = cls()
        res.header = posestmap.header
        res.pose = posestmap.pose
        return res

    def transform(self, frame_id_out, waiting=1):
        # trans charger -> frame_id_out
        try:
            transform = Controller._tf_buffer.lookup_transform(frame_id_out, self.header.frame_id, rospy.Time(0), rospy.Duration(waiting))
        # print transform
        # PoseStamped frame_id_out: any_pose
            trans_pose = tf2_geometry_msgs.do_transform_pose(self, transform)
            return self.copy(trans_pose)
        except Exception as e:
            yl_logger.error('Could not get transform from charge to base_link:{}'.format(e))
            return None

class SmoothControl(object):

    def __init__(self):
        ''' Parameters for approach controller'''
        self.k1_ = 3  # ratio in change of theta to rate of change in r
        self.k2_ = 2  # speed at which we converge to slow system
        self.min_velocity_ = 0.05
        self.max_velocity_ = 0.1
        self.max_angular_velocity_ = 2.0
        self.beta_ = 0.2  # how fast velocity drops as k increases
        self.lambda_ = 2.0

        # self.dist_ = 0.3 #used to create the tracking line
        self.dist_ = cm.charge_slow_distance

    def get_ready(self):
        Controller.send_speed(0, 0)
        self.dist_ = cm.charge_slow_distance

    # target type is geometry_msgs::PoseStamped in base_link
    def approach(self, target):
        #log.info('approach {}'.format(target))

        # print target
        pose = PoseStamped()
        pose.header = target.header
        pose.pose.position = target.pose.position
        pose.pose.orientation = target.pose.orientation
        # print pose
        orient = pose.pose.orientation
        q = [orient.x, orient.y, orient.z, orient.w]
        yaw = euler_from_quaternion(q)[2]
        # print yaw
        # Controller.pub_charge_goal(pose)

        # Normalizes the angle to be -M_PI circle to +M_PI circle
        theta = self.normalize_angle(yaw)
        # print 'theta ',theta
        pose.pose.position.x += math.cos(theta) * (-0.4)
        pose.pose.position.y += math.sin(theta) * (-0.4)
        # Controller.pub_charge_goal(pose)

        try:
            pose.header.stamp = rospy.Time.now()
            Controller._tf_buffer.lookup_transform('charger', 'base_link', rospy.Time(0), rospy.Duration(5))  # pose in base_link
        except Exception as e:
            yl_logger.error('Could not get transform from charge to base_link')
            self.get_ready()
            return False

        # distance to goal
        r = math.sqrt(pose.pose.position.x * pose.pose.position.x +
                      pose.pose.position.y * pose.pose.position.y)
        #rospy.loginfo('>>>>>>>>>>>>pose.pose.position.x: {}, pose.pose.position.y: {}'.format(pose.pose.position.x, pose.pose.position.y))
        zsm_c = 2 * pose.pose.position.y / (pose.pose.position.x * pose.pose.position.x + pose.pose.position.y * pose.pose.position.y)
        # once we get close, reduce dist_

        #if r < cm.charge_slow_distance:
        #    self.dist_ = 0.0
        if r < cm.shell_wheel_distanse:
            return True

        # orientation base frame relative to r_
        delta = math.atan2(-pose.pose.position.y, pose.pose.position.x)

        # Determine orientation of goal frame relative to r_
        # tf::Quaternion q;
        # tf::quaternionMsgToTF(pose.pose.orientation, q);
        # theta = angles::normalize_angle(tf::getYaw(q) + delta);
        theta = self.normalize_angle(yaw + delta)

        # compute the virtual control
        a = math.atan(-self.k1_ * theta)
        # compute curvature (k)
        k = -1.0 / r * (self.k2_ * (delta - a) + (1 + (self.k1_ / (1 + (self.k1_ * theta) * (self.k1_ * theta)))) * math.sin(delta))

        # Compute max_velocity based on curvature
        v = self.max_velocity_ / (1 + self.beta_ * math.pow(math.fabs(k), self.lambda_))

        # limit max velocity based on approaching target(avoids overshoot)
        if r < cm.charge_slow_distance:
            v = max(self.min_velocity_, min(min(r - cm.shell_wheel_distanse, self.max_velocity_), v))  # 百泰克
            # v = max(self.min_velocity_, min(min(r, self.max_velocity_),v))
        else:
            v = min(self.max_velocity_, max(self.min_velocity_, v))

        # compute angular velocity
        w = zsm_c * v

        #rospy.loginfo('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>= v: {}, w: {}, dock r: {}'.format(v, w, r))

        # bound angular velocity
        boubded_w = min(self.max_angular_velocity_, max(-self.max_angular_velocity_, w))

        if w != 0.0:
            v *= (boubded_w / w)

        #rospy.loginfo('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> v: {}, w: {}, dock r: {}'.format(v, boubded_w, r))

        # Send command to base
        Controller.send_speed(v, boubded_w)

        yaw = 0.0
        return False

    def normalize_angle(self, angle):
        result = (angle + math.pi) % (2.0 * math.pi)
        if result <= 0.0:
            return result + math.pi
        return result - math.pi

class MoveTocharge(object):
    def __init__(self, parent=None):
        self.sc = SmoothControl()
        self.parent = parent

    def execute(self, userdata=None):
        #rospy.loginfo('MoveTocharge:execute')

        freq = rospy.Rate(20)
        start_time = rospy.Time.now()
        self.sc.get_ready()  # 设置目标点到charger的距离

        target = ChargerPose(0.0, 0.0, math.pi, 'charger')  # target type is PoseStamped()
        yl_logger.info('-----1------')
        lost_tf_time = None         # 第一次丢失 TF 的时间
        rotate_start_time = None    # 开始旋转计时（用于 3 秒左 & 3 秒右）

        rotation_mode = 0           # 0=未开始旋转, 1=左转3秒, 2=右转3秒

        while not rospy.is_shutdown():

            # 返回命令退出
            if cm.charge_cmd is not None and cm.charge_cmd == 0:
                yl_logger.warning('charge cmd is 0')
                cm.charge_cmd = None
                return 0

            # ------------------------ 尝试获取 TF ------------------------
            try:
                trans = Controller._tf_buffer.lookup_transform(
                'charger', 'base_link', rospy.Time(0), rospy.Duration(0.1)
                )

                # **找到 TF，停止旋转并继续执行主流程**
                Controller.send_speed(0, 0)
                rotation_mode = 0
                rotate_start_time = None
                lost_tf_time = None
                break

            except Exception:
                # 第一次丢 TF，开始计时
                if lost_tf_time is None:
                    lost_tf_time = rospy.Time.now()
                    yl_logger.warning("TF lost, waiting 3 seconds before rotating...")

                dt = (rospy.Time.now() - lost_tf_time).to_sec()

                # 前 2 秒：不旋转
                if dt < 2.0:
                    Controller.send_speed(0, 0)
                    freq.sleep()
                    continue

                # ------------------------ 开始旋转搜索 TF ------------------------
                if rotation_mode == 0:
                    rotation_mode = 1
                    rotate_start_time = rospy.Time.now()
                    yl_logger.warning("Start rotating left for 3 seconds to search TF")

                rotate_dt = (rospy.Time.now() - rotate_start_time).to_sec()

                # ---- 右转 3 秒 ----
                if rotation_mode == 1:
                    if rotate_dt < 3.0:
                        Controller.send_speed(0, -0.1)
                    else:
                        # 右转完切左转
                        rotation_mode = 2
                        rotate_start_time = rospy.Time.now()
                        yl_logger.warning("TF still missing. Rotate right for 3 seconds.")

                # ---- 左转 5 秒 ----
                elif rotation_mode == 2:
                    if rotate_dt < 5.0:
                        Controller.send_speed(0, 0.1)
                    else:
                        yl_logger.error("TF not found after left 3s + right 3s. Abort.")
                        Controller.send_speed(0, 0)
                        return -1

            freq.sleep()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > TIMEOUT:
                yl_logger.warning('MoveTocharge:TimeOut')
                return -2

            if cm.charge_cmd is not None and cm.charge_cmd == 0:
                yl_logger.warning('charge cmd is 0')
                cm.charge_cmd = None
                return 0

            try:
                trans = Controller._tf_buffer.lookup_transform('charger', 'base_link', rospy.Time(0), rospy.Duration(5))
            except Exception as e:
                yl_logger.error('could not transform from charger to base_link')
                return -1
            orient_1 = trans.transform.rotation
            q_1 = [orient_1.x, orient_1.y, orient_1.z, orient_1.w]
            yaw_1 = euler_from_quaternion(q_1)[2] * 180 / math.pi

            if yaw_1 < 0:
                yaw_1 += 180
            elif yaw_1 > 0:
                yaw_1 -= 180
            yaw_1 = 0 - yaw_1
            #rospy.loginfo('yaw_1 {}'.format(yaw_1))

            target_in_base_link = target.transform('base_link')
            r = ((target_in_base_link.pose.position.x) * (target_in_base_link.pose.position.x) + (target_in_base_link.pose.position.y) * (target_in_base_link.pose.position.y)) / 2 / math.fabs((target_in_base_link.pose.position.y))
            #rospy.loginfo('r {}  x  {}  y  {}'.format(r, (target_in_base_link.pose.position.x), (target_in_base_link.pose.position.y)))
            initial_angle = math.acos((r - math.fabs(target_in_base_link.pose.position.y)) / r) * (target_in_base_link.pose.position.y) / math.fabs(target_in_base_link.pose.position.y)
            #rospy.loginfo('initial_angle {}'.format(initial_angle))
            initial_angle = math.degrees(initial_angle)
            #rospy.loginfo('initial_angle {}'.format(initial_angle))

            if abs(initial_angle - yaw_1) > 0.5:
                if (initial_angle - yaw_1) > 0:
                    Controller.send_speed(0, 0.08)
                else:
                    Controller.send_speed(0, -0.08)
            else:
                Controller.send_speed(0, 0)
                break

            freq.sleep()

        try:
            trans = Controller._tf_buffer.lookup_transform('charger', 'base_link', rospy.Time(0), rospy.Duration(5))
        except Exception as e:
            yl_logger.error('could not transform from charger to base_link:{}'.format(e))
            return -1
        yl_logger.info('-----2------')
        # 对接阶段
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > TIMEOUT:
                yl_logger.warning('MoveTocharge:TimeOut')
                return -2

            if cm.charge_cmd is not None and cm.charge_cmd == 0:
                yl_logger.warning('charge cmd is 0')
                cm.charge_cmd = None
                return 0

            target_in_base_link = target.transform('base_link')
            if target_in_base_link is None:
                yl_logger.warning('target_in_base_link is None')
                continue
            self.sc.approach(target_in_base_link)
            yl_logger.info(' -- x:{} --y:{}'.format(target_in_base_link.pose.position.x, target_in_base_link.pose.position.y))
            if target_in_base_link.pose.position.x < 0.2:
                Controller.send_speed(0.0, 0)
                break
            freq.sleep()
        yl_logger.info('-----3------')
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > TIMEOUT:
                yl_logger.warning('MoveTocharge:TimeOut')
                return -2

            if cm.charge_cmd is not None and cm.charge_cmd == 0:
                yl_logger.warning('charge cmd is 0')
                cm.charge_cmd = None
                return 0

            try:
                trans = Controller._tf_buffer.lookup_transform('charger', 'base_link', rospy.Time(0), rospy.Duration(5))
            except Exception as e:
                yl_logger.error('could not transform from charger to base_link')
                return -1

            orient = trans.transform.rotation
            q = [orient.x, orient.y, orient.z, orient.w]
            target_yaw = euler_from_quaternion(q)[2] * 180 / math.pi
            yl_logger.info('target_yaw {}'.format(target_yaw))
            if (180 - abs(target_yaw)) > 0.5:
                    if target_yaw > 0:
                        Controller.send_speed(0, 0.07)
                    else:
                        Controller.send_speed(0, -0.07)
            else:
                Controller.send_speed(0, 0)
                break
            freq.sleep()

        origin_angle = self.parent._robot_yaw
        yl_logger.info('-----4------')
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > TIMEOUT:
                yl_logger.warning('MoveTocharge:TimeOut')
                return -2

            if cm.charge_cmd is not None and cm.charge_cmd == 0:
                yl_logger.warning('charge cmd is 0')
                cm.charge_cmd = None
                return 0

            move_angle = self.parent._robot_yaw - origin_angle
            if move_angle < -180:
                move_angle += 360
            elif move_angle > 180:
                move_angle -= 360

            yl_logger.info('---------------------------move angle : {}'.format(move_angle))

            if math.fabs(move_angle - 180) <= 1.0:
                Controller.send_speed(0, 0)
                break

            Controller.send_speed(0, 0.2)

            freq.sleep()
        start_time_5 = rospy.Time.now()
        yl_logger.info('-----5------')
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time_5).to_sec() > 25:
                yl_logger.warning('MoveTocharge:TimeOut')
                return -2
            if cm.charge_cmd is not None and cm.charge_cmd == 0:
                cm.charge_cmd = None
                return 0

            if Controller.auto_charge_flag or Controller.battery_current==0.0:
                Controller.send_speed(0, 0)
                return 1
            Controller.send_speed(-0.03, 0)
            freq.sleep()


class LeaveFromCharge(object):
    def __init__(self, parent=None):
        self.parent = parent

    def execute(self, userdata=None):
        yl_logger.info('LeaveFromCharge:execute')# 离开充电桩

        freq = rospy.Rate(10)
        start_time = rospy.Time.now()

        # 获取当前位置
        try:
            trans = Controller._tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(5))
        except Exception as e:
            yl_logger.error('could not transform from odom to base_link')
            return -1

        origin_xy = trans.transform.translation.x, trans.transform.translation.y

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > TIMEOUT:
                yl_logger.warning('LeaveFromCharge:TimeOut')
                return -2

            if cm.charge_cmd is not None and cm.charge_cmd == 0:
                cm.charge_cmd = None
                return 0

            try:
                trans = Controller._tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(5))
            except Exception as e:
                yl_logger.error('could not transform from odom to base_link')
                return -1

            d = np.linalg.norm([trans.transform.translation.x - origin_xy[0], trans.transform.translation.y - origin_xy[1]])
            if d >= 0.6:
                yl_logger.info('LeaveFromCharge: success')
                Controller.send_speed(0, 0)
                return 1

            Controller.send_speed(0.08, 0)
            freq.sleep()

class AutochargeService(object):
    def __init__(self):
        Controller.init()

        self._tf_listener = tf2_ros.TransformListener(Controller._tf_buffer)
        self._br = tf2_ros.TransformBroadcaster()
        self._trans_pose = None

        self._pub_charge_tf = False
        self._robot_yaw = 0.0

        ReflactionDetector.init()

        self._scan_sub = rospy.Subscriber('/scan', LaserScan, self.on_recv_laser)
        self._charge_cmd_sub = rospy.Subscriber('/charge_cmd', Int8, self.on_recv_charge_cmd)
        self._imu_sub = rospy.Subscriber('/imu', Imu, self.recv_imu)

    def charge_thread(self):
        while not rospy.is_shutdown():
            if cm.charge_cmd == 1:
                self.charge_cmd = None
                Controller._auto_charge_control_pub.publish(0)
                rospy.Rate(1).sleep()
                Controller._auto_charge_control_pub.publish(1)
                ret = MoveTocharge(self).execute()
                yl_logger.info('charge_thread status{}'.format(ret))

                cm.charge_cmd = None
                if ret == -2:
                    time.sleep(5)
                    if Controller.auto_charge_flag:
                        ret=1
                if ret == 1:
                    yl_logger.info('charge success')
                    time.sleep(4)
                    yl_logger.info('Controller.battery_current:{}'.format(Controller.battery_current))
                    if Controller.battery_current<-0.05:
                        yl_logger.warning('Controller.battery_current<-0.05')
                        Controller.send_speed(-0.015, 0)
                        rospy.Rate(10).sleep()   # 10Hz -> 0.1s
                        Controller.send_speed(-0.015, 0)
                        rospy.Rate(10).sleep()
                        Controller.send_speed(-0.015, 0)
                        rospy.Rate(10).sleep()
                        Controller.send_speed(0.0, 0)
                        time.sleep(5)

                    while Controller.battery_current < 7.9 and not rospy.is_shutdown() and Controller.battery_current>0.1:
                        yl_logger.warning('Controller.battery_current < 8.0')
                        Controller.send_speed(0.015, 0)
                        rospy.Rate(10).sleep()   # 10Hz -> 0.1s
                        Controller.send_speed(0.015, 0)
                        rospy.Rate(10).sleep()
                        Controller.send_speed(0.015, 0)
                        rospy.Rate(10).sleep()
                        Controller.send_speed(0.0, 0)
                        time.sleep(4)
                        yl_logger.info('Controller.battery_current:{}'.format(Controller.battery_current))

                    if Controller.battery_current >7.9: #充上电后重置里程计
                        pose_msg = PoseWithCovarianceStamped()
                        try:
                            # 获取 map -> base_link
                            trans = Controller._tf_buffer.lookup_transform(
                                "map",           # target frame
                                "base_link",     # source frame
                                rospy.Time(0),   # 最新
                                rospy.Duration(1.0)
                            )
                            # header
                            pose_msg.header.stamp = rospy.Time.now()
                            pose_msg.header.frame_id = "map"
                            # position
                            pose_msg.pose.pose.position.x = trans.transform.translation.x
                            pose_msg.pose.pose.position.y = trans.transform.translation.y
                            pose_msg.pose.pose.position.z = trans.transform.translation.z
                            # orientation
                            pose_msg.pose.pose.orientation = trans.transform.rotation
                            # covariance（先给默认值）
                            pose_msg.pose.covariance = [0.0] * 36
                            # 示例：给位置一个小协方差
                            pose_msg.pose.covariance[0] = 0.05   # x
                            pose_msg.pose.covariance[7] = 0.05   # y
                            pose_msg.pose.covariance[35] = 0.1   # yaw
                            yl_logger.info("Published PoseWithCovarianceStamped")
                            Controller._reset_raw_encoder_pub.publish(1)
                            time.sleep(3)
                            Controller._localInitialPose_pub.publish(pose_msg)
                            Controller.pub_docking_flag(True)
                        except Exception as e:
                            yl_logger.error(1, "TF lookup failed: %s", str(e))
                        #yl_logger.info("828")
                    else:
                        Controller.pub_docking_flag(False)
                else:
                    Controller.pub_docking_flag(False)
                    #Controller.pub_save_bag_flag(True)
                self._pub_charge_tf = False

            elif cm.charge_cmd == 2:
                self.charge_cmd = None
                cm.charge_cmd = None
                ret = LeaveFromCharge(self).execute()
                if ret == 1:
                    yl_logger.info('leaving success')
                    Controller._auto_charge_control_pub.publish(0)
                    Controller.pub_leaving_flag(True)
                    rospy.Rate(1).sleep()
                    Controller._auto_charge_control_pub.publish(1)
                else:
                    Controller.pub_leaving_flag(False)

            rospy.Rate(1).sleep()

    def run(self):
        try:
            # 开启货架工作线程
            Thread(target=self.charge_thread, ).start()

            # 坐标转换循环
            return self.pub_transform_loop()
            # rospy.spin()
        except Exception as e:
            yl_logger.error("AutochargeService.run Exception: %s", str(e))
            yl_logger.error(traceback.format_exc())
            raise   # 如果你希望程序继续崩溃，可以保留；如果不想让节点死掉，可以去掉

    def recv_imu(self, msg):
    # 计算欧拉角
        euler = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        roll, pitch, yaw = euler
        self._robot_yaw = yaw / math.pi * 180
        #rospy.loginfo('robot angle : {}'.format(self._robot_yaw))

    def on_recv_laser(self, msg):
        #rospy.loginfo('on_recv_laser')
        #result = ReflactionDetector.find_charger(msg)
        #rospy.loginfo('find_charger {}'.format(result))

        if self._pub_charge_tf:
            result = ReflactionDetector.find_charger(msg)
            yl_logger.info('find_charger {}'.format(result))
            if result is None:
                return

            # laser link下的charger位置  PoseStamped类型
            p = result.get_pose()
            if p is None:
                yl_logger.error("result.get_pose() returned None")
            # trans base_laser_link -> odom
            # 通过tf找到laser 到 odom 的转换
            trans = Controller._tf_buffer.lookup_transform('odom', 'base_laser_link', rospy.Time(0), rospy.Duration(1))

            # PoseStamped odom: charger
            # 得到charger在odom下的位置
            try:
                self._trans_pose = tf2_geometry_msgs.do_transform_pose(p, trans)
            except Exception as e:
                yl_logger.error("do_transform_pose failed: {}".format(e))
                self._trans_pose = None

    def pub_transform_loop(self):
        freq = rospy.Rate(10)
        while not rospy.is_shutdown():
            freq.sleep()
            if not self._pub_charge_tf or self._trans_pose is None:
                continue

            p = self._trans_pose
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "odom"
            t.child_frame_id = "charger"
            t.transform.translation.x = p.pose.position.x
            t.transform.translation.y = p.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = p.pose.orientation.x
            t.transform.rotation.y = p.pose.orientation.y
            t.transform.rotation.z = p.pose.orientation.z
            t.transform.rotation.w = p.pose.orientation.w
            #rospy.loginfo('pub_transform_loop')
            self._br.sendTransform(t)

    def on_recv_charge_cmd(self, msg):
        yl_logger.info('recv cmd {}'.format(msg.data))
        cm.charge_cmd = msg.data
        if msg.data == 1:
            self._trans_pose = None
            self._pub_charge_tf = True
        else:
            self._pub_charge_tf = False

def main():
    rospy.init_node('auto_charge_service')
    yl_logger.info('auto_charge_service start----------')
    try:
        return AutochargeService().run()
    except Exception as e:
        yl_logger.error("auto_charge_service Exception: %s", str(e))
        yl_logger.error(traceback.format_exc())
        raise

if __name__ == '__main__':
    main()
