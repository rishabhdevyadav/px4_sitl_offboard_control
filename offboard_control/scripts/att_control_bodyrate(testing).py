#!/usr/bin/env python
import sys
# ROS python API
import rospy

from sensor_msgs.msg import Imu
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Vector3
import tf.transformations as transformations
from tf.transformations import *
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from trajectory_msgs.msg import MultiDOFJointTrajectory as Mdjt
from std_msgs.msg import Float32

from gazebo_msgs.msg import ModelStates

import numpy as np
from tf.transformations import *

from msg_check.msg import PlotDataMsg 


# from probnum.linalg import problinsolve
# from probnum.linalg import bayescg    


# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s"%e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)
           


class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PoseStamped()
        # set the flag to use position setpoints and yaw angle
        self.yaw_angle = Float32()
        self.yaw_angle.data = 0.0

        self.imu = Imu()
        
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.ALT_SP = 1.0
        self.sp.pose.position.z = self.ALT_SP

        self.local_pos = PoseStamped()
        self.local_vel = TwistStamped()

        self.local_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.desVel = np.zeros(3)
        self.desAcc = np.zeros(3)

        self.errInt = np.zeros(3)
        self.att_cmd = PoseStamped()
        self.thrust_cmd = Thrust()

        # Gains for simulation
        self.Lam = np.array([4, 4, 6])
        self.Phi = np.array([1.5, 1.5, 1.1])
        self.m = 1.5
        
# all data has been collected on this value
        self.Kpos_ = np.array([4, 4, 12])
        self.Kvel_ = np.array([3, 3, 3])
        self.Kint_ = np.array([0.2, 0.2, 3.0])


# m = 1 Kg
        # self.Kpos_ = np.array([4, 4, 6])
        # self.Kvel_ = np.array([2, 2, 2])
        # self.Kint_ = np.array([0.5, 0.5, 0.5])


# m = 2 Kg
        # self.Kpos_ = np.array([6, 6, 16])
        # self.Kvel_ = np.array([4, 4, 6])
        # self.Kint_ = np.array([0.5, 0.5, 0.75])


        self.norm_thrust_const = 0.056
        self.max_th = 36.0
        self.max_throttle = 0.96

        self.gravity = np.array([0, 0, 9.8])

        self.pre_time = rospy.get_time()
        self.pre_time2 =  rospy.get_time() 

        self.euler_err = np.array([0.0, 0.0, 0.0])
        self.euler_err_ = np.array([0.0, 0.0, 0.0])
        self.euler_rate_err = np.array([0.0, 0.0, 0.0])
        self.des_q_dot = np.array([0 ,0, 0])

        self.kPos_q = np.array([1.0, 1.0, 2.0])
        self.kVel_q = np.array([0.01, 0.01, 0.1])
        self.kInt_q = np.array([0.01, 0.01, 0.01])
        self.errInt_q = np.zeros(3)

        self.armed = False


        self.command = AttitudeTarget()   #Control Mode 2
        self.collective_thrust = 0.0



        # Publishers
        self.att_pub = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.thrust_pub = rospy.Publisher('mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
        self.body_rate = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        self.data_pub = rospy.Publisher('/data_out', PlotDataMsg, queue_size=10)
        self.data_out = PlotDataMsg()

   
        self.last_tau = None
        self.last_vel = np.array([0,0,0])


    ## local position callback


    def base_link_pos(self, msg):
        idx = msg.name.index('iris')
        iris = msg.pose[idx]
        # print(iris)
        # print(iris.position.x)
        # print(iris.position.y)
        # print(iris.position.z)

        self.local_pos.pose.position.x = iris.position.x
        self.local_pos.pose.position.y = iris.position.y
        self.local_pos.pose.position.z = iris.position.z

        self.local_pos.pose.orientation.x = iris.orientation.x
        self.local_pos.pose.orientation.y = iris.orientation.y
        self.local_pos.pose.orientation.z = iris.orientation.z
        self.local_pos.pose.orientation.w = iris.orientation.w

        # print('-------YES----------')

    # def posCb(self, msg):
    #     print()
        # self.local_pos.pose.position.x = msg.pose.position.x
        # self.local_pos.pose.position.y = msg.pose.position.y
        # self.local_pos.pose.position.z = msg.pose.position.z

        # self.local_pos.pose.orientation.x = msg.pose.orientation.x
        # self.local_pos.pose.orientation.y = msg.pose.orientation.y
        # self.local_pos.pose.orientation.z = msg.pose.orientation.z
        # self.local_pos.pose.orientation.w = msg.pose.orientation.w

        # print()

    def velCb(self, msg):
        self.local_vel.twist.linear.x = msg.twist.linear.x
        self.local_vel.twist.linear.y = msg.twist.linear.y
        self.local_vel.twist.linear.z = msg.twist.linear.z

        self.local_vel.twist.angular.x = msg.twist.angular.x
        self.local_vel.twist.angular.y = msg.twist.angular.y
        self.local_vel.twist.angular.z = msg.twist.angular.z


    def multiDoFCb(self, msg):
        pt = msg.points[0]
        self.sp.pose.position.x = pt.transforms[0].translation.x
        self.sp.pose.position.y = pt.transforms[0].translation.y
        self.sp.pose.position.z = pt.transforms[0].translation.z
        self.desVel = np.array([pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z])
        self.desAcc = np.array([pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z])


    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.pose.position.x = self.local_pos.pose.position.x
        self.sp.pose.position.y = self.local_pos.pose.position.y
        self.sp.pose.position.z = self.local_pos.pose.position.z

    def accCB(self,msg):
        self.imu.orientation.w = msg.orientation.w
        self.imu.orientation.x = msg.orientation.x
        self.imu.orientation.y = msg.orientation.y
        self.imu.orientation.z = msg.orientation.z

        self.imu.angular_velocity.x = msg.angular_velocity.x
        self.imu.angular_velocity.y = msg.angular_velocity.y
        self.imu.angular_velocity.z = msg.angular_velocity.z

        self.imu.linear_acceleration.x = msg.linear_acceleration.x
        self.imu.linear_acceleration.y = msg.linear_acceleration.y
        self.imu.linear_acceleration.z = msg.linear_acceleration.z


    def newPoseCB(self, msg):
        if(self.sp.pose.position != msg.pose.position):
            print("New pose received")
        self.sp.pose.position.x = msg.pose.position.x        
        self.sp.pose.position.y = msg.pose.position.y
        self.sp.pose.position.z = msg.pose.position.z
   
        self.sp.pose.orientation.x = msg.pose.orientation.x
        self.sp.pose.orientation.y = msg.pose.orientation.y
        self.sp.pose.orientation.z = msg.pose.orientation.z
        self.sp.pose.orientation.w = msg.pose.orientation.w

    def yawAngle(self,msg):
        self.yaw_angle.data = msg.data

    def vector2Arrays(self, vector):        
        return np.array([vector.x, vector.y, vector.z])

    def vector3Arrays(self, vector):        
        return np.array([vector.x, vector.y, vector.z , vector.w])

    def array2Vector3(self, array, vector):
        vector.x = array[0]
        vector.y = array[1]
        vector.z = array[2]

    def array2Vector4(self, array, vector):
        vector.x = array[0]
        vector.y = array[1]
        vector.z = array[2]
        vector.w = array[3]


    def a_des(self):
        dt = rospy.get_time() - self.pre_time
        self.pre_time = self.pre_time + dt

        if dt > 0.03:
            dt = 0.03

        curPos = self.vector2Arrays(self.local_pos.pose.position)
        desPos = self.vector2Arrays(self.sp.pose.position)
        curVel = self.vector2Arrays(self.local_vel.twist.linear)
        curAcc = self.vector2Arrays(self.imu.linear_acceleration) - self.gravity

        # print(curAcc)


        errPos = curPos - desPos
        errVel = curVel - self.desVel


        self.data_out.x_curr = curPos[0]
        self.data_out.y_curr = curPos[1]
        self.data_out.z_curr = curPos[2]

        self.data_out.x_dot_curr = curVel[0]
        self.data_out.y_dot_curr = curVel[1]
        self.data_out.z_dot_curr = curVel[2]

        self.data_out.x_ddot_curr = curAcc[0]
        self.data_out.y_ddot_curr = curAcc[1]
        self.data_out.z_ddot_curr = curAcc[2]



        self.data_out.x_des = desPos[0]
        self.data_out.y_des = desPos[1]
        self.data_out.z_des = desPos[2]

        self.data_out.x_dot_des = self.desVel[0]
        self.data_out.y_dot_des = self.desVel[1]
        self.data_out.z_dot_des = self.desVel[2]

        self.data_out.x_ddot_des = self.desAcc[0]
        self.data_out.y_ddot_des = self.desAcc[1]
        self.data_out.z_ddot_des = self.desAcc[2]



        self.data_out.x_err = errPos[0]
        self.data_out.y_err = errPos[1]
        self.data_out.z_err = errPos[2]

        self.data_out.x_dot_err = errVel[0]
        self.data_out.y_dot_err = errVel[1]
        self.data_out.z_dot_err = errVel[2]

        errAcc = curAcc - self.desAcc

        self.data_out.x_ddot_err = errAcc[0]
        self.data_out.y_ddot_err = errAcc[1]
        self.data_out.z_ddot_err = errAcc[2]


        if self.armed:
            self.errInt += errPos*dt

        # sv = errVel + np.multiply(self.Phi, errPos)
        # des_a = -np.multiply(self.Lam, sv) + self.m*(self.gravity + self.desAcc - np.multiply(self.Phi, errVel))

        des_a = -(self.Kpos_*errPos + self.Kvel_*errVel + self.Kint_*self.errInt) + self.gravity


        self.data_out.mass = self.m

        # print(errPos)
        # print(des_a)
        print('--------------')
        
        # self.last_tau = des_a
        self.last_vel = curVel

        if np.linalg.norm(des_a) > self.max_th:
            des_a = (self.max_th/np.linalg.norm(des_a))*des_a

        return (des_a ) 


    def acc2quat(self,des_a, des_yaw):
        proj_xb_des = np.array([np.cos(des_yaw), np.sin(des_yaw), 0.0])
        if np.linalg.norm(des_a) == 0.0:
            zb_des = np.array([0,0,1])
        else:    
            zb_des = des_a / np.linalg.norm(des_a)
        yb_des = np.cross(zb_des, proj_xb_des) / np.linalg.norm(np.cross(zb_des, proj_xb_des))
        xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))
       
        rotmat = np.transpose(np.array([xb_des, yb_des, zb_des]))
        return rotmat


    def geo_con_(self):
        des_a = self.a_des()

        pose = transformations.quaternion_matrix(  
                np.array([self.local_pos.pose.orientation.x, 
                             self.local_pos.pose.orientation.y, 
                             self.local_pos.pose.orientation.z, 
                             self.local_pos.pose.orientation.w]))  #4*4 matrix
        pose_temp1 = np.delete(pose, -1, axis=1)
        rot_curr = np.delete(pose_temp1, -1, axis=0)   #3*3 current rotation matrix
        zb_curr = rot_curr[:,2]
        thrust = self.norm_thrust_const * des_a.dot(zb_curr)
        self.collective_thrust = np.maximum(0.0, np.minimum(thrust, self.max_throttle))

        self.data_out.tau_x = des_a[0]
        self.data_out.tau_y = des_a[1]
        self.data_out.tau_z = des_a[2]
        self.data_out.thrust = des_a.dot(zb_curr)
        self.data_out.collective_thrust = thrust

        # ----------------------------------------------- # 

        # Current Euler orientation and 
        orientation_q = self.local_pos.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll_curr, pitch_curr, yaw_curr) = euler_from_quaternion (orientation_list)

        #---------------------------------------------#

        # Desired Euler orientation and Desired Rotation matrix

        rot_des = self.acc2quat(des_a, 0)   #desired yaw = 0
        rot_44 = np.vstack((np.hstack((rot_des,np.array([[0,0,0]]).T)), np.array([[0,0,0,1]])))

        quat_des = quaternion_from_matrix(rot_44)
        orientation_list = [quat_des[0], quat_des[1], quat_des[2], quat_des[3]]
        (roll_des, pitch_des, yaw_des) = euler_from_quaternion (orientation_list)

        # -------------------------------------------------#
        # Roll pitch yaw error Method 1
        roll_err = roll_curr - roll_des
        pitch_err = (pitch_curr - pitch_des)
        yaw_err = (yaw_curr - yaw_des)
        # -------------------------------------------- #

        # Roll pitch yaw error Method 2
        angle_error_matrix = 0.5* (np.dot(np.transpose(rot_des), rot_curr) -
                                    np.dot(np.transpose(rot_curr), rot_des) ) #skew matrix
        roll_x_err = -angle_error_matrix[1,2]
        pitch_y_err = angle_error_matrix[0,2]   
        yaw_z_err = -angle_error_matrix[0,1]


        self.euler_err = np.array([roll_err, pitch_err, yaw_err])
        self.euler_err_ = np.array([roll_x_err, pitch_y_err, yaw_z_err])

        des_euler_rate =  np.dot(np.multiply(np.transpose(rot_des), rot_curr), 
                                     self.des_q_dot)    # = 0,0,0
        curr_euler_rate = np.array([self.local_vel.twist.angular.x,
                                    self.local_vel.twist.angular.y, 
                                    self.local_vel.twist.angular.z ])

        self.euler_rate_err = curr_euler_rate - des_euler_rate


        self.data_out.roll_curr = roll_curr
        self.data_out.pitch_curr = pitch_curr
        self.data_out.yaw_curr = yaw_curr

        self.data_out.roll_dot_curr = curr_euler_rate[0]
        self.data_out.pitch_dot_curr = curr_euler_rate[1]
        self.data_out.yaw_dot_curr = curr_euler_rate[2]

        self.data_out.roll_des = roll_des
        self.data_out.pitch_des = pitch_des
        self.data_out.yaw_des = yaw_des

        self.data_out.roll_dot_des = 0
        self.data_out.pitch_dot_des = 0
        self.data_out.yaw_dot_des = 0

        self.data_out.roll_err = roll_x_err
        self.data_out.pitch_err = pitch_y_err
        self.data_out.yaw_err = yaw_z_err

        self.data_out.roll_dot_err = self.euler_rate_err[0]
        self.data_out.pitch_dot_err = self.euler_rate_err[1]
        self.data_out.yaw_dot_err = self.euler_rate_err[2]

    # ----------------------------

        dt = rospy.get_time() - self.pre_time2
        self.pre_time2 = self.pre_time2 + dt
        if dt > 0.03:
            dt = 0.03

        if self.armed:
            self.errInt_q += self.euler_err*dt
        des_mom = -(self.kPos_q*self.euler_err) - (self.kVel_q*self.euler_rate_err) - (self.kInt_q*self.errInt_q)
        # des_mom = -(self.kPos_q*self.euler_err)  - (self.kInt_q*self.errInt_q)

        self.data_out.body_moment_x = des_mom[0]
        self.data_out.body_moment_y = des_mom[1]
        self.data_out.body_moment_z = des_mom[2]


    # -------------------------------
        # Control MODE 1
        now = rospy.Time.now()
        self.att_cmd.header.stamp = now
        self.thrust_cmd.header.stamp = now
        self.data_out.header.stamp = now
        self.att_cmd.pose.orientation.x = quat_des[0]
        self.att_cmd.pose.orientation.y = quat_des[1]
        self.att_cmd.pose.orientation.z = quat_des[2]
        self.att_cmd.pose.orientation.w = quat_des[3]
        self.thrust_cmd.thrust = self.collective_thrust

    # ----------------------------------- 
        # # Control MODE 2
        # # source: mavros_controller  nonlinear_attitude_control.cpp
        attctrl_tau_ = 6.0
        inverse = np.array([1.0, -1.0, -1.0, -1.0])
        curr_att = np.array([orientation_q.x, orientation_q.y, 
                                orientation_q.z, orientation_q.w])
        ref_att = np.array([quat_des[0], quat_des[1], 
                                quat_des[2], quat_des[3]])
        q_inv = np.diag(inverse).dot(curr_att)

        # Quaternion multiplication (later make an function)
        x1, y1, z1, w1 = q_inv  # (q.x, q.y, q.z, q.w)
        x2, y2, z2, w2 = ref_att  # (q.x, q.y, q.z, q.w)
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        qe = np.array([x, y, z, w])  # (q.x, q.y, q.z, q.w)

        # Desired rate calculations
        desired_rate = np.zeros(3)
        desired_rate[0] = (2.0 / attctrl_tau_) * np.copysign(1.0, qe[3]) * qe[0]
        desired_rate[1] = (2.0 / attctrl_tau_) * np.copysign(1.0, qe[3]) * qe[1]
        desired_rate[2] = (2.0 / attctrl_tau_) * np.copysign(1.0, qe[3]) * qe[2]
        desired_rate = np.clip(desired_rate, -0.03, 0.03)
        print(desired_rate)
        
        self.command.thrust =  self.collective_thrust
        self.command.body_rate.x = desired_rate[0]      #to move in y  -> Roll Angle 
        self.command.body_rate.y = desired_rate[1]      #to move in x  -> Pitch Angle changes in Sim
        self.command.body_rate.z = desired_rate[2]

    # ----------------------------------- 
        # Control MODE 2
        # source: mavros_controller  nonlinear_geometric_control.cpp
        # attctrl_tau_ = 1.0

        # desired_rate = np.zeros(3)
        # desired_rate = (2.0 / attctrl_tau_) * self.euler_err_
        # print(desired_rate)
        
        # self.command.thrust =  self.collective_thrust
        # self.command.body_rate.x = desired_rate[0]      #to move in y  -> Roll Angle 
        # self.command.body_rate.y = desired_rate[1]      #to move in x  -> Pitch Angle changes in Sim
        # self.command.body_rate.z = desired_rate[2]

        
    def pub_att(self):

        # --------------------
        # Control MODE 1
        # self.geo_con_()
        # self.thrust_pub.publish(self.thrust_cmd)
        # self.att_pub.publish(self.att_cmd)
        # self.data_pub.publish(self.data_out)

        # --------------------
        # Control MODE 2
        self.geo_con_()
        self.body_rate.publish(self.command)


# Main function
def main(argv):
   
    rospy.init_node('setpoint_node', anonymous=True)

    modes = fcuModes()  #flight modes
    cnt = Controller()  # controller object
    rate = rospy.Rate(50)

    rospy.Subscriber('/gazebo/model_states', ModelStates, cnt.base_link_pos)

    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, cnt.velCb)
    rospy.Subscriber('mavros/imu/data', Imu, cnt.accCB)

    rospy.Subscriber('command/trajectory', Mdjt, cnt.multiDoFCb)
    rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB)
    rospy.Subscriber('yaw_in_deg',Float32,cnt.yawAngle)
    sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    print("ARMING")

    while not cnt.state.armed:
        modes.setArm()
        cnt.armed = True
        rate.sleep()

    cnt.armed = True
    k=0

    while k<20:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")

    # ROS main loop
    while not rospy.is_shutdown():

#--------------------------------------------
        cnt.pub_att()
        rate.sleep()
#--------------------------------------------  

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass

