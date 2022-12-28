#!/usr/bin/env python3
# coding=utf-8

from __future__ import division
import rospy, cv2, cv_bridge, numpy, sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String,Bool
from random import randint
import numpy as np
from roslib import message
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField     
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import time
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression, RANSACRegressor
from sklearn.pipeline import Pipeline
from math import radians
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import PoseStamped, Quaternion
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
import rosnode
from geometry_msgs.msg import Vector3, Point
from pyit2fls import IT2FLS, \
                     min_t_norm, product_t_norm, max_s_norm,IT2FS
from pyit2fls import trapezoid_mf, tri_mf,IT2Mamdani,T1FS,T1Mamdani
import rostest
import os

xr=None
yr=None
thr=None
z = None
roll = None
pitch = None
yaw = None
altitudeZ = 0
integralZ = 0
last_erroZ = 0
distH = 0
vel_linear = 0
integral = 0
ptime = 0
last_erro = 0
dt = 0.1
font = cv2.FONT_HERSHEY_COMPLEX
iter = 1

#rostest.rosrun('px4', 'mavros_offboard_posctl_test')
#########################################################################################################    

class Follower(object):

    def __init__(self):
        
        self.angulorobo = 0

        self.ze = 0
        self.phir = 0

        self.erro = 0

        self.bridge = cv_bridge.CvBridge()

        self.integral_ze = 0
        self.last_ze = 0
        self.derivative_ze = 0

        self.integral_phir = 0
        self.last_phir = 0
        self.derivative_phir = 0

        self.integral_alt = 0
        self.last_alt = 0
        self.derivative_alt = 0
        
        self.amostras = 0
        self.voltas = 0
        self.soma_z = 0
        self.media_z = 0
        self.soma_time = 0
        self.media_time = 0
        self.iter = 1
        self.cont_med = 1
        self.soma_ze = 0
        self.soma_phir = 0
        self.last_xr = 0
        self.tempo = 0
        self.dt = 0.1
        self.last_phir_true = 0
        self.acc_med = 0
        self.acc_med_phir = 0
        
        self.last_ze_filtrado = 1
        self.state = State()
        self.altitude = Altitude()
        self.global_position = HomePosition()
        self.local_position = PoseStamped()  #fcu local position
        self.local_position_odom = Odometry()
        self.extended_state = ExtendedState()
        self.mode = ''
        self.rate = rospy.Rate(10)
        self.service_timeout = 30
        self.offboard_request_threshold = rospy.Duration(5.0)
        self.arming_request_threshold = rospy.Duration(5.0)
        self.imu = Imu()
        self.current_yaw = None
        self.pos = PoseStamped()
        
        self.setup_pubsub()
        self.setup_services()
        self.current_target_pose = PositionTarget()
        self.current_pose = Point()
        self.home_pose = Point(0.0,0.0,0.0)
        self.position = (7.24,15.65,14.5) # 1m from the lines
        #self.position = (7.24,16.65,14.5) # Right above the lines


        self.waypoint_1 = Point(-23.19,-6.56,3.5)
        self.should_start_mission = False
        self.ready = False
        self.Reached = False
        
        self.radius = 0.05

        self.iter = 1
        self.cont = 1
        self.exec = 1

        self.lista_x_pto_220 = []
        self.lista_x_pto_240 = []
        self.lista_x_pto_260 = []  
        self.bridge = cv_bridge.CvBridge() 

        rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback,queue_size=1, buff_size=2**28)

        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)

        self.twistS = TwistStamped()

        rospy.Subscriber('mavros/local_position/odom',Odometry,self.callback_odom,queue_size = 10)
        self.Odom = Odometry ()
    
    ############################################################## UAV ODOMETRY FUNCION ##################################################
    def callback_odom(self,msg):
        
        #print('hehe')
        global xr
        global yr
        global thr
        global roll
        global pitch
        global yaw  
        global tf
        global r
        global ze
        global thetar


        quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        global altitudeZ
        altitudeZ = msg.pose.pose.position.z
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]  
          

    '''ros subscribers/publisher'''
    def setup_pubsub(self):
        rospy.loginfo("-------Setting pub - sub-----")
        #subscriber
        self.altitude_sub = rospy.Subscriber('/mavros/altitude',Altitude, self.altitude_cb)
        self.state_sub = rospy.Subscriber('/mavros/state',State,self.state_cb)
        self.extended_state_sub = rospy.Subscriber('/mavros/extended_state',ExtendedState,self.extended_state_cb)
        self.setpoint_raw_sub = rospy.Subscriber('/mavros/setpoint_raw/local',PositionTarget,self.setpoint_raw_cb)
        self.local_position_odom_sub = rospy.Subscriber('/mavros/local_position/odom',Odometry,self.local_position_odom_cb)
        self.local_position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_cb)
        
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)

        #publishers
        self.setpoint_raw_pub = rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=10)
        self.setmission_pub = rospy.Publisher('/mission/start',Bool,queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)
        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.twistS = TwistStamped()

    #ros services
    def setup_services(self):
        rospy.loginfo('----Waiting for services to connect----')
        try:
            rospy.wait_for_service('/mavros/param/get', self.service_timeout)
            rospy.wait_for_service('/mavros/cmd/arming',self.service_timeout)
            rospy.wait_for_service('/mavros/set_mode',self.service_timeout)
            rospy.wait_for_service('/mavros/cmd/takeoff',self.service_timeout)
            rospy.wait_for_service('/mavros/cmd/land',self.service_timeout)
            rospy.wait_for_service('/mavros/set_stream_rate',self.service_timeout)
            rospy.loginfo('Services are connected and ready')
        except rospy.ROSException as e:
            rospy.logerr('Failed to initialize service')
        
        #get services
        self.get_param_srv = rospy.ServiceProxy('/mavros/param/get',ParamGet)
        
        #set services
        self.set_stream_rate_srv = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
        self.set_arm_srv = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        self.set_takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff',CommandTOL)
        self.set_land_srv = rospy.ServiceProxy('/mavros/cmd/land',CommandTOL)

        '''set mavros stream rate to get mavros messages faster.
        mavros publishes state/setpoint messages at 1 hz by default
        '''
        #self.set_mavros_stream_rate()

    def set_mavros_stream_rate(self):
        stream_rate = StreamRateRequest()
        stream_rate.request.stream_id = 3
        stream_rate.request.message_rate = 10
        stream_rate.request.on_off = 1
        try:
            self.set_stream_rate_srv(stream_rate)
        except rospy.ServiceException as exp:
            rospy.logerr('Stream rate service failed')
            

    def set_target_pose(self,point,yaw, yaw_rate=1):
        target_pose = PositionTarget()
        target_pose.header.stamp = rospy.get_rostime()
        target_pose.coordinate_frame = 1

        target_pose.position = point
        target_pose.yaw = yaw
        target_pose.yaw_rate = yaw_rate


        target_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.FORCE


        self.current_pose = point
        return target_pose


    def extract_yaw_from_quaternion(self, q):
        quaternion = (q.x,q.y,q.z,q.w)
        yaw = euler_from_quaternion(quaternion,axes='sxyz')[2]
        return yaw


    def set_mode(self,mode):
        if self.state.mode != mode:
            try:
                mode_change_response = self.set_mode_srv(base_mode = 0, custom_mode = mode)
                last_request_time = rospy.get_rostime()
                if not mode_change_response.mode_sent:
                    rospy.logerr('---Mode change failed---')    
            except rospy.ServiceException as exception:
                rospy.logerr('Failed to change mode')


    def arm(self):
        last_request_time = rospy.get_rostime()
        if not self.state.armed:
            arm_response = self.set_arm_srv(True)
            if arm_response:
                rospy.loginfo('---Vehicle armed --')
            else:
                rospy.loginfo('---Arming failed ---')
            last_request_time = rospy.get_rostime()
        else:
            #vehicle is already armed
            pass


    def disarm(self):
        if self.set_arm_srv(False):
            rospy.loginfo('--Vehicle disarmed---')
        else:
            rospy.loginfo('---Disarming failed')


    def go_to_position(self,point,OFFSET_ENU=True):
        self.current_target_pose = self.set_target_pose(point,0)


    def return_home(self):
        self.current_target_pose = self.set_target_pose(self.home_pose,0)
    

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug("current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset
  

    #callback functions
    def altitude_cb(self,data):
        pass     
    
    def state_cb(self,data):
        self.state = data
        self.mode = data.mode

    def extended_state_cb(self,data):
        pass
    
    def setpoint_raw_cb(self,data):
        pass

    def local_position_cb(self,data):
        self.local_position = data
        pass

    def local_position_odom_cb(self,data):
        pass
        
    def imu_cb(self,data):
        self.imu = data
        self.current_yaw = self.extract_yaw_from_quaternion(self.imu.orientation)


    def image_callback(self, msg):       

        try:
            frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        except getopt.GetoptError as e:
            print(e)
        else:
            
            self.rate = rospy.Rate(10) 
            

            def start_offboard(self):
                # wait to get heartbeat from fcu
                while not self.state.connected:
                    self.rate.sleep()

                #rospy.loginfo('--Got heartbeat from FCU----')
                self.pos.header = Header()
                self.pos.header.frame_id = "base_footprint"

                #yaw_degrees = 40
                yaw_degrees = 0 # FOR ANGULAR CONTROLLER TESTS YOU NEED TO CHANGE THIS VALUE TO ADD A ANGULAR ERROR TO THE UAV
                yaw = math.radians(yaw_degrees)
                quaternion = quaternion_from_euler(0, 0, yaw)
                self.pos.pose.orientation = Quaternion(*quaternion)

                #before going offboard mode, publish setpoint.
                self.current_target_pose = self.set_target_pose(self.waypoint_1,90)
                for i in range(50,0,-1):
                    self.pos.pose.position.x = self.position[0]
                    self.pos.pose.position.y = self.position[1]
                    self.pos.pose.position.z = self.position[2]
                    self.pos_setpoint_pub.publish(self.pos)
                    self.arm()
                    self.set_mode("OFFBOARD")
                    self.rate.sleep()

                self.radius = 0.05


            if self.Reached is False :
                start_offboard(self)

            while not self.Reached :
                
                self.pos_setpoint_pub.publish(self.pos)
                self.rate.sleep()
                desired = np.array((self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z))
                pos = np.array((self.local_position.pose.position.x,self.local_position.pose.position.y,self.local_position.pose.position.z))
                diff = np.linalg.norm(desired - pos)

                if self.is_at_position(self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z, self.radius):
                    self.Reached = True
                    self.twistS.twist.angular.z = 0
                    self.twistS.twist.linear.z = 0
                    self.twistS.twist.linear.x = 0
                    self.twistS.twist.linear.y = 0 
                    self.cmd_vel_pub.publish(self.twistS)
                    break
                
            t = time.time()

            scale_percent = 100 
            scale_percent1 = 80 # Scale reduction to reduce computational efforts (May not be necessary)

            width_novo = int(frame.shape[1] * scale_percent / 100)
            height_novo = int(frame.shape[0] * scale_percent1 / 100)
            frame = cv2.resize(frame, ( int(np.ceil(width_novo)), int(np.ceil(height_novo)) ))
            frame_copy = frame.copy()
            height,width,channel = frame.shape

            #Image processing and line detection
            img_name1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_name1 = cv2.GaussianBlur(img_name1, (5,5), 0)
            linha_bw = np.zeros((height,width,channel), dtype = "uint8")
            edges = cv2.Canny(img_name1,50,100,apertureSize = 3)
            lines = cv2.HoughLinesP(edges, rho=1., theta=np.pi/180.,
                        threshold=40, minLineLength=30, maxLineGap=20.)
            
            for this_line in lines:
                cv2.line(linha_bw,
                    (this_line[0][0], this_line[0][1]),
                    (this_line[0][2], this_line[0][3]),
                    [255, 255, 255], 3, 8)
            
            ######################################### HERE YOU SELECT THE TYPE OF TEST ##########################################################

            TEST_MODE = 'lateral' # Tests involving the lateral error controller (Forward velocity = 0)
            #TEST_MODE = 'angular' # Tests involving the angular error controller (Forward velocity = 0)
            #TEST_MODE = 'following' #Tests involving the UAV tracking and following with 'Forward velocity = 0.4 m/s'.


            ######################################################################################################################################

            # Image rectification due to Image tilting as the UAV is tilted. See : https://hal-enpc.archives-ouvertes.fr/hal-00654415/document

            f = 692.81 # Camera Focal Length

            roll_ = 0
            pitch_ = -roll
            yaw_ = 0

            cx = (width-1) / 2
            cy = (height-1) / 2

            A2 = np.array([[f,0,cx],[0,f, cy],[0,0,1]])
            A1 = np.linalg.inv (A2)

            RX = np.array([[1,0,0],[0,np.cos(roll_),-(np.sin(roll_))],[0, np.sin(roll_), np.cos(roll_)]])
            RY = np.array([[np.cos(pitch_),  0, np.sin(pitch_)],[0,1,0],[-(np.sin(pitch_)), 0, np.cos(pitch_)]])
            RZ = np.array([[np.cos(yaw_), -(np.sin(yaw_)),0],[np.sin(yaw_),  np.cos(yaw_), 0],[0,0,1]]) 

            R = RX @ RY @ RZ

            H = A2 @ R @ A1

            linha_bw = cv2.warpPerspective(linha_bw, H,(frame.shape[1],frame.shape[0]),None,cv2.INTER_LINEAR)

            ######################################################## RANSAC ALGORITHM TO TRAJECTORY GENERATION ##############################################################
                    
            #Ransac receives the binary image where the white pixels correspond to the detected lines. As a result, it displays the coefficients of the curve to be considered as an input parameter for the controller.
            image_bw = cv2.rotate(linha_bw, cv2.ROTATE_90_CLOCKWISE)
            image_bw = cv2.flip(image_bw,1)

            linha_bw = cv2.cvtColor(image_bw, cv2.COLOR_BGR2GRAY)
            #cv2.imshow("c",linha_bw)

            thresh = cv2.threshold(linha_bw, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

            per = []

            coords = np.column_stack(np.where(thresh > 0))
            
            gy,gx = np.array_split(coords,[-1],1)


            model = Pipeline([('poly', PolynomialFeatures(degree=2)),
                            ('linear', LinearRegression(fit_intercept=False))])

            ransac = RANSACRegressor(model, 
                                    stop_probability=0.99,
                                    max_trials=15,
                                    min_samples=100,  
                                    residual_threshold=25, 
                                    random_state=0
            )
            ransac.fit(gx, gy)

            line_X = np.arange(0, height, 1)

            line_y_ransac = ransac.predict(line_X[:, np.newaxis])

            line_X1 = line_X.reshape(-1, 1)
            model.fit(line_X1,line_y_ransac)
            coeficientes = model.named_steps['linear'].coef_
            a = coeficientes[0][2]
            b = coeficientes[0][1] 
            c = coeficientes[0][0]

            eixo_x = []
            eixo_y = []
            eixo_y = np.arange(0, height, 1)

            for i in range(height) :
                eixo_x.append((a*(i**2) +b*i + c)) 
                    

            y_pto_1 = (10)
            x_pto_1 = eixo_x[10]
            
            y_pto_2 = (100)
            x_pto_2 = eixo_x[100]

            y_pto_3 = (150)
            x_pto_3 = eixo_x[150]

            self.y_pto_220 = (220)
            self.x_pto_220 = eixo_x[220]

            self.y_pto_240 = (240)
            self.x_pto_240 = eixo_x[240]

            self.y_pto_260 = (260)
            self.x_pto_260 = eixo_x[260] 

            y_pto_4 = (330)
            x_pto_4 = eixo_x[330]
            
            y_pto_5 = (380)
            x_pto_5 = eixo_x[380]

            y_pto_6 = (470)
            x_pto_6 = eixo_x[470]
            

            # Draw dots on frame to show the trajectory calculation 

            cv2.circle(frame, (int(self.x_pto_220),int(self.y_pto_220)), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.circle(frame, (int(self.x_pto_240),int(self.y_pto_240)), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.circle(frame, (int(self.x_pto_260),int(self.y_pto_260)), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.circle(frame, (int(x_pto_1),int(y_pto_1)), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.circle(frame, (int(x_pto_2),int(y_pto_2)), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.circle(frame, (int(x_pto_3),int(y_pto_3)), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.circle(frame, (int(x_pto_4),int(y_pto_4)), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.circle(frame, (int(x_pto_5),int(y_pto_5)), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.circle(frame, (int(x_pto_6),int(y_pto_6)), radius=3, color=(0, 0, 255), thickness=-1)       

            self.y_pto_220 = (1.2)*self.y_pto_220 # 1.2 due to scale reducion. 
            self.x_pto_220 = self.x_pto_220
            
            self.y_pto_260 = 1.2*(self.y_pto_260)
            self.x_pto_260 = self.x_pto_260
       
            f = 692.8183639889837

            ang = 21.26 # Camera tilting related to y axis.
            ang = (ang*np.pi)/180

            alt = 4.5
            x = alt/np.cos(ang)
            x1 = (width*x)/f
            conv1 = (1*x1)/width

            self.erro = self.x_pto_240 - width/2
            self.ze = (self.erro)*(conv1)
            self.ze_original = self.ze

            offset = 1
            #offset = 0

            if self.ready is False :
                self.ze = self.ze + offset        

            if self.ready is False and np.abs(self.ze) < 0.004 : # This ensures the UAV starts the tests at the desired position
                self.ready = True
                self.integral_ze = 0
                self.integral_phir = 0
                self.derivative_ze = 0
                self.derivative_ze_absolute = 0
                self.derivative_phir = 0
                self.derivative_phir_absolute = 0
                self.cont_med = 1
                self.ze = self.ze_original
                self.ze_filtrado = self.ze
                self.last_ze_filtrado = self.ze_filtrado

                print('UAV is at the specified position. Start test !')

            self.cont = self.cont+1
       
            delta_y = self.y_pto_260-self.y_pto_220
            delta_x = self.x_pto_260-self.x_pto_220

            self.phir = (np.pi/2 - np.arctan2(delta_y,delta_x))

            ################################################ IT2-FPID Lateral Controller ##########################################################
            t1 = time.time()
            
            def sat(x,value) :

                if x > value :
                    x = value
                if x < -value :
                    x = -value

                return x

            self.beta = 0.95 # Low Pass filter to derivative term. See : https://ptolemy.berkeley.edu/projects/chess/tbd/wiki/C-code/LowPassFilterForDerivativeControl

            self.derivative_ze_absolute = (np.abs(self.ze) - np.abs(self.last_ze))/self.dt
            
            self.ze_filtrado = (self.beta*self.ze) + (1-self.beta)*self.last_ze_filtrado
            self.derivative_ze = (self.ze_filtrado - self.last_ze_filtrado)/self.dt  
            self.last_ze_filtrado = self.ze_filtrado
            
            self.derivative_ze = sat(self.derivative_ze,3)
            self.derivative_ze_absolute = sat(self.derivative_ze_absolute,3)
       
            # Ultimate gain (Ku) and the oscillation period (Tu)
            KU_ze = 2.3 
            TU_ze = 1.8   

            kpmin_ = 0.32*KU_ze
            kpmax_ = 0.6*KU_ze
            kdmin_ = 0.08*KU_ze*TU_ze
            kdmax_ = 0.15*KU_ze*TU_ze

            kpmax = kpmin_ + 1/5*kpmax_ 
            kpmin = kpmin_ -1/5*kpmin_
            kdmax = kdmin_ + 1/5*kdmax_
            kdmin = kdmin_ -1/5*kdmin_
            
            kpmed = (kpmax+kpmin)/2
            kpmedtri = (kpmax-kpmin)/4
            kpmedinf = kpmed-kpmedtri
            kpmedsup = kpmed+kpmedtri

            kdmed = (kdmax+kdmin)/2
            kdmedtri = (kdmax-kdmin)/4
            kdmedinf = kdmed-kdmedtri
            kdmedsup = kdmed+kdmedtri

            ze = np.abs(self.ze)
            ze = round(ze,4)
            dot_ze = self.derivative_ze_absolute
            dot_ze = round(dot_ze,4)

            self.ze = round(self.ze,4)           
            #To avoid errors on Fuzzy controller
            self.ze = sat(self.ze,1.95)

            def fuzzySystem(algorithm, algorithm_params=[]):
                it2fls = IT2Mamdani(min_t_norm, max_s_norm, method="Centroid",algorithm=algorithm, algorithm_params=algorithm_params)
                domain1 = np.arange(0, 1.95, 0.01)
                domain2 = np.arange(-2, 2, 0.01)
                domain3 = np.arange(kpmin, kpmax, 0.001)
                domain4 = np.arange(kdmin, kdmax, 0.001)
                domain5 = np.arange(2, 4, 0.01)

                mf_inf = 1
                mf_inf_out = 1

                M = IT2FS(domain1,
                            trapezoid_mf, [0.10, 0.15, 0.20, 0.25, 1.],
                            trapezoid_mf, [0.12, 0.16, 0.19, 0.23,mf_inf])
                S = IT2FS(domain1,
                            trapezoid_mf, [-0.01, 0.0, 0.10, 0.15, 1.],
                            trapezoid_mf, [-0.01, 0.0, 0.07, 0.12,mf_inf])
                B = IT2FS(domain1,
                            trapezoid_mf, [0.20, 0.25, 1.95, 1.96, 1.],
                            trapezoid_mf, [0.23, 0.28, 1.95, 1.96,mf_inf])

                #IT2FS_plot(S,M,B)

                N = IT2FS(domain2,
                            trapezoid_mf, [-3.1, -3, -0.8, 0, 1.],
                            trapezoid_mf, [-3.1, -3, -1.2, -0.0,mf_inf])
                P = IT2FS(domain2,
                            trapezoid_mf, [0, 0.8, 3, 3.1, 1.],
                            trapezoid_mf, [0.0, 1.2, 3, 3.1,mf_inf])
                Z = IT2FS(domain2,
                            tri_mf, [-1.2, 0, 1.2, 1, ],
                            tri_mf, [-0.8, 0, 0.8,mf_inf])
                #IT2FS_plot(N,P,Z)

                S_P = IT2FS(domain3,
                            trapezoid_mf, [kpmin-0.01,kpmin,kpmedinf,kpmed, 1.],
                            trapezoid_mf, [kpmin-0.01,kpmin,0.98*kpmedinf,kpmed,mf_inf_out])
                B_P = IT2FS(domain3,
                            trapezoid_mf, [kpmed,kpmedsup,kpmax,kpmax+0.01, 1.],
                            trapezoid_mf, [kpmed,1.02*kpmedsup,kpmax,kpmax+0.01,mf_inf_out])
                M_P = IT2FS(domain3,
                            tri_mf, [kpmedinf,kpmed,kpmedsup, 1, ],
                            tri_mf, [1.02*kpmedinf,kpmed,0.98*kpmedsup,mf_inf_out])
                #IT2FS_plot(S_P,M_P,B_P)


                S_D = IT2FS(domain4,
                            trapezoid_mf, [kdmin-0.01,kdmin,kdmedinf,kdmed, 1.],
                            trapezoid_mf, [kdmin-0.01,kdmin,0.98*kdmedinf,kdmed,mf_inf_out])
                B_D = IT2FS(domain4,
                            trapezoid_mf, [kdmed,kdmedsup,kdmax,kdmax+0.01, 1.],
                            trapezoid_mf, [kdmed,1.02*kdmedsup,kdmax,kdmax+0.01,mf_inf_out])
                M_D = IT2FS(domain4,
                            tri_mf, [kdmedinf,kdmed,kdmedsup, 1, ],
                            tri_mf, [1.02*kdmedinf,kdmed,0.98*kdmedsup,mf_inf_out])
                #IT2FS_plot(S_D,M_D,B_D)

                S_I = IT2FS(domain5,
                            trapezoid_mf, [1.9, 2, 2.5, 3, 1.],
                            trapezoid_mf, [1.9, 2, 0.98*2.5, 3,mf_inf_out])
                B_I = IT2FS(domain5,
                            trapezoid_mf, [3, 3.5, 4, 4.1, 1.],
                            trapezoid_mf, [3, 1.02*3.5, 4, 4.1,mf_inf_out])
                M_I = IT2FS(domain5,
                            tri_mf, [2.5, 3, 3.5, 1, ],
                            tri_mf, [1.02*2.5, 3, 0.98*3.5,mf_inf_out])
                #IT2FS_plot(S_I,M_I,B_I)

                it2fls.add_input_variable("ze")  # E
                it2fls.add_input_variable("dot_ze")  # dot E
                it2fls.add_output_variable("P")
                it2fls.add_output_variable("D")
                it2fls.add_output_variable("alpha")
                '''
                #Original Approach
                it2fls.add_rule([("ze", S), ("dot_ze", N)], [("P", S_P),("D", B_D),("alpha", B_I)])
                it2fls.add_rule([("ze", M), ("dot_ze", N)], [("P", M_P),("D", M_D),("alpha", M_I)])
                it2fls.add_rule([("ze", B), ("dot_ze", N)], [("P", B_P),("D", S_D),("alpha", S_I)])

                it2fls.add_rule([("ze", S), ("dot_ze", Z)], [("P", B_P),("D", B_D),("alpha", M_I)])
                it2fls.add_rule([("ze", M), ("dot_ze", Z)], [("P", B_P),("D", M_D),("alpha", S_I)])
                it2fls.add_rule([("ze", B), ("dot_ze", Z)], [("P", B_P),("D", S_D),("alpha", S_I)])

                it2fls.add_rule([("ze", S), ("dot_ze", P)], [("P", S_P),("D", B_D),("alpha", B_I)])
                it2fls.add_rule([("ze", M), ("dot_ze", P)], [("P", M_P),("D", M_D),("alpha", M_I)])
                it2fls.add_rule([("ze", B), ("dot_ze", P)], [("P", B_P),("D", S_D),("alpha", S_I)])
                
                '''
                # Proposed Approach
                it2fls.add_rule([("ze", S), ("dot_ze", N)], [("P", S_P),("D", M_D),("alpha", B_I)]) 
                it2fls.add_rule([("ze", M), ("dot_ze", N)], [("P", M_P),("D", M_D),("alpha", M_I)]) 
                it2fls.add_rule([("ze", B), ("dot_ze", N)], [("P", B_P),("D", S_D),("alpha", S_I)])

                it2fls.add_rule([("ze", S), ("dot_ze", Z)], [("P", B_P),("D", B_D),("alpha", B_I)])
                it2fls.add_rule([("ze", M), ("dot_ze", Z)], [("P", M_P),("D", M_D),("alpha", S_I)])
                it2fls.add_rule([("ze", B), ("dot_ze", Z)], [("P", B_P),("D", S_D),("alpha", S_I)])

                it2fls.add_rule([("ze", S), ("dot_ze", P)], [("P", B_P),("D", M_D),("alpha", B_I)])
                it2fls.add_rule([("ze", M), ("dot_ze", P)], [("P", B_P),("D", M_D),("alpha", M_I)])
                it2fls.add_rule([("ze", B), ("dot_ze", P)], [("P", B_P),("D", S_D),("alpha", S_I)])
                
                
                return it2fls
            
           
            it2fpid_KM = fuzzySystem("KM")
            c,TR = it2fpid_KM.evaluate({"ze":ze, "dot_ze":dot_ze})
          
            P_ = TR["P"]
            D_ = TR["D"]
            alpha_ = TR["alpha"]

            P = (P_[0] + P_[1]) / 2
            D = (D_[0] + D_[1]) / 2
            alpha = (alpha_[0] + alpha_[1]) / 2
            
            kp_ze = P
            kd_ze = D
            ki_ze = (kp_ze*kp_ze)/(1*alpha*kd_ze)
             
            # Fixed Gains PID for Lateral Error Controller

            #kp_ze = kpmed
            #kd_ze = kdmed 
            #alpha = 3
            #ki_ze = (kp_ze*kp_ze)/(1*alpha*kd_ze)

            # Zigler Nichols PID for Lateral Error Controller

            #kp_ze = 0.6*KU_ze
            #kd_ze = 0.075*KU_ze*TU_ze
            #ki_ze = 1.2*(KU_ze/TU_ze)

            ################################################ IT2-FPID Angular Controller ##########################################################
            self.derivative_phir = (self.phir - self.last_phir)/self.dt
            self.derivative_phir_absolute = (np.abs(self.phir) - np.abs(self.last_phir))/self.dt

            self.derivative_phir = sat(self.derivative_phir,3)
            self.derivative_phir_absolute = sat(self.derivative_phir_absolute,3)

            KU_phir = 6.4
            TU_phir = 1.2

            kpmin_ = 0.32*KU_phir
            kpmax_ = 0.6*KU_phir
            kdmin_ = 0.08*KU_phir*TU_phir
            kdmax_ = 0.15*KU_phir*TU_phir

            kpmax = kpmin_ + 1/5*kpmax_
            kpmin = kpmin_ -1/5*kpmin_
            kdmax = kdmin_ + 1/5*kdmax_
            kdmin = kdmin_ -1/5*kdmin_

            kpmed = (kpmax+kpmin)/2
            kpmedtri = (kpmax-kpmin)/4
            kpmedinf = kpmed-kpmedtri
            kpmedsup = kpmed+kpmedtri

            kdmed = (kdmax+kdmin)/2
            kdmedtri = (kdmax-kdmin)/4
            kdmedinf = kdmed-kdmedtri
            kdmedsup = kdmed+kdmedtri

            phir = np.abs(self.phir)
            dot_phir = self.derivative_phir_absolute
            
            def fuzzySystem_phir(algorithm, algorithm_params=[]):
                it2fls = IT2Mamdani(min_t_norm, max_s_norm, method="Centroid",algorithm=algorithm, algorithm_params=algorithm_params)
                domain1 = np.arange(0, 1.57, 0.01)
                domain2 = np.arange(-3, 3, 0.01)
                domain3 = np.arange(kpmin, kpmax, 0.001)
                domain4 = np.arange(kdmin, kdmax, 0.001)
                domain5 = np.arange(2, 4, 0.01)
                mf_inf = 1

                M = IT2FS(domain1,
                            trapezoid_mf, [0.17,0.29,0.44,0.58, 1.],
                            trapezoid_mf, [0.19,0.31,0.42,0.56,mf_inf])
                S = IT2FS(domain1,
                            trapezoid_mf, [-0.03,0.0,0.19,0.22, 1.],
                            trapezoid_mf, [-0.03,0.0,0.17,0.20,mf_inf])
                B = IT2FS(domain1,
                            trapezoid_mf, [0.29,0.41,1.57,1.58, 1.],
                            trapezoid_mf, [0.31,0.43,1.57,1.58,mf_inf])

                #IT2FS_plot(S,M,B)

                N = IT2FS(domain2,
                            trapezoid_mf, [-3.1, -3, -0.8, 0, 1.],
                            trapezoid_mf, [-3.1, -3, -1.2, -0.0,mf_inf])
                P = IT2FS(domain2,
                            trapezoid_mf, [0, 0.8, 3, 3.1, 1.],
                            trapezoid_mf, [0.0, 1.2, 3, 3.1,mf_inf])
                Z = IT2FS(domain2,
                            tri_mf, [-1.2, 0, 1.2, 1, ],
                            tri_mf, [-0.8, 0, 0.8,mf_inf])
                #IT2FS_plot(N,P,Z)

                S_P = IT2FS(domain3,
                            trapezoid_mf, [kpmin-0.01,kpmin,kpmedinf,kpmed, 1.],
                            trapezoid_mf, [kpmin-0.01,kpmin,0.98*kpmedinf,kpmed,mf_inf])
                B_P = IT2FS(domain3,
                            trapezoid_mf, [kpmed,kpmedsup,kpmax,kpmax+0.01, 1.],
                            trapezoid_mf, [kpmed,1.02*kpmedsup,kpmax,kpmax+0.01,mf_inf])
                M_P = IT2FS(domain3,
                            tri_mf, [kpmedinf,kpmed,kpmedsup, 1, ],
                            tri_mf, [1.02*kpmedinf,kpmed,0.98*kpmedsup,mf_inf])
                #IT2FS_plot(S_P,M_P,B_P)


                S_D = IT2FS(domain4,
                            trapezoid_mf, [kdmin-0.01,kdmin,kdmedinf,kdmed, 1.],
                            trapezoid_mf, [kdmin-0.01,kdmin,0.98*kdmedinf,kdmed,mf_inf])
                B_D = IT2FS(domain4,
                            trapezoid_mf, [kdmed,kdmedsup,kdmax,kdmax+0.01, 1.],
                            trapezoid_mf, [kdmed,1.02*kdmedsup,kdmax,kdmax+0.01,mf_inf])
                M_D = IT2FS(domain4,
                            tri_mf, [kdmedinf,kdmed,kdmedsup, 1, ],
                            tri_mf, [1.02*kdmedinf,kdmed,0.98*kdmedsup,mf_inf])
                #IT2FS_plot(S_D,M_D,B_D)

                S_I = IT2FS(domain5,
                            trapezoid_mf, [1.9, 2, 2.5, 3, 1.],
                            trapezoid_mf, [1.9, 2, 0.98*2.5, 3,mf_inf])
                B_I = IT2FS(domain5,
                            trapezoid_mf, [3, 3.5, 4, 4.1, 1.],
                            trapezoid_mf, [3, 1.02*3.5, 4, 4.1,mf_inf])
                M_I = IT2FS(domain5,
                            tri_mf, [2.5, 3, 3.5, 1, ],
                            tri_mf, [1.02*2.5, 3, 0.98*3.5,mf_inf])
                #IT2FS_plot(S_I,B_I,M_I)

                it2fls.add_input_variable("phir")  
                it2fls.add_input_variable("dot_phir")
                it2fls.add_output_variable("P")
                it2fls.add_output_variable("D")
                it2fls.add_output_variable("alpha")


                '''
                it2fls.add_rule([("phir", S), ("dot_phir", N)], [("P", S_P),("D", B_D),("alpha", B_I)])
                it2fls.add_rule([("phir", M), ("dot_phir", N)], [("P", M_P),("D", M_D),("alpha", M_I)])
                it2fls.add_rule([("phir", B), ("dot_phir", N)], [("P", B_P),("D", S_D),("alpha", S_I)])

                it2fls.add_rule([("phir", S), ("dot_phir", Z)], [("P", B_P),("D", B_D),("alpha", M_I)])
                it2fls.add_rule([("phir", M), ("dot_phir", Z)], [("P", B_P),("D", M_D),("alpha", S_I)])
                it2fls.add_rule([("phir", B), ("dot_phir", Z)], [("P", B_P),("D", S_D),("alpha", S_I)])

                it2fls.add_rule([("phir", S), ("dot_phir", P)], [("P", S_P),("D", B_D),("alpha", B_I)])
                it2fls.add_rule([("phir", M), ("dot_phir", P)], [("P", B_P),("D", M_D),("alpha", M_I)])
                it2fls.add_rule([("phir", B), ("dot_phir", P)], [("P", B_P),("D", S_D),("alpha", S_I)])
                '''
                it2fls.add_rule([("phir", S), ("dot_phir", N)], [("P", S_P),("D", M_D),("alpha", B_I)])
                it2fls.add_rule([("phir", M), ("dot_phir", N)], [("P", M_P),("D", M_D),("alpha", M_I)])
                it2fls.add_rule([("phir", B), ("dot_phir", N)], [("P", B_P),("D", S_D),("alpha", S_I)])

                it2fls.add_rule([("phir", S), ("dot_phir", Z)], [("P", B_P),("D", B_D),("alpha", B_I)])
                it2fls.add_rule([("phir", M), ("dot_phir", Z)], [("P", M_P),("D", M_D),("alpha", S_I)])
                it2fls.add_rule([("phir", B), ("dot_phir", Z)], [("P", B_P),("D", S_D),("alpha", S_I)])

                it2fls.add_rule([("phir", S), ("dot_phir", P)], [("P", B_P),("D", M_D),("alpha", B_I)])
                it2fls.add_rule([("phir", M), ("dot_phir", P)], [("P", B_P),("D", M_D),("alpha", M_I)])
                it2fls.add_rule([("phir", B), ("dot_phir", P)], [("P", B_P),("D", S_D),("alpha", S_I)])
                
                return it2fls

          
            it2fpid_KM = fuzzySystem_phir("KM")
            c, TR = it2fpid_KM.evaluate({"phir":phir, "dot_phir":dot_phir})

            
            P_ = TR["P"]
            D_ = TR["D"]
            alpha_ = TR["alpha"]

            P = (P_[0] + P_[1]) / 2
            D = (D_[0] + D_[1]) / 2
            alpha = (alpha_[0] + alpha_[1]) / 2

            kp_phir = P
            kd_phir = D
            ki_phir = (kp_phir*kp_phir)/(1*alpha*kd_phir)

            # Fixed Gains PID - Angular Error Controller

            #kp_phir = kpmed
            #kd_phir = kdmed 
            #alpha = 3
            #ki_phir = (kp_phir*kp_phir)/(1*alpha*kd_phir)

            # Zigler Nichols PID for Angular Error Controller

            #kp_phir = 0.6*KU_phir
            #kd_phir = 0.075*KU_phir*TU_phir
            #ki_phir = 1.2*(KU_phir/TU_phir)
     
            self.integral_phir += self.phir*self.dt
            
            self.integral_phir = sat(self.integral_phir,0.25) #Anti windup

            P_term = kp_phir * self.phir
            I_term = self.integral_phir*ki_phir
            D_term = kd_phir * self.derivative_phir

            vel_phir = P_term + I_term + D_term
            #print('vel_phir',vel_phir)

            vel_phir = sat(vel_phir,4)
            
            ############################################### Attitude PID Controller #########################################
            tempo = time.time() - t1

            self.alt = 14.5 - altitudeZ

            kp_alt = 0.5
            ki_alt = 0.001
            kd_alt = 0.1
            dt = 0.1

            self.integral_alt += self.alt
            self.integral_alt = self.integral_alt*self.dt
            self.derivative_alt = (self.alt - self.last_alt)/self.dt
            self.last_alt = self.alt
            vel_alt =  kp_alt * self.alt + ki_alt * self.integral_alt + kd_alt * self.derivative_alt
            ####################################################################################################################

            self.integral_ze += self.ze*self.dt
            self.integral_ze = sat(self.integral_ze,0.25) #Anti windup

            if self.ready is False  : #Avoid oscilations before the test begins
                kp_ze = 0.3
                ki_ze = 0
                kd_ze = 0
            
            P_term = kp_ze * self.ze
            I_term = ki_ze * self.integral_ze
            D_term = kd_ze * self.derivative_ze
            
            vel_ze = P_term + I_term + D_term
            #print('vel_ze',vel_ze)
            
            vel_ze = sat(vel_ze,1.5)

            vel_lineary = -vel_ze
            #vel_linearx = 0.4

            # You can adjust the number of samples 

            if TEST_MODE == 'lateral' :
                samples = 300
            if TEST_MODE == 'angular' :
                samples = 300
            if TEST_MODE == 'following' :
                samples = 1600

            if self.ready == True and self.exec < 51 :

                if self.cont_med < samples + 1:
                    self.soma_ze += np.abs(self.ze)
                    media_ze = self.soma_ze/self.cont_med
                    self.soma_phir += np.abs(self.phir)
                    media_phir = self.soma_phir/self.cont_med
                    #print(self.ze)
                    
                    tempo = round(tempo,7)
                    vel_ze = round(vel_ze,4)
                    vel_phir = round(vel_phir,4)
                    self.phir = round(self.phir,5)

                    kp_ze = round(kp_ze,5)
                    kd_ze = round(kd_ze,5)
                    ki_ze = round(ki_ze,5)

                    kp_phir = round(kp_phir,5)
                    kd_phir = round(kd_phir,5)
                    ki_phir = round(ki_phir,5)
            
                    home_folder = os.getenv('HOME')
                    '''
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/vphi.txt", "a") as output:
                            output.write("%s \n" % vel_phir)
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/phir.txt", "a") as output:
                            output.write("%s \n" % self.phir)
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/ze.txt", "a") as output:
                            output.write("%s \n" % self.ze)           
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/vely.txt", "a") as output:
                            output.write("%s \n" % vel_ze)               
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/tempo.txt", "a") as output:
                            output.write("%s \n" % tempo)
                    
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/kp_phir.txt", "a") as output:
                            output.write("%s \n" % kp_phir)           
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/kd_phir.txt", "a") as output:
                            output.write("%s \n" % kd_phir)               
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/ki_phir.txt", "a") as output:
                            output.write("%s \n" % ki_phir) 
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/kp_ze.txt", "a") as output:
                            output.write("%s \n" % kp_ze)           
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/kd_ze.txt", "a") as output:
                            output.write("%s \n" % kd_ze)               
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/ki_ze.txt", "a") as output:
                            output.write("%s \n" % ki_ze)
                    
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/derivative_phir.txt", "a") as output:
                            output.write("%s \n" % self.derivative_phir_absolute) 
                    with open(home_folder+"/catkin_ws/src/it2fpid_ltc/Results/derivative_ze.txt", "a") as output:
                            output.write("%s \n" % self.derivative_ze_absolute) 
                    '''

                self.cont_med = self.cont_med + 1

            if self.cont_med == samples + 1:

                self.cont_med = 1
                self.acc_med += media_ze
                self.acc_med_a = self.acc_med/self.exec

                self.acc_med_phir += media_phir
                self.acc_med_b = self.acc_med_phir/self.exec
                
                print('Execution:',self.exec)
                print('Mean lateral error:',media_ze) # Mean of current simulation
                print('Mean accumulated lateral error:',self.acc_med_a) # Mean of all simulations
                #print('Mean angular error:',media_phir)
                #print('Mean accumulated angular error',self.acc_med_b)
                self.soma_ze = 0
                self.soma_phir = 0
                self.ready = False
                self.exec = self.exec + 1
                self.Reached = False

            # For line tracking tests

            #vel_lineary = 0

            if TEST_MODE == 'lateral' :
                vel_linearx = 0 
            if TEST_MODE == 'angular' :
                if self.ready == True : 
                    vel_lineary = 0
                else :
                    vel_phir = 0
            if TEST_MODE == 'following' :
                if self.ready == True :
                    vel_linearx = 0.4
                else :
                    vel_linearx = 0
                    vel_phir = 0

            stamp = rospy.get_rostime()
            self.twistS.header.stamp = stamp
            self.twistS.header.frame_id = "base_link"  
            self.twistS.header.seq = 1

            #self.twistS.twist.angular.z = 0

            self.twistS.twist.angular.z = vel_phir
            self.twistS.twist.linear.z = vel_alt
            self.twistS.twist.linear.x = vel_linearx*np.cos(yaw) - vel_lineary*np.sin(yaw)
            self.twistS.twist.linear.y = vel_linearx*np.sin(yaw) + vel_lineary*np.cos(yaw)
            self.cmd_vel_pub.publish(self.twistS)
           
            self.last_derivative_ze = self.derivative_ze
            self.last_ze = self.ze
            self.last_phir = self.phir

            self.rate.sleep()   
            self.dt = time.time() - t
            #print('dt',self.dt)

            # UNCOMMENT IF YOU WANT TO VISUALIZE THE CAMERA FRAME WITH THE CALCULATED TRAJECTORY
            '''
            cv2.imshow('a',frame) 
            cv2.waitKey(1)
            '''

       
if __name__ == '__main__':
   rospy.init_node('offboard_ros_node',anonymous=True)
   try:

      detector = Follower()
      
      rospy.spin()
      

   except rospy.ROSInterruptException:
      self.out.release()
      rospy.loginfo("Detector node terminated.")
