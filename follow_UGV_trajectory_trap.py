import sys # argv, argc
import datetime
import rospy
import math
import numpy as np
# Libraries with the messages structures used by ros services and topics
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from mavros_msgs.msg import State
from geometry_msgs.msg import Pose, Twist, Vector3, Transform, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Int8

# Libraries for the transformations between frames
import tf
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion
import tf2_ros
from tf2_ros import TransformListener
from trajectory import Trapezoidal

class Publisher(object):
    """ PUBLISHER / SUBSCRIBER """

    def __init__(self,ros_node):
        np.set_printoptions(precision=3)

        rosNode = ros_node

        #Transforms
        self.listener = tf.TransformListener()

        ## Subscribers ##
        self.sub_marker = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback_marker)
        self.sub_State = rospy.Subscriber("/mavros/state", State, self.check_Remote_mode)
        self.sub_setpoint = rospy.Subscriber("/uav_setpoint", Twist, self.callback_uav_setpoint)
        self.sub_uav_mode = rospy.Subscriber("/uav_mode", Int8, self.callback_uav_mode)

        #self.sub_odom = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.callback_odom)
        ## Publishers ##
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)

        self.pub_obs_rpy = rospy.Publisher('/observations/rpy', Float64MultiArray, queue_size=1)
        self.pub_obs_xyz = rospy.Publisher('/observations/xyz', Float64MultiArray, queue_size=1)
        self.pub_xyzy = rospy.Publisher('/estimation/xyzy', Twist, queue_size=1)

        ## Iris Variables ##

        self.obs = np.array([0,0,0])
        self.obsp = np.array([0,0,0])
        self.obsRPY = np.array([0,0,0])
        self.mean_quat_prev = np.array([0,0,0,0])
        self.hasObs = False
        self.firstObs = False
        self.hasObsRPY = False
        self.firstObsRPY = False
        self.current_state = State()
        self.prev_state = State()

        self.hz = 10.0      # Frequency
        self.T = 1./self.hz # Control period
        
        self.cmd = np.zeros((4,))
        
        self.rate = rospy.Rate(self.hz)

        self.RPY = np.zeros((5,3)) # RPY for each fidutial + mean value using quaternions
        self.XYZ = np.zeros((4,3)) # np.zeros((4,3))

        # Message for publishing XYZ Yaw estimations
        self.msgXYZY = Twist()

        # Taking your use case you want to define a 2D matrix of size 4x3=12,
        # This is almost always zero there is no empty padding at the start of your data

        self.msgRPY = Float64MultiArray()       
        self.msgRPY.layout.data_offset = 0

        self.msgXYZ = Float64MultiArray()       
        self.msgXYZ.layout.data_offset = 0

        # create two dimensions in the dim array
        # self.msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        self.msgRPY.layout.dim = [MultiArrayDimension(),]
        self.msgRPY.layout.dim[0].label = "Fiducial"
        self.msgRPY.layout.dim[0].size = 15
        self.msgRPY.layout.dim[0].stride = 15

        self.msgXYZ.layout.dim = [MultiArrayDimension(),]
        self.msgXYZ.layout.dim[0].label = "Fiducial"
        self.msgXYZ.layout.dim[0].size = 12
        self.msgXYZ.layout.dim[0].stride = 12

        self.dict={'946':0,'227':1,'168':2,'55':3}

        # Profile
        dq_max = np.array([0.8, 0.8, 0.35, 0.5])
        ddq_max = np.array([0.4, 0.4, 0.175, 0.25])
        self.tp = Trapezoidal(dq_max, ddq_max, True) # Perform trajectory planning in fixed frame
        # Controller
        self.Kp = np.array([0.35, 0.35, 0.35, 0.25]) # Proportional gain
        self.Ki = np.array([0.1, 0.1, 0.1, 0.1]) # Integral gain
        self.error = np.zeros_like(self.Kp) # Error
        self.ierror = np.zeros_like(self.Ki) # Integral error
        # self.pose_sp = np.array([-0.8, -0.8, 3.5,np.pi/2.])
        # self.pose_sp = np.array([0.,0.,1.,np.pi/2])
        # self.pose_sp = np.array([0.,0.,1.,0.]) # Set point in body frame
        # self.pose_sp = np.array([0.,0.,3.7,np.pi]) # Set point in body frame
        # self.pose_sp = np.array([0.2, 0.5, 1., 0.]) # Set point in body frame
        # self.pose_sp = np.array([0.2, 0.5, 3.7, np.pi]) # Set point in body frame
        # self.pose_sp = np.array([1.0, 1.0, 3.7, np.pi/2.]) # Set point in body frame
        # self.pose_sp = np.array([0.0, 1.0, 3.8, 0.*np.pi/4.0]) # Set point in body frame
        self.pose_sp = np.array([0.0, 0.0, 1.0, 0.*np.pi/4.0]) # Set point in body frame
        # self.pose_sp = np.array([-1.0, -1.0, 3.7, 0.0]) # Set point in body frame
        self.pose = np.zeros_like(self.pose_sp) # UAV pose provided by the aruco marks
        self.mode = 0 # 1: autonomous mode, 0: teleoperated mode
        self.prev_mode = 1
        self.reset_state()       
        self.reset_time()
        self.reset_wdt()

    def reset_time(self):
        self.time = 0.0
    
    def reset_wdt(self):
        self.wdt = 0.5 # watchdog timer for observations

    def reset_error(self, cmd=[]):
        self.error = np.zeros_like(self.Kp) # Integral error
        if cmd==[]:            
            self.ierror = np.zeros_like(self.Ki) # Integral error
        else:
            for i, Ki in enumerate(self.Ki):
                if Ki > 0.:
                    self.ierror[i] = cmd[i]/Ki
                else:
                    self.ierror[i] = 0.
        print('reset error', self.error)
        print('reset ierror', self.ierror)

    def reset_state(self):
        self.initialized = False # True when initial estimation for XYZ_YAW is obtained
        self.landing = False # True if the landing sequence is initiated        

    def start(self):

        while( not self.check_landing() and not rospy.is_shutdown()):

            if(self.firstObs == True and self.current_state.mode == "GUIDED" and self.mode==1):
                if (self.prev_state.mode != self.current_state.mode): 
                    print("Mode is: ", self.current_state.mode)
                    self.prev_state = self.current_state
                if(self.hasObs and self.hasObsRPY and self.current_state.mode == "GUIDED" and self.mode==1):
                    self.reset_wdt()                    
                    self.pose = np.array([-self.obs[0], -self.obs[1], -self.obs[2], -self.obsRPY[2]])                                      
                    if self.time >= 0. and self.time < self.T: # Compute position-speed profile and first sample
                        if self.initialized:
                            self.tp.compute_profile(self.pose, self.pose_sp)
                            print('Setpoint navigation')
                        else:
                            # self.tp.compute_profile(self.pose,  self.tp.rotate(self.pose, self.pose[3]))
                            self.tp.compute_profile(self.pose)
                            print('Aruco Locked')
                        q, dq = self.tp.generate_q_dq(self.time)
                        self.cmd = dq
                    elif self.time > 0.: # Compute next samples
                        q, dq = self.tp.generate_q_dq(self.time)
                        self.error = self.tp.wrap(q-self.pose)
                        self.ierror += self.error*(self.time-self.prev_time)
                        self.cmd = dq + self.Kp*self.error + self.Ki*self.ierror
                    self.vel_xyz_yaw(self.cmd)
                    self.prev_time = self.time
                else: # If no observation received -> decrease watchdog timer and if negative stop the drone and reset time
                    self.wdt-=self.T
                    if self.wdt<0.:
                        self.cmd = np.zeros((4,))
                        self.vel_xyz_yaw(self.cmd)
                        self.reset_time()
                        self.reset_error()
                        self.reset_state()
                self.time+=self.T
                # print(self.time)

            if (self.prev_state.mode != self.current_state.mode):
                # if flight mode changed, re initialize PID params
                print("Mode is: ", self.current_state.mode)
                self.prev_state = self.current_state
                self.cmd = np.zeros((4,))
                self.reset_time()
                self.reset_wdt()
                self.reset_error()
                self.reset_state()          

            self.rate.sleep()
        
        return 'landed'
    
    def check_landing(self):

        # HOW TO KNOW IF IT LANDED
        
        z = 0.4
        if(z <= 0.3):
            return True

        return False
    
    def check_Remote_mode(self, state):
        #check to see if remote is in guided mode
        self.current_state=state

    def callback_uav_mode(self, msg):        
        if msg.data==1:
            self.mode=1
        elif msg.data==0:
            self.mode=0
        if self.prev_mode != self.mode:
            self.prev_mode = self.mode
            self.reset_time()
            self.reset_wdt()
            self.reset_error()
            self.reset_state()
            print('mode',self.mode,' reset t w e s')

    def callback_uav_setpoint(self, msg):
        ''' Receive uav setpoint wrt aruco reference frame
        TODO check set_point values to be in the FOV of the camera
        '''
        self.pose_sp = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z])
        print('setpoint',self.pose_sp)
        print('cmd',self.cmd)
        self.reset_time()
        self.reset_error(self.cmd)
        self.reset_wdt()
        self.reset_state()
        self.initialized = True
        
    """
    > Markers Callback
    """
    def callback_marker(self, msg):
        
        observations = np.zeros( (len(msg.transforms) , 4) )
        obsQuat = np.zeros( (len(msg.transforms) , 5) )
        i = 0
        i_RPY = 0
        self.hasObs = False
        self.hasObsRPY = False

        # Parse of the fiducial_transforms message
        for marker in msg.transforms:

            try:
                id_str=str(marker.fiducial_id)
                marker_id = "fiducial_T_" + id_str
                
                (trans, rot) = self.listener.lookupTransform('base_stablized', marker_id, rospy.Time(0))
                # (trans, rot) = self.listener.lookupTransform('base_link', marker_id, rospy.Time(0))
                # print("Compute observation")
                # Store the measurements of each marker
                observations[i] = np.array([marker.fiducial_id,
                    trans[0],
                    trans[1],
                    trans[2] ]) #The x is pointing downwards in the camera frame, therefore it's the z
                self.XYZ[self.dict.get(id_str),:]=np.array(observations[i,1:])
                # print(observations[i,1:])

                # For proper detection under 3.5 m height and 848x480 camera resolution, fiducial_area should be greater than 1000.
                if marker.fiducial_area>900.:#marker.fiducial_id==946:# or marker.fiducial_id==227:#marker.fiducial_area>1:#650.0:# and (marker.fiducial_id==946 or marker.fiducial_id==227):
                    obsQuat[i_RPY]=np.array([marker.fiducial_id,
                        rot[0],
                        rot[1],
                        rot[2],
                        rot[3] ])                    
                    self.hasObsRPY = True
                    self.firstObsRPY = True
                    # print(i_RPY,marker.fiducial_id, marker.fiducial_area)
                    # print('RPY_nvg',np.array(euler_from_quaternion(obsQuat[i_RPY,1:]))*180.0/np.pi)
                    self.RPY[self.dict.get(id_str),:]=np.array(euler_from_quaternion(obsQuat[i_RPY,1:]))*180.0/np.pi
                    i_RPY+=1
                self.hasObs = True
                self.firstObs = True
                i+=1
    
            
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print ("Fail", e)
                # print ("Fail lookupTransform")
        

        # Obtain the average of all the measurements taken in this iteration
        if(i!=0):       # If matrix is not empty
            average_x = sum(observations[:,1]) / len(observations)
            average_y = sum(observations[:,2]) / len(observations)
            average_z = sum(observations[:,3]) / len(observations)    

            self.obs = np.array([average_x,
                                average_y,
                                average_z])
            # print(self.obs)
            self.msgXYZ.data = self.XYZ.reshape([12])
            self.pub_obs_xyz.publish(self.msgXYZ)  
            # print(self.obs)
        
        if(i_RPY!=0):
            # Average quaternions to RPY
            if (i_RPY==1):
                mean_quat = obsQuat[0,1:]
            else:
                Aq= np.zeros((4,4))
                for j in range(i_RPY):
                    obsQuat[j,1:]=obsQuat[j,[4,1,2,3]]
                    if obsQuat[j,1]<0:
                        obsQuat[j,1:]=-1.0*obsQuat[j,1:]
                    Aq=np.matmul(obsQuat[j,1:].reshape((4,1)),obsQuat[j,1:].reshape((1,4)))+Aq
                Aq=1.0/i*Aq
                Aq_eva, Aq_eve = np.linalg.eig(Aq)
                if np.isrealobj(Aq_eve[:,0]):
                    mean_quat = Aq_eve[[1,2,3,0],0]                    
                else:
                    # mean_quat = obsQuat[0,[1,2,3,0]]
                    mean_quat = self.mean_quat_prev
                    print('Complex, using previous value')
            self.RPY[4,:]=np.array(euler_from_quaternion(mean_quat))*180.0/np.pi # Store mean value
            self.mean_quat_prev = mean_quat.copy()
            self.obsRPY = euler_from_quaternion(mean_quat)            
            self.msgRPY.data = self.RPY.reshape([15])
            self.pub_obs_rpy.publish(self.msgRPY)  

            self.msgXYZY.linear.x = -self.obs[0]
            self.msgXYZY.linear.y = -self.obs[1]
            self.msgXYZY.linear.z = -self.obs[2]
            self.msgXYZY.angular.z = -self.obsRPY[2]
            self.pub_xyzy.publish(self.msgXYZY)
            
    def vel_xyz_yaw(self,target_angles):
        msg_iris = Twist()
        msg_iris.linear.x = target_angles[0]
        msg_iris.linear.y = target_angles[1]
        msg_iris.linear.z = target_angles[2]
        msg_iris.angular.z = target_angles[3]
        self.pub_vel.publish(msg_iris)

        
def main():
    global ros_node, publisher
    ros_node = rospy.init_node('drone_controller')

    #Other files
    publisher = Publisher(ros_node)    
    publisher.start()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
