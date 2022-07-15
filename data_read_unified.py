import rosbag
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as mp
from std_msgs.msg import Float32
# from scipy.integrate import cumtrapz
from numpy.polynomial import Polynomial

def rotate_inv(q):
    if len(q.shape) == 2:
        return np.vstack((
            q[:,0]*np.cos(q[:,3]-q[0,3])-q[:,1]*np.sin(q[:,3]-q[0,3]),
            q[:,0]*np.sin(q[:,3]-q[0,3])+q[:,1]*np.cos(q[:,3]-q[0,3]),
            q[:,2],
            q[:,3]
        )).T
    else:
        return np.array([
            q[0]*np.cos(-q[3])-q[1]*np.sin(-q[3]),
            q[0]*np.sin(-q[3])+q[1]*np.cos(-q[3]),
            q[2],
            q[3]
            ])   

# bagdir = './bags/'
# bagdir = './Simulation/bags/'
bagdir = '../Simulation/bags/'
# bagfile = '2022-06-22-16-07-11.bag' # trapezoidal profile + proportional control
# bagfile = '2022-06-23-10-11-12.bag' # trapezoidal profile only
# bagfile = '2022-06-23-10-17-35.bag'  # trapezoidal profile only
# bagfile = '2022-06-23-10-25-32.bag' #  trapezoidal profile only with xyz yaw estimation
# bagfile = '2022-06-23-10-57-55.bag' #  trapezoidal profile only with xyz yaw estimation and inital position averaged
# bagfile = '2022-06-23-11-14-54.bag'
# bagfile = '2022-06-23-11-19-11.bag'
# bagfile = '2022-06-23-11-23-46.bag'
# bagfile = '2022-06-23-11-35-20.bag'
# bagfile = '2022-06-23-11-44-15.bag'
# bagfile = '2022-06-23-13-49-13.bag'
# bagfile = '2022-06-23-14-02-19.bag'
# bagfile = '2022-06-23-14-12-31.bag'
# bagfile = '2022-06-23-14-51-22.bag'
# bagfile = '2022-06-23-15-02-35.bag'
# bagfile = '2022-06-23-15-23-05.bag'
# bagfile = '2022-06-24-12-55-34.bag'
# bagfile = '2022-06-24-15-38-01.bag'
# bagfile = '2022-06-27-15-10-37.bag'
# bagfile = '2022-06-27-15-26-57.bag'
# bagfile = '2022-06-27-15-31-33.bag'
# bagfile = '2022-06-27-15-36-22.bag'
# bagfile = '2022-06-27-16-00-17.bag'
# bagfile = '2022-06-28-10-47-03.bag'
# bagfile = '2022-06-28-10-51-32.bag'

# bagfile = '2022-06-28-12-14-42.bag'
# bagfile = '2022-06-28-12-19-43.bag'
# bagfile = '2022-06-28-12-27-07.bag'
# bagfile = '2022-06-28-12-32-08.bag'
# bagfile = '2022-06-28-12-43-10.bag' # Smaller gains
# bagfile = '2022-06-28-12-55-48.bag'
# bagfile = '2022-06-28-12-59-14.bag'
# bagfile = '2022-06-28-13-01-34.bag'
# bagfile = '2022-06-28-14-21-43.bag'
# bagfile = '2022-06-28-14-42-10.bag'
# bagfile = '2022-06-28-14-47-38.bag'
# bagfile = '2022-06-28-14-55-12.bag'
# bagfile = '2022-06-28-14-58-00.bag'
# bagfile = '2022-06-28-15-09-48.bag'
# bagfile = '2022-06-28-16-34-38.bag'
# bagfile = '2022-07-01-15-43-27.bag'
# bagfile = '2022-07-04-12-30-48.bag'

# bagdir = '../bags_test_landing/'
# bagfile = '2022-07-05-19-38-14.bag'
# bagfile = '2022-07-05-19-49-18.bag'


bagdir = './bags/'
# bagfile = '2022-07-15-13-54-59.bag'
bagfile = '2022-07-15-17-10-45.bag'

bag = rosbag.Bag(bagdir+bagfile)

topic0 = '/mavros/setpoint_velocity/cmd_vel_unstamped'
topic1 = '/estimation/xyzy'
topic2 = '/fiducial_transforms'
topic3 = '/mavros/state'
topic4 = '/uav_mode'

# T = 0.01

num_msgs = bag.get_message_count(topic_filters=topic0)
cmd_vel = np.zeros((num_msgs, 4),dtype=Float32)
t = np.zeros((num_msgs, ),dtype=Float32)

num_msgs_x = bag.get_message_count(topic_filters=topic1)
xyzy = np.zeros((num_msgs_x, 4),dtype=Float32)
t_x = np.zeros((num_msgs_x, ),dtype=Float32)

num_msgs_fi = bag.get_message_count(topic_filters=topic2)
fi_946 = -np.ones((num_msgs_fi, 10),dtype=Float32)
fi_227 = -np.ones((num_msgs_fi, 10),dtype=Float32)
fi_168 = -np.ones((num_msgs_fi, 10),dtype=Float32)
fi_55  = -np.ones((num_msgs_fi, 10),dtype=Float32)
t_fi = np.zeros((num_msgs_fi, ),dtype=Float32)

num_msgs_state = bag.get_message_count(topic_filters=topic3)
state = np.zeros((num_msgs_state, ),dtype=Float32)
t_state = np.zeros((num_msgs_state, ),dtype=Float32)

num_msgs_uav_mode = bag.get_message_count(topic_filters=topic4)
uav_mode = np.zeros((num_msgs_uav_mode, ),dtype=Float32)
t_uav_mode = np.zeros((num_msgs_uav_mode, ),dtype=Float32)


modes = {'LOITER': 0, 'GUIDED': 1, 'RTL': 2, 'STABILIZE': 3, 'ALT_HOLD': 4}

for i, (topic, msg, time) in enumerate(bag.read_messages(topics=topic3)):
    state[i] = modes[msg.mode]
    t_state[i] = time.to_time()

for i, (topic, msg, time) in enumerate(bag.read_messages(topics=topic4)):
    uav_mode[i] = msg.data
    t_uav_mode[i] = time.to_time()

for i, (topic, msg, time) in enumerate(bag.read_messages(topics=topic0)):
    cmd_vel[i,:] = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z ])
    t[i] = time.to_time()
#    yaw_rate[i] = msg.data

for i, (topic, msg, time) in enumerate(bag.read_messages(topics=topic1)):
    xyzy[i,:] = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z ])
    t_x[i] = time.to_time()

xyzy_rot = rotate_inv(xyzy.astype(float))

for i, (topic, msg, time) in enumerate(bag.read_messages(topics=topic2)):
    for j in range(len(msg.transforms)):
        # print(msg.transforms[j].fiducial_id)
        mt = msg.transforms[j].transform.translation
        mr = msg.transforms[j].transform.rotation
        fiducial_data = np.array([mt.x, mt.y, mt.z, mr.x, mr.y, mr.z, mr.w, 
        msg.transforms[j].image_error, msg.transforms[j].object_error, msg.transforms[j].fiducial_area])
        fid = msg.transforms[j].fiducial_id
        if fid==946:
            fi_946[i,:] = fiducial_data
        elif fid==227:
            fi_227[i,:] = fiducial_data
        elif fid==168:
            fi_168[i,:] = fiducial_data
        elif fid==55:
            fi_55[i,:] = fiducial_data
    t_fi[i]=msg.header.stamp.to_time()

area=fi_946[:,-1].astype(float)
z=fi_946[:,2].astype(float)
idx=area>0

p_fitted = Polynomial.fit(z[idx],area[idx],deg=2)
z_int, area_int = p_fitted.linspace()

az_fitted = Polynomial.fit(area[idx],area[idx]*z[idx],deg=1)
a_int2, az_int2 = az_fitted.linspace()

mp.figure()
mp.subplot(221)
mp.plot(t_fi-t_fi[0], fi_946[:,0],'.')
mp.plot(t_fi-t_fi[0], fi_227[:,0],'.')
mp.plot(t_fi-t_fi[0], fi_168[:,0],'.')
mp.plot(t_fi-t_fi[0], fi_55[:,0],'.')
mp.legend(('946','227','168','55'),loc='best')
mp.xlabel('t (s)')
mp.ylabel('x (m)')
mp.grid('on')

mp.subplot(222)
mp.plot(t_fi-t_fi[0], fi_946[:,1],'.')
mp.plot(t_fi-t_fi[0], fi_227[:,1],'.')
mp.plot(t_fi-t_fi[0], fi_168[:,1],'.')
mp.plot(t_fi-t_fi[0], fi_55[:,1],'.')
mp.legend(('946','227','168','55'),loc='best')
mp.xlabel('t (s)')
mp.ylabel('y (m)')
mp.grid('on')

mp.subplot(223)
mp.plot(t_fi-t_fi[0], fi_946[:,2],'.')
mp.plot(t_fi-t_fi[0], fi_227[:,2],'.')
mp.plot(t_fi-t_fi[0], fi_168[:,2],'.')
mp.plot(t_fi-t_fi[0], fi_55[:,2],'.')
mp.legend(('946','227','168','55'),loc='best')
mp.xlabel('t (s)')
mp.ylabel('z (m)')
mp.grid('on')

mp.figure()
mp.subplot(221)
mp.plot(t_fi-t_fi[0], fi_946[:,3],'.')
mp.plot(t_fi-t_fi[0], fi_227[:,3],'.')
mp.plot(t_fi-t_fi[0], fi_168[:,3],'.')
mp.plot(t_fi-t_fi[0], fi_55[:,3],'.')
mp.legend(('946','227','168','55'),loc='best')
mp.xlabel('t (s)')
mp.ylabel('q_x')
mp.grid('on')
mp.subplot(222)
mp.plot(t_fi-t_fi[0], fi_946[:,4],'.')
mp.plot(t_fi-t_fi[0], fi_227[:,4],'.')
mp.plot(t_fi-t_fi[0], fi_168[:,4],'.')
mp.plot(t_fi-t_fi[0], fi_55[:,4],'.')
mp.legend(('946','227','168','55'),loc='best')
mp.xlabel('t (s)')
mp.ylabel('q_y')
mp.grid('on')
mp.subplot(223)
mp.plot(t_fi-t_fi[0], fi_946[:,5],'.')
mp.plot(t_fi-t_fi[0], fi_227[:,5],'.')
mp.plot(t_fi-t_fi[0], fi_168[:,5],'.')
mp.plot(t_fi-t_fi[0], fi_55[:,5],'.')
mp.legend(('946','227','168','55'),loc='best')
mp.xlabel('t (s)')
mp.ylabel('q_z')
mp.grid('on')
mp.subplot(224)
mp.plot(t_fi-t_fi[0], fi_946[:,6],'.')
mp.plot(t_fi-t_fi[0], fi_227[:,6],'.')
mp.plot(t_fi-t_fi[0], fi_168[:,6],'.')
mp.plot(t_fi-t_fi[0], fi_55[:,6],'.')
mp.legend(('946','227','168','55'),loc='best')
mp.xlabel('t (s)')
mp.ylabel('q_w')
mp.grid('on')

mp.figure()
mp.subplot(221)
# mp.plot(z[idx], np.sqrt(area[idx]),'.')
mp.plot(z[idx], area[idx],'.')
# mp.plot(z_int, np.sqrt(area_int),'.')
mp.plot(az_int2/a_int2, az_int2,'.')
mp.grid('on')
mp.xlabel('z')
mp.ylabel('area')
mp.subplot(222)
mp.plot(fi_946[:,2], fi_946[:,-2],'.')
mp.grid('on')
mp.xlabel('z')
mp.ylabel('object_error')
mp.subplot(223)
mp.plot(fi_946[:,2], fi_946[:,-3],'.')
mp.grid('on')
mp.xlabel('z')
mp.ylabel('image_error')
mp.subplot(224)
mp.plot(fi_946[:,0], fi_946[:,1],'.')
mp.grid('on')
mp.xlabel('x')
mp.ylabel('y')
mp.axis('equal')

mp.figure()
t_max = np.max([t_state[-1]-t_state[0], t[-1]-t_state[0], t_x[-1]-t_x[0]])
mp.subplot(211)

if num_msgs_uav_mode > 0:
    mp.plot(t-t_state[0],cmd_vel,'.-',t_uav_mode-t_state[0],uav_mode)
    mp.legend(('v_x','v_y','v_z','\omega_z','uav_mode'))
else:
    mp.plot(t-t_state[0],cmd_vel,'.-')
    mp.legend(('v_x','v_y','v_z','\omega_z'))

mp.axis([0,t_max,-1, 1])    
mp.grid(True)
mp.subplot(212)
mp.plot(t_x-t_state[0],xyzy,'.-')
if num_msgs_uav_mode > 0:
    mp.plot(t_state-t_state[0], state,t_uav_mode-t_state[0],uav_mode,t_x-t_state[0],np.ones_like(xyzy)*np.pi,t_x-t_state[0],-np.ones_like(xyzy)*np.pi)
    mp.legend(('x','y','z','yaw','state','mode'))    
else:
    mp.plot(t_state-t_state[0], state,t_x-t_state[0],np.ones_like(xyzy)*np.pi,t_x-t_state[0],-np.ones_like(xyzy)*np.pi)
    mp.legend(('x','y','z','yaw','state'))    
mp.axis([0,t_max,-4, 5])    
mp.grid(True)
mp.show()

# mp.figure()
# t_max = np.max([t[-1]-t[0], t_x[-1]-t_x[0]])
# mp.subplot(211)
# mp.plot(t-t[0],cmd_vel,'.-')
# mp.legend(('v_x','v_y','v_z','\omega_z'))
# mp.axis([0,t_max,-1, 1])    
# mp.grid(True)
# mp.subplot(212)
# mp.plot(t_x-t_x[0],xyzy,'.-',t_x-t_x[0],np.ones_like(xyzy)*np.pi,t_x-t_x[0],-np.ones_like(xyzy)*np.pi)
# mp.legend(('x','y','z','yaw'))    
# mp.axis([0,t_max,-4, 5])    
# mp.grid(True)
# mp.show()

mp.figure()
ax=mp.axes(projection='3d')
ax.plot3D(xyzy_rot[:,0],xyzy_rot[:,1],xyzy_rot[:,2],'.')
ax.plot3D(xyzy_rot[0:10,0],xyzy_rot[0:10,1],xyzy_rot[0:10,2],'r')
mp.xlabel('x')
mp.ylabel('y')
mp.grid('on')
mp.axis('equal')
mp.show()

mp.figure()
mp.subplot(211)
mp.plot(xyzy[:,0],xyzy[:,1],'.')
mp.plot(xyzy_rot[:,0],xyzy_rot[:,1],'.')
mp.legend(('mobile','fixed'))
mp.xlabel('x')
mp.ylabel('y')
mp.grid('on')
mp.axis('equal')

mp.subplot(212)
mp.plot(xyzy[:,0],xyzy[:,2],'.')
mp.plot(xyzy_rot[:,0],xyzy_rot[:,2],'.')
mp.legend(('mobile','fixed'))
mp.xlabel('x')
mp.ylabel('z')
mp.grid('on')
mp.axis('equal')

mp.show()

# #mp.plot(t,yaw,t,yaw_t)