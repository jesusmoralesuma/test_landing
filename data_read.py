import rosbag
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as mp
from std_msgs.msg import Float32
# from scipy.integrate import cumtrapz
from numpy.polynomial import Polynomial

def rotate_inv(q):
    return np.vstack((
        q[:,0]*np.cos(q[:,3]-q[0,3])-q[:,1]*np.sin(q[:,3]-q[0,3]),
        q[:,0]*np.sin(q[:,3]-q[0,3])+q[:,1]*np.cos(q[:,3]-q[0,3]),
        q[:,2],
        q[:,3]
    )).T

bagdir = './bags/'
# bagfile = '2022-06-30-09-49-49.bag'
bagfile = '2022-06-30-09-53-53.bag'

bag = rosbag.Bag(bagdir+bagfile)

topic0 = '/mavros/setpoint_velocity/cmd_vel_unstamped'
topic1 = '/estimation/xyzy'
topic2 = '/fiducial_transforms'
topic3 = '/mavros/state'

# T = 0.01

num_msgs = bag.get_message_count(topic_filters=topic0)
cmd_vel = np.zeros((num_msgs, 4),dtype=Float32)
t = np.zeros((num_msgs, ),dtype=Float32)

num_msgs_x = bag.get_message_count(topic_filters=topic1)
xyzy = np.zeros((num_msgs_x, 4),dtype=Float32)
t_x = np.zeros((num_msgs_x, ),dtype=Float32)

num_msgs_fi = bag.get_message_count(topic_filters=topic2)
fi_946 = np.zeros((num_msgs_fi, 10),dtype=Float32)
t_fi = np.zeros((num_msgs_fi, ),dtype=Float32)

num_msgs_state = bag.get_message_count(topic_filters=topic3)
state = np.zeros((num_msgs_state, ),dtype=Float32)
t_state = np.zeros((num_msgs_state, ),dtype=Float32)

modes = {'LOITER': 0, 'GUIDED': 1, 'RTL': 2}

for i, (topic, msg, time) in enumerate(bag.read_messages(topics=topic3)):
    state[i] = modes[msg.mode]
    t_state[i] = time.to_time()


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
        if msg.transforms[j].fiducial_id==946:
            mt = msg.transforms[j].transform.translation
            mr = msg.transforms[j].transform.rotation
            fi_946[i,:] = np.array([mt.x, mt.y, mt.z, mr.x, mr.y, mr.z, mr.w, 
            msg.transforms[j].image_error, msg.transforms[j].object_error, msg.transforms[j].fiducial_area])
    t_fi[i]=msg.header.stamp.to_time()

area=fi_946[:,-1].astype(float)
z=fi_946[:,2].astype(float)
idx=area>0

p_fitted = Polynomial.fit(z[idx],area[idx],deg=2)
z_int, area_int = p_fitted.linspace()

mp.figure()
mp.subplot(221)
# mp.plot(z[idx], np.sqrt(area[idx]),'.')
mp.plot(z[idx], area[idx],'.')
# mp.plot(z_int, np.sqrt(area_int),'.')
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
mp.show()

t_max = np.max([t_state[-1]-t_state[0], t[-1]-t_state[0], t_x[-1]-t_x[0]])
mp.subplot(211)
mp.plot(t-t_state[0],cmd_vel,'.-')
mp.legend(('v_x','v_y','v_z','\omega_z'))
mp.axis([0,t_max,-1, 1])    
mp.grid(True)
mp.subplot(212)
mp.plot(t_x-t_state[0],xyzy,'.-',t_x-t_state[0],np.ones_like(xyzy)*np.pi,t_x-t_state[0],-np.ones_like(xyzy)*np.pi)
mp.plot(t_state-t_state[0], state)
mp.legend(('x','y','z','yaw'))    
mp.axis([0,t_max,-4, 5])    
mp.grid(True)
mp.show()

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