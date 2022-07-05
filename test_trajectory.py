
from data_read import rotate_inv
import trajectory
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as mp

dq_max = np.array([0.8, 0.8, 0.5, 0.5])
ddq_max = np.array([0.4, 0.4, 0.25, 0.25])

# q_init = np.array([0.0, 0.0, 1.0, 0.0])
# q_final   = np.array([0.6, 0.3, 0.4, 0.2])
q_init = np.array([0.0011756625099435779, -1.2938302568391291, 3.911878379478467,-1.5395746054673458])
q_final = np.array([0.5, 0.2, 1., -1.5395746054673458+np.pi/2.])

# q_init  = np.array([0.0, 0.0, 1.0, 1.])
# q_final = np.array([1.0, 1.0, 4.0, 1.+np.pi])


tp=trajectory.Trapezoidal(dq_max, ddq_max, False)
tp.compute_profile(q_init, q_final)

t = np.arange(0,tp._tau[0]+tp._t1[0]*2.,0.1)
q = np.zeros((len(t),len(q_init)))
dq = np.zeros((len(t),len(q_init)))

for i, time in np.ndenumerate(t):
    q[i,:], dq[i,:] = tp.generate_q_dq(time)

# mp.subplot(2,1,1)
# mp.plot(t,q,'.-')
# mp.grid(True)
# mp.subplot(2,1,2)
# mp.plot(t,dq,'.-')
# mp.grid(True)
# mp.show()

tp_cart=trajectory.Trapezoidal(dq_max, ddq_max, True)
tp_cart.compute_profile(q_init, q_final)

t_cart = np.arange(0,tp_cart._tau[0]+tp_cart._t1[0]*2.,0.1)
q_cart = np.zeros((len(t),len(q_init)))
dq_cart = np.zeros((len(t),len(q_init)))

for i, time in np.ndenumerate(t):
    q_cart[i,:], dq_cart[i,:] = tp_cart.generate_q_dq(time)
mp.figure(1)
mp.subplot(2,2,1)
mp.plot(t,q,'.-',t,np.pi*np.ones_like(q),t,-np.pi*np.ones_like(q))
mp.legend(('x','y','z','yaw'))
mp.grid(True)
mp.subplot(2,2,2)
mp.plot(t,dq,'.-')
mp.grid(True)
mp.subplot(2,2,3)
mp.plot(t_cart,q_cart,'.-',t,np.pi*np.ones_like(q),t,-np.pi*np.ones_like(q))
mp.grid(True)
mp.subplot(2,2,4)
mp.plot(t_cart,dq_cart,'.-')
mp.grid(True)
mp.show()

q_cart_rot = rotate_inv(q_cart)

mp.figure(2)
mp.subplot(211)
mp.plot(q_cart[:,0],q_cart[:,1],'.-')
mp.plot(q_cart_rot[:,0],q_cart_rot[:,1],'.-')
mp.xlabel('x')
mp.ylabel('y')
mp.legend(('mobile','fixed'))
mp.grid('on')
mp.axis('equal')

mp.subplot(212)
mp.plot(q_cart[:,0],q_cart[:,2],'.-')
mp.plot(q_cart_rot[:,0],q_cart_rot[:,2],'.-')
mp.xlabel('x')
mp.ylabel('z')
mp.legend(('mobile','fixed'))
mp.grid('on')
mp.axis('equal')

mp.show()

mp.figure()
ax=mp.axes(projection='3d')
ax.plot3D(q_cart[:,0],q_cart[:,1],q_cart[:,2],'.-')
ax.plot3D(q_cart_rot[:,0],q_cart_rot[:,1],q_cart_rot[:,2],'.-')
mp.legend(('mobile','fixed'))
mp.xlabel('x')
mp.ylabel('y')
mp.grid('on')
mp.axis('equal')
mp.show()