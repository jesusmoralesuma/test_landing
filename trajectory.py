
import numpy as np

class Trapezoidal(object):
    """ Trapezoidal trajectory generator """

    def __init__(self, dq_max, ddq_max, cartesian):
        
        """ cartesian (bool): True means planning is done in the fixed frame """
        self.cartesian = cartesian
        # Maximum speed and acceleration for XYZW (m/s, rad/s) and (m/s^2, rad/s^2)
        self._dq_max = dq_max
        self._ddq_max = ddq_max
        self._dq_max_current = dq_max.copy()
        self._ddq_max_current = ddq_max.copy()
        
        self._t1 = np.zeros_like(dq_max)
        self._tau = np.zeros_like(dq_max)
    def get_end_time(self):
        """ Return end time of the profile """
        return self._t1[0]+self._tau[0]

    def set_positions(self, q_init, q_final):
        """
        Updates current and desired positions
        Args:
            q_init (float): Current position
            q_final (float): Desired position
        """
        self._q_init = self.wrap(q_init)
        self._q_final = self.wrap(q_final)

    def rotate_inv (self, q, yaw):
        return np.array([
            q[0]*np.cos(yaw)-q[1]*np.sin(yaw),
            q[0]*np.sin(yaw)+q[1]*np.cos(yaw),
            q[2],
            q[3]
            ])

    def rotate (self, q, yaw):
        """
        Plane rotation of (x, y) vector. Angular component remains constant
        Args:
            q: position/orientation 
            yaw: Desired rotated angle
        """        
        return np.array([
            q[0]*np.cos(yaw)+q[1]*np.sin(yaw),
            -q[0]*np.sin(yaw)+q[1]*np.cos(yaw),
            q[2],
            q[3]
            ])

    def wrap (self, q):
        """
        Wrap yaw angle to [-pi, pi] range
        """
        q[3] = (q[3] + np.pi) % (2.0 * np.pi) - np.pi
        return q

    def compute_profile(self, q_init, q_final=[]):
        """
        Generates trapezoidal profile
        Args:
            q_init (array, XYZ_Yaw): Current position wrt UAV reference frame (base_stablized)
            q_final (array, XYZ_Yaw): Desired position wrt fiducial pattern reference frame           
        Returns:
            float, float:
                t_1 - acceleration stop time
                tau - deceleration start time
                  /----------\
                 / |        | \
                /  |        |  \
                  t_1       tau
        """        
        if q_final==[]:
            self.set_positions(q_init, q_init)
            self._t1 = np.zeros_like(self._t1)
            self._tau = np.zeros_like(self._tau)
            self._dq_max_current = np.zeros_like(self._dq_max_current)
            self._ddq_max_current = np.zeros_like(self._ddq_max_current)
        else:
            self.set_positions(q_init, self.rotate_inv(q_final, q_final[3]))

            if self.cartesian:            
                delta_q_sign = self.wrap(self.rotate_inv(self._q_final,self._q_final[3]-self._q_init[3]) - self._q_init)          
            else:        
                delta_q_sign = self.wrap(self._q_final - self._q_init)      
            delta_q = np.abs(delta_q_sign)                

            dq_max_prime = np.sqrt(delta_q * self._ddq_max)
            for idx, value in np.ndenumerate(dq_max_prime < self._dq_max):
                i=idx[0]
                # print(i,value)
                # Triangular profile
                if value:
                    self._t1[i] = dq_max_prime[i] / self._ddq_max[i]
                    self._tau[i] = self._t1[i]
                    self._dq_max_current[i] = dq_max_prime[i]*np.sign(delta_q_sign[i])
                # Trapezoidal profile
                else:
                    self._t1[i] = self._dq_max[i] / self._ddq_max[i]
                    self._tau[i] = delta_q[i] / self._dq_max[i]
                    self._dq_max_current[i] = self._dq_max[i]*np.sign(delta_q_sign[i])

            t1_max = np.max(self._t1)
            tau_max = np.max(self._tau)

            # Equalize profiles
            dq_max_new = delta_q/tau_max
            ddq_max_new = delta_q/(tau_max*t1_max)

            self._t1 = np.ones_like(self._t1)*t1_max
            self._tau = np.ones_like(self._tau)*tau_max

            dq_max_prime = np.sqrt(delta_q * ddq_max_new)        
            for idx, value in np.ndenumerate(dq_max_prime < dq_max_new):
                i=idx[0]
                # Triangular profile
                if value:
                    self._dq_max_current[i] = dq_max_prime[i]*np.sign(delta_q_sign[i])
                # Trapezoidal profile
                else:
                    self._dq_max_current[i] = dq_max_new[i]*np.sign(delta_q_sign[i])

            self._ddq_max_current = delta_q_sign/(tau_max*t1_max)

        # # Consider controller frequency
        # if self._control_freq != 0:
        #     n = np.ceil(t_1 * self._control_freq)
        #     m = np.ceil(tau * self._control_freq)

        #     t_1 = n / self._control_freq
        #     tau = m / self._control_freq

        print('t1 and tau',self._t1,self._tau)

        # return t_1, tau

    def generate_q_dq(self, time):
        """
        Generates position and speed sample
        Args:
            time
        Returns:
            array, array:
                q  - position
                dq - speed
        """        
        if time < self._t1[0]:
            dq = self._ddq_max_current*time
            q  = self._q_init+dq*time/2.0
        elif time < self._tau[0]:
            dq = self._dq_max_current
            q  = self._q_init+self._ddq_max_current*self._t1[0]**2./2.0+dq*(time-self._t1[0])
        elif time < self._t1[0]+self._tau[0]:
            dq = self._dq_max_current-self._ddq_max_current*(time-self._tau[0])
            if self.cartesian:
                q = self.rotate_inv(self._q_final,self._q_final[3]-self._q_init[3])+dq*(time-self._tau[0]-self._t1[0])/2.0
            else:
                q = self._q_final+dq*(time-self._tau[0]-self._t1[0])/2.0
        else:
            dq = np.zeros_like(self._dq_max_current)
            if self.cartesian:
                q  = self.rotate_inv(self._q_final,self._q_final[3]-self._q_init[3])
            else:
                q  = self._q_final.copy()

        q = self.wrap(q)       
        if self.cartesian:
            return self.rotate(q, q[3]-self._q_init[3]), self.rotate(dq, q[3]-self._q_init[3]) # Transform to the moving frame
        else:
            return q, dq