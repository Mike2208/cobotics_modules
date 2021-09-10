
class PID(object):
    def __init__(self, p, i, d, theta_des):
        self.kp = p
        self.ki = i
        self.kd = d

        self.theta_des = theta_des
        self.last_error = 0.0
        
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

    def update(self, theta, tau=0.05, dt=0.01):

        error = self.theta_des - theta
        d_error = error - self.last_error

        self.P = self.kp * error
        self.I += error * dt

        self.D = d_error / tau

        self.last_error = error

        return self.P + self.ki * self.I + self.kd * self.D