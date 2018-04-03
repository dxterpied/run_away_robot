from matrix import *


class UKF(object):
    def __init__(self, measurement):
        self.n_x = 5
        self.x = matrix([[]])
        self.x.zero(self.n_x, 1)
        self.x.value[0][0] = measurement[0]
        self.x.value[1][0] = measurement[1]
        self.P = matrix([[]])
        self.P.identity(self.n_x)
        self.P.value[0][0] = 1
        self.P.value[1][1] = 1

        self.n_aug = 5
        self.lambda_ = 3 - self.n_aug
        self.sigma_length = 2 * self.n_aug + 1

        self.n_z = 2

        self.Xsig_pred = matrix([[]])
        self.Xsig_pred.zero(self.n_x, self.sigma_length)

        # create weights
        self.weights = matrix([[]])
        self.weights.zero(self.sigma_length, 1)
        self.weights.value[0][0] = self.lambda_ / (self.lambda_ + self.n_aug)

        _ = 0.5 / (self.lambda_ + self.n_aug)
        for i in range(1, self.sigma_length):
            self.weights.value[i][0] = _

        # noise
        self.std_x = 3
        self.std_y = 3
        self.std_a = 0
        self.std_omiga = 0

        # R
        self.R = matrix([[10, 0], [0, 10]])

        # Xisg_aug
        self.Xsig_aug = matrix([[]])
        self.Xsig_aug.zero(self.n_aug, self.sigma_length)

    def AugmentedSigmaPoints(self):
        x_aug = matrix([[]])
        x_aug.zero(self.n_aug, 1)
        for i in range(self.n_x):
            x_aug.value[i][0] = self.x.value[i][0]

        P_aug = matrix([[]])
        P_aug.zero(self.n_aug, self.n_aug)
        for i in range(self.n_x):
            P_aug.value[i][0:self.n_x] = self.P.value[i]

        # P_aug.value[5][5] = self.std_a * self.std_a
        # P_aug.value[6][6] = self.std_omiga * self.std_omiga

        A = P_aug.Cholesky()

        for i in range(self.n_aug):
            self.Xsig_aug.value[i][0] = x_aug.value[i][0]
            for j in range(self.n_aug):
                self.Xsig_aug.value[i][j+1] = x_aug.value[i][0] + \
                    sqrt(self.lambda_ + self.n_aug) * A.value[i][j]
                self.Xsig_aug.value[i][j+1+self.n_aug] = x_aug.value[i][0] - \
                    sqrt(self.lambda_ + self.n_aug) * A.value[i][j]

    def SigmaPointPrediction(self, dt=1.):
        for i in range(self.sigma_length):
            p_x = self.Xsig_aug.value[0][i]
            p_y = self.Xsig_aug.value[1][i]
            p_v = self.Xsig_aug.value[2][i]
            p_theta = self.Xsig_aug.value[3][i]
            p_omiga = self.Xsig_aug.value[4][i]

            p_v_p = p_v
            p_omiga_p = p_omiga
            p_theta_p = p_theta + p_omiga * dt

            if p_omiga < 0.0001:
                px_p = p_v * cos(p_theta) * dt + p_x
                py_p = p_v * sin(p_theta) * dt + p_y
            else:
                r = p_v / p_omiga
                px_p = r * sin(p_theta_p) - r * sin(p_theta) + p_x
                py_p = r * cos(p_theta) - r * cos(p_theta_p) + p_y

            # add noise
            if self.n_aug == 7:
                nu_a = self.Xsig_aug.value[5][i]
                nu_omiga = self.Xsig_aug.value[6][i]
                px_p = px_p + 0.5 * dt * dt * nu_a * cos(p_theta)
                py_p = py_p + 0.5 * dt * dt * nu_a * sin(p_theta)
                p_v_p = p_v_p + dt * nu_a
                p_theta_p = p_theta_p + 0.5 * dt * dt * nu_omiga
                p_omiga_p = p_omiga_p + dt * nu_omiga

            self.Xsig_pred.value[0][i] = px_p
            self.Xsig_pred.value[1][i] = py_p
            self.Xsig_pred.value[2][i] = p_v_p
            self.Xsig_pred.value[3][i] = self.NormalAngle(p_theta_p)
            self.Xsig_pred.value[4][i] = self.Normal(p_omiga_p)

    def PredictMeanAndConvariance(self):
        self.x.zero(self.n_x, 1)
        for i in range(self.sigma_length):
            for j in range(self.n_x):
                self.x.value[j][0] = self.x.value[j][0] + \
                    self.weights.value[i][0] * self.Xsig_pred.value[j][i]

        self.P.zero(self.n_x, self.n_x)
        for i in range(self.sigma_length):
            x_diff = matrix(
                [self.Xsig_pred.transpose().value[i]]).transpose() - self.x
            x_diff.value[2][0] = self.NormalAngle(x_diff.value[2][0])
            x_diff.value[3][0] = self.Normal(x_diff.value[3][0])
            self.P = self.P + matrix([[self.weights.value[i][0] * x_diff.value[m][0]]
                                      for m in range(self.n_x)]) * x_diff.transpose()

    def PredictMeasurement(self):
        pass

    def UpdateState(self, measurement):
        z = matrix([[measurement[0]], [measurement[1]]])
        H = matrix([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0]])
        y = z - H * self.x
        Ht = H.transpose()
        S = H * self.P * Ht + self.R
        K = self.P * Ht * S.inverse()
        self.x = self.x + K * y
        I = matrix([[]])
        I.identity(self.n_x)
        self.P = (I - K * H) * self.P

    @staticmethod
    def NormalAngle(angle):
        while angle > 2 * pi:
            angle -= 2 * pi
        while angle < 0:
            angle += 2 * pi
        return angle

    @staticmethod
    def Normal(angle):
        while angle > pi:
            angle -= 2*pi
        while angle < -pi:
            angle += 2 * pi

        return angle


class KF(object):
    def __init__(self, measurement):
        self.position = measurement
        self.n_x = 3
        # self.x  [[h], [t], [d]]
        self.x = matrix([[]])
        self.x.zero(self.n_x, 1)
        self.F = matrix([[1, 1, 0], [0, 1, 0], [0, 0, 1]])
        self.P = matrix([[1000, 0, 0], [0, 1000, 0], [0, 0, 1000]])
        self.R = matrix([[0.01, 0],
                         [0, 0.01]])
        self.H = matrix([[1, 1, 0], [0, 0, 1]])
        self.number = 0

    def _prediction(self):
        self.x = self.F * self.x
        self.P = self.F * self.P * self.F.transpose()

    def _update(self, measurement, distance_between):
        z = matrix([[atan2(measurement[1] - self.position[1], measurement[0] - self.position[0])],
                    [distance_between(measurement, self.position)]])
        y = z - self.H * self.x
        S = self.H * self.P * self.H.transpose() + self.R
        K = self.P * self.H.transpose() * S.inverse()
        self.x = self.x + K * y
        I = matrix([[]])
        I.identity(3)
        self.P = (I - K * self.H) * self.P

    def Preocess(self, measurement, distance_between):
        self._prediction()
        self._update(measurement, distance_between)
        self.position = measurement
        x, y = measurement
        h = self.x.value[0][0]
        t = self.x.value[1][0]
        d = self.x.value[2][0]
        px = d * cos(h + t) + x
        py = d * sin(h + t) + y
        self.pred = (px, py)
