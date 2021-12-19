import numpy as np
import math

class CameraProjector(object):
    def __init__(self):
        # _K_mat[1,1]
        self._K_mat = np.asarray([[670.72030436,   0.,         265.020138  ], [  0.,         -670.75778801, 245.91319719], [  0.,           0.,           1.        ]])

        # self._K_mat = np.asarray([[841.19371541, 0.0, 517.8968481], [ 0.0, 843.58377435, 278.98469394], [ 0.0, 0.0, 1.0 ]])

        self._Rot_mat = np.asarray([[ 9.99999998e-01,  3.66661792e-05,  5.16035318e-05], [-3.69199722e-05,  9.99987863e-01,  4.92675549e-03], [-5.14222602e-05, -4.92675738e-03,  9.99987862e-01]])
        # self._Rot_mat = np.asarray([[ 2.87642066e-02,  9.99572193e-01, -5.29630851e-03], [-9.87532227e-01,  2.92373113e-02,  1.54677989e-01], [ 1.54766666e-01,  7.81085719e-04,  9.87950742e-01]])
        # self._trn_vec = np.array([[-4.29449865], [-2.27970549], [20.77305481]])
        self._trn_vec = np.array([[0], [-1.28+0.6], [1.55]])
        self._P_mat = np.hstack([self._Rot_mat, self._trn_vec])
        # self._scal_f = np.array([2.32893])
        self._scal_f = np.array([1.709061501])
        self._scal_i = 0
        self._centre_y = -0.03136924
        self._centre_y_t = -0.00836998
        self._centre_z = 0.62
        self._centre_z_off = 0.1
        self._scal_y = 2.357739
        self._scal_z = 0.525/(1.7-0.5)

    def set_K(self, K_new=np.zeros((3,3))):
        self._K_mat = K_new

    def set_P(self, R_new=np.zeros((3,3)), t_new=np.zeros((3,1))):
        self._Rot_mat = R_new
        self._trn_vec = P_new
        self._P_mat = np.hstack([R_new, t_new])

    def set_scaling_factor(self, new_s=None):
        self._scal_f = np.array([2.32893]) if (new_s is None) or (new_s is not np.ndarray) else new_s
        # sort values?
        self._scal_i = 0

    def project_to_pixel(self, X):
        return self._project_to_pixel(self._scal_f[self._scal_i], self._K_mat, self._P_mat, X)

    def _project_to_pixel(self, s, K, P, X):
        # dimension checks

        R = P[0:3,0:3]
        t = P[0:3,3].reshape((3,1))
        u = 1/s*np.matmul(K, R.dot(X)+t) # np.matmul
        return u

    def deproject(self, U):
        return self._deproject(self._scal_f[self._scal_i], self._K_mat, self._P_mat, U)

    def _deproject(self, s, K, P, U):
        # import pdb; pdb.set_trace()
        R = P[0:3,0:3]
        t = P[0:3,3].reshape((3,1))
        K_i = np.linalg.inv(K)
        R_i = np.linalg.inv(R)
        dis = (s*K_i.dot(U)-t)
        X = R_i.dot(dis)
        X[0] = -X[0]+self._centre_y
        X[1] = self._centre_z + (X[1] - self._centre_z) * 1/self._scal_z - self._centre_z_off

        return X

    def _deproject_with_disparity(self, cx, px, cy, py, fx, fy, B, D):
        z = f*B/D
        x = z*(px-cx)/fx
        y = z*(py-cy)/fy
        return np.asarray([x, y, z]).reshape((3,1))

    def _deproject_with_depth(self, cx, px, cy, py, f, depth):
        x = depth*(px-cx)/f
        y = depth*(py-cy)/f
        return np.asarray([x, y, depth]).reshape((3,1))

    def transform_camera_to_world_axes(self, X):
        t_x = math.pi/2
        Rot_x = np.asarray([[1.0, 0.0, 0.0], [0, math.cos(t_x), -math.sin(t_x)], [0, math.sin(t_x), math.cos(t_x)]])
        # t_x = np.asarray([[1.2750--0.031362],[4.7985e-1-0.54993],[0.47985-+0.027711]]) #orig
        t_x = np.asarray([[-(1.2750-0.00836998)],[4.7985e-1-0.54993],[-0.08571175]]) #mod
        X_r = Rot_x.dot(X)
        X_r[0] = X_r[0] * self._scal_y
        # X_r[2] = X_r[2] * self._scal_z
        X_r = X_r+t_x
        return X_r

    def transform_coords_swap(self, X):
        return np.array([X[1][0], X[0][0], X[2][0]]).reshape((3,1))

    def transform_world_to_camera_axes(self, X):
        t_x = -math.pi/2
        Rot_x = np.asarray([[1.0, 0.0, 0.0], [0, math.cos(t_x), -math.sin(t_x)], [0, math.sin(t_x), math.cos(t_x)]])
        t_x = -1 * np.asarray([[1.2750--0.031362],[1.5006e-1-+0.027711],[4.7985e-1-0.54993]])
        X_r = Rot_x.dot(X)+t_x
        return X_r
