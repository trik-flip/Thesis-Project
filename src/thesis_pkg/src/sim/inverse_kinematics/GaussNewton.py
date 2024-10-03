import numpy as np

from .InverseKinematics import InverseKinematics


class GaussNewton(InverseKinematics):

    # Gauss-Newton pseudocode implementation
    def _step(self, error):
        # calculate delta of joint q

        jac = self._get_correct_jac(error)

        product = jac.T @ jac

        if np.isclose(np.linalg.det(product), 0):
            j_inv = np.linalg.pinv(product) @ jac.T
        else:
            j_inv = np.linalg.inv(product) @ jac.T

        delta_q = j_inv @ error
        return delta_q
