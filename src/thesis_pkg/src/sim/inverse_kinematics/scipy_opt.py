import numpy as np
from scipy.optimize import minimize

from .InverseKinematics import InverseKinematics

__all__ = ["LevenbegMarquardt"]


class ScipyOpt(InverseKinematics):
    def __init__(
        self,
        model,
        data,
        step_size,
        tol,
        alpha,
        body_id,
        jacp=None,
        jacr=None,
        frame_rate: int = 60,
        init_q=None,
        renderer=None,
        damping=0.15,
    ):
        super().__init__(
            model,
            data,
            step_size,
            tol,
            alpha,
            body_id,
            jacp,
            jacr,
            frame_rate,
            init_q,
            renderer,
        )
        self.damping = damping

    # Levenberg-Marquardt pseudocode implementation
    def _step(self, error):

        result = minimize(
            ik_objective, initial_guess, args=(target_pose, dh_params), method="BFGS"
        )
        jac = self._get_correct_jac(error)

        # calculate delta of joint q
        n = jac.shape[1]
        I = np.identity(n)
        product = jac.T @ jac + self.damping * I

        if np.isclose(np.linalg.det(product), 0):
            j_inv = np.linalg.pinv(product) @ jac.T
        else:
            j_inv = np.linalg.inv(product) @ jac.T

        delta_q = j_inv @ error
        return delta_q
