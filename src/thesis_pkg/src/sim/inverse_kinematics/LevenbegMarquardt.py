import numpy as np

from .InverseKinematics import InverseKinematics

__all__ = ["LevenbegMarquardt"]


class LevenbegMarquardt(InverseKinematics):
    def __init__(
        self,
        model,
        data=None,
        step_size=0.5,
        tol=0.01,
        alpha=0.5,
        body_id=None,
        jacp=None,
        jacr=None,
        frame_rate=60,
        init_q=None,
        renderer=None,
        damping=0.15,
    ):
        self.damping = damping
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

    # Levenberg-Marquardt pseudocode implementation
    def _step(self, error):
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
