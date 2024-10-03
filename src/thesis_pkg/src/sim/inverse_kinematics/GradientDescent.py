from .InverseKinematics import InverseKinematics


class GradientDescent(InverseKinematics):
    """https://alefram.github.io/posts/Basic-inverse-kinematics-in-Mujoco"""

    def _step(self, error):
        # calculate gradient
        jac = self._get_correct_jac(error)

        grad = self.alpha * jac.T @ error
        return grad
