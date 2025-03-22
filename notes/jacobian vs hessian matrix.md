# Jacobian Matrix
Given a 6DOF robot, then we can translate between robot space and worldspace by simply creating 6 functions and taking the derivative of these functions like so

$\begin{bmatrix}
∂f_1/∂x & ∂f_1/∂y & ∂f_1/∂z & ∂f_1/∂u & ∂f_1/∂v & ∂f_1/∂w\cr
∂f_2/∂x & ∂f_2/∂y & ∂f_2/∂z & ∂f_2/∂u & ∂f_2/∂v & ∂f_2/∂w\cr
∂f_3/∂x & ∂f_3/∂y & ∂f_3/∂z & ∂f_3/∂u & ∂f_3/∂v & ∂f_3/∂w\cr
∂f_4/∂x & ∂f_4/∂y & ∂f_4/∂z & ∂f_4/∂u & ∂f_4/∂v & ∂f_4/∂w\cr
∂f_5/∂x & ∂f_5/∂y & ∂f_5/∂z & ∂f_5/∂u & ∂f_5/∂v & ∂f_5/∂w\cr
∂f_6/∂x & ∂f_6/∂y & ∂f_6/∂z & ∂f_6/∂u & ∂f_6/∂v & ∂f_6/∂w\cr
\end{bmatrix}$

Given multiple functions to describe a given state, for example the function for a 6DOF robot. 

# Hessian Matrix

$\begin{bmatrix}
∂^2f/∂xx & ∂^2f/∂xy & ∂^2f/∂xz & ∂^2f/∂xu & ∂^2f/∂xv & ∂^2f/∂xw\cr
∂^2f/∂yx & ∂^2f/∂yy & ∂^2f/∂yz & ∂^2f/∂yu & ∂^2f/∂yv & ∂^2f/∂yw\cr
∂^2f/∂zx & ∂^2f/∂zy & ∂^2f/∂zz & ∂^2f/∂zu & ∂^2f/∂zv & ∂^2f/∂zw\cr
∂^2f/∂ux & ∂^2f/∂uy & ∂^2f/∂uz & ∂^2f/∂uu & ∂^2f/∂uv & ∂^2f/∂uw\cr
∂^2f/∂vx & ∂^2f/∂vy & ∂^2f/∂vz & ∂^2f/∂vu & ∂^2f/∂vv & ∂^2f/∂vw\cr
∂^2f/∂wx & ∂^2f/∂wy & ∂^2f/∂wz & ∂^2f/∂wu & ∂^2f/∂wv & ∂^2f/∂ww\cr
\end{bmatrix}$