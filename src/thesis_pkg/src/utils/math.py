def lagrange_polynomial_interpolation(
    x: "tuple[float,float,float]", y: "tuple[float,float,float]"
):
    x_1, x_2, x_3 = x
    y_1, y_2, y_3 = y

    a = (
        y_1 / ((x_1 - x_2) * (x_1 - x_3))
        + y_2 / ((x_2 - x_1) * (x_2 - x_3))
        + y_3 / ((x_3 - x_1) * (x_3 - x_2))
    )

    b = (
        -y_1 * (x_2 + x_3) / ((x_1 - x_2) * (x_1 - x_3))
        - y_2 * (x_1 + x_3) / ((x_2 - x_1) * (x_2 - x_3))
        - y_3 * (x_1 + x_2) / ((x_3 - x_1) * (x_3 - x_2))
    )

    c = (
        y_1 * x_2 * x_3 / ((x_1 - x_2) * (x_1 - x_3))
        + y_2 * x_1 * x_3 / ((x_2 - x_1) * (x_2 - x_3))
        + y_3 * x_1 * x_2 / ((x_3 - x_1) * (x_3 - x_2))
    )

    return a, b, c


def quadratic_from_lagrange_polynomial_interpolation(
    x: "tuple[float,float,float]", y: "tuple[float,float,float]"
):
    """
    Create polynome from three data points
    - a*x**2 + b*x + c = (x,z)
    - a*x**2 + b*x + c = (margin,h)
    - a*x**2 + b*x + c = (w-margin,h)
    """
    x_1, x_2, x_3 = x
    y_1, y_2, y_3 = y

    a = (
        y_1 / ((x_1 - x_2) * (x_1 - x_3))
        + y_2 / ((x_2 - x_1) * (x_2 - x_3))
        + y_3 / ((x_3 - x_1) * (x_3 - x_2))
    )

    b = (
        -y_1 * (x_2 + x_3) / ((x_1 - x_2) * (x_1 - x_3))
        - y_2 * (x_1 + x_3) / ((x_2 - x_1) * (x_2 - x_3))
        - y_3 * (x_1 + x_2) / ((x_3 - x_1) * (x_3 - x_2))
    )

    c = (
        y_1 * x_2 * x_3 / ((x_1 - x_2) * (x_1 - x_3))
        + y_2 * x_1 * x_3 / ((x_2 - x_1) * (x_2 - x_3))
        + y_3 * x_1 * x_2 / ((x_3 - x_1) * (x_3 - x_2))
    )

    return lambda x: a * x**2 + b * x + c
