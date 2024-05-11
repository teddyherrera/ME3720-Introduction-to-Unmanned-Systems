import math

def wrap_to_180(angle):
    """Wrap an angle in degrees to [-180, 180]."""
    return (angle + 180) % 360 - 180

def rad_to_deg(angle):
    """Convert radians to degrees."""
    return angle * (180.0 / math.pi)

def deg_to_rad(angle):
    """Convert degrees to radians."""
    return angle * (math.pi / 180.0)

def cte(bx, by, fx, fy, tx, ty, rho):
    """
    Calculate the ordered heading based on current and target positions.

    Parameters:
        bx, by: current x/y position
        fx, fy: "from" point x/y position
        tx, ty: "to" point x/y position
        rho: desired approach point (distance in front of vehicle)
    """
    # Normal to the path vector
    N = [ty - fy, -(tx - fx)]
    # Position vector from from-point to current position
    P = [bx - fx, by - fy]
    # Error projection on the normal
    e = (P[0] * N[0] + P[1] * N[1]) / math.sqrt(N[0]**2 + N[1]**2)
    # Direction to the target point
    psitrack = math.atan2(ty - fy, tx - fx)
    # Compensated heading
    psicom = psitrack + math.atan2(e, rho)
    # Convert to degrees and wrap
    ordered_heading = wrap_to_180(rad_to_deg(psicom))

    return ordered_heading