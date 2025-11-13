from typing import Tuple, Literal
import math

def euler_ned_to_quat(
    roll: float, pitch: float, yaw: float,
    *, degrees: bool = False,
    order: Literal["wxyz", "xyzw"] = "wxyz"
) -> Tuple[float, float, float, float]:

    if degrees:
        roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    # Z-Y-X (yaw-pitch-roll)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy

    # normalize to guard against tiny numerical drift
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = (w/norm, x/norm, y/norm, z/norm)

    if order == "xyzw":
        return (x, y, z, w)
    return (w, x, y, z)

def quat_to_euler_ned(
    q: Tuple[float, float, float, float],
    *, degrees: bool = False,
    order: Literal["wxyz", "xyzw"] = "wxyz"
) -> Tuple[float, float, float]:
    if order == "xyzw":
        x, y, z, w = q
    else:
        w, x, y, z = q

    # Precompute repeated terms
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90° if out of range
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    if degrees:
        roll, pitch, yaw = map(math.degrees, (roll, pitch, yaw))

    return roll, pitch, yaw