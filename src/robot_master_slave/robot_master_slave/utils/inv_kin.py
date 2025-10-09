from math import sin, cos, atan2, sqrt, degrees

def inverse_kin_plan_3dof(l1:float, l2:float, l3:float,
                          x_des:float, y_des:float, al_des: float):
    wx = x_des - l3*cos(al_des)
    wy = y_des - l3*sin(al_des)

    r2 = wx*wx + wy*wy
    c2 = (r2 - l1*l1 - l2*l2) / (2*l1*l2)
    if abs(c2) > 1:
        raise ValueError("Target irraggiungibile")
    s2 = sqrt(max(0.0, 1 - c2*c2))
    
    q2 = atan2(s2, c2)
    q1 = atan2(wy, wx) - atan2(l2*s2, l1 + l2*c2)
    q3 = al_des - q1 - q2

    return q1,q2,q3
