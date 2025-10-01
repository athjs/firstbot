def direct_kinematics(Vd, Vg):
    Vl = (Vd + Vg) * 25 / 2
    Vo = ((Vd - Vg) / 82.5) * 25
    return (Vl, Vo)


def inverse_kinematics(Vl, Vo):
    Vd = (Vl - 41.25 * Vo) / 25
    Vg = (Vl + 41.25 * Vo) / 25
    return (Vd, Vg)


def point_direction(dx, kr, ka):
    Vl = kr * (1 / dx)
    Vo = ka * dx
    return (Vl, Vo)


