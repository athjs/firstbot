def direct_kinematics(Vd, Vg):
    Vl = (Vd + Vg) * 25 / 2
    Vo = ((Vd - Vg) / 82.5) * 25
    return (Vl, Vo)


def inverse_kinematics(Vl, Vo):
    Vd = (Vl - 41.25 * Vo) / 25
    Vg = (Vl + 41.25 * Vo) / 25
    print(Vd, Vg)
    return (Vd, Vg)


def point_direction(dx, kr, ka):
    Vl = kr
    # normalise to the image size
    Vo = ka * dx / 640
    print(Vl, Vo)
    return (Vl, Vo)
