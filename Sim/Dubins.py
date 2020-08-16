import math
import tracemalloc
import linecache
import matplotlib.pyplot as plt
import numpy as np
import sys
import gc
from pympler.asizeof import asizeof

show_animation = False


def display_top(snapshot, key_type='lineno', limit=10):
    snapshot = snapshot.filter_traces((
        tracemalloc.Filter(False, "<frozen importlib._bootstrap>"),
        tracemalloc.Filter(False, "<unknown>"),
    ))
    top_stats = snapshot.statistics(key_type)

    print("Top %s lines" % limit)
    for index, stat in enumerate(top_stats[:limit], 1):
        frame = stat.traceback[0]
        print("#%s: %s:%s: %.1f KiB"
              % (index, frame.filename, frame.lineno, stat.size / 1024))
        line = linecache.getline(frame.filename, frame.lineno).strip()
        if line:
            print('    %s' % line)

    other = top_stats[limit:]
    if other:
        size = sum(stat.size for stat in other)
        print("%s other: %.1f KiB" % (len(other), size / 1024))
    total = sum(stat.size for stat in top_stats)
    print("Total allocated size: %.1f KiB" % (total / 1024))



def mod2pi(theta):
    return theta - 2.0 * np.pi * np.floor(theta / 2.0 / np.pi)


def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def left_straight_left(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    tmp0 = d + sa - sb

    mode = ["L", "S", "L"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((cb - ca), tmp0)
    t = mod2pi(-alpha + tmp1)
    p = np.sqrt(p_squared)
    q = mod2pi(beta - tmp1)

    return t, p, q, mode


def right_straight_right(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    tmp0 = d - sa + sb
    mode = ["R", "S", "R"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((ca - cb), tmp0)
    t = mod2pi(alpha - tmp1)
    p = np.sqrt(p_squared)
    q = mod2pi(-beta + tmp1)

    return t, p, q, mode


def left_straight_right(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    p = np.sqrt(p_squared)
    tmp2 = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, p)
    t = mod2pi(-alpha + tmp2)
    q = mod2pi(-mod2pi(beta) + tmp2)

    return t, p, q, mode


def right_straight_left(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    p = np.sqrt(p_squared)
    tmp2 = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, p)
    t = mod2pi(alpha - tmp2)
    q = mod2pi(beta - tmp2)

    return t, p, q, mode


def right_left_right(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    mode = ["R", "L", "R"]
    tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp_rlr) > 1.0:
        return None, None, None, mode

    p = mod2pi(2 * np.pi - math.acos(tmp_rlr))
    t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0))
    q = mod2pi(alpha - beta - t + mod2pi(p))
    return t, p, q, mode


def left_right_left(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    mode = ["L", "R", "L"]
    tmp_lrl = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (- sa + sb)) / 8.0
    if abs(tmp_lrl) > 1:
        return None, None, None, mode
    p = mod2pi(2 * np.pi - math.acos(tmp_lrl))
    t = mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
    q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))

    return t, p, q, mode


def dubins_path_planning_from_origin(end_x, end_y, end_yaw, curvature, step_size):
    dx = end_x
    dy = end_y
    D = np.hypot(dx, dy)
    d = D * curvature

    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(- theta)
    beta = mod2pi(end_yaw - theta)

    planners = [left_straight_left, right_straight_right, left_straight_right, right_straight_left, right_left_right,
                left_right_left]

    best_cost = float("inf")
    bt, bp, bq, best_mode = None, None, None, None

    for planner in planners:
        t, p, q, mode = planner(alpha, beta, d)
        if t is None:
            continue

        cost = (abs(t) + abs(p) + abs(q))
        if best_cost > cost:
            bt, bp, bq, best_mode = t, p, q, mode
            best_cost = cost
    lengths = [bt, bp, bq]

    px, py, pyaw, directions = generate_local_course(
        sum(lengths), lengths, best_mode, curvature, step_size)

    return px, py, pyaw, best_mode, best_cost


def interpolate(ind, length, mode, max_curvature, origin_x, origin_y, origin_yaw, path_x, path_y, path_yaw, directions):
    if mode == "S":
        path_x[ind] = origin_x + length / max_curvature * np.cos(origin_yaw)
        path_y[ind] = origin_y + length / max_curvature * np.sin(origin_yaw)
        path_yaw[ind] = origin_yaw
    else:  # curve
        ldx = np.sin(length) / max_curvature
        ldy = 0.0
        if mode == "L":  # left turn
            ldy = (1.0 - np.cos(length)) / max_curvature
        elif mode == "R":  # right turn
            ldy = (1.0 - np.cos(length)) / -max_curvature
        gdx = np.cos(-origin_yaw) * ldx + np.sin(-origin_yaw) * ldy
        gdy = -np.sin(-origin_yaw) * ldx + np.cos(-origin_yaw) * ldy
        path_x[ind] = origin_x + gdx
        path_y[ind] = origin_y + gdy

    if mode == "L":  # left turn
        path_yaw[ind] = origin_yaw + length
    elif mode == "R":  # right turn
        path_yaw[ind] = origin_yaw - length

    if length > 0.0:
        directions[ind] = 1
    else:
        directions[ind] = -1

    return path_x, path_y, path_yaw, directions


def dubins_path_planning(sx, sy, syaw, ex, ey, eyaw, c, step_size=0.35):
    """
    Dubins path plannner
    input:
        sx x position of start point [m]
        sy y position of start point [m]
        syaw yaw angle of start point [rad]
        ex x position of end point [m]
        ey y position of end point [m]
        eyaw yaw angle of end point [rad]
        c turningRadius [1/m]
    output:
        px
        py
        pyaw
        mode
    """

    ex = ex - sx
    ey = ey - sy

    lex = np.cos(syaw) * ex + np.sin(syaw) * ey
    ley = - np.sin(syaw) * ex + np.cos(syaw) * ey
    leyaw = eyaw - syaw

    lpx, lpy, lpyaw, mode, clen = dubins_path_planning_from_origin(
        lex, ley, leyaw, c, step_size)
    test = zip(lpx, lpy)
    # px = [np.cos(-syaw) * x + np.sin(-syaw)
    #       * y + sx for x, y in zip(lpx, lpy)]
    # print(f'fuck this {len(lpx)}')
    # print(asizeof.asized(px, detail=1).format())

    # py = [- np.sin(-syaw) * x + np.cos(-syaw)
    #       * y + sy for x, y in zip(lpx, lpy)]
    px = []
    py = []
    tmpx = []
    tmpy = []
    tmpyaw = []
    pyaw = []
    for x, y in test:
        px.append(np.cos(-syaw) * x + np.sin(-syaw) * y + sx)
        py.append(- np.sin(-syaw) * x + np.cos(-syaw) * y + sy)
    tmpx = px
    del px
    tmpy = py
    del py
    for iyaw in lpyaw:
        pyaw.append(pi_2_pi(iyaw+syaw))
    tmpyaw = pyaw
    del pyaw
    # pyaw = [pi_2_pi(iyaw + syaw) for iyaw in lpyaw]
    # snapshot = tracemalloc.take_snapshot()
    # top_stats = snapshot.statistics('lineno')
    # print("[ Top 10 ]")
    # for stat in top_stats[:10]:
    #     print(stat)
    return tmpx, tmpy, tmpyaw, mode, clen

# def memoryLeak():
#     del px
#     del py
#     del pyaw


def generate_local_course(total_length, lengths, mode, max_curvature, step_size):
    n_point = math.trunc(total_length / step_size) + len(lengths) + 4

    path_x = [0.0 for _ in range(n_point)]
    path_y = [0.0 for _ in range(n_point)]
    path_yaw = [0.0 for _ in range(n_point)]
    directions = [0.0 for _ in range(n_point)]
    ind = 1

    if lengths[0] > 0.0:
        directions[0] = 1
    else:
        directions[0] = -1

    ll = 0.0

    for (m, l, i) in zip(mode, lengths, range(len(mode))):
        if l > 0.0:
            d = step_size
        else:
            d = -step_size

        # set origin state
        origin_x, origin_y, origin_yaw = path_x[ind], path_y[ind], path_yaw[ind]

        ind -= 1
        if i >= 1 and (lengths[i - 1] * lengths[i]) > 0:
            pd = - d - ll
        else:
            pd = d - ll

        while abs(pd) <= abs(l):
            ind += 1
            path_x, path_y, path_yaw, directions = interpolate(
                ind, pd, m, max_curvature, origin_x, origin_y, origin_yaw, path_x, path_y, path_yaw, directions)
            pd += d

        ll = l - pd - d  # calc remain length

        ind += 1
        path_x, path_y, path_yaw, directions = interpolate(
            ind, l, m, max_curvature, origin_x, origin_y, origin_yaw, path_x, path_y, path_yaw, directions)

    if len(path_x) <= 1:
        return [], [], [], []

    # remove unused data
    while len(path_x) >= 1 and path_x[-1] == 0.0:
        path_x.pop()
        path_y.pop()
        path_yaw.pop()
        directions.pop()

    return path_x, path_y, path_yaw, directions




def getDubinsPath(start_x, start_y, start_angle, end_x, end_y, end_angle, curvature):
    npx, npy, npyaw, mode, clen = dubins_path_planning(start_x, start_y, start_angle,
                                                    end_x, end_y, end_angle, curvature)

    return npx, npy, npyaw

def main(start_x, start_y, start_angle, end_x, end_y, end_angle, curvature):
    # print("Dubins path planner sample start!!")


    px, py, pyaw, mode, clen = dubins_path_planning(start_x, start_y, start_angle,
                                                    end_x, end_y, end_angle, curvature)
    # print(f'px: {px}, \n py: {py},\n pyaw: {pyaw},\n mode: {mode}, clen: {clen}')
    # if show_animation:
    #     plt.plot(px, py, label="final course " + "".join(mode))
    #
    #     # plotting
    #     plot_arrow(start_x, start_y, start_yaw)
    #     plot_arrow(end_x, end_y, end_yaw)
    #
    #     plt.legend()
    #     plt.grid(True)
    #     plt.axis("equal")
    #     plt.show()
    return px, py, pyaw

if __name__ == '__main__':
    main()