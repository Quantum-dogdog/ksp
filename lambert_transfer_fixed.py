import math
import time

import krpc
from numpy import array, dot, cross, pi
from numpy.linalg import norm, solve

# ----------------------------- 参数 -----------------------------
T_DEP_LEAD = 60.0          # 从现在起多少秒后出发（Lambert 的出发时刻）
T_DEP_UT = 2000            # 绝对 UT（可选）：填了就用它，忽略 T_DEP_LEAD
                           # scan 会打印这个值，填进来就不用担心"扫描到现在隔太久"
T_TRANSFER = 5 * 3600.0    # 转移飞行时长
G0 = 9.81                  # Isp 换算成排气速度用
THROTTLE_CUT1 = 20.0       # 剩余 dv 大于这个，全油门
THROTTLE_CUT2 = 2.0        # 剩余 dv 小于这个，关机
THROTTLE_LOW = 0.15        # 小油门
SELF_CHECK_TOL = 5000.0    # 自检允许的位置误差（米）

# ---- 调试开关：开 KSP 无限燃料自测时置 True，正式飞一定改回 False ----
CHEAT_INFINITE_FUEL = True
DV_SANITY = 1600.0         # dv 超过这个值直接不点火：多半是出发时刻选错了
AUTO_SCAN = True           # dv 超标时顺便在同一个连接里扫一遍出发窗口
PERI_ALT = 80.0e3          # 到达后目标近拱点高度（米，离目标天体地表）

# ----------------------- 纯数学部分（不依赖 KSP） -----------------------

def stumpf(z):
    '''Stumpff 函数 C(z), S(z)。z>0 椭圆，z<0 双曲线，z≈0 用级数。'''
    if z > 1e-6:
        s = math.sqrt(z)
        return (1 - math.cos(s)) / z, (s - math.sin(s)) / (z * s)
    if z < -1e-6:
        s = math.sqrt(-z)
        return (math.cosh(s) - 1) / (-z), (math.sinh(s) - s) / ((-z) * s)
    return 0.5 - z / 24.0 + z * z / 720.0, 1.0 / 6.0 - z / 120.0 + z * z / 5040.0

def lambert(r1v, r2v, tof, mu, long_way=False):
    '''
    全局变量法解 Lambert，返回 (v1, v2, z)。
    long_way=True 表示走转移角大于 180 度的那一边。
    '''
    r1 = float(norm(r1v))
    r2 = float(norm(r2v))
    cosd = dot(r1v, r2v) / (r1 * r2)
    cosd = max(-1.0, min(1.0, cosd))
    dtheta = math.acos(cosd)
    if long_way:
        dtheta = 2 * pi - dtheta
    if dtheta < 1e-6 or abs(dtheta - pi) < 1e-9:
        raise ValueError('转移角退化: %.9f rad' % dtheta)

    A = math.sin(dtheta) * math.sqrt(r1 * r2 / (1 - math.cos(dtheta)))

    def time_of_z(z):
        # y<0 或数值溢出都当作"z 太小"，返回 None 让二分往上走
        try:
            C, S = stumpf(z)
            y = r1 + r2 + A * (z * S - 1) / math.sqrt(C)
            if y < 0:
                return None, None
            x = math.sqrt(y / C)
            t = (x ** 3 * S + A * math.sqrt(y)) / math.sqrt(mu)
            return t, y
        except (ValueError, OverflowError, ZeroDivisionError):
            return None, None

    zlo = -40.0
    tlo, _ = time_of_z(zlo)
    if tlo is None:
        tlo = 0.0

    zhi = 4 * pi * pi - 0.01
    thi, _ = time_of_z(zhi)
    while thi is None:
        zhi = zlo + (zhi - zlo) * 0.8
        thi, _ = time_of_z(zhi)
        if zhi <= zlo:
            raise ValueError('找不到有效的 z 上界')

    if tof <= tlo:
        raise ValueError('TOF 太短，落在双曲线极限之外')
    if tof >= thi:
        raise ValueError('TOF 太长，接近整圈周期')

    for _ in range(200):
        mid = (zlo + zhi) / 2
        tm, _ = time_of_z(mid)
        if tm is None:
            zlo = mid
        elif tm < tof:
            zlo = mid
        else:
            zhi = mid
        if zhi - zlo < 1e-12:
            break

    z = (zlo + zhi) / 2
    t, y = time_of_z(z)
    if y is None or y <= 0:
        raise ValueError('解算失败：y<=0，z 落在无效区间')

    f = 1 - y / r1
    g = A * math.sqrt(y / mu)
    gdot = 1 - y / r2
    v1 = (r2v - f * r1v) / g
    v2 = (gdot * r2v - r1v) / g
    return v1, v2, z

def kepler_state(r0v, v0v, dt, mu, iters=60):
    '''全局变量法二体传播：给出 t+dt 时刻的位置和速度，返回 (r, v)。

    这是替换 kRPC 里不存在的 Orbit.velocity_at 的核心函数：
    从"当前 r、当前 v"往前推 dt 秒，位置和速度一起解出来。

    Newton 迭代加了步长阻尼（单步最多走自身量级的一半），防双曲线
    远端发散；阻尼后仍不收敛或中途溢出，就退化为对 chi 的二分——
    t(chi) 对 chi 单调增，二分必然收敛。
    '''
    r0 = float(norm(r0v))
    alpha = 2.0 / r0 - float(dot(v0v, v0v)) / mu
    rv0 = float(dot(r0v, v0v))
    sqm = math.sqrt(mu)

    if alpha > 1e-8:
        chi = sqm * alpha * dt
    elif alpha < -1e-8:
        # 双曲线初值：用对数形式，直接代 dt/r 会离解很远
        try:
            a_neg = -1.0 / alpha                      # 正数，等于 |a|
            denom = rv0 + sqm * math.sqrt(a_neg) * (1 - r0 * alpha)
            chi = math.sqrt(a_neg) * math.log((-2.0 * mu * alpha * dt) / denom)
        except (ValueError, ZeroDivisionError, OverflowError):
            chi = sqm * dt / r0                       # 兜底初值，交给二分救
    else:
        chi = sqm * dt / r0

    def t_of_chi(c):
        '''返回 (t, r)。r<=0 记作 (None, -1)（chi 偏小方向），
        数值溢出记作 (None, +1)（chi 偏大方向，t 视为 +inf）。'''
        try:
            z = c * c * alpha
            C, S = stumpf(z)
            r = c * c * C + rv0 / sqm * c * (1 - z * S) + r0 * (1 - z * C)
            if r <= 0:
                return None, -1
            t = (c ** 3 * S + rv0 / sqm * c * c * C + r0 * c * (1 - z * S)) / sqm
            return t, r
        except (OverflowError, ZeroDivisionError, ValueError):
            return None, +1

    # ---- 阻尼 Newton ----
    converged = False
    try:
        for _ in range(iters):
            t, r = t_of_chi(chi)
            if t is None:
                if r < 0:
                    chi = chi / 2.0
                    continue
                break                      # 溢出：直接进二分兜底
            step = (dt - t) * sqm / r
            # 阻尼：单步最多走自身量级的一半，双曲线远端防发散
            cap = 0.5 * max(1.0, abs(chi))
            if abs(step) > cap:
                step = math.copysign(cap, step)
            chi += step
            if abs(step) < 1e-10 * max(1.0, abs(chi)):
                converged = True
                break
    except (OverflowError, ZeroDivisionError, ValueError):
        pass

    if not converged:
        # ---- 二分兜底：从零起指数扩张找上界，再二分 ----
        lo = 0.0
        hi = max(1.0, abs(chi))
        for _ in range(200):
            t_hi, flag = t_of_chi(hi)
            if t_hi is None and flag < 0:
                lo = hi                    # t 还不够大（r<=0，罕见）
                hi *= 2.0
                continue
            if t_hi is None or t_hi >= dt:
                break                      # 溢出或 t 已超过 dt，hi 是合法上界
            lo = hi
            hi *= 2.0
        for _ in range(300):
            mid = 0.5 * (lo + hi)
            t_m, flag = t_of_chi(mid)
            if t_m is None and flag < 0:
                lo = mid                   # r<=0 视作 t 偏小
            elif t_m is None or t_m < dt:
                lo = mid                   # 溢出视作 t=+inf，归到偏大一侧
            else:
                hi = mid
            if hi - lo < 1e-12 * max(1.0, hi):
                break
        chi = 0.5 * (lo + hi)
        t_c, _ = t_of_chi(chi)
        if t_c is None or abs(t_c - dt) > 1e-6 * max(1.0, dt):
            raise ValueError('传播求解失败：二分未能收敛到目标时刻')

    # ---- 由 chi 一次算出位置和速度（f, g, fdot, gdot）----
    z = chi * chi * alpha
    C, S = stumpf(z)
    f = 1 - chi * chi * C / r0
    g = dt - chi ** 3 * S / sqm
    r = f * r0v + g * v0v
    rn = float(norm(r))
    if rn < 1e-6:
        raise ValueError('传播结果半径异常：%.3f m' % rn)
    fdot = sqm / (rn * r0) * chi * (z * S - 1.0)
    gdot = 1 - chi * chi * C / rn
    v = fdot * r0v + gdot * v0v
    return r, v

def kepler_propagate(r0v, v0v, dt, mu, iters=60):
    '''只要位置的接口，保留给自检用。'''
    return kepler_state(r0v, v0v, dt, mu, iters)[0]

# ---------------------------- KSP 部分 ----------------------------

def unit(v):
    return v / norm(v)

def node_components(dv_vec, rv, vv):
    """把惯性系里的 dv 向量分解成 KSP 节点的 (prograde, radial, normal)。

    关键点：KSP 节点的 prograde 轴是"速度方向"，不是"横向"。
    轨道偏心率大时（比如刚进 Mun SOI、速度几乎指向月球中心），
    prograde 和 radial 两个轴几乎重合、并不正交，
    不能像圆轨道那样拿 t̂/r̂/n̂ 直接点乘；必须解 M x = dv 这个 3x3 方程。
    """
    r_hat = unit(rv)
    v_hat = unit(vv)
    n_hat = unit(cross(rv, vv))
    M = array([[v_hat[0], r_hat[0], n_hat[0]],
               [v_hat[1], r_hat[1], n_hat[1]],
               [v_hat[2], r_hat[2], n_hat[2]]])
    comp = solve(M, dv_vec)
    return float(comp[0]), float(comp[1]), float(comp[2])

def body_radius(body):
    '''kRPC 的 CelestialBody 没有 radius 属性，只有 equatorial_radius。'''
    try:
        return float(body.equatorial_radius)
    except AttributeError:
        return float(body.radius)      # 兼容某些分支/旧版

def burn_mag(stream):
    '''剩余燃烧矢量的模长。注意不能取 [1]，那是某一个分量。'''
    v = stream()
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

def burn_to_node(conn, vessel, node, burn_time, burn_start, ut, label):
    '''按剩余 dv 的模长分段截止，不做开环计时。'''
    rem = conn.add_stream(node.remaining_burn_vector, node.reference_frame)

    print('%s：等待点火时刻 %.0f' % (label, burn_start))
    while ut() < burn_start:
        time.sleep(0.05)

    vessel.control.throttle = 1.0
    t_begin = ut()
    while True:
        m = burn_mag(rem)
        if m < THROTTLE_CUT2:
            break
        if m < THROTTLE_CUT1:
            vessel.control.throttle = THROTTLE_LOW
        if ut() - t_begin > burn_time * 3.0 + 30.0:
            print('%s：超时，强制关机' % label)
            break
        time.sleep(0.02)
    vessel.control.throttle = 0.0
    rem.remove()
    print('%s：结束，实际用时 %.1f s' % (label, ut() - t_begin))

def burn_time_for(vessel, dv_mag, ve):
    '''火箭方程反解燃烧时长，并检查燃料。

    CHEAT_INFINITE_FUEL=True 时只警告不报错——注意 KSP 的无限燃料作弊
    不会改变 vessel.mass / dry_mass，这个检查纯是算质量算出来的，
    所以作弊了照样会"燃料不够"，必须靠这个开关跳过。
    '''
    mass = vessel.mass
    thrust = vessel.available_thrust
    if thrust <= 0.0:
        raise RuntimeError('当前级没有可用推力（available_thrust = 0）')
    m_after = mass * math.exp(-dv_mag / ve)
    burn = (mass - m_after) / (thrust / ve)
    if m_after < vessel.dry_mass:
        msg = ('按火箭方程烧完只剩 %.1f kg，低于干重 %.1f kg'
               % (m_after, vessel.dry_mass))
        if CHEAT_INFINITE_FUEL:
            print('警告：%s —— 已开无限燃料，跳过燃料检查继续' % msg)
            print('        无限燃料下质量恒定，实际燃烧会比 %.0f s 长一些，'
                  '但按剩余 dv 截止会自动修正' % burn)
        else:
            raise RuntimeError('燃料不够：' + msg)
    return burn, mass, m_after

def flyby_periapsis(rv, vv, mu):
    '''给定相对位置/速度，返回这条圆锥曲线的 (近拱点半径, 偏心率, 半长轴)。'''
    r = float(norm(rv))
    v2 = float(dot(vv, vv))
    a = 1.0 / (2.0 / r - v2 / mu)
    e_vec = ((v2 - mu / r) * rv - dot(rv, vv) * vv) / mu
    e = float(norm(e_vec))
    return a * (1.0 - e), e, a

def solve_tune_dv(rv, vv, mu, rp_goal, iters=5):
    '''算"把近拱点设到 rp_goal 需要的横向 dv"。

    近拱点只由角动量 h = |r × v| 决定，双曲线下
        h_des = sqrt(2 mu rp + (rp v_inf)^2)
    所以在 SOI 入口沿顺行方向补一点横向速度，就能精确改 h，
    不必重做 Lambert，也不必等飞到近拱点。
    '''
    r = float(norm(rv))
    n_hat = unit(cross(rv, vv))
    t_hat = unit(cross(n_hat, unit(rv)))     # 顺行横向：加它 = 增大角动量
    dv_t = 0.0
    rp_new = rp_goal
    for _ in range(iters):
        v_new = vv + t_hat * dv_t
        rp_new, e_new, a_new = flyby_periapsis(rv, v_new, mu)
        h_new = float(norm(cross(rv, v_new)))
        if a_new < 0.0:                       # 双曲（无界）
            h_des = math.sqrt(2.0 * mu * rp_goal
                              + (rp_goal * math.sqrt(-mu / a_new)) ** 2)
        else:                                 # 意外变成束缚椭圆
            e_des = 1.0 - rp_goal / a_new
            if not (0.0 < e_des < 1.0):
                raise RuntimeError('到达轨道是束缚椭圆（a=%.1f km），'
                                   '靠横向推力抬不到 %.1f km 的近拱点'
                                   % (a_new / 1e3, rp_goal / 1e3))
            h_des = math.sqrt(mu * a_new * (1.0 - e_des * e_des))
        if abs(rp_new - rp_goal) < 200.0:
            break
        dv_t += (h_des - h_new) / r
    return dv_t, t_hat, rp_new

def tune_periapsis(conn, vessel, ut, rp_goal):
    '''SOI 入口做一次短促的横向推力，把近拱点定到 rp_goal（米，距质心）。

    Lambert 瞄的是 Mun 质心，轨迹就从质心穿过去 → 天然是一条"对月导弹"。
    与其在出发端死磕瞄准点（那东西对近拱点极其不敏感），
    不如在刚进 SOI、离 Mun 还有 2400 km 的时候补一发几十 m/s 的横向推力：
        · 花掉 ~80 m/s
        · 圆化 dv 从 -805 降到 -426，净省 300+ m/s
    '''
    sc = conn.space_center
    body = vessel.orbit.body
    mu = body.gravitational_parameter
    frame = body.non_rotating_reference_frame
    rv = array(vessel.position(frame))
    vv = array(vessel.velocity(frame))
    r = float(norm(rv))

    rp_now = flyby_periapsis(rv, vv, mu)[0]
    print('进入 SOI: 距 %s 中心 %.0f km，近拱点 %.1f km（目标 %.1f km）'
          % (body.name, r / 1e3, rp_now / 1e3, rp_goal / 1e3))

    dv_t, t_hat, rp_new = solve_tune_dv(rv, vv, mu, rp_goal)
    if abs(dv_t) < 1.0:
        print('近拱点已合格，跳过修正')
        return

    ve = vessel.specific_impulse * G0
    burn, _, _ = burn_time_for(vessel, abs(dv_t), ve)
    t_burn = ut() + burn / 2.0 + 3.0
    # 目标是一个"横向"推力，但节点轴是(速度, 径向, 法向)，这里必须转换。
    # 速度几乎全在径向上时，prograde/radial 两个分量会是一对大数近似抵消，
    # 这是几何决定的，不是算错。
    dv_vec = t_hat * dv_t
    np_p, np_r, np_n = node_components(dv_vec, rv, vv)
    node = vessel.control.add_node(t_burn, prograde=np_p, radial=np_r,
                                   normal=np_n)
    print('修正近拱点：横向 dv = %+.2f m/s -> 节点 (P %+.2f / R %+.2f / N %+.2f)，'
          '燃烧 %.1f s' % (dv_t, np_p, np_r, np_n, burn))
    vessel.control.sas = True
    time.sleep(0.2)
    vessel.control.sas_mode = sc.SASMode.maneuver
    burn_to_node(conn, vessel, node, burn, t_burn - burn / 2.0, ut, '修正近拱点')
    node.remove()
    print('修正后近拱点 %.1f km' % (vessel.orbit.periapsis / 1e3))

def scan_windows(vessel, mun, mu, frame, ut0, r0v, v0v, period,
                 tof_list=(2.0 * 3600.0, 2.5 * 3600.0, 3.0 * 3600.0), step=120.0):
    '''在一个停泊周期内扫出发时刻，返回 [(tof, lead, dv, theta_deg, v2), ...]。

    每个 TOF 只保留 dv 最小的那一个出发时刻。
    '''
    out = []
    for tof in tof_list:
        best = None
        lead = 0.0
        while lead < period:
            try:
                r1v, v_now = kepler_state(r0v, v0v, lead, mu)
                r1 = float(norm(r1v))
                r2v = array(mun.orbit.position_at(ut0 + lead + tof, frame))
                r2 = float(norm(r2v))
                h1v = cross(r1v, v_now)
                cosd = max(-1.0, min(1.0, float(dot(r1v, r2v)) / (r1 * r2)))
                theta = math.acos(cosd)
                long_way = dot(cross(r1v, r2v), h1v) < 0.0
                v1, v2, _ = lambert(r1v, r2v, tof, mu, long_way=long_way)
                dv = float(norm(v1 - v_now))
                if best is None or dv < best[1]:
                    best = (lead, dv, math.degrees(theta), v2)
            except (ValueError, ZeroDivisionError):
                pass
            lead += step
        if best is not None:
            out.append((tof, best[0], best[1], best[2], best[3]))
    return out

def scan_departure(tof_list=(2.0 * 3600.0, 2.5 * 3600.0, 3.0 * 3600.0),
                   step=120.0):
    '''扫一个停泊周期内的出发时刻，打印 dv 最小的窗口。

    用法：python lanbo.py scan
    把打印出来的 T_DEP_UT / T_TRANSFER 填回文件顶部再跑 main。
    '''
    conn = krpc.connect(name='lambert scan')
    sc = conn.space_center
    vessel = sc.active_vessel
    mun = sc.bodies['Mun']
    body = vessel.orbit.body
    mu = body.gravitational_parameter
    frame = body.non_rotating_reference_frame

    ut0 = sc.ut
    r0v = array(vessel.position(frame))
    v0v = array(vessel.velocity(frame))
    period = vessel.orbit.period
    print('飞船 %s，停泊轨道周期 %.0f s（%.2f h）'
          % (vessel.name, period, period / 3600.0))
    print('扫描步长 %.0f s，每个 TOF 扫 %.0f 个点' % (step, period / step))

    res = scan_windows(vessel, mun, mu, frame, ut0, r0v, v0v, period,
                       tof_list=tof_list, step=step)
    for tof, lead, dv, th, v2 in res:
        print('--- T_TRANSFER = %.2f h ---' % (tof / 3600.0))
        print('  dv = %.1f m/s @ T_DEP_LEAD = %.0f s（转移角 %.1f deg，'
              '|v2| = %.1f m/s）' % (dv, lead, th, norm(v2)))
        print('  对应绝对时刻 T_DEP_UT = %.1f' % (ut0 + lead))

    if not res:
        print('一个窗口都没解出来，检查飞船是否还在 Kerbin 轨道上')
        return

    res = sorted(res, key=lambda r: r[2])
    tof, lead, dv, th, v2 = res[0]
    print('')
    print('最佳：T_TRANSFER = %.2f h, T_DEP_UT = %.1f（= 现在 + %.0f s），'
          'dv = %.1f m/s' % (tof / 3600.0, ut0 + lead, lead, dv))
    print('把 T_TRANSFER 和 T_DEP_UT 两个值填到文件顶部，再跑 python lanbo.py')
    if dv > 1300.0:
        print('注：dv 仍偏大，当前相位不理想，可以等飞船再绕一圈重新 scan')

def main():
    conn = krpc.connect(name='lambert transfer')
    sc = conn.space_center
    vessel = sc.active_vessel
    mun = sc.bodies['Mun']
    body = vessel.orbit.body
    mu = body.gravitational_parameter
    ut = conn.add_stream(getattr, sc, 'ut')
    frame = body.non_rotating_reference_frame

    print('飞船:', vessel.name)

    # ---- 1. 出发状态 ----
    # kRPC 的 Orbit 只有 position_at / radius_at / orbital_speed_at，没有 velocity_at，
    # 所以这里取当前的 r、v，用二体传播推到 t_dep，位置和速度一起得到。
    r0v = array(vessel.position(frame))
    v0v = array(vessel.velocity(frame))
    # 读完状态再取"现在"（ut 流可能滞后一帧，直接用 sc.ut 更准），
    # 保证 dt 和状态是同一时刻，交叉验证误差从 km 级降到几十米
    ut0 = float(sc.ut)
    t_dep = T_DEP_UT if T_DEP_UT is not None else ut0 + T_DEP_LEAD
    dt_dep = t_dep - ut0
    if dt_dep < 1.0:
        raise RuntimeError('出发时刻已过（t_dep - now = %.0f s），'
                           '检查 T_DEP_UT / T_DEP_LEAD' % dt_dep)
    r1v, v_now = kepler_state(r0v, v0v, dt_dep, mu)
    r1 = norm(r1v)
    print('出发: |r1| = %.2f km, |v| = %.2f m/s, 停泊半长轴 = %.2f km'
          % (r1 / 1e3, norm(v_now), vessel.orbit.semi_major_axis / 1e3))

    # 顺带用 kRPC 自己的位置做一次交叉验证（位置应该有，速度没有）
    r_krpc = array(vessel.orbit.position_at(t_dep, frame))
    print('  位置交叉验证：与 position_at 相差 %.1f m' % norm(r1v - r_krpc))

    # ---- 2. 到达时刻 Mun 的位置 ----
    t_arr = t_dep + T_TRANSFER
    r2v = array(mun.orbit.position_at(t_arr, frame))
    r2 = norm(r2v)
    print('到达: |r2| = %.2f km' % (r2 / 1e3))

    # ---- 3. 转移角与分支 ----
    h1v = cross(r1v, v_now)
    cosd = dot(r1v, r2v) / (r1 * r2)
    cosd = max(-1.0, min(1.0, cosd))
    theta_short = math.acos(cosd)
    long_way = dot(cross(r1v, r2v), h1v) < 0.0
    dtheta = 2 * pi - theta_short if long_way else theta_short
    print('转移角 %.2f deg (%s)'
          % (math.degrees(dtheta), '长程' if long_way else '短程'))

    # ---- 4. Lambert ----
    v1, v2, z = lambert(r1v, r2v, T_TRANSFER, mu, long_way=long_way)
    print('Lambert: |v1| = %.2f m/s, |v2| = %.2f m/s'
          % (norm(v1), norm(v2)))

    # ---- 5. 自检 ----
    err = -1.0
    try:
        r_check = kepler_propagate(r1v, v1, T_TRANSFER, mu)
        err = float(norm(r_check - r2v))
        print('自检: 传播 %.0f s 后与目标位置相差 %.1f m' % (T_TRANSFER, err))
    except ValueError as e:
        print('自检跳过：%s' % e)
    if err > SELF_CHECK_TOL:
        raise RuntimeError('Lambert 解自检失败，误差 %.0f m，先别点火' % err)

    # ---- 6. dv 分解 ----
    dv = v1 - v_now
    # 用节点真实轴（速度方向 / 径向 / 法向）解出分量，别用 t̂ 直接点乘
    dv_p, dv_r, dv_n = node_components(dv, r1v, v_now)
    dv_mag = float(norm(dv))
    print('dv = %.2f m/s  (prograde %.2f, radial %.2f, normal %.2f)'
          % (dv_mag, dv_p, dv_r, dv_n))

    # dv 明显偏大 = 出发时刻/飞行时长选错了（正常 LKO->Mun 约 800~1100 m/s）
    if dv_mag > DV_SANITY:
        print('')
        print('× dv = %.0f m/s 明显偏大（radial 分量 %.0f m/s），'
              '这是一条近乎垂直的高抛弹道，点火也打不中 Mun' % (dv_mag, dv_r))
        if AUTO_SCAN:
            print('自动扫描出发窗口...')
            tofs = sorted(set([T_TRANSFER, 2.0 * 3600.0, 2.5 * 3600.0,
                               3.0 * 3600.0]))
            res = scan_windows(vessel, mun, mu, frame, ut0, r0v, v0v,
                               vessel.orbit.period, tof_list=tofs)
            for tof, lead, dv_s, th, v2s in res:
                print('  T_TRANSFER = %.2f h -> dv = %.1f m/s  '
                      '（转移角 %.1f deg，T_DEP_UT = %.1f）'
                      % (tof / 3600.0, dv_s, th, ut0 + lead))
            if res:
                res = sorted(res, key=lambda r: r[2])
                tof, lead, dv_s, th, v2s = res[0]
                print('  推荐：T_TRANSFER = %.2f h，T_DEP_UT = %.1f，dv = %.1f m/s'
                      % (tof / 3600.0, ut0 + lead, dv_s))
        raise RuntimeError('导航几何不对，本次不点火。'
                           '把上面推荐的 T_TRANSFER / T_DEP_UT 填到文件顶部再跑')

    # ---- 7. 燃烧时长 ----
    # 注意必须在点火前现取 Isp / 质量 / 推力，别用旧值
    ve = vessel.specific_impulse * G0
    if ve <= 0.0:
        raise RuntimeError('specific_impulse 为 0，发动机没点火/被禁用')
    burn_time, mass, m_after = burn_time_for(vessel, dv_mag, ve)
    print('预计燃烧 %.1f s（%.1f kg -> %.1f kg）' % (burn_time, mass, m_after))

    # ---- 8. 建节点 ----
    node = vessel.control.add_node(t_dep,
                                   prograde=dv_p, radial=dv_r, normal=dv_n)
    vessel.control.sas = True
    time.sleep(0.2)
    vessel.control.sas_mode = sc.SASMode.maneuver

    # ---- 9. 出发点火：以 t_dep 为中心，提前 burn_time/2 开始 ----
    burn_start = t_dep - burn_time / 2.0
    if burn_start < ut() + 5.0:
        raise RuntimeError('点火时刻太近，先把飞船开出去或调大 T_DEP_LEAD')

    sc.warp_to(burn_start - 25.0)
    vessel.control.sas = True
    vessel.control.sas_mode = sc.SASMode.maneuver
    burn_to_node(conn, vessel, node, burn_time, burn_start, ut, '出发点火')
    node.remove()

    print('点火后: 近拱点 %.1f km, 远拱点 %.1f km'
          % (vessel.orbit.periapsis / 1e3, vessel.orbit.apoapsis / 1e3))

    # ---- 10. 等 SOI 切换 ----
    # time_to_soi_change 在"不会切换"时返回的是 NaN，不是 None，必须用 isfinite 判
    tts = vessel.orbit.time_to_soi_change
    if tts is None or not math.isfinite(tts) or tts <= 0.0:
        raise RuntimeError('没有找到 SOI 切换，转移没打中')
    print('SOI 切换还有 %.0f s' % tts)
    sc.warp_to(ut() + tts + 20.0)

    mu = vessel.orbit.body.gravitational_parameter
    print('已进入 %s 的 SOI' % vessel.orbit.body.name)

    # ---- 11. 修正近拱点 ----
    # Lambert 瞄的是质心 → 轨迹穿心而过，天然是"对月导弹"。
    # 这里在 SOI 入口补一发横向推力，把近拱点抬到最后一段的目标高度。
    tune_periapsis(conn, vessel, ut, body_radius(vessel.orbit.body) + PERI_ALT)

    # ---- 12. 圆化 ----
    rp = vessel.orbit.periapsis
    a1 = vessel.orbit.semi_major_axis
    rp_alt = rp - body_radius(vessel.orbit.body)
    if rp_alt < 2000.0:
        raise RuntimeError('近拱点只有 %.1f km 高（地表），会撞 %s'
                           % (rp_alt / 1e3, vessel.orbit.body.name))
    v_peri = math.sqrt(mu * (2.0 / rp - 1.0 / a1))
    v_circ = math.sqrt(mu * (2.0 / rp - 1.0 / rp))
    dv2 = v_circ - v_peri
    print('圆化 dv = %.2f m/s（近拱点 %.1f km，地表以上 %.1f km）'
          % (dv2, rp / 1e3, rp_alt / 1e3))
    if dv2 > 0:
        raise RuntimeError('dv 应为负（需要减速），检查转移是否打过了头')

    # 转移点火可能已经抛掉了一级，Isp / 质量 / 推力都要重新取
    ve = vessel.specific_impulse * G0
    burn_time2, m0, m1 = burn_time_for(vessel, abs(dv2), ve)

    t_peri = ut() + vessel.orbit.time_to_periapsis
    node2 = vessel.control.add_node(t_peri, prograde=dv2)
    burn_start2 = t_peri - burn_time2 / 2.0     # 这次真的用上了
    if burn_start2 > ut() + 1.0:
        sc.warp_to(burn_start2 - 25.0)
    else:
        print('圆化点火时刻太近，跳过跃迁直接等')
    vessel.control.sas = True
    vessel.control.sas_mode = sc.SASMode.maneuver
    burn_to_node(conn, vessel, node2, burn_time2, burn_start2, ut, '圆化')
    node2.remove()

    print('完成: 近拱点 %.1f km, 远拱点 %.1f km, 周期 %.0f s'
          % (vessel.orbit.periapsis / 1e3, vessel.orbit.apoapsis / 1e3,
             vessel.orbit.period))

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'scan':
        scan_departure()
    else:
        main()
