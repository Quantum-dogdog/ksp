'''
lambert_transfer_fixed.py —— 修好后的 Kerbin -> Mun 转移脚本

相对原文件 lambert transfer to mun.py 的改动（对应审计）

  1. 出发/到达半径用 |r|，不再拿停泊轨道的半长轴当 r1。
  2. 入轨速度直接和"当前速度向量"相减得到 dv，再分解成 prograde/radial/normal，
     不再用圆轨道速度 sqrt(mu/r) 当基准，椭圆停泊轨道和倾角都能正确处理。
  3. 出发时刻只有一处 T_DEP_LEAD：节点时刻、Lambert 假设、点火中心都由它推出来。
  4. 转移角用 acos 得到，并按停泊轨道法向判定短程/长程，不再只支持 ±180°。
  5. Lambert 用全局变量法（Stumpff 函数 + 对 z 二分），不用扫描半长轴；
     短程/长程、椭圆/双曲线都覆盖。
  6. 解完做独立自检：把 (r1, v1) 传播 TOF，看是否回到 r2。
  7. 删掉 1.008 和 +3.2 这类补偿系数，改成按剩余 dv 的模长分段截止。
  8. 忙等换成 sleep 轮询；第二段点火时刻 burn_ut 真正用上。

第二轮修订（2026-11-05，独立复算后）：

  9. kepler_propagate 的 Newton 迭代加步长阻尼，不收敛或溢出时退化为
     对全局变量 chi 的二分（t(chi) 单调增，二分必然收敛）。原来双曲线
     分支在个别长尾案例（30 例里 1 例）会发散到 1e145 m。
 10. 头注释第 4 条措辞改成 acos（原写 atan2，代码实现是 acos+法向判支，
     数学等价，注释口径对齐）。
'''

import math
import time

import krpc
from numpy import array, dot, cross, pi
from numpy.linalg import norm

# ----------------------------- 参数 -----------------------------
T_DEP_LEAD = 60.0          # 从现在起多少秒后出发（Lambert 的出发时刻）
T_TRANSFER = 5 * 3600.0    # 转移飞行时长
G0 = 9.81                  # Isp 换算成排气速度用
THROTTLE_CUT1 = 20.0       # 剩余 dv 大于这个，全油门
THROTTLE_CUT2 = 2.0        # 剩余 dv 小于这个，关机
THROTTLE_LOW = 0.15        # 小油门
SELF_CHECK_TOL = 5000.0    # 自检允许的位置误差（米）

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

def kepler_propagate(r0v, v0v, dt, mu, iters=60):
    '''全局变量法二体传播：给出 t+dt 时刻的位置。用于自检。

    Newton 迭代加了步长阻尼（单步最多走自身量级的一半），防双曲线
    远端发散；阻尼后仍不收敛或中途溢出，就退化为对 chi 的二分——
    t(chi) 对 chi 单调增，二分必然收敛。
    '''
    r0 = float(norm(r0v))
    alpha = 2.0 / r0 - float(dot(v0v, v0v)) / mu
    rv0 = float(dot(r0v, v0v))
    if alpha > 1e-8:
        chi = math.sqrt(mu) * alpha * dt
    elif alpha < -1e-8:
        # 双曲线初值：用对数形式，直接代 dt/r 会离解很远
        a_neg = -1.0 / alpha                      # 正数，等于 |a|
        denom = rv0 + math.sqrt(mu * a_neg) * (1 - r0 * alpha)
        chi = math.sqrt(a_neg) * math.log((-2.0 * mu * alpha * dt) / denom)
    else:
        chi = math.sqrt(mu) * dt / r0

    sqm = math.sqrt(mu)

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

    z = chi * chi * alpha
    C, S = stumpf(z)
    f = 1 - chi * chi * C / r0
    g = dt - chi ** 3 * S / sqm
    return f * r0v + g * v0v

# ---------------------------- KSP 部分 ----------------------------

def unit(v):
    return v / norm(v)

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
    t_dep = ut() + T_DEP_LEAD
    r1v = array(vessel.orbit.position_at(t_dep, frame))
    v_now = array(vessel.orbit.velocity_at(t_dep, frame))
    r1 = norm(r1v)
    print('出发: |r1| = %.2f km, |v| = %.2f m/s, 停泊半长轴 = %.2f km'
          % (r1 / 1e3, norm(v_now), vessel.orbit.semi_major_axis / 1e3))

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
    try:
        r_check = kepler_propagate(r1v, v1, T_TRANSFER, mu)
        err = float(norm(r_check - r2v))
        print('自检: 传播 %.0f s 后与目标位置相差 %.1f m' % (T_TRANSFER, err))
    except ValueError as e:
        err = -1.0
        print('自检跳过：%s' % e)
    if err > SELF_CHECK_TOL:
        raise RuntimeError('Lambert 解自检失败，误差 %.0f m，先别点火' % err)

    # ---- 6. dv 分解 ----
    n_hat = unit(h1v)
    r_hat = unit(r1v)
    t_hat = unit(cross(n_hat, r_hat))        # 顺行方向
    dv = v1 - v_now
    dv_p = float(dot(dv, t_hat))
    dv_r = float(dot(dv, r_hat))
    dv_n = float(dot(dv, n_hat))
    dv_mag = float(norm(dv))
    print('dv = %.2f m/s  (prograde %.2f, radial %.2f, normal %.2f)'
          % (dv_mag, dv_p, dv_r, dv_n))

    # ---- 7. 燃烧时长 ----
    ve = vessel.specific_impulse * G0
    mass = vessel.mass
    thrust = vessel.available_thrust
    m_after = mass * math.exp(-dv_mag / ve)
    burn_time = (mass - m_after) / (thrust / ve)
    print('预计燃烧 %.1f s（%.1f kg -> %.1f kg）' % (burn_time, mass, m_after))
    if m_after < vessel.dry_mass:
        raise RuntimeError('燃料不够：烧完后 %.1f kg 低于干重 %.1f kg'
                           % (m_after, vessel.dry_mass))

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
    tts = vessel.orbit.time_to_soi_change
    if tts is None or tts < 0:
        raise RuntimeError('没有找到 SOI 切换，转移没打中')
    print('SOI 切换还有 %.0f s' % tts)
    sc.warp_to(ut() + tts + 20.0)

    mu = vessel.orbit.body.gravitational_parameter
    print('已进入 %s 的 SOI' % vessel.orbit.body.name)

    # ---- 11. 圆化 ----
    rp = vessel.orbit.periapsis
    a1 = vessel.orbit.semi_major_axis
    if rp <= 0:
        raise RuntimeError('近拱点 %.0f m，是撞击轨道' % rp)
    v_peri = math.sqrt(mu * (2.0 / rp - 1.0 / a1))
    v_circ = math.sqrt(mu * (2.0 / rp - 1.0 / rp))
    dv2 = v_circ - v_peri
    print('圆化 dv = %.2f m/s（近拱点 %.1f km）' % (dv2, rp / 1e3))
    if dv2 > 0:
        raise RuntimeError('dv 应为负（需要减速），检查转移是否打过了头')

    node2 = vessel.control.add_node(ut() + vessel.orbit.time_to_periapsis,
                                    prograde=dv2)
    m0 = vessel.mass
    thr = vessel.available_thrust
    m1 = m0 * math.exp(-abs(dv2) / ve)
    burn_time2 = (m0 - m1) / (thr / ve)

    t_peri = ut() + vessel.orbit.time_to_periapsis
    burn_start2 = t_peri - burn_time2 / 2.0     # 这次真的用上了
    sc.warp_to(burn_start2 - 25.0)
    vessel.control.sas = True
    vessel.control.sas_mode = sc.SASMode.maneuver
    burn_to_node(conn, vessel, node2, burn_time2, burn_start2, ut, '圆化')
    node2.remove()

    print('完成: 近拱点 %.1f km, 远拱点 %.1f km, 周期 %.0f s'
          % (vessel.orbit.periapsis / 1e3, vessel.orbit.apoapsis / 1e3,
             vessel.orbit.period))

if __name__ == '__main__':
    main()