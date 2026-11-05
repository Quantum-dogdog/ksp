'''
G-FOLD 修好的 pyomo 版（对应原文件 G-FOLD(pyomo_version).py）

相对原文件改了四处模型问题：

1. 推力改成三维向量 (ux, uy, uz)，再加一个"推力大小" un 和
   un**2 == ux**2 + uy**2 + uz**2 的约束。原文件只有一个标量 u，
   三个力方程都用它，等于把推力方向锁死在 (1,1,1)。

2. m.T 改成"剩余飞行时长"(秒)，bounds=(5, 150)。原文件把绝对 UT
   塞进了 T，量级差了 1e7。

3. 归一化时间 tau ∈ [0,1]，物理时间 t = tau * T。所有导数都带 T 因子：
       dx/dtau = T * vx
       dvx/dtau = T * (F/m)
   这样力方程里就不用再补 1/T**2（原文件漏了这个因子，03.py 里是对的）。

4. 目标函数改成 ∫ T * un dtau，也就是总燃料质量。原文件用的是 ∫ un dtau，
   那等于 燃料/T，T 自由时会被推到上界。

坐标约定：x 竖直向上，y、z 水平。和 kRPC target_frame 返回的 (x, y, z)
顺序直接对应，重力写在 x 那条方程上。

这份仍然是非线性规划（m * a 是双线性），所以必须给初值，并且要检查
求解器的 termination_condition。想要严格的凸版本，看 gfold_convex.py。

第二轮修订（2026-11-05，独立复算后）：

  5. ContinuousSet 改从 pyomo.dae 导入。pyomo >= 6.x 已把它从
     pyomo.environ 移走，原写法在新版直接 ImportError（旧版两个入口
     都在，改后两边都能跑）。
  6. solve_and_report 先查 solver.available()，ipopt 不在 PATH 时给出
     可读的报错，而不是 pyomo 内部的 ApplicationError。
'''

import time
import numpy as np

from pyomo.environ import (ConcreteModel, Var, Constraint, Objective,
                           TransformationFactory, SolverFactory,
                           value)
from pyomo.dae import ContinuousSet, DerivativeVar, Integral

def build_model(p0, v0, m0, dry_mass, u_max, v_exhaust, g,
                t_min=5.0, t_max=150.0, nfe=80, ncp=3):
    '''
    p0, v0 : (x, y, z) 三元的初始位置、速度，x 向上
    m0     : 初始质量 kg
    u_max  : 最大质量流量 kg/s，u_max = 最大推力 / 排气速度
    '''
    assert u_max > 0
    assert m0 > dry_mass

    m = ConcreteModel()

    m.t = ContinuousSet(bounds=(0, 1))          # 归一化时间 tau
    m.T = Var(bounds=(t_min, t_max))            # 剩余飞行时长 [s]

    # ---- 状态 ----
    m.x = Var(m.t, bounds=(-20000, 20000))
    m.y = Var(m.t, bounds=(-20000, 20000))
    m.z = Var(m.t, bounds=(-20000, 20000))

    m.vx = Var(m.t, bounds=(-3000, 3000))
    m.vy = Var(m.t, bounds=(-3000, 3000))
    m.vz = Var(m.t, bounds=(-3000, 3000))

    m.mass = Var(m.t, bounds=(dry_mass, m0))

    # ---- 控制：三维质量流量 + 它的大小 ----
    m.ux = Var(m.t, bounds=(-u_max, u_max))
    m.uy = Var(m.t, bounds=(-u_max, u_max))
    m.uz = Var(m.t, bounds=(-u_max, u_max))
    m.un = Var(m.t, bounds=(0, u_max))

    # ---- 对 tau 的导数 ----
    m.dx = DerivativeVar(m.x, wrt=m.t)
    m.dy = DerivativeVar(m.y, wrt=m.t)
    m.dz = DerivativeVar(m.z, wrt=m.t)
    m.dvx = DerivativeVar(m.vx, wrt=m.t)
    m.dvy = DerivativeVar(m.vy, wrt=m.t)
    m.dvz = DerivativeVar(m.vz, wrt=m.t)
    m.dm = DerivativeVar(m.mass, wrt=m.t)

    # ---- 运动学：d/dtau = T * d/dt ----
    m.kin_x = Constraint(m.t, rule=lambda m, t: m.dx[t] == m.T * m.vx[t])
    m.kin_y = Constraint(m.t, rule=lambda m, t: m.dy[t] == m.T * m.vy[t])
    m.kin_z = Constraint(m.t, rule=lambda m, t: m.dz[t] == m.T * m.vz[t])

    # ---- 动力学：m * a = 推力，a = dv/dtau / T ----
    m.dyn_x = Constraint(m.t, rule=lambda m, t:
        m.dvx[t] == m.T * (m.ux[t] * v_exhaust / m.mass[t] - g))
    m.dyn_y = Constraint(m.t, rule=lambda m, t:
        m.dvy[t] == m.T * (m.uy[t] * v_exhaust / m.mass[t]))
    m.dyn_z = Constraint(m.t, rule=lambda m, t:
        m.dvz[t] == m.T * (m.uz[t] * v_exhaust / m.mass[t]))

    # ---- 推力大小：这就是原来没有的范数约束 ----
    m.thrust_norm = Constraint(m.t, rule=lambda m, t:
        m.un[t] ** 2 == m.ux[t] ** 2 + m.uy[t] ** 2 + m.uz[t] ** 2)

    # ---- 质量：dm/dtau = -T * 推力大小 ----
    m.mass_rate = Constraint(m.t, rule=lambda m, t: m.dm[t] == -m.T * m.un[t])

    # ---- 目标：总燃料 ----
    m.fuel = Integral(m.t, wrt=m.t, rule=lambda m, t: m.T * m.un[t])
    m.obj = Objective(expr=m.fuel)

    # ---- 边界条件 ----
    m.x[0].fix(p0[0])
    m.y[0].fix(p0[1])
    m.z[0].fix(p0[2])
    m.vx[0].fix(v0[0])
    m.vy[0].fix(v0[1])
    m.vz[0].fix(v0[2])
    m.mass[0].fix(m0)

    m.x[1].fix(0)
    m.y[1].fix(0)
    m.z[1].fix(0)
    m.vx[1].fix(0)
    m.vy[1].fix(0)
    m.vz[1].fix(0)

    # ---- 离散化 ----
    TransformationFactory('dae.collocation').apply_to(m, nfe=nfe, ncp=ncp)

    # ---- 初值：从起点线性插到终点，IPOPT 必须要有 ----
    for tau in m.t:
        m.x[tau].value = p0[0] * (1 - tau)
        m.y[tau].value = p0[1] * (1 - tau)
        m.z[tau].value = p0[2] * (1 - tau)
        m.vx[tau].value = v0[0] * (1 - tau)
        m.vy[tau].value = v0[1] * (1 - tau)
        m.vz[tau].value = v0[2] * (1 - tau)
        m.mass[tau].value = m0 - 0.3 * (m0 - dry_mass) * tau
        m.ux[tau].value = 0.0
        m.uy[tau].value = 0.0
        m.uz[tau].value = 0.0
        m.un[tau].value = 0.1 * u_max
    m.T.value = 60.0

    return m

def solve_and_report(m, tee=True):
    solver = SolverFactory('ipopt')
    if not solver.available():
        raise RuntimeError('找不到 ipopt 可执行文件。先安装（如 conda install '
                           '-c conda-forge ipopt）并确认它在 PATH 里')
    solver.options['max_iter'] = 5000
    solver.options['tol'] = 1e-5
    t0 = time.time()
    results = solver.solve(m, tee=tee)
    print('solver time: %.1f s' % (time.time() - t0))

    status = results.solver.termination_condition
    print('termination_condition:', status)
    if str(status) != 'optimal':
        print('没有收敛到最优，后面的轨迹不要用')
        return None

    print('飞行时长 T = %.2f s' % value(m.T))
    print('总燃料 = %.1f kg' % value(m.fuel))

    print('  tau    un(kg/s)    |u|check')
    for tau in m.t:
        ux = value(m.ux[tau]) if value(m.ux[tau]) is not None else 0.0
        uy = value(m.uy[tau]) if value(m.uy[tau]) is not None else 0.0
        uz = value(m.uz[tau]) if value(m.uz[tau]) is not None else 0.0
        un = value(m.un[tau]) if value(m.un[tau]) is not None else 0.0
        check = (ux * ux + uy * uy + uz * uz) ** 0.5
        print('  %5.2f  %8.2f   %8.2f' % (tau, un, check))
    return m

def sample_profile(m):
    '''把离散点上的解取出来，按 tau 升序排好。'''
    prof = []
    for tau in sorted(m.t):
        prof.append((tau,
                     value(m.ux[tau]) or 0.0,
                     value(m.uy[tau]) or 0.0,
                     value(m.uz[tau]) or 0.0,
                     value(m.un[tau]) or 0.0))
    return prof

def sample_at(prof, tau):
    '''线性插值：返回该 tau 处的 (ux, uy, uz, un)。'''
    if tau <= prof[0][0]:
        return prof[0][1:]
    if tau >= prof[-1][0]:
        return prof[-1][1:]
    for i in range(len(prof) - 1):
        a, b = prof[i], prof[i + 1]
        if a[0] <= tau <= b[0]:
            w = 0.0 if b[0] == a[0] else (tau - a[0]) / (b[0] - a[0])
            return tuple(a[k] * (1 - w) + b[k] * w for k in (1, 2, 3, 4))
    return prof[-1][1:]

def fly(m, conn, vessel, target_frame, ut, u_max, t_start=None):
    '''把解出来的推力剖面喂给飞船：油门按 un/u_max，方向用节点+SAS 指向。'''
    prof = sample_profile(m)
    T = value(m.T)
    nonrot = vessel.orbit.body.non_rotating_reference_frame
    node = None
    next_dir = -1.0
    t0 = ut() if t_start is None else t_start
    lag = ut() - t0
    if lag > 2.0:
        print('注意：状态采样到现在已经过了 %.1f s，剖面已滞后，前段会有偏差' % lag)
    print('开始执行：T = %.1f s，共 %d 个采样点' % (T, len(prof)))

    while True:
        t_rel = ut() - t0
        if t_rel > T + 3.0:
            print('时间到')
            break
        pos = vessel.position(target_frame)
        if pos[0] < 1.5:
            print('已接地')
            break

        ux, uy, uz, un = sample_at(prof, t_rel / T)
        fmag = (ux * ux + uy * uy + uz * uz) ** 0.5

        if un <= 1e-9 or fmag <= 1e-9:
            vessel.control.throttle = 0.0
        else:
            vessel.control.throttle = min(1.0, un / u_max)
            if t_rel >= next_dir:
                # 方向：目标系 -> 惯量系，再拆成 (顺行, 径向, 法向)
                d = np.array(conn.space_center.transform_direction(
                    (ux / fmag, uy / fmag, uz / fmag), target_frame, nonrot))
                d = d / np.linalg.norm(d)
                pv = np.array(vessel.position(nonrot))
                vv = np.array(vessel.velocity(nonrot))
                r_hat = pv / np.linalg.norm(pv)
                n_hat = np.cross(pv, vv)
                n_hat = n_hat / np.linalg.norm(n_hat)
                t_hat = np.cross(n_hat, r_hat)

                if node is not None:
                    node.remove()
                node = vessel.control.add_node(
                    ut() + 2.0,
                    prograde=float(d @ t_hat),
                    radial=float(d @ r_hat),
                    normal=float(d @ n_hat))
                vessel.control.sas = True
                vessel.control.sas_mode = conn.space_center.SASMode.maneuver
                next_dir = t_rel + 0.5

        time.sleep(0.05)

    vessel.control.throttle = 0.0
    if node is not None:
        node.remove()
    p_end = vessel.position(target_frame)
    v_end = vessel.velocity(target_frame)
    print('结束：高度 %.2f m，速度 %.2f m/s'
          % (p_end[0], (v_end[0] ** 2 + v_end[1] ** 2 + v_end[2] ** 2) ** 0.5))

if __name__ == '__main__':
    import krpc
    from math import sin, cos, pi

    conn = krpc.connect(name='gfold')
    space_center = conn.space_center
    vessel = space_center.active_vessel
    body = vessel.orbit.body
    g = 9.81

    # 着陆点坐标系，和原文件一致
    lat = -0.10
    lon = -74.55
    body_frame = body.reference_frame
    temp1 = space_center.ReferenceFrame.create_relative(body_frame,
        rotation=(0., sin(-lon / 2. * pi / 180), 0., cos(-lon / 2. * pi / 180)))
    temp2 = space_center.ReferenceFrame.create_relative(temp1,
        rotation=(0., 0., sin(lat / 2. * pi / 180), cos(lat / 2. * pi / 180)))
    height = body.surface_height(lat, lon) + body.equatorial_radius
    target_frame = space_center.ReferenceFrame.create_relative(temp2,
        position=(height, 0., 0.))

    ut = conn.add_stream(getattr, space_center, 'ut')
    t_start = ut()          # 采样状态的时刻，执行时以它为 t = 0

    # 注意：position/velocity 返回的就是 (x, y, z)，按同一顺序喂进去
    p0 = vessel.position(target_frame)
    v0 = vessel.velocity(target_frame)
    m0 = vessel.mass
    dry_mass = vessel.dry_mass
    v_exhaust = vessel.specific_impulse * g
    u_max = vessel.available_thrust / v_exhaust

    print('p0 =', p0)
    print('v0 =', v0)
    print('m0 =', m0, ' dry =', dry_mass, ' u_max =', u_max)

    model = build_model(p0, v0, m0, dry_mass, u_max, v_exhaust, g)
    if solve_and_report(model) is None:
        raise SystemExit('没有可行解，不执行')

    fly(model, conn, vessel, target_frame, ut, u_max, t_start)