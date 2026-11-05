'''
gfold_convex.py —— G-FOLD 的凸版本（SOCP），带 KSP 飞控

和 gfold_fixed.py 的区别：
  * 这里假设质量在一个制导周期内不变（a = T/m 里的 m 当常数），
    所以问题严格是 SOCP：不需要初值，不会陷入局部解，有全局最优证书。
  * gfold_fixed.py 里质量是变量，m*a 是双线性，只能用 IPOPT 解 NLP。

两种用法：

    python gfold_convex.py            # 连 KSP，解算并执行
    python gfold_convex.py --demo     # 不连 KSP，跑内置算例，检查"无损"性质

--demo 会做的事：
  1. 对每个飞行时长 T 解一次 SOCP，取燃料最少的那次
  2. 检查最优解的推力大小是不是只落在 0 和 [a_min, a_max] 上
  3. 把 (0, a_min) 的换挡步吸附到 0，用同一套 Euler 步进开环复算，
     报告吸附后偏离目标落点的位置/速度——这就是离散化的实际代价

第二轮修订（2026-11-05，独立复算后）：

  4. gam 虚约束（gam>=a_min 只绑在松弛变量上，对 ‖a‖ 不起作用）换成
     规范的 σ-z 松弛：‖a‖<=σ，a_min·z<=σ<=a_max·z，z∈[0,1]。
     连续时间下这是无损凸化，最优解 bang；离散 Euler 网格上与旧写法
     数值等价（换挡步的中间值仍会出现），但形式对齐了文献，之后想升
     级成混合整数（z 真二值）只需换求解器。
  5. 新增 make_executable + rollout：解完把剖面吸附成引擎可执行的
     （0 或 >=a_min），再开环复算一遍终点偏差。锥下界非凸，SOCP
     结构上无法硬禁 (0,a_min)，可执行性由这一步兜底。fly() 执行前
     也走同一套吸附。
'''

import math
import sys
import time

import cvxpy as cp
import numpy as np

class GfoldConvex:

    def __init__(self, N=100, g=9.81, a_min=5.0, a_max=20.0,
                 p0=None, v0=None):
        self.N = N
        self.g = g
        self.a_min = a_min
        self.a_max = a_max
        self.p0 = np.zeros(3) if p0 is None else np.asarray(p0, float)
        self.v0 = np.zeros(3) if v0 is None else np.asarray(v0, float)

    def solve(self, T):
        '''给定飞行时长 T，解一次 SOCP。返回 (最优代价, 加速度序列)。'''
        N = self.N
        h = T / N

        p = [cp.Variable(3) for _ in range(N + 1)]
        v = [cp.Variable(3) for _ in range(N + 1)]
        a = [cp.Variable(3) for _ in range(N)]
        sig = [cp.Variable() for _ in range(N)]   # 推力大小的上界（松弛变量）
        z = [cp.Variable() for _ in range(N)]     # 0=关机 1=开机

        cons = [p[0] == self.p0, v[0] == self.v0,
                p[N] == np.zeros(3), v[N] == np.zeros(3)]

        grav = np.array([-self.g, 0.0, 0.0])
        cost = 0
        for k in range(N):
            # 规范 G-FOLD 松弛（lossless convexification）：
            #   ‖a‖ <= sig，  a_min*z <= sig <= a_max*z，  z ∈ [0,1]
            # 连续时间下可证无损：最优解 z 取 0/1、sig 只落在 0 和
            # [a_min, a_max] 上。离散 Euler 网格上这一组约束与旧写法
            # （下限绑在 gam 上）等价——换挡步仍可能出 (0, a_min) 的
            # 中间值，锥下界本身非凸，SOCP 结构上禁不掉；这部分由
            # make_executable 的吸附 + rollout 开环复核兜底。
            cons.append(cp.SOC(sig[k], a[k]))
            cons.append(sig[k] >= self.a_min * z[k])
            cons.append(sig[k] <= self.a_max * z[k])
            cons.append(z[k] <= 1.0)
            cons.append(z[k] >= 0.0)
            cons.append(p[k + 1] == p[k] + h * v[k])
            cons.append(v[k + 1] == v[k] + h * (grav + a[k]))
            cost = cost + h * sig[k]

        prob = cp.Problem(cp.Minimize(cost), cons)
        prob.solve(solver=cp.CLARABEL)
        if prob.status != cp.OPTIMAL:
            return float('inf'), None
        return prob.value, a

    def mag(self, a, k):
        ax, ay, az = a[k].value
        return math.sqrt(ax * ax + ay * ay + az * az)

def make_executable(tb, a_list):
    '''把解出的加速度剖面转成引擎能执行的：每步要么关机（0），要么 >= a_min。

    落在 (0, a_min) 的换挡步吸附到 0（比吸附到 a_min 省燃料）。
    返回 (剖面, 被吸附的步数)。
    '''
    prof = []
    snapped = 0
    for k in range(tb.N):
        vec = np.array([a_list[k].value[0], a_list[k].value[1], a_list[k].value[2]])
        s = float(np.linalg.norm(vec))
        if s < 1e-3:                # 关机步（含求解器的数值噪声）
            prof.append(np.zeros(3))
        elif s < tb.a_min:
            prof.append(np.zeros(3))
            snapped += 1
        else:
            prof.append(vec)
    return prof, snapped

def rollout(tb, prof, T):
    '''用与模型完全相同的 Euler 步进开环复算一遍，返回终点 (位置, 速度)。

    p[k+1] = p[k] + h*v[k]；v[k+1] = v[k] + h*(grav + a[k])，与 solve()
    里的约束逐项相同。终点偏差就是吸附（离散换挡）付出的实际代价。
    '''
    N = tb.N
    h = T / N
    p = np.array(tb.p0, float)
    v = np.array(tb.v0, float)
    grav = np.array([-tb.g, 0.0, 0.0])
    for k in range(N):
        p = p + h * v
        v = v + h * (grav + prof[k])
    return p, v

def best_over_T(tb, T_list):
    '''扫一遍飞行时长，取代价最小的那次。'''
    best_val, best_T, best_a = 1e9, 0, None
    for T in T_list:
        val, a = tb.solve(T)
        if a is None:
            print('%5d      无可行解' % T)
            continue
        print('%5d      %10.3f' % (T, val))
        if val < best_val:
            best_val, best_T, best_a = val, T, a
    return best_val, best_T, best_a

# ===================== 飞控 =====================

def sample_at(prof, T, t_rel):
    '''加速度剖面按时间线性插值。'''
    N = len(prof)
    h = T / N
    s = t_rel / h
    i = int(math.floor(s))
    if i < 0:
        return prof[0]
    if i >= N - 1:
        return prof[N - 1]
    w = s - i
    return prof[i] * (1 - w) + prof[i + 1] * w

def fly(tb, a_list, T, conn, vessel, target_frame, ut, thrust_max, t_start=None):
    '''把解出来的加速度剖面喂给飞船：油门按 m·‖a‖/推力上限，方向用节点 + SAS。'''
    # 引擎执行不了 (0, a_min) 的换挡步：先吸附成可执行剖面
    prof, snapped = make_executable(tb, a_list)
    if snapped:
        print('注意：%d 个换挡步低于最小油门，已吸附到 0' % snapped)
    nonrot = vessel.orbit.body.non_rotating_reference_frame
    node = None
    next_dir = -1.0
    t0 = ut() if t_start is None else t_start
    lag = ut() - t0
    if lag > 2.0:
        print('注意：状态采样到现在已经过了 %.1f s，剖面已滞后，前段会有偏差' % lag)
    print('开始执行：T = %.1f s，步长 %.2f s' % (T, T / tb.N))

    while True:
        t_rel = ut() - t0
        if t_rel > T + 3.0:
            print('时间到')
            break
        pos = vessel.position(target_frame)
        if pos[0] < 1.5:
            print('已接地')
            break

        acc = sample_at(prof, T, t_rel)
        amag = float(np.linalg.norm(acc))
        mass = vessel.mass

        if amag <= 1e-9:
            vessel.control.throttle = 0.0
        else:
            # 模型里 a 是"推力/质量"，实际油门 = 推力/推力上限 = m·‖a‖/T_max
            vessel.control.throttle = min(1.0, mass * amag / thrust_max)
            if t_rel >= next_dir:
                d = np.array(conn.space_center.transform_direction(
                    tuple(acc / amag), target_frame, nonrot))
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

# ===================== 两种入口 =====================

def run_demo():
    '''不连 KSP：跑内置算例，验证"无损"性质。'''
    tb = GfoldConvex()
    tb.p0 = np.array([1500.0, 500.0, 0.0])
    tb.v0 = np.array([-20.0, 0.0, 0.0])

    print('T [s]     燃料代价(∫‖a‖dt)')
    best_val, best_T, best_a = best_over_T(tb, range(20, 61, 5))
    if best_a is None:
        print('所有 T 都无可行解')
        raise SystemExit(1)

    print()
    print('最好的 T = %d s，代价 = %.3f' % (best_T, best_val))

    print()
    print('每步的推力大小（只打印有变化的那些）：')
    last = None
    for k in range(tb.N):
        s = tb.mag(best_a, k)
        tag = ''
        if 1e-3 < s < tb.a_min - 1e-3:
            tag = '   <-- 落在禁区内'
        if last is None or abs(s - last) > 0.5:
            print('  k=%3d   s=%7.2f%s' % (k, s, tag))
            last = s

    bad = [k for k in range(tb.N) if 1e-3 < tb.mag(best_a, k) < tb.a_min - 1e-3]
    print()
    print('推力落在 (0, a_min) 禁区的步数:', len(bad), '/', tb.N)
    for k in bad:
        print('  k=%3d   s=%.4f' % (k, tb.mag(best_a, k)))

    # ---- 吸附成可执行剖面，开环复算，量化离散换挡的代价 ----
    prof, snapped = make_executable(tb, best_a)
    p_end, v_end = rollout(tb, prof, best_T)
    print()
    print('吸附到可执行剖面：改了 %d 步' % snapped)
    print('开环复核（目标 p=0, v=0）：')
    print('  终点位置偏差 = %.4f m（各分量 %s）'
          % (float(np.linalg.norm(p_end)), np.array2string(p_end, precision=3)))
    print('  终点速度偏差 = %.4f m/s（各分量 %s）'
          % (float(np.linalg.norm(v_end)), np.array2string(v_end, precision=3)))

def run_ksp():
    '''连 KSP：读状态 → 扫 T 求最优 → 执行。'''
    import krpc

    conn = krpc.connect(name='gfold-convex')
    sc = conn.space_center
    vessel = sc.active_vessel
    body = vessel.orbit.body
    g = 9.81

    # 着陆点坐标系，和 gfold_fixed.py / 原文件一致
    lat = -0.10
    lon = -74.55
    body_frame = body.reference_frame
    temp1 = sc.ReferenceFrame.create_relative(body_frame,
        rotation=(0., math.sin(-lon / 2. * math.pi / 180), 0.,
                  math.cos(-lon / 2. * math.pi / 180)))
    temp2 = sc.ReferenceFrame.create_relative(temp1,
        rotation=(0., 0., math.sin(lat / 2. * math.pi / 180),
                  math.cos(lat / 2. * math.pi / 180)))
    height = body.surface_height(lat, lon) + body.equatorial_radius
    target_frame = sc.ReferenceFrame.create_relative(temp2,
        position=(height, 0., 0.))

    ut = conn.add_stream(getattr, sc, 'ut')
    t_start = ut()

    p0 = np.array(vessel.position(target_frame))
    v0 = np.array(vessel.velocity(target_frame))
    m0 = vessel.mass
    thrust_max = vessel.available_thrust
    a_max = thrust_max / m0
    a_min = 0.3 * a_max          # 发动机最小油门对应 30%，按你的引擎改

    print('p0 =', p0)
    print('v0 =', v0)
    print('m0 = %.1f kg   推力上限 = %.1f N   a_max = %.2f m/s^2' % (m0, thrust_max, a_max))

    tb = GfoldConvex(a_min=a_min, a_max=a_max, p0=p0, v0=v0)
    print()
    print('T [s]     燃料代价(∫‖a‖dt)')
    best_val, best_T, best_a = best_over_T(tb, range(10, 121, 5))
    if best_a is None:
        raise SystemExit('没有可行解，不执行')
    print()
    print('最好的 T = %d s，代价 = %.3f' % (best_T, best_val))

    fly(tb, best_a, best_T, conn, vessel, target_frame, ut, thrust_max, t_start)

if __name__ == '__main__':
    if '--demo' in sys.argv:
        run_demo()
    else:
        run_ksp()