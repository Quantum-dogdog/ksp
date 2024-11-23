import krpc
import numpy as np
import time
import math
from math import sin, cos, pi

conn = krpc.connect(name = 'diedai')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
vessel_frame = vessel.reference_frame # 机体系
print('conn succesed!')
print('Project name:', vessel.name)

time.sleep(2)

lat = -1.58
lon = -71.97
#lat = -0.185355657540052
#lon = -74.4730633292066
body_frame = body.reference_frame # 地固系
# 绕y轴旋转-lon度
temp1 = space_center.ReferenceFrame.create_relative(body_frame,
    rotation=(0., sin(-lon / 2. * pi / 180), 0., cos(-lon / 2. * pi / 180)))
# 绕z轴旋转lat度
temp2 = space_center.ReferenceFrame.create_relative(temp1,
    rotation=(0., 0., sin(lat / 2. * pi / 180), cos(lat / 2. * pi / 180)))

target_frame = space_center.ReferenceFrame.create_relative(temp2,
    position=(0., 0., 0.))


#绕x轴旋转90度
zhuan1 = space_center.ReferenceFrame.create_relative(target_frame,
    rotation=(sin(90 / 2. * pi / 180), 0., 0.,  cos(90 / 2. * pi / 180)))
#绕z轴旋转90度
zhuan2 = space_center.ReferenceFrame.create_relative(zhuan1,
    rotation=(0., 0., sin(-90 / 2. * pi / 180), cos(-90 / 2. * pi / 180)))

xyz = space_center.ReferenceFrame.create_relative(zhuan2,
    position=(0., 0., 0.))








def drawReferenceFrame(frame):
    x_axis = conn.drawing.add_line((900000,0,0),(0,0,0), frame)
    x_axis.color = (1, 0, 0)#红
    x_axis.thickness = 200
    y_axis = conn.drawing.add_line((0,900000,0),(0,0,0), frame)
    y_axis.color = (0, 1, 0)#绿
    y_axis.thickness = 200
    z_axis = conn.drawing.add_line((0,0,900000),(0,0,0), frame)
    z_axis.color = (0, 0, 1)#蓝
    z_axis.thickness = 200

drawReferenceFrame(xyz) # 调用上述函数，画出之前建立的着陆点坐标系





    

    

def qiu_tg(v1, v0, gm, tao, ve, tolerance=1e-6):
    # 初始化 tg
    tg = 20
    
    # 循环迭代，直到满足容忍度
    while True:
        # 计算速度差
        dv = v1 - v0 - gm * tg
        # 更新 tg
        tg1 = tao * (1 - math.exp(-np.linalg.norm(dv) / ve))
        # 检查是否满足容忍度
        if abs(tg - tg1) < tolerance:
            break
        # 更新 tg 为新的 tg1，为下一次迭代做准备
        tg = tg1
    
    return tg

def diedai_guid(r0, r1, v0, v1, m0, tao, ve, dt, tolerance=1e-6):
    xl = np.array(r0)
    dwxl = -xl / np.linalg.norm(r0)  # 单位向量
    gm = g0 * dwxl
    tg = qiu_tg(v1, v0, gm, tao, ve, tolerance)
    dv = v1 - v0 - gm * tg

    # 计算辅助变量
    f0 = ve * np.log(tao / (tao - tg))
    f1 = tao * f0 - ve * tg
    f2 = f0 * tg - f1
    f3 = f2 * tao - 0.5 * ve * tg**2

    # 计算方向角
    phiv = math.atan2((v1[1] - v0[1] - gm[1] * tg), (v1[0] - v0[0] - gm[0] * tg))
    psiv = -math.asin((v1[2] - v0[2] - gm[2] * tg) / np.linalg.norm(dv))

    # 计算方向系数
    kphi1 = (r1[1] - f2 * math.sin(phiv) * math.cos(psiv) - 0.5 * gm[1] * tg**2 - v0[1] * tg - r0[1]) / (math.cos(phiv) * (-f2 + f3 * f0 / f1))
    kpsi1 = (r1[2] + f2 * math.sin(psiv) - 0.5*gm[2]*tg**2 - v0[2] * tg - r0[2]) / (math.cos(psiv) * (f2 - f3 * f0 / f1))
    kphi2 = kphi1 * f0 / f1
    kpsi2 = kpsi1 * f0 / f1

    phic = phiv - kphi1+ kphi2*dt
    psic = psiv - kpsi1 + kpsi2*dt

    ux = 2*(r1[0]-r0[0]-v0[0]*tg-0.5*gm[0]*tg**2)/tg**2
    uy = T*math.sin(phic)*math.cos(psic)/m0
    uz = -T*math.sin(psic)/m0

    u_norm = math.sqrt(ux**2+uy**2+uz**2)

    
    phi = math.atan(uy/ux)
    psi = -math.asin(uz/u_norm)

    return phi,psi


r0 = vessel.position(xyz)  # 初始位置    
v0 = vessel.velocity(xyz)  # 初始速度
r1 = np.array([0,  706000, 0])  # 目标位置
v1 = np.array([0, 0, 0])  # 目标速度



g0 = 9.80665  # 标准重力加速度 (m/s^2)
T = vessel.available_thrust  # 引擎总推力
Isp = vessel.specific_impulse  # 引擎有效排气速度
ve = Isp * g0
GM = 3.5316 * 10e12

tg = 0
tg1 = 0


m0 = vessel.mass  # 航天器质量
m1 = vessel.dry_mass
tao = (m0-m1) * ve / T     #能够燃烧时间
dt = 0
game_prev_time = space_center.ut # 记录上一帧时间
while (True):
    time.sleep(0.001)
    ut = space_center.ut # 获取游戏内时间
    game_delta_time = ut - game_prev_time # 计算上一帧到这一帧消耗的时间
    if game_delta_time < 0.019: # 如果游戏中还没有经过一个物理帧，不进行计算
        continue
    r0 = vessel.position(xyz)  # 初始位置    
    v0 = vessel.velocity(xyz)  # 初始速度
    m0 = vessel.mass  # 航天器质量
    dt = dt + game_delta_time
    phi,psi = diedai_guid(r0,r1,v0,v1,m0,tao,ve,dt)     

    print(phi,psi)
    vessel.auto_pilot.engage()
    time.sleep(0.001)
    vessel.control.throttle = 1
    time.sleep(0.001)
    vessel.control.yaw =psi*57.3
    vessel.control.pitch =phi*57.3
    time.sleep(1)
    game_prev_time = ut # 更新上一帧时间记录
    
