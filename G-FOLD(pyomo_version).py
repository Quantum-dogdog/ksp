from pyomo.environ import *
from pyomo.dae import *
from pyomo.opt import SolverFactory
import pyomo.dae as pyd
import numpy as np
import matplotlib.pyplot as plt
from tqdm import trange
from pyomo.dae import Integral
import krpc
from math import sin, cos, pi
import time
import math

conn = krpc.connect(name = 'pyomo')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
vessel_frame = vessel.reference_frame # 机体系
print('conn succesed!')
print('Project name:', vessel.name)

time.sleep(2)
#建立的是z轴向东，y轴向北，x轴向上的yzx坐标系
lat = -0.10
lon = -74.55
#lat = -0.185355657540052
#lon = -74.4730633292066
body_frame = body.reference_frame # 地固系
# 绕y轴旋转-lon度
temp1 = space_center.ReferenceFrame.create_relative(body_frame,
    rotation=(0., sin(-lon / 2. * pi / 180), 0., cos(-lon / 2. * pi / 180)))
# 绕z轴旋转lat度
temp2 = space_center.ReferenceFrame.create_relative(temp1,
    rotation=(0., 0., sin(lat / 2. * pi / 180), cos(lat / 2. * pi / 180)))
# 沿x轴平移
height = body.surface_height(lat, lon) + body.equatorial_radius
target_frame = space_center.ReferenceFrame.create_relative(temp2,
    position=(height, 0., 0.))

def drawReferenceFrame(frame):
    x_axis = conn.drawing.add_line((10,0,0),(0,0,0), frame)
    x_axis.color = (1, 0, 0)
    x_axis.thickness = 2
    y_axis = conn.drawing.add_line((0,10,0),(0,0,0), frame)
    y_axis.color = (0, 1, 0)
    y_axis.thickness = 2
    z_axis = conn.drawing.add_line((0,0,10),(0,0,0), frame)
    z_axis.color = (0, 0, 1)
    z_axis.thickness = 2

drawReferenceFrame(target_frame) # 调用上述函数，画出之前建立的着陆点坐标系


g = 9.81

weizhi = vessel.position(target_frame)  #矢量
sudu = vessel.velocity(target_frame)    #矢量
mass = vessel.mass
dry_mass = vessel.dry_mass
fuel_mass = mass -dry_mass
bichong = vessel.specific_impulse       #居然随着高度变化
tuili = vessel.available_thrust

ut = space_center.ut

v_exhaust = bichong * g
# 设置初始参数
target_position = [0, 0, 0]  # 初始位置
initial_position = weizhi  # 目标位置
initial_velocity = sudu  # 初始速度
max_thrust = tuili  # 最大推力
u_max = tuili / v_exhaust  # 最大质量流量kg/s
initial_mass = mass  # 初始质量
time_lower_bound = ut  # 时间下界
time_upper_bound = ut + 100  # 时间上界
drag_coefficient = 0.2  # 空气阻力系数



model = ConcreteModel()
model.name="G-FOLD"
model.t = ContinuousSet(bounds=(0, 1))  # 时间范围
model.T = Var(bounds=(time_lower_bound, time_upper_bound))  # 总时间

# 定义变量
model.z = Var(model.t)  # z位置
model.y = Var(model.t)  # y位置
model.x = Var(model.t)  # x位置

model.m = Var(model.t, bounds=(dry_mass, initial_mass))  # 质量

model.u = Var(model.t, bounds=(0, u_max))  # 推力

model.vz = DerivativeVar(model.z, wrt=model.t)
model.vy = DerivativeVar(model.y, wrt=model.t)
model.vx = DerivativeVar(model.x, wrt=model.t)

model.az = DerivativeVar(model.vz, wrt=model.t)
model.ay = DerivativeVar(model.vy, wrt=model.t)
model.ax = DerivativeVar(model.vx, wrt=model.t)

model.mdot = DerivativeVar(model.m, wrt=model.t)

# Variables must satisfy mechanical force equilibrium at each time step

model.force_eq_z = Constraint(model.t, rule=lambda m, t: model.m[t] * model.az[t] == model.u[t] * v_exhaust)
model.force_eq_y = Constraint(model.t, rule=lambda m, t: model.m[t] * model.ay[t] == model.u[t] * v_exhaust)
model.force_eq_x = Constraint(model.t, rule=lambda m, t: model.m[t] * model.ax[t] == -model.m[t] * g + model.u[t] * v_exhaust)

                       
model.v_x = Constraint(model.t, rule = lambda m, t: model.vx[t] <= 0)   #你最好在火箭已经向下掉的时候运行这个脚本

# 质量变化率约束
def mass_rate_constraint(m, t):
    return m.mdot[t] / m.T == -m.u[t]

model.mass_rate_con = Constraint(model.t, rule=mass_rate_constraint)

# 目标函数
model.fuel = Integral(model.t, wrt=model.t, rule = lambda m, t: model.u[t]*model.T)
model.obj = Objective(expr=Integral(model.t, wrt=model.t, rule=lambda m, t: model.u[t]))


# 边界条件
model.z[0].fix(initial_position[0])
model.y[0].fix(initial_position[1])
model.x[0].fix(initial_position[2])
model.vz[0].fix(initial_velocity[0])
model.vy[0].fix(initial_velocity[1])
model.vx[0].fix(initial_velocity[2])
model.m[0].fix(initial_mass)

model.z[1].fix(0)
model.y[1].fix(0)
model.x[1].fix(0)
model.vz[1].fix(0)
model.vy[1].fix(0)
model.vx[1].fix(0)

# 离散化时间集
discretizer = TransformationFactory('dae.collocation')
discretizer.apply_to(model, nfe=100, ncp=1)  # nfe 是离散点的数量，ncp 是每个区间内的插值点数量


# 求解器设置
solver = SolverFactory('ipopt')
results = solver.solve(model)

tsim = [t for t in model.t]
ysim = [model.y[t]() for t in model.t]
zsim = [model.z[t]() for t in model.t]
xsim = [model.x[t]() for t in model.t]





trajectory = [tsim, zsim, ysim, xsim]

#print(trajectory)

def draw_line(origin, terminal, colour, reference_frame):
    line = conn.drawing.add_line(reference_frame=reference_frame, start=origin, end=terminal)
    line.thickness = 1
    line.color = colour


def draw_trajectory(trajectory, reference_frame):
    tsim, zsim, ysim, xsim = trajectory
    for i in range(len(tsim) - 1):  # 循环到 len(tsim) - 1，以确保不会超出范围
        origin = [zsim[i], ysim[i], xsim[i]]
        terminal = [zsim[i+1], ysim[i+1], xsim[i+1]]  # 使用下一个点作为终点

        draw_line(origin=origin, terminal=terminal, colour=(255, 255, 255), reference_frame=reference_frame)


# 现在可以尝试绘制轨迹
draw_trajectory(trajectory, reference_frame=target_frame)




