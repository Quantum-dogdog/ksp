import krpc
import numpy as np
import time
import math
from math import sin, cos, pi

conn = krpc.connect(name = 'pid')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
vessel_frame = vessel.reference_frame # 机体系
print('conn succesed!')
print('Project name:', vessel.name)

time.sleep(2)



# 设置新的目标飞行器（例如某个特定的飞行器）
vessels = space_center.vessels
for vessel in vessels:
    if vessel.name == 'mini':
        space_center.target_vessel = vessel
        print(f"New target vessel set to: {vessel.name}")    
        break               
##################################################
#
#不分段写就报错，不是很懂
##################################################    
conn = krpc.connect(name = 'pid2')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
vessel_frame = vessel.reference_frame # 机体系
print('conn succesed!')
print('Project name:', vessel.name)

time.sleep(2)

lat = -0.185355657540052
lon = -74.4730633292066
body_frame = body.reference_frame # 地固系
# 绕y轴旋转-lon度
temp1 = space_center.ReferenceFrame.create_relative(body_frame,
    rotation=(0., sin(-lon / 2. * pi / 180), 0., cos(-lon / 2. * pi / 180)))
# 绕z轴旋转lat度
temp2 = space_center.ReferenceFrame.create_relative(temp1,
    rotation=(0., 0., sin(lat / 2. * pi / 180), cos(lat / 2. * pi / 180)))
# 沿x轴平移
height = body.surface_height(lat, lon) + body.equatorial_radius
xyz = space_center.ReferenceFrame.create_relative(temp2,
    position=(height, 0., 0.))



def drawReferenceFrame(frame):
    x_axis = conn.drawing.add_line((90,0,0),(0,0,0), frame)
    x_axis.color = (1, 0, 0)#红
    x_axis.thickness = 2
    y_axis = conn.drawing.add_line((0,90,0),(0,0,0), frame)
    y_axis.color = (0, 1, 0)#绿
    y_axis.thickness = 2
    z_axis = conn.drawing.add_line((0,0,90),(0,0,0), frame)
    z_axis.color = (0, 0, 1)#蓝
    z_axis.thickness = 2

drawReferenceFrame(xyz) # 调用上述函数，画出之前建立的着陆点坐标系



# 初始化参数
g0 = 9.80665  # 标准重力加速度 (m/s^2)

r1 = np.array([0, 700000, 0])  # 目标位置
v1 = np.array([0, 0, 0])  # 目标速度
T = vessel.available_thrust  # 引擎总推力
Isp = vessel.specific_impulse  # 引擎有效排气速度
ve = Isp * g0
GM = 3.5316 * 10e12

m0 = vessel.mass  # 航天器质量
m1 = vessel.dry_mass
tao = (m0-m1) * ve / T     #能够燃烧时间
game_prev_time = space_center.ut # 记录上一帧时间
kaishi = space_center.ut
flag = True

vessel.control.throttle = 1

while (True):
    time.sleep(0.001)
    ut = space_center.ut # 获取游戏内时间
    game_delta_time = ut - game_prev_time # 计算上一帧到这一帧消耗的时间
    if game_delta_time < 0.019: # 如果游戏中还没有经过一个物理帧，不进行计算
        continue
   
    gao =vessel.position(xyz)[0]

    print(gao)
    #vessel.control.throttle = 1
    #vessel.control.pitch = phi
    #vessel.control.yaw = psi
    
    #if ut < kaishi + tao:
        
    if gao > 200 and flag==True:
        
           vessel.control.throttle = 0.3
           time.sleep(0.1)
           vessel.control.sas = True
           time.sleep(0.1)
           vessel.control.sas_mode = conn.space_center.SASMode.target
           flag = False
    

    
    time.sleep(1)
    game_prev_time = ut # 更新上一帧时间记录  
















        
     
