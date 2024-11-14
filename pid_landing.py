import krpc
from math import sin, cos, pi
import numpy as np
import time
import math

conn = krpc.connect(name = 'pid')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
vessel_frame = vessel.reference_frame # 机体系
print('conn succesed!')
print('Project name:', vessel.name)

time.sleep(2)

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




# 用于限制一个数的范围
def clamp(num, limit1, limit2):
    lower_limit = min(limit1, limit2)
    upper_limit = max(limit1, limit2)
    return max(min(num, upper_limit), lower_limit)

# PID控制器
class PID:
    def __init__(self, kp, ki, kd, integral_output_limit = 1):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral_output_limit = integral_output_limit
        self.integral = 0
        self.error_prev = 0

    def update(self, error, dt):
        # 计算 P 分量
        p = error * self.kp
        # 计算 I 分量
        self.integral += error * dt * self.ki
        self.integral = clamp(self.integral, self.integral_output_limit, -self.integral_output_limit)
        i = self.integral
        # 计算 D 分量
        d = (error - self.error_prev) / dt * self.kd
        self.error_prev = error
        # 加起来得到结果
        return p + i + d





vessel.control.sas = True

# 初始化一个PID控制器
height_pid = PID(kp=0.1, ki=0.05, kd=0.2)
yaw_pid = PID(kp=-2.0, ki = 0, kd=-2.2)
pitch_pid = PID(kp=2.0, ki = 0, kd=2.2)
position_y_pid = PID(kp=-0.02, ki = 0, kd=-0.1)
position_p_pid = PID(kp=-0.02, ki = 0, kd=-0.1)
landing_cnt = 0
landing_time = 5
h_landing = 5
flag = False
flag2 = False
height_ref = 150
game_prev_time = space_center.ut # 记录上一帧时间
print('当前时间',game_prev_time)
while (True):
    time.sleep(0.001)
    ut = space_center.ut # 获取游戏内时间
    game_delta_time = ut - game_prev_time # 计算上一帧到这一帧消耗的时间
    if game_delta_time < 0.019: # 如果游戏中还没有经过一个物理帧，不进行计算
        continue

    # 在这里写控制代码
    height = vessel.position(target_frame)[0]
    #balanced_throttle = vessel.mass * 9.81 / vessel.available_thrust
    balanced_throttle = 0.2
    
    error = height_ref - height
    vessel.control.throttle = balanced_throttle + height_pid.update(error, game_delta_time)

    
    
    if space_center.ut > 50 and flag == False :
        print('自动着陆开始')
        flag = True

    if flag == True:
        height_ref = height - landing_cnt/(landing_time*10)*(height - h_landing)
        landing_cnt += 1
        flag2 = True
    if height < 6 and flag2 == True:
        vessel.control.throttle = 0
        print('着陆完毕')
        break
    game_prev_time = ut # 更新上一帧时间记录
    
    
