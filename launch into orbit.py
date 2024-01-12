import krpc
import math
import threading

from os import system
from time import sleep

conn = krpc.connect(name = 'launch into orbit')
vessel = conn.space_center.active_vessel
print('conn succesed!')
print('Project name:', vessel.name)


orbit = 'orbit'
surface = 'surface'

g = vessel.orbit.body.surface_gravity  # 海平面重力加速度
para = vessel.orbit.body.gravitational_parameter  # 天体引力常量
local_xyz = vessel.orbit.body.non_rotating_reference_frame  # 建立天体质心参考系
earth_xyz = vessel.orbit.body.reference_frame  # 建立地表参考系
gameTime = conn.add_stream(getattr, conn.space_center, 'ut')  # 获取全局时间


atmosphere_altitude = 70000  # 大气高度
turn_start_altitude = 500  # 程序转弯起始高度
turn_end_altitude = 25000  # 程序转弯结束高度
end_angle = 75  # 转向结束角度
target_altitude = 100000  # 目标轨道高度
turn_angle = 0  # 偏转角度初始化
flag1 = 0
flag2 = 0
flag3 = 0
flag4 = 0
flag5 = 0


sleep(2)
print('Launch Start！')

for i in range(5):  # 点火倒计时
    system('cls')
    print('Countdown：')
    print(5 - i)
    sleep(1)
system('cls')
print('Engine start！')

vessel.auto_pilot.target_pitch_and_heading(90, 90) # 设置自动驾驶目标姿态为竖直向上
vessel.auto_pilot.engage()
vessel.control.throttle = 0.3
vessel.control.activate_next_stage()
sleep(0.3)
vessel.control.throttle = 1
vessel.control.activate_next_stage()


def fenli():

   fuel_amount = conn.get_call(vessel.resources.amount, 'SolidFuel')
   
   expr = conn.krpc.Expression.less_than(
       conn.krpc.Expression.call(fuel_amount),
       conn.krpc.Expression.constant_float(0.1))
   event = conn.krpc.add_event(expr)
   with event.condition:
       event.wait()
   print('Booster separation')
   vessel.control.activate_next_stage() 
def fangxiang():
   if turn_start_altitude < vessel.flight().mean_altitude < turn_end_altitude: 
      
      xielv = ((vessel.flight().mean_altitude - turn_start_altitude) / (turn_end_altitude - turn_start_altitude))
      turn_angle = xielv * end_angle
      vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)         
      sleep(0.2)
       
       
t2 = threading.Thread(target=fenli)
t2.start()

    
while True:      
    if vessel.flight().mean_altitude > 500:
       fangxiang()
    if flag1 == 0 and vessel.orbit.apoapsis_altitude > target_altitude - 200:
       flag1 = 1
       vessel.control.throttle = 0 
    if flag1 == 1 and flag2 == 0 and vessel.flight().mean_altitude > atmosphere_altitude:
       flag2 = 1
       v0 = math.sqrt(para * (2 / vessel.orbit.apoapsis - 1 / vessel.orbit.semi_major_axis))  # 当前轨道远点速度
       v1 = math.sqrt(para * 1 / vessel.orbit.apoapsis)  # 目标轨道速度
       delta_v = v1 - v0  # 所需△v
       
       F = vessel.available_thrust  # 引擎总推力
       C = vessel.specific_impulse * 9.82  # 引擎有效排气速度
       flow_rate = F / C  # 燃料流速
       m0 = vessel.mass  # 点火前航天器质量
       m1 = m0 / math.exp(delta_v / C)  # 点火后航天器质量
       burn_time = (m0 - m1) / flow_rate  # 点火时长
       
       burn_ut = gameTime() + vessel.orbit.time_to_apoapsis - (burn_time / 2)  # 确定点火时间
       node = vessel.control.add_node(gameTime() + vessel.orbit.time_to_apoapsis, prograde=delta_v)  # 建立机动节点
       remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)  # 剩余所需△v
       burntime = node.remaining_delta_v / 33                                             #GDLV3的液体引擎的燃料流速大概是33.8
       sleep(1)
      
       break

vessel.auto_pilot.disengage()
vessel.control.sas = True
sleep(0.1)
vessel.control.sas_mode = conn.space_center.SASMode.maneuver
print('burntime', burntime)    
conn.space_center.warp_to(burn_ut - 20)

while node.time_to > 0:
    pass
print('Executing burn')

vessel.control.throttle = 1.0


sleep(burntime * 0.8)    
print('Fine tuning')
vessel.control.throttle = 0.3

while remaining_burn()[1] > 15:
    pass
sleep(2)
vessel.control.throttle = 0.0

sleep(3)
node.remove()

print('Launch complete') 
       


