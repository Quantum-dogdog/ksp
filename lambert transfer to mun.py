import krpc
import math
import threading
import os

from numpy import pi, dot, cross, array, arccos, sign
from numpy.linalg import norm
from os import system
from time import sleep

conn = krpc.connect(name = 'lambert transfer')
vessel = conn.space_center.active_vessel
print('conn succesed!')
print('Project name:', vessel.name)

mu = vessel.orbit.body.gravitational_parameter  # 天体引力常量  # 天体引力常量
ut = conn.add_stream(getattr, conn.space_center, 'ut')  # 获取全局时间

sleep(2)
for i in range(3):  # 转移倒计时
    system('cls')
    print('Countdown：')
    print(3 - i)
    sleep(1)
system('cls')
print('Start Transfer!')

r_kerbin = vessel.orbit.semi_major_axis
mun = conn.space_center.bodies['Mun']
r_mun = mun.orbit.semi_major_axis


print('r_k:', r_kerbin)
print('r_m:', r_mun)

def angle_between(obj1, obj2, conn):
    frame = obj1.orbit.body.non_rotating_reference_frame
    pos1 = array(obj1.orbit.position_at(ut() + 60, frame))                      #假设60秒后出发
    pos2 = array(obj2.orbit.position_at(ut() + 60 + 60 * 60 * 5, frame))        #假设出发后5小时到达
    # find the cross product from pos1 to pos2
    pos1 /= norm(pos1)
    pos2 /= norm(pos2)

    # angle from dot product
    angle = arccos(dot(pos1, pos2))

    # flip if cross product opposite orbital normal of first object
    cp = cross(pos1, pos2)
    cp_local = conn.space_center.transform_direction(cp, frame, obj1.orbital_reference_frame)
    if cp_local[2] > 0:  # left handed coordinate system
        angle = - angle

    return angle

angle = angle_between(vessel, mun, conn)

frame = vessel.orbit.body.non_rotating_reference_frame
pos1 = array(vessel.orbit.position_at(ut() + 60, frame))                      #假设60秒后出发
pos2 = array(mun.orbit.position_at(ut() + 60 + 60 * 60 * 5, frame)) 

print('angle:', angle)

c = math.sqrt(r_kerbin ** 2 + r_mun ** 2 - 2 * r_kerbin * r_mun * math.cos(angle))

print('c:', c)

s = (r_kerbin + r_mun + c) / 2

print('s:', s)

num_s = int(s / 2 + 500)
num_e = int(s)

for a in range(num_s, num_e, 500):

  alpha = math.acos(1 - s / a)
  baita = math.acos(1 - (s - c) / a)
  youbian = a ** 1.5 *(alpha - baita -(math.sin(alpha) - math.sin(baita)))
  zuobian = math.sqrt(mu) * 60 * 60 * 5
  print('youbian:', youbian)  
  print('zuobian:', zuobian)
  if youbian < zuobian:
  
    print('a:', a)
    break

alpha = math.acos(1 - s / a)
baita = math.acos(1 - (s - c) / a)

e = math.sqrt(1- 4 * (s - r_kerbin) * (s - r_mun) * (math.sin((alpha + baita) / 2) ** 2) / (c ** 2))

print('e:' ,e)

p = a * (1 - e ** 2)

print('p:' ,p)


v1 = math.sqrt(mu * p) * ((pos2 - pos1) + r_mun * (1 - math.cos(angle)) * pos1 / p) / (r_kerbin * r_mun * math.sin(angle))

v2 = math.sqrt(mu * p) * ((pos2 - pos1) - r_kerbin * (1 - math.cos(angle)) * pos2 / p) / (r_kerbin * r_mun * math.sin(angle))


print('v1:' ,v1)
print('v2:' ,v2)

v1_norm = norm(v1)

print('v1_norm:' ,v1_norm)

v1_local = conn.space_center.transform_direction(v1, frame, vessel.orbital_reference_frame)

print('v1_local:' ,v1_local)

speed = math.sqrt(mu / r_kerbin)
print('speed:' ,speed)

dv_prograde = (v1_local[1] - speed) * 1.008
dv_radial = (- v1_local[0]) * 1.008

node = vessel.control.add_node(ut() + 50, prograde=dv_prograde,radial=dv_radial)
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
vessel.control.sas = True
sleep(0.1)  # allow SAS to turn on
vessel.control.sas_mode = conn.space_center.SASMode.maneuver

burntime = node.remaining_delta_v / 21           #调参
print('burntime:', burntime)    

while node.time_to > 0:
    pass
print('Executing burn')
vessel.control.throttle = 1.0
sleep(burntime + 3.2)                             #调参

vessel.control.throttle = 0
sleep(3)
node.remove()


sleep(3)




conn.space_center.warp_to(ut() + 60 * 60 * 4.5)

time_to_change_of_soi = vessel.orbit.time_to_soi_change   #进入mun的引力影响范围
assert time_to_change_of_soi > 0
conn.space_center.warp_to(ut() + time_to_change_of_soi + 20)

# get new orbit parameters and prepare to stabilize orbit
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.periapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu*((2./r)-(1./a1)))
v2 = math.sqrt(mu*((2./r)-(1./a2)))
delta_v = v2 - v1
assert delta_v < 0
node = vessel.control.add_node(
    ut() + vessel.orbit.time_to_periapsis, prograde=delta_v)  # negative delta v

# Calculate burn time (using rocket equation)
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(- delta_v/Isp)
flow_rate = F / Isp
burn_time = node.remaining_delta_v / 26             #调参
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
# Wait until burn
print('Waiting until second circularization burn')
time_to_periapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_periapsis')
burn_ut = ut() + time_to_periapsis() - (burn_time / 2)
lead_time = 20  # warp turns off SAS, give time to reorient
conn.space_center.warp_to(burn_ut - lead_time)


vessel.control.sas_mode = conn.space_center.SASMode.maneuver

# Orientate ship
print('Orientating ship for second circularization burn')
while node.time_to > 0:
    pass
print('Executing burn - {} seconds'.format(burn_time))
vessel.control.throttle = 1.0
while remaining_burn()[1] > 60:
    pass
vessel.control.throttle = 0.2
while remaining_burn()[1] > 15:
    pass
sleep(2)
vessel.control.throttle = 0.0

sleep(3)
node.remove()



