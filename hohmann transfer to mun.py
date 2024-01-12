import krpc
import math
import threading

import os
from numpy import pi, dot, cross, array, arccos, sign
from numpy.linalg import norm

from os import system
from time import sleep

conn = krpc.connect(name = 'hohmann transfer')
vessel = conn.space_center.active_vessel
print('conn succesed!')
print('Project name:', vessel.name)

mu = vessel.orbit.body.gravitational_parameter  # 天体引力常量
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
transfer_phase = pi * (1 - (((r_kerbin + r_mun) / (2 * r_mun)) ** 1.5)) #phase是一个角度
print("phase", transfer_phase)

def angle_between(obj1, obj2, conn):
    frame = obj1.orbit.body.non_rotating_reference_frame
    pos1 = array(obj1.position(frame))
    pos2 = array(obj2.position(frame))

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


def time_until_phase(obj1, obj2, phase, conn):
    ''' find time until angle between obj1 and obj2 is phase '''
    angle = angle_between(obj1, obj2, conn)
    print("angle between is", angle)

    # find orbital radial velocity
    radians_per_second_1 = 2 * pi / obj1.orbit.period
    radians_per_second_2 = 2 * pi / obj2.orbit.period

    # assume obj2 is running away, obj1 is chasing
    relative_angle_change_per_second = radians_per_second_2 - radians_per_second_1
    print("p, a, r", phase, angle, relative_angle_change_per_second)
    delta = phase - angle
    while sign(delta) != sign(relative_angle_change_per_second):
        delta += sign(relative_angle_change_per_second) * 2 * pi
    return delta / relative_angle_change_per_second

# compute node and delta-v for transfer
dt = time_until_phase(vessel, mun, transfer_phase, conn)
v1 = math.sqrt(mu / r_kerbin)
r_ellipse = (r_kerbin + r_mun) / 2
v2 = math.sqrt(mu * ((2/r_kerbin) - (1/r_ellipse)))
delta_v = v2 - v1 - 10        #调参
print("dV", delta_v)

node = vessel.control.add_node(ut() + dt, prograde=delta_v)
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)  # 剩余所需△v
conn.space_center.warp_to(ut() + dt - 20)

sleep(1)  # reorient
vessel.control.sas = True
sleep(0.1)
vessel.control.sas_mode = conn.space_center.SASMode.maneuver

burntime = node.remaining_delta_v / 21           #调参
print('burntime:', burntime)    

while node.time_to > 0:
    pass
print('Executing burn')
vessel.control.throttle = 1.0
sleep(burntime * 0.9)    
print('Fine tuning')
vessel.control.throttle = 0.2

while remaining_burn()[1] > 10:
    pass
sleep(2)
vessel.control.throttle = 0
sleep(3)
node.remove()


conn.space_center.warp_to(ut() + 60 * 60 * 6)
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
burn_time = node.remaining_delta_v / 24          
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
# Wait until burn
print('Waiting until second circularization burn')
time_to_periapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_periapsis')
burn_ut = ut() + time_to_periapsis() - (burn_time / 2)
lead_time = 20  # warp turns off SAS, give time to reorient
conn.space_center.warp_to(burn_ut - lead_time)

# use SAS for reotrgrade burn
vessel.control.sas = True
sleep(0.1)  # allow SAS to turn on
vessel.control.sas_mode = conn.space_center.SASMode.maneuver

# Orientate ship
print('Orientating ship for second circularization burn')
while node.time_to > 0:
    pass
print('Executing burn - {} seconds'.format(burn_time))
vessel.control.throttle = 1.0
sleep(burn_time * 0.8)    
print('Fine tuning')
vessel.control.throttle = 0.2

while remaining_burn()[1] > 15:         #调参
    pass 
sleep(2)
vessel.control.throttle = 0.0

sleep(3)
node.remove()

