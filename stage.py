import krpc
import math


from os import system
from time import sleep
import numpy as np

conn = krpc.connect(name = 'admin')
vessel = conn.space_center.active_vessel
parts = vessel.parts

print('连接成功!')
print()
print('Project name:', vessel.name)
print()

para = vessel.orbit.body.gravitational_parameter  # 天体引力常量
ut = conn.add_stream(getattr, conn.space_center, 'ut')  # 获取全局时间
orbit = 'orbit'
surface = 'surface'

#stage_0_resources = vessel.resources_in_decouple_stage(stage=9, cumulative=False)  # 获取助推器燃料信息
#stage_1_resources = vessel.resources_in_decouple_stage(stage=8, cumulative=False)  # 获取芯一级燃料信息

#fuel_amount = conn.get_call(vessel.resources.amount, 'SolidFuel')

print()

#name = vessel.parts.all

#print(name)


#name = vessel.parts.controlling

#print(name)


'''

stage0 = parts.in_stage(0)

print('stage0:', stage0)
print()
stage1 = parts.in_stage(1)

print('stage1:', stage1)
print()
stage2 = parts.in_stage(2)

print('stage2:', stage2)
print()
stage3 = parts.in_stage(3)

print('stage3:', stage3)
print()



for part in stage0:
    print('part_title:', part.title)
print()
for part in stage1:
    print('part_title:', part.title)
print()
for part in stage2:
    print('part_title:', part.title)
    
    
print()
for part in stage3:
    print('part_name:', part.name)
print()
for part in stage3:
    print('part_title:', part.title)
    if part.title == 'RE-I5 "Skipper" Liquid Fuel Engine':
        print('部件存在')
print()

stage_0_resources = vessel.resources_in_decouple_stage(stage=-1, cumulative=False)
print('stage_-1_resources:',stage_0_resources.names)


stage_0_resources = vessel.resources_in_decouple_stage(stage=0, cumulative=False)
print('stage_0_resources:',stage_0_resources.names)

stage_0_resources = vessel.resources_in_decouple_stage(stage=1, cumulative=False)     #solidfuel
print('stage_1_resources:',stage_0_resources.names)

stage_0_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
print('stage_2_resources:',stage_0_resources.names)

stage_0_resources = vessel.resources_in_decouple_stage(stage=3, cumulative=False)
print('stage_3_resources:',stage_0_resources.names)

stage_0_resources = vessel.resources_in_decouple_stage(stage=4, cumulative=False)
print('stage_4_resources:',stage_0_resources.names)

stage_0_resources = vessel.resources_in_decouple_stage(stage=5, cumulative=False)
print('stage_5_resources:',stage_0_resources.names)

stage_0_resources = vessel.resources_in_decouple_stage(stage=6, cumulative=False)
print('stage_6_resources:',stage_0_resources.names)

stage_0_resources = vessel.resources_in_decouple_stage(stage=7, cumulative=False)
print('stage_7_resources:',stage_0_resources.names)

stage_0_resources = vessel.resources_in_decouple_stage(stage=8, cumulative=False)
print('stage_8_resources:',stage_0_resources.names)

stage_0_resources = vessel.resources_in_decouple_stage(stage=9, cumulative=False)
print('stage_9_resources:',stage_0_resources.names)

print()



stage0 = parts.in_decouple_stage(1)

for part in stage0:
    print('part_title:', part.title)
print()
'''

stage0 = parts.in_decouple_stage(-1)
'''
while vessel.flight().mean_altitude > 3000:

  for part in stage0:
      #print('part_title:', part.title)
      if part.title == 'Rockomax Jumbo-64 Fuel Tank':
          #print('部件存在')
          liquidfuel_amount = part.resources.amount('LiquidFuel')
          oxidizer_amount = part.resources.amount('Oxidizer')
        
          print('liquidfuel_amount:', liquidfuel_amount)
          print('oxidizer_amount:', oxidizer_amount)
          print()
          sleep(2)
'''
while vessel.flight().mean_altitude > 100000:
  for part in stage0:
      #print('part_title:', part.title)
      if part.title == 'Rockomax Jumbo-64 Fuel Tank':
          #print('部件存在')
          liquidfuel_amount = part.resources.amount('LiquidFuel')
          oxidizer_amount = part.resources.amount('Oxidizer')
          print('liquidfuel_amount:', liquidfuel_amount)
          print('oxidizer_amount:', oxidizer_amount)
          if liquidfuel_amount < 300:
              vessel.control.activate_next_stage()
          sleep(2)     






























