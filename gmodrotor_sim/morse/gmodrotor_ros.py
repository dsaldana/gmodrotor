from morse.builder import *
import sys
sys.path.append('.' )
from params import N, width, init_sepraration

for i in range(1, N+1):
    modrotor = Robot('brick.blend')    
    robotid =  '/crazyflie' + str(i).zfill(2)
    modrotor.name = robotid
    modrotor.translate(y=init_sepraration * i, z=.10)

    subgroup = robotid

    # Attitude controller
    force = ForceTorque()
    force.frequency(60.)
    modrotor.append(force)
    force.add_interface('ros', topic=subgroup + '/morseforce')
    # force.add_interface('ros',  topic='morseforce')

    # Pose sensor
    pose = Pose()
    pose.frequency(60.)
    modrotor.append(pose)
    pose.add_interface('ros', topic=subgroup+'/pose')

    # Velocity sensor
    velocity = Velocity()
    velocity.frequency(60.)
    modrotor.append(velocity)
    velocity.add_interface('ros', topic=subgroup+'/vel')




env = Environment('indoors-1/boxes', fastmode=False)

#env.simulator_frequency(1000)

env.set_camera_location([2, init_sepraration*(N/2.), 1])
env.set_camera_rotation([math.pi/3, 0, math.pi/2])
