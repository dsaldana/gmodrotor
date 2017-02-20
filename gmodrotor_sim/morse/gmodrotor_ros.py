from morse.builder import *
import sys
sys.path.append('.' )
from params import N, width, init_sepraration

for i in range(1, N+1):
    modrotor = Robot('brick.blend')
    modrotor.name = "gmodrotor_" + str(i)
    modrotor.translate(y=init_sepraration * i, z=.10)

    subgroup = '/crazy' + str(i).zfill(2)

    # Attitude controller
    force = ForceTorque()
    force.frequency(100.)
    modrotor.append(force)
    force.add_interface('ros', topic=subgroup+'/cmd_vel')
    #force.add_interface('ros')

    # Pose
    pose = Pose()
    pose.frequency(200.)
    modrotor.append(pose)
    #pose.add_interface('ros', topic=subgroup+'/pose')
    pose.add_interface('ros')




#env = Environment('indoors-1/boxes', fastmode=False)
env = Environment('indoors-1/boxes', fastmode=False)
#env = Environment('indoors-1/cage', fastmode=False)
env.simulator_frequency(1000)

env.set_camera_location([2, init_sepraration*(N/2.), 1])
env.set_camera_rotation([math.pi/3, 0, math.pi/2])
