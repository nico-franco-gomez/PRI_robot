import numpy as np
from parcours_create import Parcours

'''
This parcours is done to define the used base and mark it on the ground.
'''

# Create object and set limits
par = Parcours(base='[2]:straight_sol')

initial_rot = [0,-90,180]
other_rot = [0,0,180]

par.add_point_SPTP([0,0,800],initial_rot,75)
par.add_point_SLIN([0,0,0],initial_rot)
par.add_point_SLIN([0,-400,0],initial_rot)
par.add_point_SLIN([0,400,0],initial_rot)

par.add_point_SLIN([-200,0,0],other_rot)
par.add_point_SPTP([0,0,500],initial_rot,75)
par.add_point_SLIN([200,0,0],initial_rot)

par.name = f'alineacion'

# Export
par.export(path='real_parcours/')