import numpy as np
from parcours_create import Parcours

'''
This script creates a moving frontal wall of measuring points for an exemplary
loudspeaker, see dimensions in par.set_object_lims.

Attention: This parcours comes very near to the object, special attention
for defining the paths is required. Trust mode is on
'''

# Create object and set limits
par = Parcours(base='[2]:straight_sol')
par.set_trust(True) ## ============= <
par.set_object_lims([(-183.5,183.5),(-80,80),(0,400)]) # Box

## Security points 1
initial_rot = np.array([0,-90,180])
par.add_point_SPTP([0,0,800],initial_rot,75)

nom_dist = -40-183.5 # nominal distance for plane location
## Create coordinates
# Frontal plane
rot_frontal = np.array([0,-45,180])
par.add_point_SLIN([nom_dist-30,0,800],rot_frontal,1)

distances = np.arange(nom_dist,nom_dist-501,-100)

for x in distances:
    yy = np.linspace(-100,100,10)
    zz = np.linspace(0,405,10)
    points = []
    y_pos = True
    if x < -400:
        rot_frontal = [0,0,180]
    for z in zz:
        if y_pos:
            for y in yy:
                points.append( ( (x,y,z), rot_frontal ,'SLIN') )
        else:
            for y in np.flip(yy):
                points.append( ( (x,y,z), rot_frontal ,'SLIN') )
        y_pos = not y_pos
    # Add points
    for n in points:
        eval(f'''par.add_point_{n[2]}(n[0],n[1])''')

# Security point
par.add_point_SLIN([nom_dist-30,0,800],rot_frontal,1)

par.name = f'moving_front{-nom_dist:.0f}'

# Export
par.export(path='real_parcours/')