import numpy as np
from parcours import Parcours

'''
This script creates a frontal wall of measuring points for an exemplary
loudspeaker, see dimensions in par.set_object_lims
'''

# Create object and set limits
par = Parcours(base='[2]:straight_sol')
par.set_object_lims([(-183.5,183.5),(-80,80),(0,401)]) # Box

## Security points 1
initial_rot = np.array([0,-90,180])
par.add_point_SPTP([0,0,800],initial_rot,75)
par.add_point_SLIN([-250,0,800],initial_rot,1)

nom_dist = -570 # nominal distance for plane location
## Create coordinates
# Frontal plane
rot_frontal = np.array([0,0,180])
par.add_point_SLIN([-250,0,800],rot_frontal,1)
x = nom_dist # Place for the mesh
yy = np.linspace(-400,400,10)
zz = np.linspace(0,800,10)
points = []
y_pos = True
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

## Security points 2
par.add_point_SLIN([nom_dist,0,800],rot_frontal,1)
par.add_point_SLIN([0,nom_dist,800],initial_rot,1)

# One side
xx = np.linspace(0,nom_dist,10) # Place for the mesh
y = nom_dist
zz = np.linspace(0,800,10)
points = []
x_pos = True
for z in zz:
    if x_pos:
        for x in xx:
            points.append( ( (x,y,z), initial_rot ,'SLIN') )
    else:
        for x in np.flip(xx):
            points.append( ( (x,y,z), initial_rot ,'SLIN') )
    x_pos = not x_pos
# Add points
for n in points:
    eval(f'''par.add_point_{n[2]}(n[0],n[1])''')

## Security points 3
par.add_point_SLIN([0,nom_dist,800],initial_rot,1)
par.add_point_SLIN([0,-nom_dist,800],initial_rot,1)

# Other side
xx = np.linspace(0,nom_dist,10) # Place for the mesh
y = -nom_dist
zz = np.linspace(0,800,10)
points = []
x_pos = True
for z in zz:
    if x_pos:
        for x in xx:
            points.append( ( (x,y,z), initial_rot ,'SLIN') )
    else:
        for x in np.flip(xx):
            points.append( ( (x,y,z), initial_rot ,'SLIN') )
    x_pos = not x_pos
# Add points
for n in points:
    eval(f'''par.add_point_{n[2]}(n[0],n[1])''')

par.name = f'grid_dist{-nom_dist}'

# Export
par.export(path='real_parcours/')