import numpy as np
from parcours_create import Parcours

'''
This script creates a box-like grid of measuring points for an exemplary
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

nom_dist = -50-183.5 # nominal distance for plane location
## Create coordinates
# Frontal plane
rot_frontal = np.array([0,-45,180])
par.add_point_SLIN([nom_dist-30,0,800],rot_frontal,1)
x = nom_dist # Place for the mesh
yy = np.linspace(-80,80,10)
zz = np.linspace(0,405,10)
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

rot_side1 = np.array([45,-45,180])
par.add_point_SLIN([0,nom_dist,800],rot_side1,1)

# One side
xx = np.linspace(0,nom_dist,10) # Place for the mesh
y = nom_dist
zz = np.linspace(0,405,10)
points = []
x_pos = True
for z in zz:
    if x_pos:
        for x in xx:
            points.append( ( (x,y,z), rot_side1 ,'SLIN') )
    else:
        for x in np.flip(xx):
            points.append( ( (x,y,z), rot_side1 ,'SLIN') )
    x_pos = not x_pos
# Add points
for n in points:
    eval(f'''par.add_point_{n[2]}(n[0],n[1])''')

## Security points 3
par.add_point_SLIN([0,nom_dist,800],initial_rot,1)

rot_side2 = np.array([-45,-45,180])
par.add_point_SLIN([0,-nom_dist,800],rot_side2,1)

# Other side
xx = np.linspace(0,nom_dist,10) # Place for the mesh
y = -nom_dist
zz = np.linspace(0,405,10)
points = []
x_pos = True
for z in zz:
    if x_pos:
        for x in xx:
            points.append( ( (x,y,z), rot_side2 ,'SLIN') )
    else:
        for x in np.flip(xx):
            points.append( ( (x,y,z), rot_side2 ,'SLIN') )
    x_pos = not x_pos
# Add points
for n in points:
    eval(f'''par.add_point_{n[2]}(n[0],n[1])''')

## Security points 3
par.add_point_SLIN([0,-nom_dist,800],rot_side2,1)
rot_upper = np.array([0,0,180])
par.add_point_SLIN([0,0,800],rot_upper,1)

# Upper plane
xx = np.linspace(0,nom_dist,10) # Place for the mesh
yy = np.linspace(-80,80,10)
zz = 405
points = []
z_pos = True
for y in yy:
    if z_pos:
        for x in xx:
            points.append( ( (x,y,z), rot_upper ,'SLIN') )
    else:
        for x in np.flip(xx):
            points.append( ( (x,y,z), rot_upper ,'SLIN') )
    z_pos = not z_pos
# Add points
for n in points:
    eval(f'''par.add_point_{n[2]}(n[0],n[1])''')

par.name = f'near_field{-nom_dist:.0f}'

# Export
par.export(path='real_parcours/')