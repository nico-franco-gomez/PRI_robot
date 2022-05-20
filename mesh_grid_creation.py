import numpy as np
from parcours_create import Parcours

'''
This script creates a frontal wall of measuring points for an exemplary
loudspeaker, see dimensions in par.set_object_lims
'''

# Create object and set limits
par = Parcours(name='frontal_mesh',base='[2]:straight_sol')
par.set_object_lims([(-183.5,183.5),(-80,80),(0,401)]) # Box

# Set first points to move robot safely without touching object
ro = [85,-84,90]
par.add_point_SPTP([0,0,1000],ro,75)
par.add_point_SPTP([-250,0,1000],ro,75)

# Create coordinates
x = -570 # Place for the mesh
yy = np.linspace(-100,100,10)
zz = np.linspace(0,800,10)
points = []
y_pos = True
for z in zz:
    if y_pos:
        for y in yy:
            points.append( ( (x,y,z), ro ,'SLIN') )
    else:
        for y in np.flip(yy):
            points.append( ( (x,y,z), ro ,'SLIN') )
    y_pos = not y_pos

# Create points in parcours
for n in points:
    eval(f'''par.add_point_{n[2]}(n[0],n[1])''')

# Export
par.export(path='real_parcours/')