import numpy as np
import matplotlib.pyplot as plt
from parcours import Parcours

'''
This script creates a box-like grid of measuring points for an exemplary
loudspeaker, see dimensions in par.set_object_lims.

Attention: This parcours comes very near to the object, special attention
for defining the paths is required. Trust mode is on
'''

# Visualization
visualize = True
if visualize:
    fig,ax = plt.subplots(1,1,subplot_kw={'projection':'3d'})

# Create object and set limits
par = Parcours(base='[2]:straight_sol')
par.set_trust(True) ## ============= <
par.set_object_lims([(-183.5,183.5),(-140,140),(0,400)]) # Box
point_dist = 40

if visualize:
    # Upper
    xx,yy = np.meshgrid(np.arange(-18.3,18.3,1),np.arange(-14,14,1))
    ax.scatter(xx, yy, np.ones_like(xx)*36,color='xkcd:green',label='Haut-parleur')
    # Sides 1
    xx,zz = np.meshgrid(np.arange(-18.3,18.3,1),np.arange(0,40,1))
    ax.scatter(xx,np.ones_like(xx)*14,zz,color='xkcd:green')
    ax.scatter(xx,np.ones_like(xx)*-14,zz,color='xkcd:green')
    # Sides 2
    yy,zz = np.meshgrid(np.arange(-14,14,1),np.arange(0,40,1))
    ax.scatter(np.ones_like(yy)*18.3,yy,zz,color='xkcd:green')
    ax.scatter(np.ones_like(yy)*-18.3,yy,zz,color='xkcd:green')

    vec = np.arange(0,30,1)
    ax.scatter(vec,np.zeros_like(vec),np.zeros_like(vec),color='xkcd:purple',\
        label='Base')
    ax.scatter(np.zeros_like(vec),np.zeros_like(vec),vec,color='xkcd:purple')
    ax.scatter(np.zeros_like(vec),vec,np.zeros_like(vec),color='xkcd:purple')

## Security points 1
initial_rot = np.array([0,-90,180])
par.add_point_SPTP([0,0,800],initial_rot,75,marker=0)

nom_dist = -50-183.5 # nominal distance for plane location
## Create coordinates
# Frontal plane
rot_frontal = np.array([0,-45,180])
par.add_point_SLIN([nom_dist-30,0,800],rot_frontal,1,marker=0)
x = nom_dist # Place for the mesh
yy = np.arange(nom_dist,-nom_dist+1,point_dist)
zz = np.arange(0,405+1,point_dist)
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

if visualize:
    xx = []
    yy = []
    zz = []
    for n in points:
        xx.append(n[0][0]/10)
        yy.append(n[0][1]/10)
        zz.append(n[0][2]/10)
    ax.scatter(xx,yy,zz,color='xkcd:orange',label='Points \nde mesure')

## Security points 2
par.add_point_SPTP([nom_dist,0,800],rot_frontal,75,marker=0)
rot_side1 = np.array([45,-45,180])
par.add_point_SLIN([0,nom_dist,800],rot_side1,1,marker=0)

# One side
xx = np.arange(0,nom_dist-1,-point_dist) # Place for the mesh
y = nom_dist
zz = np.arange(0,405+1,point_dist)
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

if visualize:
    xx = []
    yy = []
    zz = []
    for n in points:
        xx.append(n[0][0]/10)
        yy.append(n[0][1]/10)
        zz.append(n[0][2]/10)
    ax.scatter(xx,yy,zz,color='xkcd:orange')

## Security points 3
par.add_point_SLIN([0,nom_dist,800],initial_rot,1,marker=0)
rot_side2 = np.array([-45,-45,180])
par.add_point_SLIN([0,-nom_dist,800],rot_side2,1,marker=0)

# Other side
xx = np.arange(0,nom_dist-1,-point_dist) # Place for the mesh
y = -nom_dist
zz = np.arange(0,405+1,point_dist)
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

if visualize:
    xx = []
    yy = []
    zz = []
    for n in points:
        xx.append(n[0][0]/10)
        yy.append(n[0][1]/10)
        zz.append(n[0][2]/10)
    ax.scatter(xx,yy,zz,color='xkcd:orange')

## Security points 3
par.add_point_SLIN([0,-nom_dist,800],rot_side2,1,marker=0)
rot_upper = np.array([0,0,180])
par.add_point_SLIN([0,0,800],rot_upper,1,marker=0)

# Upper plane
xx = np.arange(0,nom_dist+1,-point_dist) # Place for the mesh
yy = np.arange(nom_dist,-nom_dist+1,point_dist)
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

if visualize:
    xx = []
    yy = []
    zz = []
    for n in points:
        xx.append(n[0][0]/10)
        yy.append(n[0][1]/10)
        zz.append(n[0][2]/10)
    ax.scatter(xx,yy,zz,color='xkcd:orange')

par.name = f'near_field{-nom_dist:.0f}'

# Export
par.export(path='real_parcours/')

if visualize:
    ax.set_title(f'Planes – Distance {-nom_dist//10:.0f} cm')
    ax.set_xlabel('$x$ [cm]')
    ax.set_ylabel('$y$ [cm]')
    ax.set_zlabel('$z$ [cm]')
    fig.tight_layout()
    ax.legend(loc='center',bbox_to_anchor=(-0.09,0.5))
    # fig.savefig(f'plots/grid_dist{-nom_dist//10}.pdf')
    # fig.savefig(f'''/Users/nicolas/Documents/Uni/Master – 4. Semester 
    #  INSA/Projet de recherche/Latex/Bilder/parcours/
    # grid_dist{-nom_dist//10}.pdf'''.replace('\n',''))
    plt.show()