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

# Create object and set limits
base_parcours = [1272, -51, -355]  # [2]:straight_sol
par = Parcours(base_coord = base_parcours)
par.set_trust(True) ## ============= <
object_lim_base = [[-183.5,183.5],[-140,140],[0,400]]
par.set_object_lims([[-183.5,183.5],[-140,140],[0,400]]) # Box
point_dist = 60


if visualize:
    fig,ax = plt.subplots(1,1,subplot_kw={'projection':'3d'})
    # Upper
    xx,yy = np.meshgrid(np.arange(par.object_lims[0][0]/10,par.object_lims[0][1]/10,1), # x
                        np.arange(par.object_lims[1][0]/10,par.object_lims[1][1]/10,1)) # y
    ax.scatter(xx, yy, np.ones_like(xx)*par.object_lims[2][1]/10,
                color='xkcd:green',label='Haut-parleur')
    # Sides 1
    xx,zz = np.meshgrid(np.arange(par.object_lims[0][0]/10,par.object_lims[0][1]/10,1),
                        np.arange(par.object_lims[2][0]/10,par.object_lims[2][1]/10,1))
    ax.scatter(xx,np.ones_like(xx)*par.object_lims[1][0]/10,
                zz,color='xkcd:green')
    ax.scatter(xx,np.ones_like(xx)*par.object_lims[1][1]/10,
                zz,color='xkcd:green')
    # Sides 2
    yy,zz = np.meshgrid(np.arange(par.object_lims[1][0]/10,par.object_lims[1][1]/10,1),
                        np.arange(par.object_lims[2][0]/10,par.object_lims[2][1]/10,1))
    ax.scatter(np.ones_like(yy)*par.object_lims[0][0]/10,
                yy,zz,color='xkcd:green')
    ax.scatter(np.ones_like(yy)*par.object_lims[0][1]/10,
                yy,zz,color='xkcd:green')

    vec = np.arange(0,30,1)
    ax.scatter(vec,np.zeros_like(vec),np.zeros_like(vec),color='xkcd:purple',\
        label='Base')
    ax.scatter(np.zeros_like(vec),np.zeros_like(vec),vec,color='xkcd:purple')
    ax.scatter(np.zeros_like(vec),vec,np.zeros_like(vec),color='xkcd:purple')

## Security points 1
initial_rot = np.array([0, -90, 180])
par.add_point_SPTP([0, 0, 800], initial_rot, 20, marker=0)

nom_dist = 50 # nominal distance for plane location (from object boundaries)
## Create coordinates
# Frontal plane
rot_frontal = np.array([0, -45, 180])
par.add_point_SLIN([object_lim_base[0][0] - nom_dist*2, 0, 800],
                   rot_frontal, 0.25, marker=0)
x = object_lim_base[0][0] - nom_dist # Place for the mesh
yy = np.arange(object_lim_base[1][0]-nom_dist,
                object_lim_base[1][1]+nom_dist+1,point_dist)
zz = np.arange(object_lim_base[2][0],
                object_lim_base[2][1]+nom_dist+1,point_dist)
points = []
y_pos = True
for z in zz:
    if y_pos:
        for y in yy:
            points.append( ( [x,y,z], rot_frontal ,'SLIN') )
    else:
        for y in np.flip(yy):
            points.append( ( [x,y,z], rot_frontal ,'SLIN') )
    y_pos = not y_pos
# Add points
for n in points:
    eval(f'''par.add_point_{n[2]}(n[0], n[1])''')

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
par.add_point_SPTP([object_lim_base[0][0] - nom_dist*2, 0, 800], rot_frontal,
                   40, marker=0)
rot_side1 = np.array([45, -45, 180])
par.add_point_SLIN([0,object_lim_base[1][0] - nom_dist*2, 800], rot_side1,
                   0.5, marker=0)

# One side
xx = np.arange(0, object_lim_base[0][0]-nom_dist-1, -point_dist) # Place for the mesh
y = object_lim_base[1][0] - nom_dist
zz = np.arange(object_lim_base[2][0],
                object_lim_base[2][1]+nom_dist+1,point_dist)
points = []
x_pos = True
for z in zz:
    if x_pos:
        for x in xx:
            points.append( ( [x,y,z], rot_side1 ,'SLIN') )
    else:
        for x in np.flip(xx):
            points.append( ( [x,y,z], rot_side1 ,'SLIN') )
    x_pos = not x_pos
# Add points
for n in points:
    eval(f'''par.add_point_{n[2]}(n[0], n[1])''')

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
par.add_point_SLIN([0, object_lim_base[1][0] - nom_dist*2, 800], initial_rot,
                   0.5, marker=0)
rot_side2 = np.array([-45, -45, 180])
par.add_point_SLIN([0, object_lim_base[1][1] + nom_dist*2, 800], rot_side2,
                   0.5, marker=0)

# Other side
xx = np.arange(0,object_lim_base[0][0]-nom_dist-1,-point_dist)
y = object_lim_base[1][1]+nom_dist
zz = np.arange(object_lim_base[2][0],
                object_lim_base[2][1]+nom_dist+1,point_dist)
points = []
x_pos = True
for z in zz:
    if x_pos:
        for x in xx:
            points.append( ( [x,y,z], rot_side2 ,'SLIN') )
    else:
        for x in np.flip(xx):
            points.append( ( [x,y,z], rot_side2 ,'SLIN') )
    x_pos = not x_pos
# Add points
for n in points:
    eval(f'''par.add_point_{n[2]}(n[0], n[1])''')

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
par.add_point_SLIN([0, object_lim_base[1][1] + nom_dist*2, 800], rot_side2,
                   0.5, marker=0)
rot_upper = np.array([0, 0, 180])
par.add_point_SLIN([0, 0, 800], rot_upper, 0.5, marker=0)

# Upper plane
xx = np.arange(0,object_lim_base[0][0]-nom_dist-1,-point_dist)
yy = np.arange(object_lim_base[1][0]-nom_dist,
                object_lim_base[1][1]+nom_dist+1,point_dist)
z = object_lim_base[2][1] + nom_dist
points = []
z_pos = True
for y in yy:
    if z_pos:
        for x in xx:
            points.append( ( [x,y,z], rot_upper ,'SLIN') )
    else:
        for x in np.flip(xx):
            points.append( ( [x,y,z], rot_upper ,'SLIN') )
    z_pos = not z_pos
# Add points
for n in points:
    eval(f'''par.add_point_{n[2]}(n[0], n[1])''')

if visualize:
    xx = []
    yy = []
    zz = []
    for n in points:
        xx.append(n[0][0]/10)
        yy.append(n[0][1]/10)
        zz.append(n[0][2]/10)
    ax.scatter(xx,yy,zz,color='xkcd:orange')

par.name = f'near_field{nom_dist:.0f}'

# Export
par.export(path='real_parcours/')

if visualize:
    ax.set_title(f'Plans – Distance {nom_dist//10:.0f} cm')
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