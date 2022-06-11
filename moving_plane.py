import matplotlib.pyplot as plt
import numpy as np
from parcours import Parcours
from scipy.io import savemat

'''
This script creates a moving frontal wall of measuring points for an exemplary
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
par.set_object_lims([[-183.5,183.5],[-140,140],[0,400]]) # Box
point_dist = 85

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
initial_rot = np.array([0,-90,180])
par.add_point_SPTP([0,0,800],initial_rot,20,marker=0)

nom_dist = -40-183.5 # nominal distance for plane location
## Create coordinates
# Frontal plane
rot_frontal = np.array([0,-45,180])
par.add_point_SLIN([nom_dist-30,0,800],rot_frontal,0.25,marker=0)

distances = np.arange(nom_dist,nom_dist-201,-50)

label = True
for x in distances:
    yy = np.arange(-200,200+1,point_dist)
    zz = np.arange(0,405+1,point_dist)
    points = []
    y_pos = True
    if x < -400:
        rot_frontal = [0,0,180]
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
        eval(f'''par.add_point_{n[2]}(n[0],n[1])''')
    if visualize:
        xx = []
        yy = []
        zz = []
        for n in points:
            xx.append(n[0][0]/10)
            yy.append(n[0][1]/10)
            zz.append(n[0][2]/10)
        if label:
            ax.scatter(xx,yy,zz,color='xkcd:orange',label='Points \nde mesure')
            label = False
        else:
            ax.scatter(xx,yy,zz,color='xkcd:orange')

# Security point
par.add_point_SLIN([nom_dist-30,0,800],rot_frontal,0.25,marker=0)

par.name = f'moving_front{-nom_dist:.0f}'

# Export
par.export(path='real_parcours/')

if visualize:
    ax.set_title(f'Plans – Devant')
    ax.set_xlabel('$x$ [cm]')
    ax.set_ylabel('$y$ [cm]')
    ax.set_zlabel('$z$ [cm]')
    fig.tight_layout()
    ax.legend(loc='center',bbox_to_anchor=(-0.09,0.5))
    # fig.savefig(f'plots/moving_plane.pdf')
    # fig.savefig(f'''/Users/nicolas/Documents/Uni/Master – 4. Semester 
    #  INSA/Projet de recherche/Latex/Bilder/parcours/
    # moving_plane.pdf'''.replace('\n',''))
    plt.show()