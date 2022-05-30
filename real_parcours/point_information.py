from scipy.io import loadmat

# data = loadmat('grid_dist570.mat')
data = loadmat('near_field40.mat')

# Number of points
n_point = 0
for key in data.keys():
    try:
        key = int(key)
        if key>n_point:
            n_point=key
    except:
        pass

markers = []
for n in range(n_point):
    markers.append(data[f'{n}'][0][-1]) # Marker selection

print(f''' 
        Point information
        ––––––––––––––––––––––––––
        Number of points: {n_point+1}
        Measuring points: {sum(markers)}
        Security points: {n_point+1-sum(markers)}
        ''')