import numpy as np
import matplotlib.pyplot as plt

def get_circle(n, ray):
    phis = np.arange(0, 2*np.pi, 2*np.pi/n)
    points = np.zeros([n, 2])

    for i, phi in enumerate(phis):
        x = ray * np.cos(phi)
        y = ray * np.sin(phi)
        points[i, 0] = x
        points[i, 1] = y

    return points

def get_random_points(points, n):
    indexes = np.random.choice(len(points), n)
    return points[indexes, :]

def get_phi(point):
    x = point[0]
    y = point[1]
    phi = np.arctan(y/x) 
    if x < 0 and y > 0:
        phi = phi + np.pi
    elif x < 0 and y < 0:
        phi = phi + np.pi
    elif x > 0 and y < 0:
        phi = phi + 2*np.pi

    return phi

def get_equilater_triangle(point, ray):
    phi = get_phi(point)
    phi_l = phi + 2/3 * np.pi
    phi_r = phi - 2/3 * np.pi

    x = point[0]
    y = point[1]

    x_l = ray * np.cos(phi_l)
    y_l = ray * np.sin(phi_l)
    x_r = ray * np.cos(phi_r)
    y_r = ray * np.sin(phi_r)

    return np.array([[x,y], [x_l,y_l], [x_r, y_r]])


class combinations:
    _vector = 0
    _combination = 0
    _dim = 0

    def __init__(self):
        self._vector = np.array([])
        self._combination = np.array([])
        # to use for array of array - maybe a list is sufficient, creating the obj point ..
        self._dim = 0

    def comb(self, points, m, k):
        if m==k:
            self._combination = np.append(self._combination, self._vector)
        else:
            for i in range(len(points)):
                self._vector = np.append(self._vector, points[i, :])
                self.comb(points[i+1:,:], m+1, k)
                self._vector = np.delete(self._vector, [-1 , -2])

                #self._vector = np.append(self._vector, points[i])
                #self.comb(points[i:], m+1, k)
                #self._vector = np.delete(self._vector, -1 )
    
        return self._combination

def distance(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**(1/2)

def get_perimeter(triangle):
    a = [triangle[0], triangle[1]]
    b = [triangle[2], triangle[3]]
    c = [triangle[4], triangle[5]]
    perimeter = distance(a, b) + distance(b, c) + distance(a, c)
    return perimeter


n_points = 1000
ray = 2
points = get_circle(n_points, ray)

n_sampled = 5
sampled_points = get_random_points(points, n_sampled)
#print(sampled_points)

compute_comb = combinations()
points_comb = compute_comb.comb(sampled_points, 0, 3)
points_comb = points_comb.reshape(int(len(points_comb)/6), 6)

# get triangle with biggest perimeter computing all perimeter
max_triangle = np.array([])
max_perimeter = 0
for triangle in points_comb:
    perimeter = get_perimeter(triangle)
    if perimeter > max_perimeter:
        max_triangle = triangle
        max_perimeter = perimeter

# get triangle with biggest perimeter using the equilater triangle
max_perimeter = 0
max_triangle_equil = 0
phis = [get_phi(point) for point in sampled_points]
for point in sampled_points:
    max = get_equilater_triangle([point[0], point[1]], ray)
    # compute phis of max 
    phi1 = get_phi(max[1,:])
    phi2 = get_phi(max[2,:])
    # nearest points to phi1 and phi2

    # brute force. there is for sure a better method.
    dist1 = abs(phis - phi1)
    dist2 = abs(phis - phi2)
    ind1 = np.where(dist1==min(dist1))[0]
    ind2 = np.where(dist2==min(dist2))[0]

    point1 = sampled_points[ind1, :]
    point2 = sampled_points[ind2, :]

    triangle = np.append(point, point1)
    triangle = np.append(triangle, point2)

    perimeter = get_perimeter(triangle)
    if perimeter > max_perimeter:
        max_triangle_equil = triangle
        max_perimeter = perimeter

# print results of both algo
print(max_triangle)
print(max_triangle_equil)

# plot results
plt.figure()
plt.plot(points[:,0], points[:,1])
plt.plot(sampled_points[:,0], sampled_points[:,1], 'o')

x_ind = [True, False, True, False, True, False]
y_ind = [False, True, False, True, False, True]

x_tr = max_triangle[x_ind]
y_tr = max_triangle[y_ind]
plt.plot(np.append(x_tr, x_tr[0]), np.append(y_tr, y_tr[0]), 'b--')

x_tr = max_triangle_equil[x_ind]
y_tr = max_triangle_equil[y_ind]
plt.plot(np.append(x_tr, x_tr[0]), np.append(y_tr, y_tr[0]), 'r-.')

plt.show() 












"""
for triangle in points_comb:
    plt.figure()
    plt.plot(points[:,0], points[:,1])
    plt.plot(sampled_points[:,0], sampled_points[:,1], 'o')

    #print(triangle)
    triangle = np.array(triangle)
    x_ind = [True, False, True, False, True, False]
    x_tr = triangle[x_ind]
    y_ind = [False, True, False, True, False, True]
    y_tr = triangle[y_ind]
    plt.plot(np.append(x_tr, x_tr[0]), np.append(y_tr, y_tr[0]))

    max = get_equilater_triangle([x_tr[0], y_tr[0]], ray)
    #print(max)
    #plt.plot(max[:,0], max[:,1])
    plt.plot(np.append(max[:,0], x_tr[0]), np.append(max[:,1], y_tr[0]))
    plt.show() 
"""
