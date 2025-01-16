'''
this can be used for pose estimation
but the starting point for fsolve is important and needs to be picked with care
which can be done using the approxiamtion for v and w
ur > 1.7478ul or ul > 1.7478ur causes the program to have a runtime warning
because the equations do not intersect in the expected region
for these cases, we can still use the output of fsolve and embrace the error
we can also try to avoid these scenarios when driving
'''

from scipy.optimize import fsolve
T = 1
r = 1
ur = 2
ul = 3 * ur
L = 1
def eqs(p):
    v, w = p
    eq1 = v + T*w/2 + (L*w/2)**2/(v+T*w/2) - ur*r
    eq2 = v - T*w/2 + (L*w/2)**2/(v-T*w/2) - ul*r
    return [eq1, eq2]


v = r/2 * (ur+ul)
w = r/T * (ur-ul)
print(eqs((v,w)))
print((v,w))
v,w = fsolve(eqs,(v,w))

print(eqs((v,w)))
print((v,w))