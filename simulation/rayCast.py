import numpy as np
#use ray cast algorithm to judge whether a point is in a polygon(4 vertices)
def rayCast(x):
    result = False
    x0 = x[0]
    x1 = x[1]
    x2 = x[2]
    x3 = x[3]
    p = x[4]
    pFar = x[5]
    cnt = 0
    x = np.stack((x0,x1,p,pFar))
    cnt = cnt + crossOrNot(x)
    x = np.stack((x1,x2,p,pFar))
    cnt = cnt + crossOrNot(x)
    x = np.stack((x2,x3,p,pFar))
    cnt = cnt + crossOrNot(x)
    x = np.stack((x3,x0,p,pFar))
    cnt = cnt + crossOrNot(x)

    if cnt <-10:
        result = False
    elif cnt%2 == 0:
        result = False
    else:
        result = True
    return result

#https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
def crossOrNot(x):
    x0 = x[0]
    x1 = x[1]
    x2 = x[2]
    x3 = x[3]
    p = x0
    r = x1 - x0
    q = x2
    s = x3 - x2
    if np.sum(np.cross(r,s)**2) == 0:
        return -100
    t = np.cross((q-p), s)/(np.cross(r,s)+0.0001)
    u = np.cross((q-p), r)/(np.cross(r,s)+0.0001)
    t = t[0]
    u = u[0]

    if t>0 and t<1 and u>0 and u<1:
        return 1
    elif t == 0 or t == 1 or u == 0 or u ==1:
        return -100
    else:
        return 0
