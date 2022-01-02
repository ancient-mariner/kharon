#!/usr/bin/env python2
import sys
from random import random
from scipy import optimize
import numpy as np

# fit algorithm taken from 
#     https://stackoverflow.com/questions/44647239/how-to-fit-a-circle-to-a-set-of-points-with-a-constrained-radius

if len(sys.argv) != 2:
   print("Usage: %s <imu log file>" % sys.argv[0])
   sys.exit(1)

def residuals(parameters,dataPoint):
    xc,yc,Ri = parameters
    distance = [np.sqrt( (x-xc)**2 + (y-yc)**2 ) for x,y in dataPoint]
    res = [(Ri-dist)**2 for dist in distance]
    return res

def f0(phi,x0,y0,r):
    return [x0+r*np.cos(phi),y0+r*np.sin(phi)]



mag = []
infile = sys.argv[1]
with open(infile, 'r') as f:
   contents = f.readlines()
   for line in contents:
      if len(line) < 5:
         continue
      toks = line.split()
      mag.append([float(toks[7]), float(toks[9])])
data = np.array(mag)

estimate = [0, 0, 10]
bestFitValues, ier = optimize.leastsq(residuals, estimate,args=(data))
print("# estimated x,z mag offset as installed (no scaling)")
print("# generated from compass_correction.py")
print("%f %f" % (bestFitValues[0], bestFitValues[1]))
print("# fit circle radius is %f" % bestFitValues[2])


pList=np.linspace(0,2*np.pi,35)
rList=np.array([f0(p,*bestFitValues) for p in pList])


#import matplotlib
#matplotlib.use('Qt4Agg')
#from matplotlib import pyplot as plt
#fig = plt.figure()
#ax = fig.add_subplot(111)
#ax.scatter(data[:,0],data[:,1])
#ax.plot(rList[:,0],rList[:,1])
## 2nd to change color as don't recall param
#ax.plot(rList[:,0],rList[:,1])
#plt.show()
