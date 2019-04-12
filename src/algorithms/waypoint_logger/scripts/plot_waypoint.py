import scipy.interpolate
import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
import pdb

mydata = genfromtxt('levine-fast-extra-right-waypoints.csv', delimiter=',')
spt = 0 # starting point
ept = 50 # ending point

# This is simpler code to just plot waypoints as given
#x = mydata[:,0]
#y = mydata[:,1]

#plt.plot(x, y, 'k--')
#plt.show()

#print(x)

# s is smoothing parameter
tck,u = scipy.interpolate.splprep([mydata[spt:-ept,0], mydata[spt:-ept,1]], s=0)
unew = np.arange(0,1.01,0.01)
out_raw = scipy.interpolate.splev(unew,tck)

tck,u = scipy.interpolate.splprep([mydata[spt:-ept,0], mydata[spt:-ept,1]], s=50, k=3)
unew = np.arange(0,1.01,0.01)
out_smooth = scipy.interpolate.splev(unew,tck)
out_smooth = np.asarray(out_smooth)
out_smooth = out_smooth[:,0:-1]
plt.plot(out_raw[0], out_raw[1], 'k--',
         out_smooth[0], out_smooth[1], 'c--', 
         out_smooth[0], out_smooth[1], 'b*',
         out_smooth[0,0], out_smooth[1,0], 'g*', 
         out_smooth[0,-1], out_smooth[1,-1], 'r*')
plt.show()
