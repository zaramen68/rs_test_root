from scipy.stats import norm
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

import math
import numpy as np

from scipy.optimize import curve_fit

def func(x,  a1, s1, M1, a2, s2, M2, a3, s3, M3):

    return  a1/(np.exp((x-M1)**2/(2*s1**2))*(s1*math.sqrt(2*math.pi))) + a2/(np.exp((x-M2)**2/(2*s2**2))*(s2*math.sqrt(2*math.pi)))+\
                a3/(np.exp((x-M3)**2/(2*s3**2))*(s3*math.sqrt(2*math.pi)))

# read data from a text file. One number per line
arch = "/home/tenderbook/rs_test/1_150.in"
datos = []
bins = []
for item in open(arch,'r'):
    try:
        item = item.strip('\n')
        item_ = item.split(',')
        y = item_[1]
        bin = item_[0]
    except Exception as ex:
        print(ex)
    else:
        if y != '':
            try:
                datos.append(float(y))
                bins.append(float(bin))
            except ValueError:
                pass
p0 = [1.0, 0.00001, 1.0, 2.0, 0.01, 0.4, 3.0, 0.01, 0.35]
bound=(0, [10.0, 0.1, 2.0, 10., 0.1, 2.0, 10., 0.1, 2.0])
# plt.subplot(111)
plt.plot(bins, datos, 'r--', label='data')
# plt.subplot(112)
plt.bar(bins, datos, width=bins[1])

popt, pcov = curve_fit(f = func, xdata = bins, ydata = datos, p0=p0, absolute_sigma = True, check_finite=True, method='dogbox')
perr = np.sqrt(np.diag(pcov))

# best fit of data
# (mu, sigma) = norm.fit(datos)

# the histogram of the data
# n, bins, patches = plt.hist(datos, bins, normed=1, facecolor='green', alpha=0.75)
# n, bins, patches = plt.hist(x=bins, bins=datos)

# add a 'best fit' line
# y = mlab.normpdf( bins, mu, sigma)

y = func(bins, *popt)


ydata = y 

# plt.plot(bins, func(bins, *popt), 'r-', label='fit: s1=%5.3f, M1=%5.3f, s2=%5.3f, M2=%5.3f, s3=%5.3f, M3=%5.3f' % tuple(popt))
# plt.subplot(114)
plt.plot(bins, func(bins, *popt), 'g^')

plt.show()





a=1