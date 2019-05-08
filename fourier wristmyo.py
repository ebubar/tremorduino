import csv
import math
from scipy.fftpack import fft
import matplotlib.pyplot as plt
import numpy as np

time = []
ax = []
ay = []
az = []
muscle = []
vec = []

with open(r'C:\Users\ebuba\Desktop\test.csv') as csvDatafile:
    csvReader = csv.reader(csvDatafile, delimiter=',')
    for row in csvReader:
        time.append(float(row[0]))
        ax.append(float(row[1]))
        ay.append(float(row[2]))
        az.append(float(row[3]))
        muscle.append(float(row[7]))

axmag = np.square(ax)
aymag = np.square(ay)
azmag = np.square(az)

magn = axmag+aymag+azmag


t = np.linspace(0,1,1000)
f = np.fft.fftfreq(len(magn),t[1]-t[0])

data_fft = np.abs(np.fft.fft(magn)) / len(magn)

print(f)


