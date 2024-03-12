import numpy as np

np.set_printoptions(suppress=True)
f = 100
fs = 48000
frel = 48000/100
angle = np.linspace(0, 2*np.pi - (1/480)*2*np.pi, 480)
sine = np.array([int(np.sin(i) * 2**15-1) for i in angle])
print(np.array2string(sine, separator=','))