#%%

import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy import signal

from astropy.timeseries import LombScargle

#%%

file = open("data_out.csv",'r')
reader = csv.reader(file)

headers = next(reader, None)
print(headers)

data = {}
for h in headers:
    data[h] = []
for row in reader:
    for h, v in zip(headers, row):
        data[h].append(v)

i=0;

%matplotlib
fig_raw = plt.figure(figsize=(10,8))
fig_raw.suptitle("Raw Head Points Data")
for output in data:
    if output == "timestamps":
        ts = np.array(data[output]).astype(np.float)
        x = (ts - ts[0])
    else: 
        i+=1
        raw = np.array(data[output]).astype(np.float).astype(np.int)
        plt.subplot(5,2,i)
        plt.plot(x,raw)
        plt.ylim((-1,480)), plt.xlim(left=0)
        plt.xlabel("Time (s)"), plt.ylabel("px")
        plt.yticks((-1, 160, 320, 480))
        plt.subplots_adjust(hspace=1.3)
        plt.title(headers[i])

#%%
fig_post = plt.figure(figsize=(10,8))
fig_post.suptitle("Spectral Power")
i=0
for output in data:
    if output != "timestamps":
        i+=1
        raw = np.array(data[output]).astype(np.float).astype(np.int)
        frequency, power = LombScargle(x, signal).autopower()
        plt.subplot(5,2,i)
        plt.plot(frequency, power)
        plt.xlim(left=0)
        plt.yticks((0, 0.2, 0.4, 0.6))
        plt.xlabel("Frequency [Hz]"), plt.ylabel("Power [a.u.]")
        plt.subplots_adjust(hspace=1.3)
        plt.title(headers[i]) 


#%%
#   
inter_frame_time = np.diff(x)
instant_fps = 1/inter_frame_time
mean_fps = np.ones(instant_fps.shape)*np.mean(instant_fps)

%matplotlib
fig = plt.figure()
plt.plot(istant_fps, label="Instantaneous FPS")
plt.plot(mean_fps, label="Mean FPS: {:.1f}".format(mean_fps[0]))
plt.ylabel("FPS")
plt.xlabel("Frame")
plt.legend()

#%%
fig_post = plt.figure(figsize=(10,8))
fig_post.suptitle("Filtered Data Points")
i=0
for output in data:
    if output != "timestamps":
        i+=1
        raw = np.array(data[output]).astype(np.float).astype(np.int)
        sos = signal.butter(16, 0.3, 'lp',output='sos')
        filtered_signal = signal.sosfiltfilt(sos, raw)
        plt.subplot(5,2,i)
        plt.plot(x,filtered_signal, label='filtered')
        plt.plot(x, raw, color='r', label='raw')
        plt.xlim(left=0)
        plt.yticks((-1, 160, 320, 480))
        plt.xlabel("Time (s)"), plt.ylabel("px")
        plt.subplots_adjust(hspace=1.3)
        plt.title(headers[i]) 
# %%

fig_post = plt.figure(figsize=(10,8))
fig_post.suptitle("Spectral Power (Filtered??)")
i=0
for output in data:
    if output != "timestamps":
        i+=1
        raw = np.array(data[output]).astype(np.float).astype(np.int)
        sos = signal.butter(16, 0.7, 'lp',output='sos')
        filtered_signal = signal.sosfiltfilt(sos, raw)
        frequency, power = LombScargle(x, filtered_signal.astype(np.int)).autopower()
        plt.subplot(5,2,i)
        plt.plot(frequency, power)
        plt.xlim(left=0)
        plt.yticks((0, 0.2, 0.4, 0.6))
        plt.xlabel("Frequency [Hz]"), plt.ylabel("Power [a.u.]")
        plt.subplots_adjust(hspace=1.3)
        plt.title(headers[i]) 
