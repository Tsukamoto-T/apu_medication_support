#!/usr/bin/env python
# -*- coding: utf-8 -*-

import statistics
import math
import numpy as np
import pandas as pd
import csv
import matplotlib.pyplot as plt
from scipy.stats import norm
from scipy.optimize import curve_fit

#閾値
threshold = 26
target = 'case'
target_num = '8'
dataset = pd.read_table('../data/ex_1_'+ target +'/target_map_'+ target +'_'+ target_num +'.csv',header=None, dtype=float)
arr = dataset.values

dataset_hist = pd.read_table('../data/ex_1_'+ target +'/histgram_'+ target +'_'+ target_num +'.csv',header=None, dtype=int)
hist = dataset_hist.values
hist_x, hist_y = zip(*hist)
sum_y = sum(hist_y)
hist_y = [float(n)/sum_y for n in hist_y]

#入力データ表示
plt.xlim(0,255)
plt.ylim(0,0.20)
plt.ylabel('Ratio of pixels')
plt.xlabel('Feature value')

plt.bar(hist_x,hist_y)
plt.show()
plt.close()

#移動平均
hist_y = pd.Series(hist_y)
hist_y_mean = hist_y.rolling(10).mean()

#閾値以上のデータ
th_hist_y = list(hist_y)
#for i in range(threshold+1):
#    th_hist_y[i] = 0.0
plt.axvline(x=threshold,color="g")
plt.xlim(0,255)
plt.ylim(0,0.20)
plt.ylabel('Ratio of pixels')
plt.xlabel('Feature value')

plt.bar(hist_x,th_hist_y)
plt.plot(hist_x,hist_y_mean,color="r")
plt.show()
plt.close()

#閾値以上のデータ
plt.xlim(0,255)
plt.ylim(0,0.005)
plt.ylabel('Ratio of pixels')
plt.xlabel('Feature value')

plt.plot(hist_x,hist_y_mean,color="r")
plt.axvline(x=threshold,color="g")
plt.bar(hist_x,th_hist_y)
plt.legend(['moving average','threshold','feature data'])
plt.show()
plt.close()


#データから一次元正規分布フィッティング
data_arr = [int(round(n*255)) for n in arr]
max_data = max(data_arr)

mean,std=norm.fit(data_arr)
print("mean : ", mean, "std : ", std)

bin_num = max_data
n, bins, patches=plt.hist(data_arr, normed=True,range=(0,max_data),bins=bin_num,width=0.8)
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
y = norm.pdf(x, mean, std)

#print (plt.hist(data_arr, normed=True))
plt.xlim(0,255)
plt.ylim(0,0.20)
plt.plot(x, y,color='r')
plt.ylabel('Ratio of pixels')
plt.xlabel('Feature value')
plt.legend(['gaussian fitting','feature data'])
plt.show()

#plt.scatter(hist_x,hist_y)
#plt.show()
