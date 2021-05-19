# from pylab import subplots, mpl, plt, np

import matplotlib as mpl
from matplotlib.pyplot import subplots
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import time as time_package


def get_data(fname):
    try:
        data = pd.read_csv(fname) #, skiprows=2, nrows=7) # na_values = ['no info', '.', '']
    except pd.errors.EmptyDataError as error:
        # print(str(error))
        # time_package.sleep(1)
        raise error
    keys = data.keys()
    return data, keys

fname = './algorithm.dat'
data, keys = get_data(fname)


fig, ax_list = plt.subplots(5, sharex=True, figsize=(16*0.8, 9*0.8), dpi=80, facecolor='w', edgecolor='k', constrained_layout=False)

SAMP_TS = 10/4000.000000 # TS=1/8000, down_exe=2, down-sampling 10: SAMP_TS=1/400
n = number_points_draw_at_once = 100 # plot n points at a time
POST_TS = SAMP_TS * n

_index = 0
def animate(_):
    global data, keys, _index
    if not bool_animate_pause:      #键击事件 发生了就会停止绘图，不再绘图
        if _index > 0:
            time = np.arange(1, n*_index+1) * SAMP_TS
            for idx, ax in enumerate(ax_list):
                ax.cla()
                try:
                    ax.plot(time, data[keys[idx]][:n*_index])

                except ValueError as error:
                    # print(str(error))
                    data_backup = data
                    data, keys = get_data(fname)
                    print('Read data.')
                    if len(data[keys[0]]) == len(data_backup[keys[0]]):   # 判断该 axes是否结束，一个axes结束了绘图就全部结束了
                        print('Plotting data are exausted. Quit now.')
                        _index = -1
                        break
                else:
                    ax.set_ylabel(keys[idx])         # 没有错误就更新 纵坐标 label
            ax_list[-1].set_xlabel('Time [s]')    # 统一了横坐标， 实际上是为每个 axes 配置相同的横轴
        if _index == -1:
            raise KeyboardInterrupt
        _index += 1

def onClick(event):    #键击 事件
    global bool_animate_pause
    bool_animate_pause ^= True   # 取反
 
import matplotlib.animation as animation
bool_animate_pause = False
ani = animation.FuncAnimation(fig, animate, blit=False, interval=10, repeat=True)
fig.canvas.mpl_connect('button_press_event', onClick)
time_package.sleep(0.1)
print('Animate starts...')
plt.show()






