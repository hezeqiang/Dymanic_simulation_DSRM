import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()   
# 生成子图，相当于fig = plt.figure(),ax = fig.add_subplot(),其中ax的函数参数表示把当前画布进行分割，
# 例：fig.add_subplot(2,2,2).表示将画布分割为两行两列　　　　　　　　　　　　　　　　
# ax在第2个子图中绘制，其中行优先，
xdata, ydata = [], []      #初始化两个数组
line= ax.plot(xdata, ydata, 'r-', animated=False)  
#第三个参数表示画曲线的颜色和线型，具体参见：https://blog.csdn.net/tengqingyong/article/details/78829596
frame_total=30


def init():
    ax.set_xlim(0, 2*np.pi)  #设置x轴的范围pi代表3.14...圆周率，
    ax.set_ylim(-1, 1)  
    return line         #返回曲线

def update(n):
    xdata.append(n)         #将每次传过来的n追加到xdata中
    ydata.append(np.sin(n))
    line= ax.plot(xdata, ydata, 'r-', animated=False)      #重新设置曲线的值
    
    if len(xdata)==(frame_total-1):   # 不可以画最后一个点(0,0), 判断后使用 KeyboardInterrupt 来做暂停
        raise KeyboardInterrupt

    return line

ani = FuncAnimation(fig, update, frames=frame_total, init_func=None, blit=False,interval=100)

"""frames设定帧数,总共执行100个update就会进行下一次循环，并且frames还会作为参数传入animate()函数，init_func设定初始函数图像,
interval设置更新间隔此处设置为100毫秒，(仔细想想20毫秒其实是很小的一个间隔)
blit如果是只有变化了的像素点才更新就设置为True,如果是整张图片所有像素点全部更新的话就设置为False
"""
plt.show()