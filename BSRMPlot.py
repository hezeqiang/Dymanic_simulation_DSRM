#coding:u8
from pylab import plt, mpl, np
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
# from pprint import pprint
from collections import OrderedDict as O
import pandas as pd

# plot style
#produce a one-demesion array between （0，5） containing 4 element
#a = np.random.randint(0, 5, (4,))
#plt.style.available print all available style in plt
style = np.random.choice(plt.style.available);print(style);
#use specific style named ggplot
plt.style.use('ggplot')
#frequently used style
#['bmh', 'classic', 'dark_background', 'fast', 'fivethirtyeight', 'ggplot', 'grayscale']

# plot setting
mpl.rcParams['mathtext.fontset'] = 'stix'
mpl.rcParams['font.family'] = 'STIXGeneral'
mpl.rcParams['legend.fontsize'] = 12.5
# mpl.rcParams['legend.family'] = 'Times New Roman'
mpl.rcParams['font.family'] = ['Times New Roman']
mpl.rcParams['font.size'] = 14.0
# mpl.style.use('classic')
font = {'family' : 'Times New Roman', #'serif',
        'color' : 'darkblue',
        'weight' : 'normal',
        'size' : 14,}
textfont = {'family' : 'Times New Roman', #'serif',
            'color' : 'darkblue',
            'weight' : 'normal',
            'size' : 11.5,}

#print(matplotlib.rc_params())
#print default setting
#mpl.rcdefaults()
#use the default setting
######################
# Plotting
def get_axis(cNr):
    # fig, axes = plt.subplots(ncols=cNr[0], nrows=cNr[1], dpi=150, sharex=True);
    fig, axes = plt.subplots(ncols=cNr[0], nrows=cNr[1], sharex=True, figsize=(16*0.8, 9*0.8), dpi=80, facecolor='w', edgecolor='k');
    # column number  cNr[0], row number cNr[1], share the x axis in figure, size setting, dpi , background color,edge color
    fig.subplots_adjust(right=0.95, bottom=0.1, top=0.95, hspace=0.2, wspace=0.02)
    #figure ax size setting
    if sum(cNr)<=2:
        return axes
    else:
        #create a variable for every axes
        return axes.ravel()

def plot_key(ax, key, df):
    # plot the curve with time as x and values as y
    ax.plot(time, df[key].values, '-', lw=1)
    #set y label name
    ax.set_ylabel(key, fontdict=font)

def plot_it(ax, ylabel, d, time=None):
    count = 0
    for k, v in d.items():
        if count == 0:
            count += 1
            # ax.plot(time, v, '--', lw=2, label=k)
            ax.plot(time, v, '-', lw=1)
        else:
            # ax.plot(time, v, '-', lw=2, label=k)
            ax.plot(time, v, '-', lw=1)

    # ax.legend(loc='lower right', shadow=True)
    # ax.legend(bbox_to_anchor=(1.08,0.5), borderaxespad=0., loc='center', shadow=True)
    ax.set_ylabel(ylabel, fontdict=font)
    # ax.set_xlim(0,35) # shared x
    # ax.set_ylim(0.85,1.45)

if __name__ == '__main__':

    df_info = pd.read_csv(r"./info.dat", na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])
    # if reading string is '1.#QNAN', '-1#INF00', '-1#IND00, then set the string as NaN
    data_file_name = df_info['DATA_FILE_NAME'].values[0].strip()
    # the first row is defaulted name for every column. strip() means element is a string
    #   df_info['DATA_FILE_NAME'].values[0].strip() represent the first element in colunm ['DATA_FILE_NAME']
    print(data_file_name)
    df_profiles = pd.read_csv(data_file_name, na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])
    #   read the data file containing the simulation results.

    no_samples = df_profiles.shape[0]
    # get number of rows
    no_traces = df_profiles.shape[1]
    # get number of column, namely number of keys
    print(df_info, 'Simulated time: %f s.'%(no_samples * df_info['WRITE_PER_TIME'].values[0]), 'Key list:', sep='\n')
    for key in df_profiles.keys():
        print('\t', key)
    #   print all key of the data ,which is the first row string in data file
    #   the value of key is corresponding list
    time = np.arange(1, no_samples+1) * df_info['WRITE_PER_TIME'].values[0]

    ax_list = []
    for i in range(0, no_traces, 2):    #   for i in range(1, 101，1) print from 1 to 101
        ax_list += list(get_axis((1,2)))    #   add axis list to ax_list
#    for i in range(0, no_traces, 1):    #   for i in range(1, 101，1) print from 1 to 101
#        ax_list.append(get_axis((1,1)))    #   add axis list to ax_list


    for ID_x, key in enumerate(df_profiles.keys()):     #   enumerate() is utilized to give num for every key
        plot_it(ax_list[ID_x], key, O([
                                        (str(ID_x), df_profiles[key]),
                                        # (str(idx), df_profiles[key]),  
                                        ]), time)

    #   fig.savefig('figure_name.png')  must be placed before   plt.show
    #   the final direction in the script to creat the defined figure
    plt.show()

    quit()


