##WIP code to plot 6D data (XYZ for 2 joints), using RGB for XYZ#2

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from scipy import stats
%matplotlib qt

##No way to install pandas in Maya, needs to be rewritten to work with just numpy

def seriesGen(max,length):
    x = np.random.choice(max+1,length+1,replace=True)
    np.random.shuffle(x)
    return(x)

def sixAxDataGen(max,length):
    x,y,z,r,g,b = [seriesGen(max, length) for i in range(6)]
    axis = np.array([x,y,z,r,g,b]).transpose()
    return(axis)

def findAngularRangeNp(data):
    rgbComp = data[[-3,-2,-1]]
    rMax, gMax, bMax = [data[:,i].max() for i in range(0,3)]
    rgbMax = max(rMax, gMax, bMax)
    rgbMax = (round(rgbMax,-1)+10)
    return(rgbMax)

def findAngularRangePd(data):
    rgbComp = data[['r','g','b']]
    rMax, gMax, bMax = [data.iloc[:,i].max() for i in range(0,3)]
    rgbMax = max(rMax, gMax, bMax)
    rgbMax = (round(rgbMax,-1)+10)
    return(rgbMax)

#returns 6 columns + 1 additional rgb column
def dataFormatNp(data):
    df = np.array(data, copy=1)
    angleRange = findAngularRangeNp(df)
    print('RGB values are normalized to axis of greatest rotation: '+str(angleRange))
    rScaled=df[:,3]/angleRange
    gScaled=df[:,4]/angleRange
    bScaled=df[:,5]/angleRange
    rgbScaled = np.array((rScaled, gScaled, bScaled)).T
    rgbScaled = [tuple(x) for x in rgbScaled.tolist()]
    df = np.append(df,rgbScaled,axis=1)
    return(df, angleRange)

def dataFormatPd(data):
    df = pd.DataFrame(data=data, columns=['x','y','z','r','g','b'])
    angleRange = findAngularRangePd(df)
    print('RGB values are normalized to axis of greatest rotation: '+str(angleRange))
    df['rgb'] = df.apply(lambda row: (row.r/angleRange,row.g/angleRange,row.b/angleRange), axis=1)
    return(df, angleRange)

def plot6D(data,alpha=0.5):
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(xs=data['x'],ys=data['y'],zs=data['z'], c=data['rgb'], alpha=alpha, s=2)
    plt.show()

test6D = sixAxDataGen(90,800)
test7D,d = dataFormatPd(test6D)
plot6D(test7D)
