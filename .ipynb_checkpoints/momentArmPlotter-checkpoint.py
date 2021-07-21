import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import statsmodels.api as sm
from statsmodels.api import OLS
from sklearn import preprocessing
from sklearn.preprocessing import PolynomialFeatures
from sklearn.metrics import r2_score
from sklearn.model_selection import train_test_split
%matplotlib inline
sns.set(style="darkgrid")

axes = ['X','Y','Z']
axis_colors = ['pale red','medium green','denim blue']
color_dict = dict(zip(axes, [sns.xkcd_rgb[axis] for axis in axis_colors]))


def cmToMm(df):
    ## multiplies by 10 to convert from centimeters to millimeters
    df = df*10
    return df


def getIndices(range_string: str):
    ## takes q 1-indexed range as a string and parses it into a 0-indexed list of indices
    chunks = range_string.split(',')
    selection = []
    for i in range(0,len(chunks)):
        subchunk = chunks[i].split('-')
        if len(subchunk) == 1:
            selection.append(int(subchunk[0]))
        else:
            nums = range(int(subchunk[0]),int(subchunk[1])+1)
            selection.extend(list(nums))
    selection_index = [x-1 for x in selection]
    return selection_index

def importData(csv: str, rows: str, cols: str, convert_cm_mm=0):
    ## takes a csv with column headers but no frame column (XROMM tools default)
    ## pass frame ranges and column ranges as strings in 'first-last' syntax, using commas as separators
    df = pd.read_csv(csv)
    rows_selected = []
    cols_selected = []
    df['frame'] = df.index+1
    frame_offset = 0
    if rows == '' or rows == 'all':
        firstrow, lastrow = df.index[0,], df.index[-1]
        rows_selected = list(range(firstrow,lastrow+1))
    else:
        rows_selected = getIndices(rows)
        frame_offset = rows_selected[0]
    if cols == '' or cols == 'all':
        cols_selected = [-1]+list(range(0,len(df.columns)-1))
    else:
        cols_selected = [-1]+getIndices(cols)
    df = df.iloc[rows_selected,cols_selected]
    df['frame'] = df['frame']-frame_offset
    if convert_cm_mm == 1:
        df.iloc[:,1:] = cmToMm(df.iloc[:,1:])
    return df


def wideToLong(df, muscle_name, notes):
    df = df.rename(index = str, columns = {df.columns.drop('frame')[0]:'X',df.columns.drop('frame')[1]:'Y',df.columns.drop('frame')[2]:'Z'})
    df = pd.melt(df, id_vars=[('frame')], value_vars=list(df.columns.drop('frame')), var_name='axis', value_name='moment_arm')
    df['muscle'] = muscle_name
    df['notes'] = notes
    df['moment_arm'] = df['moment_arm']
    df = df[['frame','muscle','notes','axis','moment_arm']]
    return df


def plotMomentArms(dfs, panels, figsize):
    assert abs(panels[0]*panels[1]-len(dfs)) <= 1, "# of panels must equal # of dataframes!"
    fig, axs = plt.subplots(nrows = panels[0], ncols = panels[1], sharex=True, sharey=True, figsize=(figsize[0],figsize[1]))
    # axs_list = [item for sublist in axs for item in sublist]
    axs_list = axs.ravel()
    fig.subplots_adjust(hspace=0.1, wspace=0.05)
    for df in dfs:
        # ax = axs_list.pop(0)
        ax = axs_list[0]
        axs_list = axs_list[1:]
        sns.lineplot(data=df, ax=ax, x='frame',y='moment_arm', hue='axis', palette = color_dict, style = 'notes').set_title(df['notes'][0])
    fig.show()


#48R4
SIMM_48_brevis = importData('csv/SIMM48LHS_tr4.csv', '', '2,4,6')
SIMM_48_brevis = wideToLong(SIMM_48_brevis, 'biceps', 'SIMM_brevis')
SIMM_48_longus = importData('csv/SIMM48LHS_tr4.csv', '', '3,5,7')
SIMM_48_longus = wideToLong(SIMM_48_longus, 'biceps', 'SIMM_longus')
maya_48_scaled = importData('csv/clean48LHSr4_biceps_includeunscaled.csv', '75-800', '1-3', convert_cm_mm=1)
maya_48_unscaled = importData('csv/clean48LHSr4_biceps_includeunscaled.csv', '75-800', '4-6', convert_cm_mm=1)
maya_48_scaled = wideToLong(maya_48_scaled, 'biceps', 'implanted_scaled')
maya_48_unscaled = wideToLong(maya_48_unscaled, 'biceps', 'implanted_unscaled')
echidna48run4figs = [SIMM_48_brevis,SIMM_48_longus,maya_48_scaled, maya_48_unscaled]
plotMomentArms(echidna48run4figs, panels = (2,2), figsize = (20,10))


#46r15
SIMM_46_pec1= importData('csv/SIMM46LHS_tr15.csv', '', '2,5,8')
SIMM_46_pec1 = wideToLong(SIMM_46_pec1, 'pectoralis', 'SIMM_pec1')
SIMM_46_pec2= importData('csv/SIMM46LHS_tr15.csv', '', '3,6,9')
SIMM_46_pec2 = wideToLong(SIMM_46_pec2, 'pectoralis', 'SIMM_pec2')
SIMM_46_pec3= importData('csv/SIMM46LHS_tr15.csv', '', '4,7,10')
SIMM_46_pec3 = wideToLong(SIMM_46_pec3, 'pectoralis', 'SIMM_pec3')
maya_46_caudal_scaled = importData('csv/clean46LHSr15_peccd_includeunscaled.csv', '3-800', '1-3', convert_cm_mm=1)
maya_46_caudal_unscaled = importData('csv/clean46LHSr15_peccd_includeunscaled.csv', '3-800', '4-6', convert_cm_mm=1)
maya_46_caudal_scaled = wideToLong(maya_46_caudal_scaled, 'pectoralis', 'implanted_pec_cd_scaled')
maya_46_caudal_unscaled = wideToLong(maya_46_caudal_unscaled, 'pectoralis', 'implanted_pec_cd_unscaled')
echidna46run15figs = [SIMM_46_pec1,SIMM_46_pec2,SIMM_46_pec3, maya_46_caudal_scaled, maya_46_caudal_unscaled]
plotMomentArms(echidna46run15figs, panels = (1,5), figsize = (26,10))
plotMomentArms([SIMM_46_pec3,maya_46_caudal_scaled], panels = (2,1), figsize = (20,10))


#44r9
SIMM_44_triceps= importData('csv/SIMM44LHS_tr9.csv', '', '2,3,4')
SIMM_44_triceps = wideToLong(SIMM_44_triceps, 'triceps', 'SIMM_tri')
maya_44_triceps_scaled = importData('csv/clean44LHSr9_triceps_includeunscaled.csv', '36-800', '1-3',convert_cm_mm=1)
maya_44_triceps_unscaled = importData('csv/clean44LHSr9_triceps_includeunscaled.csv', '36-800', '4-6',convert_cm_mm=1)
maya_44_triceps_scaled = wideToLong(maya_44_triceps_scaled, 'triceps', 'implanted_tri_scaled')
maya_44_triceps_unscaled = wideToLong(maya_44_triceps_unscaled, 'triceps', 'implanted_tri_unscaled')
echidna44run9figs = [SIMM_44_triceps,maya_44_triceps_scaled,maya_44_triceps_unscaled]
plotMomentArms(echidna44run9figs, panels = (3,1), figsize = (20,10))


maya_44_relJCS = importData('csv/transrot_clean44LHSr9.csv', '36-799', '')
maya_44_relJCS_noframe = maya_44_relJCS.loc[:, maya_44_relJCS.columns != 'frame']
corr_maya_44_relJCS = maya_44_relJCS_noframe.corr()
maya_44_relJCS_noframe.describe()
maya_44_relJCS_abdadd = maya_44_relJCS[(abs(maya_44_relJCS['mdata.ry'])<=5) & (abs(maya_44_relJCS['mdata.rz'])<=5)]
maya_44_relJCS_abdadd.shape
maya_44_relJCS_lar = maya_44_relJCS[(abs(maya_44_relJCS['mdata.rx'])<=5) & (abs(maya_44_relJCS['mdata.rz'])<=5)]
maya_44_relJCS_lar.shape
maya_44_relJCS_flexext = maya_44_relJCS[(abs(maya_44_relJCS['mdata.rx'])<=5) & (abs(maya_44_relJCS['mdata.ry'])<=5)]
maya_44_relJCS_flexext.shape



maya_46_relJCS = importData('csv/transrot_clean46LHSr15.csv', '3-800', '')
maya_46_relJCS_noframe = maya_46_relJCS.loc[:, maya_46_relJCS.columns != 'frame']
corr_maya_46_relJCS = maya_46_relJCS_noframe.corr()
maya_46_relJCS_abdadd = maya_46_relJCS[(abs(maya_46_relJCS['mdata.ry'])<=5) & (abs(maya_46_relJCS['mdata.rz'])<=5)]
maya_46_relJCS_abdadd.shape
maya_46_relJCS_lar = maya_46_relJCS[(abs(maya_46_relJCS['mdata.rx'])<=5) & (abs(maya_46_relJCS['mdata.rz'])<=5)]
maya_46_relJCS_lar.shape
maya_46_relJCS_flexext = maya_46_relJCS[(abs(maya_46_relJCS['mdata.rx'])<=5) & (abs(maya_46_relJCS['mdata.ry'])<=5)]
maya_46_relJCS_flexext.shape



maya_48_relJCS = importData('csv/transrot_clean48LHSr4.csv', '75-800', '')
maya_48_relJCS_noframe = maya_48_relJCS.loc[:, maya_48_relJCS.columns != 'frame']
corr_maya_48_relJCS = maya_48_relJCS_noframe.corr()
maya_48_relJCS_noframe.describe()
maya_48_relJCS_abdadd = maya_48_relJCS[(abs(maya_48_relJCS['mdata.ry'])<=10) & (abs(maya_48_relJCS['mdata.rz'])<=10)]
maya_48_relJCS_abdadd.shape
maya_48_relJCS_lar = maya_48_relJCS[(abs(maya_48_relJCS['mdata.rx'])<=10) & (abs(maya_48_relJCS['mdata.rz'])<=10)]
maya_48_relJCS_lar.shape
maya_48_relJCS_flexext = maya_48_relJCS[(abs(maya_48_relJCS['mdata.rx'])<=10) & (abs(maya_48_relJCS['mdata.ry'])<=10)]
maya_48_relJCS_flexext.shape



sns.pairplot(maya_44_relJCS_noframe, diag_kind="kde")
sns.pairplot(maya_46_relJCS_noframe,diag_kind="kde")
sns.pairplot(maya_48_relJCS_noframe,diag_kind="kde")

maya_444648_relJCS_pooled = pd.concat([maya_44_relJCS_noframe, maya_46_relJCS_noframe, maya_48_relJCS_noframe])
sns.pairplot(maya_444648_relJCS_pooled,diag_kind="kde")
maya_444648_relJCS_pooled.describe()
