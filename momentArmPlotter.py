import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
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

def importData(csv: str, rows: str, cols: str):
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
maya_48_distal = importData('csv/clean48LHSr4_jcs_ds.csv', '75-800', '')
maya_48_distal = wideToLong(maya_48_distal, 'biceps', 'implanted')
maya_48_distal.moment_arm = cmToMm(maya_48_distal.moment_arm)
echidna48run4figs = [SIMM_48_brevis,SIMM_48_longus,maya_48_distal]
plotMomentArms(echidna48run4figs, panels = (3,1), figsize = (20,10))


#46r15
SIMM_46_pec1= importData('csv/SIMM46LHS_tr15.csv', '', '2,5,8')
SIMM_46_pec1 = wideToLong(SIMM_46_pec1, 'pectoralis', 'SIMM_pec1')
SIMM_46_pec2= importData('csv/SIMM46LHS_tr15.csv', '', '3,6,9')
SIMM_46_pec2 = wideToLong(SIMM_46_pec2, 'pectoralis', 'SIMM_pec2')
SIMM_46_pec3= importData('csv/SIMM46LHS_tr15.csv', '', '4,7,10')
SIMM_46_pec3 = wideToLong(SIMM_46_pec3, 'pectoralis', 'SIMM_pec3')
maya_46_cranial = importData('csv/clean46LHSr15_pecO1.csv', '3-800', '')
maya_46_cranial = wideToLong(maya_46_cranial, 'pectoralis', 'implanted_pec_cr')
maya_46_cranial.moment_arm = cmToMm(maya_46_cranial.moment_arm)
maya_46_caudal = importData('csv/clean46LHSr15_pecO2.csv', '3-800', '')
maya_46_caudal = wideToLong(maya_46_caudal, 'pectoralis', 'implanted_pec_cd')
maya_46_caudal.moment_arm = cmToMm(maya_46_caudal.moment_arm)
echidna46run15figs = [SIMM_46_pec1,SIMM_46_pec2,SIMM_46_pec3, maya_46_cranial, maya_46_caudal]
plotMomentArms(echidna46run15figs, panels = (1,5), figsize = (26,10))
plotMomentArms([SIMM_46_pec3,maya_46_caudal], panels = (2,1), figsize = (20,10))


#44r9
SIMM_44_triceps= importData('csv/SIMM44LHS_tr9.csv', '', '2,3,4')
SIMM_44_triceps = wideToLong(SIMM_44_triceps, 'triceps', 'SIMM_tri')
maya_44_triceps = importData('csv/clean44LHSr9_triceps.csv', '36-800', '')
maya_44_triceps = wideToLong(maya_44_triceps, 'triceps', 'implanted_tri')
maya_44_triceps.moment_arm = cmToMm(maya_44_triceps.moment_arm)
echidna44run9figs = [SIMM_44_triceps,maya_44_triceps]
plotMomentArms(echidna44run9figs, panels = (2,1), figsize = (20,10))
