from pymel.all import *
import pymel.core.datatypes as dt

## does the thing, runs on UI button click
def doTheThing(muscle_name, radios):
    rotation_order = radioCollection(radios, q=1, sl=1)
    sel = getSelectionSet()
    ## setup: make muscle arrow for visualization
    mus_arrow = makeMuscle(sel['C'], sel['B'], 'mus_'+muscle_name)
    ## setup: make proxy
    proxy = makeUnitAxes(muscle_name+'_'+rotation_order, sel, united=0)
    ## evaluate every frame:
    updateFrame(sel, proxy, rotation_order)

## every frame: move proxy to new location, transform and key proxy axes using X, XY, and XYZ rotations from offset matrix
def updateProxy(sel, proxy, order):
    offset_mat = findOffsetMatrix(order, sel['A'], sel['A*'])
    ## reset warp:
    xform(proxy['x_a'],proxy['y_a'],proxy['z_a'], ro=[0,0,0], r=0, os=1)
    if order == 'XYZ':
        ## rotate XYZ by rX:
        xform(proxy['x_a'],proxy['y_a'],proxy['z_a'], ro=[offset_mat['rX'],0,0], r=1, os=1)
        ## rotate YZ by rY:
        xform(proxy['y_a'],proxy['z_a'], ro=[0,offset_mat['rY'],0], r=1, os=1)
        ## rotate Z by rZ:
        xform(proxy['z_a'], ro=[0,0,offset_mat['rZ']], r=1, os=1)
    else:    
        ## rotate ZYX by rZ:
        xform(proxy['z_a'],proxy['y_a'],proxy['x_a'], ro=[0,0,offset_mat['rZ']], r=1, os=1)
        ## rotate YX by rY:
        xform(proxy['y_a'],proxy['x_a'], ro=[0,offset_mat['rY'],0], r=1, os=1)
        ## rotate X by rX:
        xform(proxy['x_a'], ro=[offset_mat['rX'],0,0], r=1, os=1)
    ## key proxy transformations:
    for axis in ['x_a','y_a','z_a']:
        setKeyframe(proxy[axis], at='rotate')

## every frame: calculate XYZ unit vectors from transformed proxy axes
def updateUnitVecs(proxy):
    unit_vecs = {}
    for axis in ['x','y','z']:
        unit = makeVector(proxy['joint'],proxy[axis])
        unit_vecs[axis] = unit
    return unit_vecs

## every frame: get and key moment arms
def calculateMomentArms(sel, proxy, mus_vec, unit_vecs, frame):
    ## calculate moment arm as shortest distance between two skew lines
    unscaled_moment_arms = {}
    for axis in ['x','y','z']:
        axisdir = unit_vecs[axis]
        axispos = xform(proxy['joint'],q=1,t=1, ws=1)
        muscledir = mus_vec/(mus_vec.length())
        musclepos = xform(sel['B'],q=1,t=1, ws=1)
        cross_axismusc = cross(axisdir,muscledir)
        difference_pos = dt.Vector([(axispos[0]-musclepos[0]),(axispos[1]-musclepos[1]),(axispos[2]-musclepos[2])])
        numerator = dot(cross_axismusc,difference_pos)
        denominator = cross_axismusc.length()
        unscaled_moment_arms[axis] = numerator/denominator
    ## dot muscle vector with each plane to get scale factor
    muscle_projs = {}
    for axis in ['x','y','z']:
        axis_proj = dot(mus_vec,unit_vecs[axis])
        plane_proj = mus_vec - (axis_proj*unit_vecs[axis])
        scale_factor = plane_proj.length()/mus_vec.length()
        muscle_projs[axis+'_axis'] = axis_proj
        muscle_projs[axis+'_plane'] = plane_proj
        muscle_projs[axis+'_scale'] = scale_factor
    ## calculate true moment arm as product of moment arm and scale factor
    scaled_moment_arms = {}
    for axis in ['x','y','z']:
        scaled = unscaled_moment_arms[axis]*muscle_projs[axis+'_scale']
        scaled_moment_arms[axis] = scaled
    ## key Xma_scaled Yma_scaled Zma_scaled
    for axis in ['x','y','z']:
        setAttr(proxy['joint']+'.'+axis.upper()+'ma', scaled_moment_arms[axis])
        setKeyframe(proxy['joint'], at=axis.upper()+'ma')
    print('X moment arm for frame '+str(int(frame))+' is: '+str(scaled_moment_arms['x']))
    print('Y moment arm for frame '+str(int(frame))+' is: '+str(scaled_moment_arms['y']))
    print('Z moment arm for frame '+str(int(frame))+' is: '+str(scaled_moment_arms['z']))
    return scaled_moment_arms

## frame updater, inspiration from David Baier's outputRelMotion shelf tool
def updateFrame(sel, proxy, order):
    transformList = ls(tr=1)
    frame=findKeyframe(transformList, hi="both",which="first")
    lastframe=findKeyframe(transformList, hi="both",which="last")
    assert(frame-lastframe != 0), "No animation found!"
    previousframe = lastframe+1
    currentTime(frame, update=1, edit=1)
    progressWindow(title="Grabbing moment arms...", min=frame, max=lastframe, progress=frame, status = "Crunching frame: "+str(frame), isInterruptable=True)
    while frame<=lastframe:
        frame=findKeyframe(transformList, hi="both",which="next")
        if progressWindow(q=1, isCancelled=1):
            break
        if (frame-previousframe) == 0:
            break
        previousframe = frame
        mus_vec = makeVector(sel['C'],sel['B'])
        updateProxy(sel, proxy, order)
        unit_vecs = updateUnitVecs(proxy)
        calculateMomentArms(sel, proxy, mus_vec, unit_vecs, frame)
        progressWindow(edit=1, progress=frame, status=('Processing frame: '+str(frame)))
        currentTime(frame, update=1, edit=1)
    progressWindow(endProgress=1)

## calculates a worldspace offset matrix to capture transformations between distal and proximal joint coordinate systems
def findOffsetMatrix(order, obj1, obj2):
    inverse_order = order[::-1]
    obj2_mat = obj2.attr('worldMatrix').get()
    obj1_imat = obj1.attr('worldMatrix').get().inverse()
    offset_o1o2 = dt.Matrix(obj2_mat*obj1_imat)
    offset_o1o2_Euler = degrees(dt.EulerRotation.decompose(offset_o1o2,inverse_order))
    output = {  'mat':offset_o1o2,
                'rX':dt.round(offset_o1o2_Euler[0],8),
                'rY':dt.round(offset_o1o2_Euler[1],8),
                'rZ':dt.round(offset_o1o2_Euler[2],8),
                'tX':offset_o1o2[3][0],
                'tY':offset_o1o2[3][1],
                'tZ':offset_o1o2[3][2]}
    # print('new offset found, rX='+str(output['rX']))
    return output

## makes unit vector proxy for composite joint coordinate system
def makeUnitAxes(name, sel, united=0):
    def axesShaderMake():
        if not objExists('xShader'):
            shadingNode('blinn', asShader=True, name='xShader')
            sets(renderable=True, noSurfaceShader=True, empty=True, name='xShaderSG')
            connectAttr('xShader.outColor', 'xShaderSG.surfaceShader', force=True)
            setAttr('xShader.color', 1,0,0, type='double3')
        if not objExists('yShader'):
            shadingNode('blinn', asShader=True, name='yShader')
            sets(renderable=True, noSurfaceShader=True, empty=True, name='yShaderSG')
            connectAttr('yShader.outColor', 'yShaderSG.surfaceShader', force=True)
            setAttr('yShader.color', 0,1,0, type='double3')
        if not objExists('zShader'):
            shadingNode('blinn', asShader=True, name='zShader')
            sets(renderable=True, noSurfaceShader=True, empty=True, name='zShaderSG')
            connectAttr('zShader.outColor', 'zShaderSG.surfaceShader', force=True)
            setAttr('zShader.color', 0,0,1, type='double3')
    axesShaderMake()
    size=1
    cyl_length = size*0.8
    cone_length = size*0.2
    cyl_radius = size*0.03
    cone_radius = size*0.06
    originLoc = spaceLocator(n=name+'_origin')
    xLoc = spaceLocator(n=name+'_x')
    xform(xLoc, t=[1,0,0])
    yLoc = spaceLocator(n=name+'_y')
    xform(yLoc, t=[0,1,0])
    zLoc = spaceLocator(n=name+'_z')
    xform(zLoc, t=[0,0,1])
    xCyl = polyCylinder(r=cyl_radius,h=cyl_length, ax=[1,0,0], n=name+'_x_cyl')
    sets('xShaderSG', e=1, forceElement=xCyl[0])
    xform(xCyl[0], piv=[(-1*(cyl_length/2)),0,0], t=[(cyl_length/2),0,0])
    xCone = polyCone(r=cone_radius,h=cone_length, ax=[1,0,0], n=name+'_x_cone')
    sets('xShaderSG', e=1, forceElement=xCone[0])
    xform(xCone[0], piv=[(-1*(cone_length/2)),0,0], t=[(cone_length/2+cyl_length),0,0])
    yCyl = polyCylinder(r=cyl_radius,h=cyl_length, ax=[0,1,0], n=name+'_y_cyl')
    sets('yShaderSG', e=1, forceElement=yCyl[0])
    xform(yCyl[0], piv=[0,(-1*(cyl_length/2)),0], t=[0,(cyl_length/2),0])
    yCone = polyCone(r=cone_radius,h=cone_length, ax=[0,1,0], n=name+'_y_cone')
    sets('yShaderSG', e=1, forceElement=yCone[0])
    xform(yCone[0], piv=[0,(-1*(cone_length/2)),0], t=[0,(cone_length/2+cyl_length),0])
    zCyl = polyCylinder(r=cyl_radius,h=cyl_length, ax=[0,0,1], n=name+'_z_cyl')
    sets('zShaderSG', e=1, forceElement=zCyl[0])
    xform(zCyl[0], piv=[0,0,(-1*(cyl_length/2))], t=[0,0,(cyl_length/2)])
    zCone = polyCone(r=cone_radius,h=cone_length, ax=[0,0,1], n=name+'_z_cone')
    sets('zShaderSG', e=1, forceElement=zCone[0])
    xform(zCone[0], piv=[0,0,(-1*(cone_length/2))], t=[0,0,(cone_length/2+cyl_length)])
    if united==1:
        unitedaxes = polyUnite(xCyl,yCyl,zCyl,xCone,yCone,zCone, n=name+'_arrows', constructionHistory=0)
        output = group(empty=1, name=name+'_jnt')
        parent(unitedaxes[0],originLoc,xLoc,yLoc,zLoc,output)
        proxy = {   'joint':output,
                    'o':originLoc,
                    'x':xLoc,
                    'y':yLoc,
                    'z':zLoc}
    else:
        x_arrow = polyUnite(xCyl,xCone, n=name+'_x_a', constructionHistory=0)
        y_arrow = polyUnite(yCyl,yCone, n=name+'_y_a', constructionHistory=0)
        z_arrow = polyUnite(zCyl,zCone, n=name+'_z_a', constructionHistory=0)
        output = group(empty=1, name=name+'_jnt')
        parent(xLoc,x_arrow[0])
        parent(yLoc,y_arrow[0])
        parent(zLoc,z_arrow[0])
        parent(x_arrow,y_arrow,z_arrow,originLoc,output)
        proxy = {   'joint':output,
                    'o':originLoc,
                    'x':xLoc,
                    'y':yLoc,
                    'z':zLoc,
                    'x_a':x_arrow,
                    'y_a':y_arrow,
                    'z_a':z_arrow}
    addAttr(proxy['joint'], shortName='Xma', longName='XaxisMomentArm', at="float" , keyable=1)
    addAttr(proxy['joint'], shortName='Yma', longName='YaxisMomentArm', at="float" , keyable=1)
    addAttr(proxy['joint'], shortName='Zma', longName='ZaxisMomentArm', at="float" , keyable=1)
    pointConstraint(sel['A*'], proxy['joint'])
    orientConstraint(sel['A'], proxy['joint'])
    return proxy

## return currently selected objects as dictionary
    ## order: 1. proximal joint center, 2. distal joint center, 3. proximal muscle marker, 4. distal muscle marker
def getSelectionSet():
    sel = ls(sl=1)
    assert(len(sel)==4),"Please select (in order): 1. proximal joint center, 2. distal joint center, 3. proximal muscle marker, 4. distal muscle marker"
    dict = {}
    dict['A'], dict['A*'], dict['B'], dict['C'] = sel[0], sel[1], sel[2], sel[3]
    return dict

## make a vector from two locators
def makeVector(locA,locB):
    posA = dt.Vector(xform(locA,q=1,t=1,ws=1))
    posB = dt.Vector(xform(locB,q=1,t=1,ws=1))
    vectAB = posB - posA
    return vectAB


## make muscle cylinder arrow for visualization
def makeMuscle(sp, ep, obj_name, radius = 0.05, tint='red', alpha=0.75):
    ## sp: start point as dt.Vector,
    ## ep: end point as dt.Vector,
    ## obj_name: name of vector object,
    ## set up variables
    size = makeVector(sp,ep).length()
    obj_length_name = "length_"+obj_name
    obj_scale_name = "sf_"+obj_name
    obj_mult_name = "multiply_"+obj_name
    cyl_height = size
    cyl_radius = radius
    ## make cylinder
    muscle = polyCylinder(r=cyl_radius,h=cyl_height, ax=[1,0,0], n='cyl_'+obj_name)[0]
    assignColor(muscle, tint, alpha)
    xform(muscle, piv=[(cyl_height*-1)/2,0,0])
    ## constrain muscle to follow ep
    pointConstraint(sp,muscle)
    aimConstraint(ep,muscle,aimVector=[1,0,0],worldUpType="vector")
    hide(listRelatives(muscle)[-2:])
    ## scale muscle by distance between sp and ep,
    ## parentMatrix distanceBetween node always gets worldspace distances, regardless of parenting
    createNode("distanceBetween", n=obj_length_name)
    connectAttr(sp+".parentMatrix", obj_length_name+'.inMatrix1',f=1)
    connectAttr(ep+".parentMatrix", obj_length_name+'.inMatrix2',f=1)
    for axis in ['X', 'Y', 'Z']:
        connectAttr(sp+".translate"+axis, obj_length_name+'.point1'+axis,f=1)
        connectAttr(ep+".translate"+axis, obj_length_name+'.point2'+axis,f=1)
    createNode("multiplyDivide", n=obj_scale_name)
    setAttr(obj_scale_name+".operation", 2)
    connectAttr(obj_length_name+".distance", obj_scale_name+".input1X", f=1)
    setAttr(obj_scale_name+".input2X",size)
    connectAttr(obj_scale_name+".outputX",muscle+".scaleX",f=1)
    return muscle

## make vector arrow for visualization
def makeArrow(sp, ep, obj_name, tint='yellow', alpha=0.75):
    ## sp: start point as dt.Vector,
    ## ep: end point as dt.Vector,
    ## obj_name: name of vector object,
    ## set up variables
    size = makeVector(sp,ep).length()
    obj_length_name = "length_"+obj_name
    obj_scale_name = "sf_"+obj_name
    obj_mult_name = "multiply_"+obj_name
    cyl_height = size*0.8
    cone_height = size*0.2
    cyl_radius = size*0.02
    cone_radius = size*0.04
    ## make primitives and combine into arrow
    cyl_primitive = polyCylinder(r=cyl_radius,h=cyl_height, ax=[1,0,0], n='cyl_'+obj_name)
    cone_primitive = polyCone(r=cone_radius,h=cone_height, ax=[1,0,0], n='cone_'+obj_name)
    xform(cone_primitive, piv=[cone_height/2,0,0], t=[(cyl_height+cone_height)/2,0,0] )
    arrow = polyUnite(cyl_primitive, cone_primitive, n=obj_name, ch=0)
    assignColor(arrow, tint, alpha)
    xform(arrow, piv=[(cyl_height*-1)/2,0,0])
    ## constrain arrow to follow ep
    pointConstraint(sp,arrow)
    aimConstraint(ep,arrow,aimVector=[1,0,0],worldUpType="vector")
    hide(listRelatives(arrow)[-2:])
    ## scale arrow by distance between sp and ep,
    ## parentMatrix distanceBetween node always gets worldspace distances, regardless of parenting
    createNode("distanceBetween", n=obj_length_name)
    connectAttr(sp+".parentMatrix", obj_length_name+'.inMatrix1',f=1)
    connectAttr(ep+".parentMatrix", obj_length_name+'.inMatrix2',f=1)
    for axis in ['X', 'Y', 'Z']:
        connectAttr(sp+".translate"+axis, obj_length_name+'.point1'+axis,f=1)
        connectAttr(ep+".translate"+axis, obj_length_name+'.point2'+axis,f=1)
    createNode("multiplyDivide", n=obj_scale_name)
    setAttr(obj_scale_name+".operation", 2)
    connectAttr(obj_length_name+".distance", obj_scale_name+".input1X", f=1)
    setAttr(obj_scale_name+".input2X",size)
    connectAttr(obj_scale_name+".outputX",arrow[0]+".scaleX",f=1)
    ## store vector magnitude as "Vector Length" attribute
    return arrow

## check to see if muscle material exists. if yes, do nothing. if no, make one.
def assignColor(target, tint, alpha):
    if objExists(target):
        mat_name = tint+'_mat'
        red_rgb = [0.800, 0.000, 0.000]
        yellow_rgb = [0.800, 0.500, 0.000]
        green_rgb = [0.000, 0.800, 0.000]
        blue_rgb = [0.000, 0.000, 0.800]
        if not objExists(mat_name):
            material = shadingNode("lambert", asShader=True, name=mat_name)
            material_SG = sets(renderable=True, noSurfaceShader=True, empty=True, name=mat_name+'SG' )
            if tint == 'red':
                material.color.set(red_rgb)
            elif tint == 'yellow':
                material.color.set(yellow_rgb)
            elif tint == 'green':
                material.color.set(green_rgb)
            elif tint == 'blue':
                material.color.set(blue_rgb)
            material.transparency.set(alpha, alpha, alpha)
            material.outColor >> material_SG.surfaceShader
        else:
            print("Muscle material already exists!")
            material = mat_name
            material_SG = mat_name+'SG'
        sets(material_SG, edit=True, forceElement=target)


def zeroMissingData(*args):
    sel = ls(sl=1)
    targets = sel[:-1]
    reference = sel[-1]
    TxKeys = set(keyframe(reference.translateX, query=True))
    TyKeys = set(keyframe(reference.translateY, query=True))
    TzKeys = set(keyframe(reference.translateZ, query=True))
    RxKeys = set(keyframe(reference.rotateX, query=True))
    RyKeys = set(keyframe(reference.rotateY, query=True))
    RzKeys = set(keyframe(reference.rotateZ, query=True))
    keyedFrames = set.intersection(TxKeys, TyKeys, TzKeys, RxKeys, RyKeys, RzKeys)
    playbackRange = { float(time) for time in range(int(playbackOptions(q=1, minTime = True)), int(playbackOptions(q=1, maxTime = True)+1))}
    missingFrames = sorted(playbackRange - keyedFrames)
    for frame in missingFrames:
        currentTime(frame, update=1, edit=1)
        for target in targets:
            if target.hasAttr('XaxisMomentArm'):
                for maType in ['XaxisMomentArm','YaxisMomentArm','ZaxisMomentArm']:
                    attrNameM = target+'.'+maType
                    setAttr(attrNameM,0)
                    setKeyframe(attrNameM)
            elif 'data' in target[-4:]:
                for transformationType in ['translateX','translateY','translateZ','rotateX','rotateY','rotateZ']:
                    attrNameT = target+'.'+transformationType
                    setAttr(attrNameT,0)
                    setKeyframe(attrNameT)
            else:
                pass

def hideByMissing(*args):
    sel = ls(sl=1)
    targets = sel[:-1]
    reference = sel[-1]
    TxKeys = set(keyframe(reference.translateX, query=True))
    TyKeys = set(keyframe(reference.translateY, query=True))
    TzKeys = set(keyframe(reference.translateZ, query=True))
    RxKeys = set(keyframe(reference.rotateX, query=True))
    RyKeys = set(keyframe(reference.rotateY, query=True))
    RzKeys = set(keyframe(reference.rotateZ, query=True))
    keyedFrames = set.intersection(TxKeys, TyKeys, TzKeys, RxKeys, RyKeys, RzKeys)
    playbackRange = { float(time) for time in range(int(playbackOptions(q=1, minTime = True)), int(playbackOptions(q=1, maxTime = True)+1))}
    missingFrames = sorted(playbackRange - keyedFrames)
    for frame in playbackRange:
        currentTime(frame, update=1, edit=1)
        if frame in missingFrames:
            for selectedObject in sel:
                vizName = selectedObject+'.visibility'
                setAttr(vizName,False)
                setKeyframe(vizName)
        else:
            for selectedObject in sel:
                vizName = selectedObject+'.visibility'
                setAttr(vizName,True)
                setKeyframe(vizName)

def hideByHidden(*args):
    sel = ls(sl=1)
    targets = sel[:-1]
    reference = sel[-1]
    visibilityFrames = sorted(keyframe(reference.visibility, query=True))
    for frame in visibilityFrames:
        currentTime(frame, update=1, edit=1)
        referenceValue = reference.visibility.get()
        for target in targets:
            vizName = target+'.visibility'
            setAttr(vizName,referenceValue)
            setKeyframe(vizName)

## prompt for zeroMissingData()
def zeroMissingDataUI():
    if window('zero_window',exists=1) == True:
        deleteUI('zero_window')
    mainWindow = window('zero_window', title='Zero Missing Data',rtf=1, w=160, h=160)
    frameLayout(label='Zero Missing')
    columnLayout(columnOffset=('both',20))
    text(label='')
    text(label='Select (in order):')
    text(label=' 1) Target Object(s)')
    text(label=' 2) Reference Object (this has to come last!)')
    text(label='')
    text(label='')
    text(label="Looks for frames with missing translation/rotation data on the reference object.")
    text(label="Zeroes out translation/rotation and moment arm attributes on each target object.")
    text(label='')
    text(label='')
    button(label='Zero Missing',command=zeroMissingData)
    text(label='')
    showWindow(mainWindow)

## prompt for hideByMissing()
def hideByMissingUI():
    if window('hidemissing_window',exists=1) == True:
        deleteUI('hidemissing_window')
    mainWindow = window('hidemissing_window', title='Hide Missing',rtf=1, w=300, h=160)
    frameLayout(label='Hide Objects by Missing')
    columnLayout(columnOffset=('both',20))
    text(label='')
    text(label='Select (in order):')
    text(label=' 1) Target Object(s)')
    text(label=' 2) Reference Object (this has to come last!)')
    text(label='')
    text(label='')
    text(label="Looks for frames with missing translation/rotation data on the reference object.")
    text(label="Hides all selected objects at every frame where data is missing for the reference object.")
    text(label="The reference object will also be hidden.")
    text(label='')
    text(label='')
    button(label='Hide Missing',command=hideByMissing)
    text(label='')
    showWindow(mainWindow)


## prompt for hideByHidden()
def hideByHiddenUI():
    if window('hidehidden_window',exists=1) == True:
        deleteUI('hidehidden_window')
    mainWindow = window('hidehidden_window', title='Match Visibility',rtf=1, w=300, h=160)
    frameLayout(label='Hide Objects by Reference')
    columnLayout(columnOffset=('both',20))
    text(label='')
    text(label='Select (in order):')
    text(label=' 1) Target Object(s)')
    text(label=' 2) Reference Object (this has to come last!)')
    text(label='')
    text(label='')
    text(label='This will set keys matching the visibility attribute of each target object to the reference object')
    text(label='')
    text(label='')
    button(label='Match Visibility',command=hideByHidden)
    text(label='')
    showWindow(mainWindow)


## get muscle name from user input
def getMuscleNameUI():
    if window('muscle_window',exists=1) == True:
        deleteUI('muscle_window')
    mainWindow = window('muscle_window', title='Create Muscle',rtf=1, w=300, h=160)
    frameLayout(label='Muscle Name Input')
    columnLayout(columnOffset=('both',20))
    text(label='')
    text(label='Select (in order):')
    text(label=' 1) Proximal Joint Coordinate System (JCS)')
    text(label=' 2) Distal Joint Coordinate System (JCS)')
    text(label=' 3) Proximal Muscle Marker')
    text(label=' 4) Distal Muscle Marker')
    text(label='')
    text(label='')
    text(label='Muscle Name')
    textField('muscle_name_field', text='MuscleName')
    text(label='')
    text(label='')
    radios = radioCollection('rotationOrderRadios')
    radioButton('ZYX', label='ZYX intrinsic (XROMM default)', sl=True)
    radioButton('XYZ', label='XYZ intrinsic')
    text(label='')
    text(label='')
    muscle_name = textField('muscle_name_field', q=1,text=1)
    button(label='Create Muscle', command=lambda *args: doTheThing(muscle_name, radios))
    text(label='')
    showWindow(mainWindow)
