from pymel.all import *
import pymel.core.datatypes as dt
import maya.mel as mel
from functools import partial
from collections import OrderedDict
from maya import OpenMaya as om
import csv
import re

## does the thing, runs on UI button click
def doTheThing(muscle_name, radios):
    name = textField(muscle_name, q=1, tx=1)
    rotation_order = radioCollection(radios, q=1, sl=1)
    sel = getSelectionSet()
    ## setup: make muscle arrow for visualization
    mus_arrow = makeMuscle(sel['C'], sel['B'], 'mus_'+name)
    ## setup: make proxy
    proxy = makeUnitAxes(name+'_'+rotation_order, sel, united=0)
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
def calculateMomentArms(sel, proxy, mus_vec, unit_vecs, frame, get_moments):
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
        if get_moments:
            moment = scaled_moment_arms[axis] * mus_vec.length()
            setAttr(proxy['joint']+'.tq'+axis.upper(), moment)
            setKeyframe(proxy['joint'], at='tq'+axis.upper())
    print('X moment arm for frame '+str(int(frame))+' is: '+str(scaled_moment_arms['x']))
    print('Y moment arm for frame '+str(int(frame))+' is: '+str(scaled_moment_arms['y']))
    print('Z moment arm for frame '+str(int(frame))+' is: '+str(scaled_moment_arms['z']))
    return scaled_moment_arms


########### pose scene limb wrt reference limb, align reference limb to scene limb, key
def getScapularSegmentTransforms(ventral_cranial, ventral_caudal, dorsal, scene_scapula, ref_sternum, ref_sternum_jcs, ref_acromion_jcs, leftSide, tegu):
    #example:
    ##ventral_cranial = 'Body_vn1_crn'
    ##ventral_caudal = 'body:vn3dummy'
    ##dorsal= 'Body_ds1_crn'
    ##scene_scapula = 'Scapula_9:Mesh'
    ##ref_sternum = 'SEP87forOpenSim:sternum'
    ##ref_sternum_jcs = 'SEP87forOpenSim:jcs_origin'
    ##ref_acromion_jcs = 'SEP87forOpenSim:jcs_acromion_right'
    ##leftSide = False
    ##tegu=False
    ##currentTime(getPlaybackRange()[1]+1, update=1, edit=1)
    ##getScapularSegmentTransforms(ventral_cranial, ventral_caudal, dorsal, scene_scapula, ref_sternum, ref_sternum_jcs, ref_acromion_jcs, leftSide, tegu)

    ref_sternum = duplicate(ref_sternum, rr=True)
    timeRange = getPlaybackRange()
    bodyplane = mel.eval('axesCreate bodyplane 5 Hemi "Single Object"')
    sternum_acromion = mel.eval('axesCreate stern_scap 5 Hemi "Single Object"')
    acromion = mel.eval('axesCreate acs_scapula 5 Hemi "Single Object"')
    for obj in [bodyplane, sternum_acromion, acromion]:
        xform(obj, roo='zyx')
    makeBodyCS(bodyplane, ventral_cranial, ventral_caudal, dorsal, leftSide=leftSide)
    bakeResults(bodyplane, simulation=True,  time=str(timeRange[0])+":"+str(timeRange[1]), sampleBy=1, oversamplingRate = 1 , disableImplicitControl =True, preserveOutsideKeys = True,  sparseAnimCurveBake = False, removeBakedAttributeFromLayer = False, removeBakedAnimFromLayer = False , bakeOnOverrideLayer =  False , minimizeRotation = True , controlPoints = False,  shape = True)
    disconnectAttr('bodyCS_decompose.outputRotate', bodyplane+'.rotate')
    disconnectAttr(ventral_cranial+'.translate', bodyplane+'.translate')
    matchTransform( bodyplane, ref_sternum_jcs, piv=True, pos=True, rot=False, scl=False)
    matchTransform( acromion, ref_acromion_jcs, piv=True, pos=True, rot=False, scl=False)
    if leftSide:
        rotate(bodyplane, [180, 0,-90])
        rotate(acromion, [180, 0,-90])
    else:
        if tegu:
            rotate(bodyplane, [0, 0,-90])
            rotate(acromion, [0, 0,-90])
        else:
            rotate(bodyplane, [0, 0,90])
            rotate(acromion, [0, 0,90])
    matchTransform( sternum_acromion, bodyplane, piv=True, pos=True, rot=True, scl=False)
    if tegu:
        move(bodyplane, 0.15,1.0,-0.3, relative=True)
    setKeyframe(bodyplane)
    parent(acromion, scene_scapula)
    parent(ref_sternum, bodyplane)
    scriptedJCS([{'axes': sternum_acromion,  'dist': acromion,  'prox': bodyplane}], dummyFrame=timeRange[1]+1)


def makeBodyCS(axes_obj, ventral_cran, ventral_caud, dorsal, leftSide=False):
    # assumes ZYX rotation order on axes_obj
    if leftSide:
        caudcran_vec = makeVectorAB(ventral_cran,ventral_caud, nameA='vnA', nameB='vnB', normalize=True)
        dorsoventral_vec = makeVectorAB(ventral_cran,dorsal, nameA='vnA', nameB='ds', normalize=True)
    else:
        caudcran_vec = makeVectorAB(ventral_caud,ventral_cran, nameA='vnB', nameB='vnA', normalize=True)
        dorsoventral_vec = makeVectorAB(dorsal,ventral_cran, nameA='ds', nameB='vnA', normalize=True)
    lateral_vec = makeCrossProduct(caudcran_vec, dorsoventral_vec, name='bodyY', normalize=True)
    real_dorsoventral_vec = makeCrossProduct(caudcran_vec, lateral_vec, name='bodyZ', normalize=True)
    rotation_mat = makeRotationMatrixFromVecs(caudcran_vec, lateral_vec, real_dorsoventral_vec, name='bodyCS', decompose=True)
    connectAttr(ventral_cran+'.translate', axes_obj+'.translate', force=True)
    connectAttr(rotation_mat+'.outputRotate', axes_obj+'.rotate', force=True)
    setAttr(rotation_mat+'.inputRotateOrder',0)

def makeCrossProduct(vectorNodeA, vectorNodeB, name=None, normalize=False):
    name = name if name else "xvec_"+vectorNodeA+'_X_'+vectorNodeB
    product = createNode("vectorProduct", name=name)
    setAttr(product+'.operation', 2)
    if normalize:
        setAttr(product+'.normalizeOutput', 1)
    for i, j in zip([vectorNodeA, vectorNodeB],[1,2]):
        outputIs3D = attributeQuery( 'output3D', node=i, exists=True )
        if outputIs3D:
            connectAttr(i+'.output3D',  product+'.input'+str(j), force=True)
        else:
            connectAttr(i+'.output',  product+'.input'+str(j), force=True)
    return product

def makeRotationMatrixFromVecs(vectorNodeX, vectorNodeY, vectorNodeZ, name=None, decompose=False):
    name = name if name else "mvec_"+vectorNodeX+'_'+vectorNodeY+'_'+vectorNodeZ
    rot_mat = createNode("fourByFourMatrix", name=name)
    connectAttr(vectorNodeX+'.output.outputX',  rot_mat+'.in00', force=True)
    connectAttr(vectorNodeX+'.output.outputY',  rot_mat+'.in01', force=True)
    connectAttr(vectorNodeX+'.output.outputZ',  rot_mat+'.in02', force=True)
    connectAttr(vectorNodeY+'.output.outputX',  rot_mat+'.in10', force=True)
    connectAttr(vectorNodeY+'.output.outputY',  rot_mat+'.in11', force=True)
    connectAttr(vectorNodeY+'.output.outputZ',  rot_mat+'.in12', force=True)
    connectAttr(vectorNodeZ+'.output.outputX',  rot_mat+'.in20', force=True)
    connectAttr(vectorNodeZ+'.output.outputY',  rot_mat+'.in21', force=True)
    connectAttr(vectorNodeZ+'.output.outputZ',  rot_mat+'.in22', force=True)
    if decompose:
        decompose_mat = createNode("decomposeMatrix", name=name+'_decompose')
        connectAttr(rot_mat+'.output',  decompose_mat+'.inputMatrix', force=True)
    return decompose_mat if decompose else rot_mat


## frame updater, inspiration from David Baier's outputRelMotion shelf tool
def updateFrame(sel, proxy, order, frame_range=None, get_moments = False):
    transformList = ls(tr=1)
    frame = frame_range[0] if frame_range else findKeyframe(transformList, hi="both",which="first")
    lastframe= frame_range[1] if frame_range else findKeyframe(transformList, hi="both",which="last")
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
        calculateMomentArms(sel, proxy, mus_vec, unit_vecs, frame, get_moments)
        progressWindow(edit=1, progress=frame, status=('Processing frame: '+str(frame)))
        currentTime(frame, update=1, edit=1)
    progressWindow(endProgress=1)

## calculates a worldspace offset matrix to capture transformations between distal and proximal joint coordinate systems
def findOffsetMatrix(order, obj1, obj2):
    inverse_order = order[::-1]
    obj2_mat = getAttr(obj2+'.worldMatrix')
    obj1_imat = getAttr(obj1+'.worldInverseMatrix')
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
def makeUnitAxes(name, sel, united=0, include_moments=False):
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
    if include_moments:
        addAttr(proxy['joint'], shortName='tqX', longName='XaxisTorque', at="float" , keyable=1)
        addAttr(proxy['joint'], shortName='tqY', longName='YaxisTorque', at="float" , keyable=1)
        addAttr(proxy['joint'], shortName='tqZ', longName='ZaxisTorque', at="float" , keyable=1)
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
def makeArrow(sp, ep, obj_name, size, tint='yellow', alpha=0.75):
    ## sp: start point as dt.Vector,
    ## ep: end point as dt.Vector,
    ## obj_name: name of vector object,
    ## set up variables
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
    return arrow


## make 3D arrows for visualization
def makeForceArrows(sp, ep_dict, scale_dict, obj_name, alpha=0.75):
    obj_scale_nameX = "sfX_"+obj_name
    obj_scale_nameY = "sfY_"+obj_name
    obj_scale_nameZ = "sfZ_"+obj_name
    obj_scale_nameXYZ = "sfXYZ_"+obj_name
    obj_sum_name = "sum_"+obj_name
    obj_square_name = "sq_"+obj_name
    obj_sqrt_name = "sqrt_"+obj_name
    obj_pythsq_name = "pythsq_"+obj_name
    obj_pyth_name = "pyth_"+obj_name


    cyl_height_factor = 0.8
    cyl_rad_factor = 0.02
    cone_height_factor = 0.2
    cone_rad_factor = 0.04

    cyl_heights = {'XYZ':scale_dict['XYZ']*cyl_height_factor, 'X':scale_dict['X']*cyl_height_factor,'Y':scale_dict['Y']*cyl_height_factor,'Z':scale_dict['Z']*cyl_height_factor}
    cyl_rads = {'XYZ':scale_dict['XYZ']*cyl_rad_factor, 'X':scale_dict['X']*cyl_rad_factor,'Y':scale_dict['Y']*cyl_rad_factor,'Z':scale_dict['Z']*cyl_rad_factor}
    cone_heights = {'XYZ':scale_dict['XYZ']*cone_height_factor, 'X':scale_dict['X']*cone_height_factor,'Y':scale_dict['Y']*cone_height_factor,'Z':scale_dict['Z']*cone_height_factor}
    cone_rads = {'XYZ':scale_dict['XYZ']*cone_rad_factor, 'X':scale_dict['X']*cone_rad_factor,'Y':scale_dict['Y']*cone_rad_factor,'Z':scale_dict['Z']*cone_rad_factor}

    cyl_primitiveXYZ = polyCylinder(r=cyl_rads['XYZ'],h=cyl_heights['XYZ'], ax=[1,0,0], n='cyl_XYZ')
    cyl_primitiveX = polyCylinder(r=cyl_rads['X'],h=cyl_heights['X'], ax=[1,0,0], n='cyl_X')
    cyl_primitiveY = polyCylinder(r=cyl_rads['Y'],h=cyl_heights['Y'], ax=[0,1,0], n='cyl_Y')
    cyl_primitiveZ = polyCylinder(r=cyl_rads['Z'],h=cyl_heights['Z'], ax=[0,0,1], n='cyl_Z')

    cone_primitiveXYZ = polyCone(r=cone_rads['XYZ'],h=cone_heights['XYZ'], ax=[1,0,0], n='cone_XYZ')
    cone_primitiveX = polyCone(r=cone_rads['X'],h=cone_heights['X'], ax=[1,0,0], n='cone_X')
    cone_primitiveY = polyCone(r=cone_rads['Y'],h=cone_heights['Y'], ax=[0,1,0], n='cone_Y')
    cone_primitiveZ = polyCone(r=cone_rads['Z'],h=cone_heights['Z'], ax=[0,0,1], n='cone_Z')

    xform(cone_primitiveXYZ, piv=[cone_heights['XYZ']/2,0,0], t=[(cyl_heights['XYZ']+cone_heights['XYZ'])/2,0,0] )
    xform(cone_primitiveX, piv=[cone_heights['X']/2,0,0], t=[(cyl_heights['X']+cone_heights['X'])/2,0,0] )
    xform(cone_primitiveY, piv=[0,cone_heights['Y']/2,0], t=[0,(cyl_heights['Y']+cone_heights['Y'])/2,0] )
    xform(cone_primitiveZ, piv=[0,0,cone_heights['Z']/2], t=[0,0,(cyl_heights['Z']+cone_heights['Z'])/2] )

    arrowXYZ = polyUnite(cyl_primitiveXYZ, cone_primitiveXYZ, n=obj_name+'_XYZ', ch=0)
    arrowX = polyUnite(cyl_primitiveX, cone_primitiveX, n=obj_name+'_X', ch=0)
    arrowY = polyUnite(cyl_primitiveY, cone_primitiveY, n=obj_name+'_Y', ch=0)
    arrowZ = polyUnite(cyl_primitiveZ, cone_primitiveZ, n=obj_name+'_Z', ch=0)
    xform(arrowXYZ, t=[cyl_heights['XYZ']/2,0,0])
    xform(arrowX, t=[cyl_heights['X']/2,0,0])
    xform(arrowY, t=[0,cyl_heights['Y']/2,0])
    xform(arrowZ, t=[0,0,cyl_heights['Z']/2])
    xform(arrowXYZ, piv=[-cyl_heights['XYZ']/2,0,0])
    xform(arrowX, piv=[-cyl_heights['X']/2,0,0])
    xform(arrowY, piv=[0,-cyl_heights['Y']/2,0])
    xform(arrowZ, piv=[0,0,-cyl_heights['Z']/2])
    XYZ_mat = assignColor(arrowXYZ, 'dynamic', 0.0)
    assignColor(arrowX, 'red', alpha)
    assignColor(arrowY, 'green', alpha)
    assignColor(arrowZ, 'blue', alpha)
    dynLoc = spaceLocator(n='GRF_dynamicRGB')
    arrowGroup = group(arrowXYZ, arrowX, arrowY, arrowZ, dynLoc, n='GRF_grp')
    xform(arrowGroup, piv=[0,0,0])
    #aim xyz arrow
    aimConstraint(ep_dict['XYZ'],arrowXYZ,aimVector=[1,0,0],worldUpType="vector")
    #scale other arrows
    createNode("multiplyDivide", n=obj_scale_nameX)
    createNode("multiplyDivide", n=obj_scale_nameY)
    createNode("multiplyDivide", n=obj_scale_nameZ)
    setAttr(obj_scale_nameX+".operation", 2)
    setAttr(obj_scale_nameY+".operation", 2)
    setAttr(obj_scale_nameZ+".operation", 2)
    setAttr(obj_scale_nameX+".input2X",scale_dict['X'])
    setAttr(obj_scale_nameY+".input2Y",scale_dict['Y'])
    setAttr(obj_scale_nameZ+".input2X",scale_dict['Z'])
    setAttr(obj_scale_nameZ+".input2Y",scale_dict['Z'])
    setAttr(obj_scale_nameZ+".input2Z",scale_dict['Z'])
    connectAttr(ep_dict['X']+".translateX",obj_scale_nameX+".input1X",f=1)
    connectAttr(ep_dict['Y']+".translateY",obj_scale_nameY+".input1Y",f=1)
    connectAttr(ep_dict['X']+".translateX",obj_scale_nameZ+".input1X",f=1)
    connectAttr(ep_dict['Y']+".translateY",obj_scale_nameZ+".input1Y",f=1)
    connectAttr(ep_dict['Z']+".translateZ",obj_scale_nameZ+".input1Z",f=1)
    connectAttr(obj_scale_nameX+".outputX",arrowX[0]+".scaleX",f=1)
    connectAttr(obj_scale_nameX+".outputX",arrowX[0]+".scaleY",f=1)
    connectAttr(obj_scale_nameX+".outputX",arrowX[0]+".scaleZ",f=1)
    connectAttr(obj_scale_nameY+".outputY",arrowY[0]+".scaleX",f=1)
    connectAttr(obj_scale_nameY+".outputY",arrowY[0]+".scaleY",f=1)
    connectAttr(obj_scale_nameY+".outputY",arrowY[0]+".scaleZ",f=1)
    connectAttr(obj_scale_nameZ+".outputZ",arrowZ[0]+".scaleX",f=1)
    connectAttr(obj_scale_nameZ+".outputZ",arrowZ[0]+".scaleY",f=1)
    connectAttr(obj_scale_nameZ+".outputZ",arrowZ[0]+".scaleZ",f=1)
    #scale XYZ arrow
    createNode("multiplyDivide", n=obj_pythsq_name)
    setAttr(obj_pythsq_name+".operation", 3)
    connectAttr(ep_dict['XYZ']+".translateX",obj_pythsq_name+'.input1X',f=1)
    connectAttr(ep_dict['XYZ']+".translateY",obj_pythsq_name+'.input1Y',f=1)
    connectAttr(ep_dict['XYZ']+".translateZ",obj_pythsq_name+'.input1Z',f=1)
    setAttr(obj_pythsq_name+".input2X",2)
    setAttr(obj_pythsq_name+".input2Y",2)
    setAttr(obj_pythsq_name+".input2Z",2)
    createNode("plusMinusAverage", n=obj_sum_name)
    connectAttr(obj_pythsq_name+".outputX",obj_sum_name+'.input1D[0]',f=1)
    connectAttr(obj_pythsq_name+".outputY",obj_sum_name+'.input1D[1]',f=1)
    connectAttr(obj_pythsq_name+".outputZ",obj_sum_name+'.input1D[2]',f=1)
    createNode("multiplyDivide", n=obj_pyth_name)
    setAttr(obj_pyth_name+".operation", 3)
    connectAttr(obj_sum_name+".output1D",obj_pyth_name+'.input1X',f=1)
    setAttr(obj_pyth_name+".input2X",0.5)
    createNode("multiplyDivide", n=obj_scale_nameXYZ)
    setAttr(obj_scale_nameXYZ+".operation", 2)
    connectAttr(obj_pyth_name+".outputX", obj_scale_nameXYZ+".input1X", f=1)
    setAttr(obj_scale_nameXYZ+".input2X",scale_dict['XYZ'])
    connectAttr(obj_scale_nameXYZ+".outputX",arrowXYZ[0]+".scaleX",f=1)
    connectAttr(obj_scale_nameXYZ+".outputX",arrowXYZ[0]+".scaleY",f=1)
    connectAttr(obj_scale_nameXYZ+".outputX",arrowXYZ[0]+".scaleZ",f=1)
    #set dynamic color for XYZ arrow
    createNode("multiplyDivide", n=obj_square_name)
    setAttr(obj_square_name+".operation", 3)
    connectAttr(obj_scale_nameZ+".outputX",obj_square_name+'.input1X',f=1)
    connectAttr(obj_scale_nameZ+".outputY",obj_square_name+'.input1Y',f=1)
    connectAttr(obj_scale_nameZ+".outputZ",obj_square_name+'.input1Z',f=1)
    setAttr(obj_square_name+".input2X",2)
    setAttr(obj_square_name+".input2Y",2)
    setAttr(obj_square_name+".input2Z",2)
    createNode("multiplyDivide", n=obj_sqrt_name)
    setAttr(obj_sqrt_name+".operation", 3)
    connectAttr(obj_square_name+".outputX",obj_sqrt_name+'.input1X',f=1)
    connectAttr(obj_square_name+".outputY",obj_sqrt_name+'.input1Y',f=1)
    connectAttr(obj_square_name+".outputZ",obj_sqrt_name+'.input1Z',f=1)
    setAttr(obj_sqrt_name+".input2X",0.5)
    setAttr(obj_sqrt_name+".input2Y",0.5)
    setAttr(obj_sqrt_name+".input2Z",0.5)
    connectAttr(obj_sqrt_name+".outputX",dynLoc+'.translateX',f=1)
    connectAttr(obj_sqrt_name+".outputY",dynLoc+'.translateY',f=1)
    connectAttr(obj_sqrt_name+".outputZ",dynLoc+'.translateZ',f=1)
    connectAttr(dynLoc+'.translateX',XYZ_mat+'.colorR',f=1)
    connectAttr(dynLoc+'.translateY',XYZ_mat+'.colorG',f=1)
    connectAttr(dynLoc+'.translateZ',XYZ_mat+'.colorB',f=1)
    return arrowGroup


## check to see if muscle material exists. if yes, do nothing. if no, make one.
def assignColor(target, tint, alpha):
    if objExists(target):
        mat_name = tint+'_mat'
        material_SG = mat_name+'SG'
        red_rgb = [0.800, 0.000, 0.000]
        yellow_rgb = [0.800, 0.500, 0.000]
        green_rgb = [0.000, 0.800, 0.000]
        blue_rgb = [0.000, 0.000, 0.800]
        white_rgb = [1.000, 1.000, 1.000]
        black_rgb = [0.000, 0.000, 0.000]
        if not objExists(material_SG):
            material_SG = sets(renderable=True, noSurfaceShader=True, empty=True, name=material_SG )
        if not objExists(mat_name):
            material = shadingNode("lambert", asShader=True, name=mat_name)
            if tint == 'red':
                material.color.set(red_rgb)
            elif tint == 'yellow':
                material.color.set(yellow_rgb)
            elif tint == 'green':
                material.color.set(green_rgb)
            elif tint == 'blue':
                material.color.set(blue_rgb)
            elif tint == 'white':
                material.color.set(white_rgb)
            elif tint == 'black':
                material.color.set(black_rgb)
            elif tint == 'dynamic':
                material.color.set(black_rgb)
            material.transparency.set(alpha, alpha, alpha)
            material.outColor >> material_SG.surfaceShader
        else:
            print("Muscle material already exists!")
            material = mat_name
        sets(material_SG, edit=True, forceElement=target)
        return material


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


def quickExport(path, species):
    xformList = ['translateX','translateY','translateZ','rotateX','rotateY','rotateZ']
    torqueList = ['XaxisMomentArm','YaxisMomentArm','ZaxisMomentArm','XaxisTorque','YaxisTorque','ZaxisTorque']
    nodes = OrderedDict([
        ("tegu", OrderedDict([
            ('animal','dvec_animal'),
            ('acromion','IKscap:jcs_acromiondata'),
            ('glenoid','IKscap:jcs_glenohumeraldata'),
            ('elbow','IKscap:jcs_elbowdata'),
            ('wrist','IKscap:jcs_wristdata'),
            ('glenoidTorque','shoulder_ZYX_jnt'),
            ('elbowTorque','elbow_ZYX_jnt'),
        ])),
        ("opossum", OrderedDict([
            ('animal','dvec_animal'),
            ('acromion','IKscap:jcs_acromiondata'),
            ('glenoid','IKscap:jcs_glenohumeraldata'),
            ('elbow','IKscap:jcs_elbowdata'),
            ('wrist','IKscap:jcs_wristdata'),
            ('glenoidTorque','shoulder_ZYX_jnt'),
            ('elbowTorque','elbow_ZYX_jnt'),
        ])),
    ])
    vars = OrderedDict([
        ('animal', xformList + [    'shoulderHeight','spineBend',
                                    'localShoulderPosX','localShoulderPosY','localShoulderPosZ',
                                    'localElbowPosX','localElbowPosY','localElbowPosZ',
                                    'localWristPosX','localWristPosY','localWristPosZ',
                                    'localGRFX','localGRFY','localGRFZ']),
        ('acromion', xformList),
        ('glenoid', xformList),
        ('elbow', xformList),
        ('wrist', xformList),
        ('glenoidTorque', torqueList),
        ('elbowTorque', torqueList),
    ])
    frame_range = getPlaybackRange()
    headerList = [var+'.'+attr for var in vars for attr in vars[var]]
    attrDict = {var+'.'+attr : nodes[species][var]+'.'+attr for var in vars for attr in vars[var]}
    with open(path, 'w') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=["frame"]+headerList)
        writer.writeheader()
        frame = frame_range[0]
        lastframe= frame_range[1]
        assert(frame-lastframe != 0), "No animation found!"
        previousframe = lastframe+1
        currentTime(frame, update=1, edit=1)
        progressWindow(title="Exporting data...", min=frame, max=lastframe, progress=frame, status = "Parsing frame: "+str(frame), isInterruptable=True)
        while frame<=lastframe:
            if progressWindow(q=1, isCancelled=1):
                break
            if (frame-previousframe) == 0:
                break
            previousframe = frame
            dataRow = dict.fromkeys(headerList)
            dataRow['frame'] = int(frame)
            for key in headerList:
                dataRow[key] = getAttr(attrDict[key], sl=1)
            writer.writerow(dataRow)
            progressWindow(edit=1, progress=frame, status=('Processing frame: '+str(frame)))
            frame += 1
            currentTime(frame, update=1, edit=1)

        progressWindow(endProgress=1)
    return

## imports XYZ forces from csv
def importForces(path):
    data = {}
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for h in header:
            data[h] = []
        data['FXYZ'] = []
        for row in reader:
            for h, v in zip(header, row):
                data[h].append(float(v))
    for row in data['frame']:
        i = int(row)
        data['FXYZ'].append(sqrt(data['FX'][i]**2+data['FY'][i]**2+data['FZ'][i]**2))
    return(data)

def keyForces(data, target, endFrame=None, mag=5):
    # key end locator offsets using imported forces
    startLoc = spaceLocator(n='GRF_start')
    dataLoc = spaceLocator(n='GRF_data')
    endLoc3D = spaceLocator(n='GRF_end3D')
    endLocX = spaceLocator(n='GRF_endX')
    endLocY = spaceLocator(n='GRF_endY')
    endLocZ = spaceLocator(n='GRF_endZ')
    createNode("multiplyDivide", n='GRF_mag')
    setAttr('GRF_mag.operation', 1)
    connectAttr('GRF_data.translate', 'GRF_mag.input1', f=1)
    setAttr('GRF_mag.input2',[mag,mag,mag])
    connectAttr("GRF_mag.output","GRF_end3D.translate",f=1)
    connectAttr("GRF_mag.outputX","GRF_endX.translateX",f=1)
    connectAttr("GRF_mag.outputY","GRF_endY.translateY",f=1)
    connectAttr("GRF_mag.outputZ","GRF_endZ.translateZ",f=1)
    parent(dataLoc,endLoc3D, endLocX, endLocY, endLocZ, startLoc)
    matchTransform( startLoc, target, piv=True, pos=True, rot=True, scl=True)
    parent(startLoc,target)
    for i in data['frame']:
        idx = int(i)
        mayaFrame = idx+1
        setKeyframe(dataLoc, at='tx', v=data['FX'][idx], t=[mayaFrame])
        setKeyframe(dataLoc, at='ty', v=data['FY'][idx], t=[mayaFrame])
        setKeyframe(dataLoc, at='tz', v=data['FZ'][idx], t=[mayaFrame])
    if endFrame:
        keyframe('GRF_data', relative=True, timeChange=-(len(data['frame'])-endFrame))
    arrowScale = mag*max(data['FXYZ'])
    XScale = mag*max(data['FX'])
    YScale = mag*max(data['FY'])
    ZScale = mag*max(data['FZ'])
    arrowXYZ = makeForceArrows(startLoc, {'XYZ':endLoc3D,'X':endLocX,'Y':endLocY,'Z':endLocZ}, {'XYZ':arrowScale,'X':XScale,'Y':YScale,'Z':ZScale}, 'GRF',alpha=0.75)
    parent(arrowXYZ, startLoc, r=True )

def callGetOtherData(origin_field, sternum_field, glenohumeral_field, elbow_field, wrist_field, grfStart_field, grfEnd_field, *args):
    origin = textField(origin_field, q=1, tx=1)
    sternum = textField(sternum_field, q=1, tx=1)
    glenohumeral = textField(glenohumeral_field, q=1, tx=1)
    elbow = textField(elbow_field, q=1, tx=1)
    wrist = textField(wrist_field, q=1, tx=1)
    GRF_start = textField(grfStart_field, q=1, tx=1)
    GRF_end = textField(grfEnd_field, q=1, tx=1)
    getOtherData(origin, sternum, glenohumeral, elbow, wrist, GRF_start, GRF_end)

def getOtherDataUI(speciesSel):
    def handleTegu(*args):
        textField('origin_field', edit=1, text='IKscap:SEP73_roto')
        textField('sternum_field', edit=1, text='IKscap:interclavicle_j_roto')
        textField('glenohumeral_field', edit=1, text='IKscap:glenohumeral_j_roto')
        textField('elbow_field', edit=1, text='IKscap:elbow_j')
        textField('wrist_field', edit=1, text='IKscap:wrist_j')
        textField('grfStart_field', edit=1, text='GRF_start')
        textField('grfEnd_field', edit=1, text='GRF_data')
    def handleOpossum(*args):
        textField('origin_field', edit=1, text='IKscap:SEP87_roto')
        textField('sternum_field', edit=1, text='IKscap:manubrium_j_roto')
        textField('glenohumeral_field', edit=1, text='IKscap:glenoid_j')
        textField('elbow_field', edit=1, text='IKscap:elbow_j')
        textField('wrist_field', edit=1, text='IKscap:wrist_j')
        textField('grfStart_field', edit=1, text='GRF_start')
        textField('grfEnd_field', edit=1, text='GRF_data')
    def handleOrigin(*args):
        selection = ls(sl=1)
        textField('origin_field', edit=1, text=selection[0])
    def handleStart(*args):
        selection = ls(sl=1)
        textField('start_field', edit=1, text=selection[0])
    def handleEnd(*args):
        selection = ls(sl=1)
        textField('end_field', edit=1, text=selection[0])
    def handleSternum(*args):
        selection = ls(sl=1)
        textField('sternum_field', edit=1, text=selection[0])
    def handleGlenohumeral(*args):
        selection = ls(sl=1)
        textField('glenohumeral_field', edit=1, text=selection[0])
    def handleElbow(*args):
        selection = ls(sl=1)
        textField('elbow_field', edit=1, text=selection[0])
    def handleWrist(*args):
        selection = ls(sl=1)
        textField('wrist_field', edit=1, text=selection[0])
    def handleGRFstart(*args):
        selection = ls(sl=1)
        textField('grfStart_field', edit=1, text=selection[0])
    def handleGRFend(*args):
        selection = ls(sl=1)
        textField('grfEnd_field', edit=1, text=selection[0])
    def handleRun(*args):
        callGetOtherData(origin, sternum, glenohumeral, elbow, wrist, GRF_start, GRF_end)
        deleteUI('other_window')
    if window('other_window',exists=1) == True:
        deleteUI('other_window')
    mainWindow = window('other_window', title='Get Other Data',rtf=1, w=150, h=150)
    frameLayout(label='A) Choose a preset:')
    rowLayout(numberOfColumns=2)
    button(label='Tegu', command=handleTegu)
    button(label='Opossum', command=handleOpossum)
    setParent('..')
    frameLayout(label='B) OR select joints manually:')
    columnLayout()
    text(label='Forceplate origin')
    rowLayout(numberOfColumns=2, adjustableColumn=1)
    origin = textField('origin_field', text='')
    button(label='Select currently selected', command = handleOrigin)
    setParent('..')
    text(label='Sternum/interclavicle')
    rowLayout(numberOfColumns=2, adjustableColumn=1)
    sternum = textField('sternum_field', text='')
    button(label='Select currently selected', command = handleSternum)
    setParent('..')
    text(label='Glenohumeral')
    rowLayout(numberOfColumns=2, adjustableColumn=1)
    glenohumeral = textField('glenohumeral_field', text='')
    button(label='Select currently selected', command = handleGlenohumeral)
    setParent('..')
    text(label='Elbow')
    rowLayout(numberOfColumns=2, adjustableColumn=1)
    elbow = textField('elbow_field', text='')
    button(label='Select currently selected', command = handleElbow)
    setParent('..')
    text(label='Wrist')
    rowLayout(numberOfColumns=2, adjustableColumn=1)
    wrist = textField('wrist_field', text='')
    button(label='Select currently selected', command = handleWrist)
    setParent('..')
    text(label='GRF start')
    rowLayout(numberOfColumns=2, adjustableColumn=1)
    GRF_start = textField('grfStart_field', text='')
    button(label='Select currently selected', command = handleWrist)
    setParent('..')
    text(label='GRF end')
    rowLayout(numberOfColumns=2, adjustableColumn=1)
    GRF_end = textField('grfEnd_field', text='')
    button(label='Select currently selected', command = handleWrist)
    setParent('..')
    text(label='')
    button(label='Make Nodes', command=handleRun)
    text(label='')
    showWindow(mainWindow)
    if speciesSel == "tegu":
        handleTegu()
    elif speciesSel == "opossum":
        handleOpossum()


## prompt for importing GRF forces and visualizing with arrows
def keyForcesUI():
    def handleImport(*args):
        browse_path=fileDialog2(ff="*.csv", fm=1)
        textField('data_field', edit=1, text=browse_path[0])
    def handleSelect(*args):
        selection = ls(sl=1)
        textField('target_field', edit=1, text=selection[0])
    def handleRun(*args):
        callKeyForces(data_path, target, endFrame, mag)
        deleteUI('force_window')
    if window('force_window',exists=1) == True:
        deleteUI('force_window')
    mainWindow = window('force_window', title='Import Forces',rtf=1, w=150, h=150)
    frameLayout(label='1) Select a csv to import. Columns must be in format [frame#, X, Y, Z].')
    rowLayout(numberOfColumns=2, adjustableColumn=1)
    data_path = textField('data_field', text='')
    button(label='Browse', command=handleImport)
    setParent('..')
    frameLayout(label='2) Select a single object representing center of pressure location.')
    rowLayout(numberOfColumns=2, adjustableColumn=1)
    target = textField('target_field', text='')
    button(label='Select currently selected', command = handleSelect)
    setParent('..')
    frameLayout(label='3) (optional) Enter frame # representing end of loading duration.')
    columnLayout()
    text(label='')
    endFrame = textField('end_field', text='')
    text(label='')
    setParent('..')
    frameLayout(label='4) (optional) Enter magnification factor for visualizing forces (defaults to 20).')
    columnLayout()
    text(label='')
    mag = textField('mag_field', text='20')
    text(label='')
    setParent('..')
    button(label='Import Forces', command=handleRun)
    text(label='')
    showWindow(mainWindow)

def callKeyForces(data_path, target, end_field, mag_field, *args):
    data_val = textField(data_path, q=1, tx=1)
    target_val = textField(target, q=1, tx=1)
    end_val = textField(end_field, q=1, tx=1)
    mag_val = textField(mag_field, q=1, tx=1)
    data = importForces(data_val)
    end_int = int(end_val) if end_val else None
    keyForces(data, target_val, end_int, int(mag_val))

def momentAnalysisUI():
    def queryRadios(*args):
        speciesSel = radioCollection(radios, q=1, sl=1)
        if speciesSel == 'NONE':
            confirmDialog(title='No species selected', message='Please select a species from section 0.', button=['OK'],  cancelButton='OK', dismissString='OK')
            return False
        else:
            return speciesSel
    def handleBake(*args):
        bakeIKSim()
        saveBakedCopy()
        textField('destination_field', edit=1, text=getDefaultName())
    def handleJCS(*args):
        speciesSel = queryRadios()
        if speciesSel:
            dictSel = teguDict if speciesSel == "tegu" else possumDict
            scriptedJCS(dictSel)
    def handleNodes(*args):
        speciesSel = queryRadios()
        if speciesSel:
            getOtherDataUI(speciesSel)
    def handleDelete(*args):
        vecs = ls('*vec_*')
        delete(vecs)
    def handleMoments(*args):
        speciesSel = queryRadios()
        if speciesSel:
            startFrame = int(float(textField('startFrame_field', q=1, tx=1)))
            endFrame = int(float(textField('endFrame_field', q=1, tx=1)))
            quickMoments(speciesSel, [startFrame, endFrame])
    def handleBrowse(*args):
        browse_path=fileDialog2(cap="Save as", dir=workspace(dir=1, q=1), fm=0, ff="CSV files (*.csv)", ds=1)
        textField('destination_field', edit=1, text=browse_path[0])
    def handleExport(*args):
        speciesSel = queryRadios()
        if speciesSel:
            quickExport(textField(destination, q=1, tx=1),speciesSel)
    def getDefaultName(*args):
        wd, basename = os.path.split(os.path.splitext(cmds.file(q=1, sn=1, exn=1))[0])
        defaultname = os.path.join(wd, basename+'_data.csv')
        return defaultname
    teguDict = [
        {'axes': 'IKscap:jcs_acromion',  'dist': 'IKscap:acromion_j',  'prox': 'IKscap:interclavicle_j_roto'},
        {'axes': 'IKscap:jcs_glenohumeral',  'dist': 'IKscap:glenohumeral_j_roto',  'prox': 'IKscap:acromion_j'},
        {'axes': 'IKscap:jcs_elbow',  'dist': 'IKscap:elbow_j', 'prox': 'IKscap:glenohumeral_j_roto'},
        {'axes': 'IKscap:jcs_wrist',  'dist': 'IKscap:wrist_j', 'prox': 'IKscap:elbow_j'}
    ]
    possumDict = [
        {'axes': 'IKscap:jcs_acromion',  'dist': 'IKscap:acromion_roto',  'prox': 'IKscap:manubrium_j_roto'},
        {'axes': 'IKscap:jcs_glenohumeral',  'dist': 'IKscap:glenoid_j',  'prox': 'IKscap:acromion_roto'},
        {'axes': 'IKscap:jcs_elbow',  'dist': 'IKscap:elbow_j', 'prox': 'IKscap:glenoid_j'},
        {'axes': 'IKscap:jcs_wrist',  'dist': 'IKscap:wrist_j', 'prox': 'IKscap:elbow_j'}
    ]
    if window('moment_window',exists=1) == True:
        deleteUI('moment_window')
    mainWindow = window('moment_window', title='Moment Analysis Wizard',rtf=1, w=150, h=150)
    frameLayout(label='0) Select species.')
    rowColumnLayout(numberOfColumns=2)
    radios = radioCollection('speciesRadios')
    radioButton('tegu', label='Tegu')
    radioButton('opossum', label='Opossum')
    setParent('..')
    frameLayout(label='1) Manually key center of pressure location through all frames of interest.')
    text(label='')
    setParent('..')
    frameLayout(label='2) Import ground reaction forces from .csv.')
    button(label='Import GRF', command=lambda *args: keyForcesUI())
    setParent('..')
    frameLayout(label='3) Bake keys from IK simulation and save as a separate file (suffix "_baked").')
    button(label='Bake and Save', command=handleBake)
    setParent('..')
    frameLayout(label='4) Key zero pose at frame -1 and (optional) clear existing JCS data')
    rowLayout(numberOfColumns=3)
    button(label='Zero joints', command=zeroForJCS)
    text(label='')
    button(label='Clear JCS', command=clearKeyedJCS)
    setParent('..')
    frameLayout(label='5) Set joint coordinate systems for selected species (make sure XROMM shelf tools are installed)')
    button(label='Set JCS', command=handleJCS)
    setParent('..')
    frameLayout(label='6) Create nodes to measure spine bending, shoulder height, and shoulder, elbow, and wrist positions.')
    rowLayout(numberOfColumns=3)
    button(label='Create Nodes', command=handleNodes)
    text(label='')
    button(label='Delete Created Nodes', command=handleDelete)
    setParent('..')
    frameLayout(label='7) Calculate elbow and shoulder moments')
    rowLayout(numberOfColumns=6)
    text(label='Start frame')
    start = textField('startFrame_field', text=getPlaybackRange()[0])
    text(label='End frame')
    end = textField('endFrame_field', text=getPlaybackRange()[1])
    text(label='')
    button(label='Calculate moments', command=handleMoments)
    setParent('..')
    frameLayout(label='8) Export data')
    rowLayout(numberOfColumns=3, adjustableColumn=1)
    destination = textField('destination_field', text=getDefaultName())
    button(label='Browse', command=handleBrowse)
    button(label='Export', command=handleExport)
    setParent('..')
    showWindow(mainWindow)

def quickMoments(species, frameRange=None):
    lookup =    {   'tegu': {
                        'shoulder':{'A':'IKscap:jcs_glenohumeralProx', 'A*':'IKscap:jcs_glenohumeral', 'B':'GRF_data','C': 'GRF_start'},
                        'elbow':{'A':'IKscap:jcs_elbowProx', 'A*':'IKscap:jcs_elbow', 'B':'GRF_data', 'C':'GRF_start' },
                    },
                    'opossum':  {
                        'shoulder':{'A':'IKscap:jcs_glenohumeralProx', 'A*':'IKscap:jcs_glenohumeral', 'B':'GRF_data','C': 'GRF_start'},
                        'elbow':{'A':'IKscap:jcs_elbowProx', 'A*':'IKscap:jcs_elbow', 'B':'GRF_data', 'C':'GRF_start' },
                    }}
    rotation_order = "ZYX"
    frame_range = frameRange if frameRange else getPlaybackRange()
    for joint in ["shoulder", "elbow"]:
        sel = lookup[species][joint]
        proxy = makeUnitAxes(joint+'_'+rotation_order, sel, united=0, include_moments=True)
        updateFrame(sel, proxy, rotation_order, frame_range, get_moments=True)

def makeOffsetMatrix(objectA, objectB, nameA=None, nameB=None):
    nameA = nameA if nameA else objectA
    nameB= nameB if nameB else objectB
    diffName = "diffvec_"+nameB+"_FROM_"+nameA
    if objExists(diffName):
        diff = diffName
    else:
        diff = createNode("multMatrix", name=diffName)
        connectAttr(objectB+'.worldMatrix', diff+'.matrixIn[0]', force=True)
        connectAttr(objectA+'.worldInverseMatrix', diff+'.matrixIn[1]', force=True)
    return(diff)

def makeDecomposeMatrix(matrix, name=None):
    name = name if name else matrix
    decompName = "xformvec_"+matrix.split("_",1)[-1]
    if objExists(decompName):
        decomp = decompName
    else:
        decomp = createNode("decomposeMatrix", name=decompName)
        connectAttr(matrix+'.matrixSum', decomp+'.inputMatrix', force=True)
    return decomp

def makeUnitVector(vector, name=None):
    name = name if name else vector
    unitName = "uvec_"+vector.split("_",1)[-1]
    if objExists(unitName):
        unit = unitName
    else:
        unit = duplicate(rr=1, ic=1, n=unitName)
        setAttr(unitName+'.normalizeOutput', 1)
    return unit

def makeMagnitude(vector, name=None):
    name = name if name else vector
    nameBase = vector.split("_",1)[-1]
    magnitudeName = "magvec_"+nameBase
    selfDotName = 'sqmag_'+nameBase
    if objExists(magnitudeName):
        magnitude = magnitudeName
    else:
        selfDot =  createNode("vectorProduct", name=selfDotName)
        setAttr(selfDot+'.operation', 1)
        connectAttr(vector+'.output', selfDot+'.input1', force=True)
        connectAttr(vector+'.output', selfDot+'.input2', force=True)
        magnitude = createNode("multiplyDivide", name=magnitudeName)
        setAttr(magnitude+'.operation', 3)
        setAttr(magnitude+'.input2X', 0.5)
        connectAttr(selfDot+'.outputX', magnitude+'.input1X', force=True)
        addAttr(magnitude, shortName='mag', longName='magnitude', at="float" , keyable=0)
        setAttr(magnitude+'.magnitude', channelBox=1)
        connectAttr(magnitude+'.outputX',  magnitude+'.magnitude',force=True)
    return magnitude

def makePositionVector(object, name=None):
    name = name if name else object
    posName = "pvec_"+name
    if objExists(posName):
        pos = posName
    else:
        pos = createNode("vectorProduct", name=posName)
        setAttr(pos+'.operation', 4)
        connectAttr(object+'.worldMatrix[0]',  pos+'.matrix', force=True)
        addAttr(pos, shortName='pos', longName='position', at="float3" , keyable=0)
        for axis in ['X','Y','Z']:
            addAttr(pos, shortName='pos'+axis, longName='position'+axis, at="float" , keyable=0, parent='position')
        for axis in ['X','Y','Z']:
            setAttr(pos+'.position'+axis, channelBox=1)
        connectAttr(pos+'.output',  pos+'.position',force=True)
    return pos

def makeVectorAB(objectA, objectB, nameA=None, nameB=None, normalize=False):
    nameA = nameA if nameA else objectA
    nameB= nameB if nameB else objectB
    vecABName = "vec_"+nameA+'_TO_'+nameB
    if objExists(vecABName):
        vecAB = vecABName
    else:
        vecAB = createNode("plusMinusAverage", name=vecABName)
        setAttr(vecAB+'.operation', 2)
        addAttr(vecAB, shortName='XYZ', longName='components', at="float3" , keyable=0)
        for axis in ['X','Y','Z']:
            addAttr(vecAB, shortName=axis, longName='component'+axis, at="float" , keyable=0, parent='components')
        for axis in ['X','Y','Z']:
            setAttr(vecAB+'.component'+axis, channelBox=1)
        connectAttr(vecAB+'.output3D',  vecAB+'.components',force=True)
        posA = makePositionVector(objectA, nameA)
        posB = makePositionVector(objectB, nameB)
        connectAttr(posB+'.output',  vecAB+'.input3D[0]',force=True)
        connectAttr(posA+'.output',  vecAB+'.input3D[1]',force=True)
        if normalize:
            normAB = createNode("vectorProduct", name=vecABName+'_norm')
            connectAttr(vecAB+'.output3D', normAB+'.input1', force=True)
            setAttr(normAB+'.operation', 0)
            setAttr(normAB+'.normalizeOutput', 1)
    return normAB if normalize else vecAB

def useReferenceFrame(object, targetFrame, mode, objectName=None, targetFrameName=None):
    objectName = objectName if objectName else object
    targetFrameName= targetFrameName if targetFrameName else targetFrame
    resultName = objectName + "_IN_" + targetFrameName
    if objExists(resultName):
        result = resultName
    else:
        result = createNode("vectorProduct", name=resultName)
        connectAttr(targetFrame+'.worldInverseMatrix', result+'.matrix', force=True)
        if mode == "vec":
            setAttr(result+'.operation', 3)
            connectAttr(object+'.output3D', result+'.input1', force=True)
        elif mode == "pvec":
            setAttr(result+'.operation', 4)
            connectAttr(object+'.output', result+'.input1', force=True)
        addAttr(result, shortName='XYZ', longName='components', at="float3" , keyable=0)
        for axis in ['X','Y','Z']:
            addAttr(result, shortName=axis, longName='component'+axis, at="float" , keyable=0, parent='components')
        for axis in ['X','Y','Z']:
            setAttr(result+'.component'+axis, channelBox=1)
        connectAttr(result+'.output',  result+'.components',force=True)
    return result

def getOtherData(originTransform, sternalJoint, shoulderJoint, elbowJoint, wristJoint, GRF_start, GRF_end):
    ## body reference frame using start-end of sternum as vector
    sternalLoc = spaceLocator(name="dvec_sternalLoc")
    pointConstraint(sternalJoint, sternalLoc)
    orientConstraint(sternalJoint,sternalLoc)
    timeRange = getPlaybackRange()
    startMatrix = getAttr(sternalJoint+'.worldMatrix', time=timeRange[0])
    startMatrixInOriginFrame = startMatrix*getAttr(originTransform+'.worldInverseMatrix')
    endMatrix = getAttr(sternalJoint+'.worldMatrix', time=timeRange[1])
    endMatrixInOriginFrame = endMatrix*getAttr(originTransform+'.worldInverseMatrix')
    startMMatrix = om.MTransformationMatrix(startMatrixInOriginFrame)
    endMMatrix = om.MTransformationMatrix(endMatrixInOriginFrame)
    startTranslation = startMMatrix.translation(om.MSpace.kWorld)
    endTranslation = endMMatrix.translation(om.MSpace.kWorld)
    displacementVector = endTranslation - startTranslation
    wholeAnimalLoc = spaceLocator(name="dvec_animal")
    parent(wholeAnimalLoc, sternalLoc, originTransform)
    xform(wholeAnimalLoc, ro=[0,0,0])
    pointConstraint(sternalJoint, wholeAnimalLoc)
    angleToRotate = angleBetween(v1=(1.0,0.0,0.0), v2=(displacementVector.x,displacementVector.y,0.0), euler=True)
    xform(wholeAnimalLoc, ro=angleToRotate)
    # get shoulder position in forceplate reference frame
    shoulderPos = makePositionVector(shoulderJoint, name='glenoid')
    shoulderPosInForceplate = useReferenceFrame(shoulderPos, originTransform, 'pvec', targetFrameName='forcePlate')
    # get shoulder position in animal reference frame
    shoulderPosInAnimal = useReferenceFrame(shoulderPos, wholeAnimalLoc, 'pvec', targetFrameName='animal')
    # get elbow position in animal reference frame
    elbowPos = makePositionVector(elbowJoint, name='elbow')
    elbowPosInAnimal = useReferenceFrame(elbowPos, wholeAnimalLoc, 'pvec', targetFrameName='animal')
    # get wrist position in animal reference frame
    wristPos = makePositionVector(wristJoint, name='wrist')
    wristPosInAnimal = useReferenceFrame(wristPos, wholeAnimalLoc, 'pvec', targetFrameName='animal')
    #get spine bending: difference between body orientation and ic orientation
    animalSternumDiff = makeOffsetMatrix(wholeAnimalLoc, sternalLoc, nameA="animal", nameB="sternum")
    animalSternumDiffDecomp = makeDecomposeMatrix(animalSternumDiff)
    #get GRF vector in animal reference frame
    grfVec = makeVectorAB(GRF_start, GRF_end)
    grfVecInAnimal = useReferenceFrame(grfVec, wholeAnimalLoc, 'vec', targetFrameName='animal')
    #put new attributes on animal locator
    addAttr(wholeAnimalLoc, shortName='sHt', longName='shoulderHeight', at="float" , keyable=0)
    connectAttr(shoulderPosInForceplate+'.outputZ',  wholeAnimalLoc+'.shoulderHeight')
    setAttr(wholeAnimalLoc+'.shoulderHeight', channelBox=1)
    addAttr(wholeAnimalLoc, shortName='sBend', longName='spineBend', at="float" , keyable=0)
    connectAttr(animalSternumDiffDecomp+'.outputRotateZ',  wholeAnimalLoc+'.spineBend')
    setAttr(wholeAnimalLoc+'.spineBend', channelBox=1)
    # float3 attribute for local GRF
    addAttr(wholeAnimalLoc, shortName='lGrf', longName='localGRF', at="float3" , keyable=0)
    for axis in ['X','Y','Z']:
        addAttr(wholeAnimalLoc, shortName='lGrf'+axis, longName='localGRF'+axis, at="float" , keyable=0, parent='localGRF')
    for axis in ['X','Y','Z']:
        setAttr(wholeAnimalLoc+'.localGRF'+axis, channelBox=1)
    connectAttr(grfVecInAnimal+'.output',  wholeAnimalLoc+'.localGRF')
    # float3 attribute for local shoulder
    addAttr(wholeAnimalLoc, shortName='sPos', longName='localShoulderPos', at="float3" , keyable=0)
    for axis in ['X','Y','Z']:
        addAttr(wholeAnimalLoc, shortName='sPos'+axis, longName='localShoulderPos'+axis, at="float" , keyable=0, parent='localShoulderPos')
    for axis in ['X','Y','Z']:
        setAttr(wholeAnimalLoc+'.localShoulderPos'+axis, channelBox=1)
    connectAttr(shoulderPosInAnimal+'.output',  wholeAnimalLoc+'.localShoulderPos')
    # float3 attribute for local elbow
    addAttr(wholeAnimalLoc, shortName='ePos', longName='localElbowPos', at="float3" , keyable=0)
    for axis in ['X','Y','Z']:
        addAttr(wholeAnimalLoc, shortName='ePos'+axis, longName='localElbowPos'+axis, at="float" , keyable=0, parent='localElbowPos')
    for axis in ['X','Y','Z']:
        setAttr(wholeAnimalLoc+'.localElbowPos'+axis, channelBox=1)
    connectAttr(elbowPosInAnimal+'.output',  wholeAnimalLoc+'.localElbowPos')
    # float3 attribute for local wrist
    addAttr(wholeAnimalLoc, shortName='wPos', longName='localWristPos', at="float3" , keyable=0)
    for axis in ['X','Y','Z']:
        addAttr(wholeAnimalLoc, shortName='wPos'+axis, longName='localWristPos'+axis, at="float" , keyable=0, parent='localWristPos')
    for axis in ['X','Y','Z']:
        setAttr(wholeAnimalLoc+'.localWristPos'+axis, channelBox=1)
    connectAttr(wristPosInAnimal+'.output',  wholeAnimalLoc+'.localWristPos')
    return wholeAnimalLoc


def getPlaybackRange():
    timeRange = [playbackOptions(minTime=1, q=1),playbackOptions(maxTime=1, q=1)]
    return timeRange

def saveBakedCopy():
    path = cmds.file(q=True, exn=True)
    dirName, oldFullName = os.path.split(path)
    oldName, extension = os.path.splitext(oldFullName)
    newName = oldName + '_baked' + extension
    newPath = os.path.join(dirName, newName)
    cmds.file(rename=newPath)
    cmds.file( save=True, force=True, type='mayaBinary' )
    return

def bakeIKSim():
    jointsSel = ls(type='joint')
    timeRange = getPlaybackRange()
    bakeResults(jointsSel, simulation=True, time=str(timeRange[0])+":"+str(timeRange[1]), sampleBy=1, oversamplingRate = 1 , disableImplicitControl =True, preserveOutsideKeys = True,  sparseAnimCurveBake = False, removeBakedAttributeFromLayer = False, removeBakedAnimFromLayer = False , bakeOnOverrideLayer =  False , minimizeRotation = True , controlPoints = False,  shape = True)
    return

def clearKeyedJCS():
    sure = confirmDialog( title='Confirm', message='Clear all JCS data? This will delete all "Prox" axis objects and JCS data nodes.', button=['Yes','No'], defaultButton='Yes', cancelButton='No', dismissString='No' )
    if sure == "Yes":
        proxSel = ls('*:*jcs*Prox*')
        dataSel = ls('*:*jcs*data*')
        delete(proxSel+dataSel)
    return

def zeroForJCS(dummyFrame=-1):
    mel.eval('doEnableNodeItems true all')
    currentTime(dummyFrame, update=1, edit=1)
    jointsSel = ls(type='joint')
    ikHandleSel = ls(type='ikHandle')
    ikEffectorSel = ls(type='ikEffector')
    ikSel = ikHandleSel + ikEffectorSel
    coracoid_j =filter(lambda v: re.search('coracoid_j_roto',str(v)), jointsSel)
    if coracoid_j:
        setAttr(coracoid_j[0]+'.translateX', 0)
        select(ikSel, r=1)
    mel.eval('doEnableNodeItems false all') #disable ik solvers, constraints, and expressions that might prevent joints from being zeroed
    for selectedJoint in jointsSel:
        for axis in ['X','Y','Z']:
            attrName = selectedJoint+'.rotate'+axis
            isReferenced = referenceQuery( attrName, isNodeReferenced=True )
            settable = getAttr(attrName, settable=1, silent=1)
            if settable:
                setAttr(attrName, 0)
            elif not isReferenced:
                setAttr(attrName, 0, lock=0)
        setKeyframe(selectedJoint)

def scriptedJCS(listOfDicts, dummyFrame=-1):
    #paste in desired jcs as list of dicts, e.g. {'prox':'proxjoint', 'dist':'distjoint', 'axes':'axes'}
    mel.eval('doEnableNodeItems false all')
    for dict in listOfDicts:
        currentTime(dummyFrame, update=1, edit=1)
        mel.eval('jointAxesDual')
        textFieldButtonGrp('proxFieldGroup', e=1, text = dict['prox'])
        textFieldButtonGrp('distFieldGroup', e=1, text = dict['dist'])
        textFieldButtonGrp('axesFieldGroup', e=1, text = dict['axes'])
        # button('setAxesBButton', e=1, enable=1)
        mel.eval('setAxesB')
    deleteUI('jointAxesWindow')
    mel.eval('doEnableNodeItems true all')


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
    muscle_name = textField('muscle_name_field', text='MuscleName')
    text(label='')
    text(label='')
    radios = radioCollection('rotationOrderRadios')
    radioButton('ZYX', label='ZYX intrinsic (XROMM default)', sl=True)
    radioButton('XYZ', label='XYZ intrinsic')
    text(label='')
    text(label='')
    button(label='Create Muscle', command=lambda *args: doTheThing(muscle_name, radios))
    text(label='')
    showWindow(mainWindow)
