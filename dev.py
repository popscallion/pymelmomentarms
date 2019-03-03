### calculate muscle moment arms w/r/t a defined joint center
from pymel.all import *
import pymel.core.datatypes as dt

def getMuscleName():
    if window('muscleWindow',exists=1) == True:
        deleteUI('muscleWindow')
    mainWindow = window('muscleWindow', title='Create Muscle',rtf=1, w=300, h=200)
    frameLayout(label='Muscle Name Input')
    columnLayout(columnOffset=('both',20))
    text(label=" ")
    text(label="Select (in order):")
    text(label=" 1) Joint Coordinate System (JCS)")
    text(label=" 2) Muscle Marker 1 (proximal)")
    text(label=" 3) Muscle Marker 2 (distal)")
    text(label=" ")
    text(label="Muscle Name")
    textField('muscleNameField', text="muscleName")
    button(label="Create Muscle",command=doTheThing)
    showWindow(mainWindow)

### check to see if muscle material exists. if yes, do nothing. if not, make one.
### maya assigns materials(shaders) by adding meshes to a shader group, which must first be created
def assignMuscleColor(target):
    if objExists(target):
        if not objExists("muscleMat"):
            muscleMaterial = shadingNode("lambert", asShader=True, name="muscleMat")
            muscleMaterialSG = sets(renderable=True, noSurfaceShader=True, empty=True, name="muscleMatSG" )
            muscleMaterial.color.set(0.604, 0.036, 0.014)
            muscleMaterial.transparency.set(0.5, 0.5, 0.5)
            muscleMaterial.outColor >> muscleMaterialSG.surfaceShader
        else:
            print("Muscle material already exists!")
            muscleMaterial = "muscleMat"
            muscleMaterialSG = "muscleMatSG"
        sets(muscleMaterialSG, edit=True, forceElement=target)

### get currently selected objects in order:
### 1. joint center, 2. muscle marker #1, 3. muscle marker #2
def getSelectionSet():
    sel = ls(sl=1)
    assert(0<len(sel)<=3),"Please select (in order): 1. joint center, 2. proximal muscle marker, 3. distal muscle marker"
    return sel

##convenience:make vectors from two locators
def makeVector(locA,locB):
    posA = dt.Vector(xform(locA,q=1,t=1,ws=1))
    posB = dt.Vector(xform(locB,q=1,t=1,ws=1))
    vectAB = posB - posA
    return vectAB

### make muscle cylinder for visualization
def makeMuscleCyl(sel,muscleName):
    mus_name = "mus_"+muscleName
    mus_length_name = "length_"+mus_name
    mus_mult_name = "multiply"+mus_name
    polyCylinder(r=0.05,h=2, sx=8, ax=[0,1,0], n=mus_name)
    assignMuscleColor(mus_name)
    select(mus_name)
    xform(piv=[0,-1,0]) #moves pivot point to one end of cylinder
    pointConstraint(sel['B'],mus_name)
    aimConstraint(sel['C'],mus_name,aimVector=[0,1,0],worldUpType="vector")
    hide(listRelatives(mus_name)[-2:])
    createNode("distanceBetween", n=mus_length_name)
    connectAttr(sel['B']+".translate",mus_length_name+'.point1',f=1)
    connectAttr(sel['C']+".translate",mus_length_name+'.point2',f=1)
    createNode("multiplyDivide", n=mus_mult_name)
    connectAttr(mus_length_name+".distance",mus_mult_name+".input1Y",f=1)
    connectAttr(mus_mult_name+".outputY",mus_name+".scaleY",f=1)
    setAttr(mus_mult_name+".input2Y",0.5)
    return(mus_name)

### calculate orthogonal distance between joint center and muscle vector
def calcMAVector(sel):
    A, B, C = sel[0], sel[1], sel[2]
    vectBA = makeVector(B,A)
    vectBC = makeVector(B,C)
    unitBC = vectBC/vectBC.length()
    distT = vectBA * unitBC #get distance T of point B to projection P of point A on vector BC
    projectionP = dt.Vector(xform(B,q=1,translation=1,worldSpace=1)) + distT * unitBC #get projection P as sum of position B and distance T along vector BC
    vectAP = projectionP - dt.Vector(xform(A,q=1,translation=1,worldSpace=1))
    distanceResult = vectAP.length()
    interMarkerDistance = vectBC.length()
    dict = {'vectAP': vectAP, 'distanceResult': distanceResult, 'distT':distT, 'projectionP':projectionP, 'interMarkerDistance':interMarkerDistance}
    return(dict)

### make moment arm locator object
def makeMAobj(sel, muscleName):
    #if MA locator doesn't exist, make one and parent under JCS to get object/local space translations
    MA_muscle = spaceLocator(n="MA_"+muscleName)
    addAttr(MA_muscle, shortName='lMA', longName='MomentArmLength', defaultValue=0.0, keyable=1)
    addAttr(MA_muscle, shortName='iMD', longName='IntermarkerDistance', defaultValue=0.0, keyable=1)
    addAttr(MA_muscle, shortName='xPA', longName='xProjectionArm', defaultValue=0.0, keyable=1)
    addAttr(MA_muscle, shortName='yPA', longName='yProjectionArm', defaultValue=0.0, keyable=1)
    addAttr(MA_muscle, shortName='zPA', longName='zProjectionArm', defaultValue=0.0, keyable=1)
    addAttr(MA_muscle, shortName='xPM', longName='xProjectionMuscle', defaultValue=0.0, keyable=1)
    addAttr(MA_muscle, shortName='yPM', longName='yProjectionMuscle', defaultValue=0.0, keyable=1)
    addAttr(MA_muscle, shortName='zPM', longName='zProjectionMuscle', defaultValue=0.0, keyable=1)
    addAttr(MA_muscle, shortName='xMA', longName='xMomentArm', defaultValue=0.0, keyable=1)
    addAttr(MA_muscle, shortName='yMA', longName='yMomentArm', defaultValue=0.0, keyable=1)
    addAttr(MA_muscle, shortName='zMA', longName='zMomentArm', defaultValue=0.0, keyable=1)
    matchTransform(MA_muscle,sel[0],rot=1, pos=1)
    parent(MA_muscle, sel[0])
    print("made "+muscleName+"'s moment arm locator!")
    return(MA_muscle)

### decompose 3D moment arm into its projections onto X, Y, and Z planes, but returns 0 if the plane projection is parallel to a given axis
def findProjection(MA_muscle):
    MA_xyz = xform(MA_muscle,q=1,t=1,objectSpace=1)
    vectMA_xyz = dt.Vector(MA_xyz)
    vectX = dt.Vector(1,0,0)
    vectY = dt.Vector(0,1,0)
    vectZ = dt.Vector(0,0,1)
    XYPlane = cross(vectX,vectY)
    YZPlane = cross(vectY,vectZ)
    XZPlane = cross(vectX,vectZ)
    projX = vectMA_xyz - dot(vectMA_xyz,YZPlane)*YZPlane
    projY = vectMA_xyz - dot(vectMA_xyz,XZPlane)*XZPlane
    projZ = vectMA_xyz - dot(vectMA_xyz,XYPlane)*XYPlane
    areaX = projX[1]*projX[2]
    areaY = projY[0]*projY[2]
    areaZ = projZ[0]*projZ[1]
    if areaX == 0:
        resultX = 0
    else:
        resultX = projX.length()
    if areaY == 0:
        resultY = 0
    else:
        resultY = projY.length()
    if areaZ == 0:
        resultZ = 0
    else:
        resultZ = projZ.length()
    dict = {'xMA': resultX,'yMA': resultY,'zMA': resultZ}
    return(dict)


### store decomposed moment arms as custom attributes
def keyMA(sel,muscleName,MA_muscle):
    #run calcMAVector to get projectionP
    result = calcMAVector(sel)
    xform(MA_muscle,t=result['projectionP'],ws=1)
    #get translation of MA locator
    momentArms = findProjection(MA_muscle)
    #store MA projections as attributes in MA locator, then key
    setAttr(MA_muscle.xMA, momentArms['xMA'])
    setAttr(MA_muscle.yMA, momentArms['yMA'])
    setAttr(MA_muscle.zMA, momentArms['zMA'])
    setAttr(MA_muscle.lMA, result['distanceResult'])
    setAttr(MA_muscle.iMD, result['interMarkerDistance'])
    setKeyframe(MA_muscle, v=momentArms['xMA'], at='xMA')
    setKeyframe(MA_muscle, v=momentArms['yMA'], at='yMA')
    setKeyframe(MA_muscle, v=momentArms['zMA'], at='zMA')
    setKeyframe(MA_muscle, v=result['distanceResult'], at='lMA')
    setKeyframe(MA_muscle, v=result['interMarkerDistance'], at='iMD')
    setKeyframe(MA_muscle, at='translateX')
    setKeyframe(MA_muscle, at='translateY')
    setKeyframe(MA_muscle, at='translateZ')
    #check:sqrt(xMA**2+MA_x**2) = result['distanceResult']
    return(MA_muscle)

### step through frames, inspiration from David Baier's outputRelMotion shelf tool
def keyFrameHelper(sel,muscleName):
    if objExists("MA_"+muscleName):
        print(muscleName+"'s moment arm locator already exists! deleting...")
        delete("MA_"+muscleName)
    transformList = ls(tr=1)
    frame=findKeyframe(transformList, hi="both",which="first")
    lastframe=findKeyframe(transformList, hi="both",which="last")
    assert(frame-lastframe != 0), "No animation found!"
    previousframe = lastframe+1
    currentTime(frame, update=1, edit=1)
    progressWindow(title="Grabbing moment arms...", min=frame, max=lastframe, progress=frame, status = "Crunching frame: "+str(frame), isInterruptable=True)
    MA_muscle=makeMAobj(sel,muscleName)
    while frame<=lastframe:
        #key and advance
        MA_muscle = (keyMA(sel,muscleName,MA_muscle))
        frame=findKeyframe(transformList, hi="both",which="next")
        if progressWindow(q=1, isCancelled=1):
            break
        if (frame-previousframe) == 0:
            break
        previousframe = frame
        progressWindow(edit=1, progress=frame, status=('Processing frame: '+str(frame)))
        currentTime(frame, update=1, edit=1)
    progressWindow(endProgress=1)

### make measure tool to double check moment arm length calculation
def measureToolMA(locA, locB, muscleName, measureType):
    sceneBefore = ls(l=1, transforms=1)
    newMeasure = distanceDimension(sp=getAttr(locA.translate),ep=getAttr(locB.translate))
    newMeasure = rename(newMeasure,'msr_'+muscleName+measureType)
    return(newMeasure)

### all together now
def doTheThing(muscleName):
    muscleName = textField('muscleNameField', q=1,text=1)
    sel = getSelectionSet()
    makeMuscleCyl(sel,muscleName)
    keyFrameHelper(sel,muscleName)
    return(muscleName, sel)


### devblock: setting up
muscle_name = 'testMuscle'
sel={}
sel['A'], sel['B'], sel['C'] = getSelectionSet()
#run once:
mus_cyl = makeMuscleCyl(sel,muscle_name)
proxy_cs = makeJointProxy(sel['A'], muscle_name)
#run each frame:
xyzUnits = updateCoordinateSystem(proxy_cs)
muscle_CB = makeVector(sel['C'],sel['B'])
joint_muscle_BA = makeVector(sel['B'],sel['A'])
armDecomposer(muscle_CB, joint_muscle_BA, xyzUnits)


##delete:

dotcheckX=projectionHero('xPlane', muscle_CB, xyzUnits)
dotcheckY=projectionHero('yPlane', muscle_CB, xyzUnits)
dotcheckZ=projectionHero('zPlane', muscle_CB, xyzUnits)

dot(dotcheckZ,xyzUnits['zUnit'])
round(dotcheckZ.length(),4)
sumofsquares([dotcheckX.length(),dotcheckY.length(),dotcheckZ.length()])
muscle_CB.length()
####GET VALENCE:dot muscle vector with joint center vector to find projP in world space
unitCB = muscle_CB/muscle_CB.length()
distT = dot(joint_muscle_BA, unitCB)
projP = dt.Vector(xform(sel['B'], q=1, t=1, ws=1)+(distT*unitCB))
joint_proj_PA = projP-(xform(sel['A'], q=1, t=1, ws=1))
joint_proj_PA.length()
projectionHero('xPlane', joint_proj_PA, xyzUnits)


projectionHero(mode, target, cs)

##make local space moment arm vector
xyzMA = projP-dt.Vector(xform(A, q=1, t=1, ws=1))
xyzMAlength = xyzMA.length()
##check to see if moment arm vector and muscle vector are perpendicular




#### WAY 2: PROJECT MUSCLE INTO LOCAL REFERENCE FRAME, THEN GET MOMENT ARM
##get worldspace translations of proxy axis locators

##once:make proxy coordinate system with unit axis locators
def makeJointProxy(target, name):
    joint_proxy = spaceLocator(n="jnt_"+name)
    x_proxy = spaceLocator(n="x_"+name)
    y_proxy = spaceLocator(n="y_"+name)
    z_proxy = spaceLocator(n="z_"+name)
    xform(x_proxy, t=[1,0,0])
    xform(y_proxy, t=[0,1,0])
    xform(z_proxy, t=[0,0,1])
    parent(x_proxy,y_proxy,z_proxy,joint_proxy)
    pointConstraint(target, joint_proxy)
    orientConstraint(target, joint_proxy)
    hide(listRelatives(joint_proxy)[-5:])
    proxy = {   'joint':joint_proxy,
                'x':x_proxy,
                'y':y_proxy,
                'z':z_proxy}
    return(proxy)

##perframe:rcalculate world space vectors for joint proxy
def updateCoordinateSystem(cs):
    xUnit = makeVector(cs['joint'],cs['x'])
    yUnit = makeVector(cs['joint'],cs['y'])
    zUnit = makeVector(cs['joint'],cs['z'])
    xyzUnits = {'xUnit':xUnit,'yUnit':yUnit,'zUnit':zUnit}
    return(xyzUnits)



##dot muscle vector with each axis to get component in plane
def projectionHero(mode, target, cs):
    xUnit = cs['xUnit']
    yUnit = cs['yUnit']
    zUnit = cs['zUnit']
    xAxisP = dot(target,xUnit)
    yAxisP = dot(target,yUnit)
    zAxisP = dot(target,zUnit)
    xPlaneP = target-(xAxisP*xUnit)
    yPlaneP = target-(yAxisP*yUnit)
    zPlaneP = target-(zAxisP*zUnit)
    xAxisP = dot(target,xUnit)
    yAxisP = dot(target,yUnit)
    zAxisP = dot(target,zUnit)
    result = {'xPlane':xPlaneP,
              'yPlane':yPlaneP,
              'zPlane':zPlaneP,
              'xAxis':xAxisP,
              'yAxis':yAxisP,
              'zAxis':zAxisP}
    return(result[mode])

##cross vectors to find area of parallelogram as magnitude of orthogonal vector
##find perpendicular distance as height of parallelogram
def armDecomposer(muscle_CB, joint_muscle_BA, cs):
    result = {}
    for case in ['xPlane','yPlane','zPlane']:
        muscle_CB_proj = projectionHero(case, muscle_CB, cs)
        joint_muscle_BA_proj = projectionHero(case, joint_muscle_BA, cs)
        parallelogram = cross(muscle_CB_proj,joint_muscle_BA_proj)
        momentArm = round(parallelogram.length()/muscle_CB_proj.length(),5)
        ###valence = (muscleVectorProj-jointMuscleVectorProj)/(muscleVectorProj-jointMuscleVectorProj).length()
        print('moment arm in '+case+' is: '+str(momentArm))
        ###print('valence is:'+str(valence))
        result[case]=momentArm
    return(result)





####BEGIN CHECK origin case checker, using locators 1, 2, and 3
one = dt.Vector(xform('locator1',q=1,t=1,ws=1))
two = dt.Vector(xform('locator2',q=1,t=1,ws=1))
three = dt.Vector(xform('locator3',q=1,t=1,ws=1))
vect1CB = one-two
vect1BA = two-three
unit1X = dt.Vector([1,0,0])
unit1Y = dt.Vector([0,1,0])
unit1Z = dt.Vector([0,0,1])
xyzUnits1 = {'xUnit':unit1X,'yUnit':unit1Y,'zUnit':unit1Z}


vect1CBalongX = dot(vect1CB,unit1X)
vect1CBalongY = dot(vect1CB,unit1Y)
vect1CBalongZ = dot(vect1CB,unit1Z)

vect1CBonplaneX = vect1CB-dot(vect1CB,unit1X)*unit1X
vect1CBonplaneY = vect1CB-dot(vect1CB,unit1Y)*unit1Y
vect1CBonplaneZ = vect1CB-dot(vect1CB,unit1Z)*unit1Z
vect1CBonplaneZ.length()

armDecomposer(vect1CB, vect1BA, xyzUnits1)
####END CHECK



####USE THIS TO GET PROJECTION POINT?
#### WAY 1: CALCULATE IN 3D, THEN REPROJECT
##dot muscle vector with joint center vector to find projP in world space
distT = dot(vectBA, unitCB)
projP = dt.Vector(xform(B, q=1, t=1, ws=1)+(distT*unitCB))
##make local space moment arm vector
xyzMA = projP-dt.Vector(xform(A, q=1, t=1, ws=1))
xyzMAlength = xyzMA.length()
##check to see if moment arm vector and muscle vector are perpendicular
round(dot(xyzMA,vectCB),6)==0





###NEXT:FIND WAY TO CALCULATE NEGATIVE VALUES



def sumofsquares(arr):
    result = sqrt(arr[0]**2+arr[1]**2+arr[2]**2)
    return result







###<MONTUESTHURS: ROTATION ORDER-AWARE PROJECTION, THEN SCALE BY PLANAR PROJECTION OF MUSCLE VECTOR>
muscle_name = 'testMuscle'
selProx={}
selDist={}

#### make proximal and distal proxies
selProx['A'], selProx['B'], selProx['C'] = getSelectionSet()
selDist['A'], selDist['B'], selDist['C'] = getSelectionSet()
mus_cyl = makeMuscleCyl(selDist,muscle_name)

proxy_Prox = makeJointProxy(selProx['A'], muscle_name)
proxy_Dist = makeJointProxy(selDist['A'], muscle_name)
#proxy_Prox_move = makeJointProxy(selProx['A'], muscle_name)
proxy_Prox_t_Dist = makeJointProxy(selProx['A'], muscle_name)
#translate proxy_Prox_t_Dist to match proxy_Dist
translation_proxy_Prox = xform(proxy_Prox['joint'],q=1,t=1,ws=1)
translation_proxy_Dist = xform(proxy_Dist['joint'],q=1,t=1,ws=1)
xform(proxy_Prox_t_Dist['joint'], t=translation_proxy_Dist, ws=1)
xform(proxy_Prox_t_Dist['joint'],q=1,t=1,ws=1)
xform(proxy_Prox_t_Dist['joint'],q=1,ro=1,ws=1) == xform(proxy_Prox['joint'],q=1,ro=1,ws=1)
xform(proxy_Prox_t_Dist['joint'],q=1,ro=1,ws=1) == xform(proxy_Dist['joint'],q=1,ro=1,ws=1)


#case0: ignore rotation order JCS
u_Prox = updateCoordinateSystem(proxy_Prox)
u_Dist = updateCoordinateSystem(proxy_Dist)

#case1: Z only
rotate(proxy_Prox_t_Dist['joint'], [0,0,24], os=1, fo=1, r=1)
zRotProx_u_tdist = updateCoordinateSystem(proxy_Prox_t_Dist)
#case2: ZY only
rotate(proxy_Prox_t_Dist['joint'], [0,6.4,0], os=1, fo=1, r=1)
zyRotProx_u_tdist = updateCoordinateSystem(proxy_Prox_t_Dist)
#case3: ZYX only
rotate(proxy_Prox_t_Dist['joint'], [37,0,0], os=1, fo=1, r=1)
zyxRotProx_u_tdist = updateCoordinateSystem(proxy_Prox_t_Dist)

#make muscle and joint vectors ABC
muscle_CB = makeVector(selProx['C'],selProx['B'])
joint_muscle_BA = makeVector(selProx['B'],selDist['A'])


case1_tdist_z = armDecomposer(muscle_CB, joint_muscle_BA, zRotProx_u_tdist)
case2_tdist_zy = armDecomposer(muscle_CB, joint_muscle_BA, zyRotProx_u_tdist)
case3_tdist_zyx = armDecomposer(muscle_CB, joint_muscle_BA, zyxRotProx_u_tdist)


regularcase_tprox = armDecomposer(muscle_CB, joint_muscle_BA, u_Prox)
regularcase_tdist = armDecomposer(muscle_CB, joint_muscle_BA, u_Dist)
compositeCase_tdist_xyzsequential = {'xPlane':case1_tdist_z['xPlane'],'yPlane':case2_tdist_zy['yPlane'],'zPlane':case3_tdist_zyx['zPlane']} ##maybe?

def musclePlaneFraction(target, cs):
    planes = {'xPlane':0, 'yPlane':0, 'zPlane':0}
    for case in planes:
        result = projectionHero(case, target, cs)
        planes[case] = result.length()/target.length()
    return planes

proxFractions = musclePlaneFraction(muscle_CB, u_Prox)
distFractions = musclePlaneFraction(muscle_CB, u_Dist)
rotationOrderFractions = musclePlaneFraction(muscle_CB, zyxRotProx_u_tdist)

def scaleMomentArm(MAs, fractions):
    planes = {}
    for case in MAs.keys():
        planes[case] = MAs[case]*fractions[case]
    return planes

prox_eMA = scaleMomentArm(regularcase_tprox,proxFractions)
dist_eMA = scaleMomentArm(regularcase_tdist,distFractions)
rotationOrder_eMA = scaleMomentArm(compositeCase_tdist_xyzsequential,rotationOrderFractions)



###</MONTUESTHURS>


###<WEDS27FEB5PM:GET LENGTH CHANGE OF BICEPS IN 48r4>
biceps = distanceDimension(sp=xform('Mus_BI_O',q=1,t=1,ws=1),ep=xform('Mus_BI_I',q=1,t=1,ws=1))
def printAttributePerFrame(target):
    transformList = ls(tr=1)
    frame=findKeyframe(transformList, hi="both",which="first")
    lastframe=findKeyframe(transformList, hi="both",which="last")
    assert(frame-lastframe != 0), "No animation found!"
    previousframe = lastframe+1
    currentTime(frame, update=1, edit=1)
    vals = {}
    while frame<=lastframe:
        attr_value = easyFindAttribute(target)
        vals[frame]=attr_value
        print
        frame=findKeyframe(transformList, hi="both",which="next")
        if progressWindow(q=1, isCancelled=1):
            break
        if (frame-previousframe) == 0:
            break
        previousframe = frame
        progressWindow(edit=1, progress=frame, status=('Processing frame: '+str(frame)))
        currentTime(frame, update=1, edit=1)
    return vals


def easyFindAttribute(target):
    attr_value = target.get()
    print(attr_value)
    return attr_value

vals = printAttributePerFrame(biceps.distance)
strain = max(vals.values())- min(vals.values())
bicepsSIMM48r4 = (73.5-61)/73.5
bicepsMaya48r4 = (strain/max(vals.values()))
###</WEDS27FEB5PM:GET LENGTH CHANGE OF BICEPS IN 48r4>

###<THURS28FEB4PM: CALCULATE VALENCE VIA MOMENT>

unit_muscle_CB = muscle_CB/muscle_CB.length()
distT = dot(joint_muscle_BA, unit_muscle_CB)
projP = dt.Vector(xform(selProx['B'], q=1, t=1, ws=1)+(distT*unit_muscle_CB))
xyzMA = projP-dt.Vector(xform(selDist['A'], q=1, t=1, ws=1))
xyzMAlength = xyzMA.length()
round(dot(xyzMA,muscle_CB),6)==0  ##check for orthogonality between moment arm vector xyzMA and muscle_CB
#calculate decomposed moment arm and muscle vectors
xyzMA_decomp_prox = musclePlane(xyzMA, u_Prox)
xPlaneMuscle = musclePlane(muscle_CB, u_Prox)

#calculate torques around each axis
xTorque = findMoment(xyzMA_decomp_prox['xPlane'], xPlaneMuscle['xPlane'], u_Prox['xUnit'])
yTorque = findMoment(xyzMA_decomp_prox['yPlane'], xPlaneMuscle['yPlane'], u_Prox['yUnit'])
zTorque = findMoment(xyzMA_decomp_prox['zPlane'], xPlaneMuscle['zPlane'], u_Prox['zUnit'])

#calculate polarity
getSign = lambda a: (a>0) - (a<0)
xSign = getSign(xTorque)
ySign = getSign(yTorque)
zSign = getSign(zTorque)

#make simm-compatible moment arms
actualMAs = {}
actualMAs['xMA']=(prox_eMA['xPlane']*xSign*10)
actualMAs['yMA']=(prox_eMA['yPlane']*ySign*10)
actualMAs['zMA']=(prox_eMA['zPlane']*zSign*10)
actualMAs

def musclePlane(target, cs):
    planes = {'xPlane':0, 'yPlane':0, 'zPlane':0}
    for case in planes:
        result = projectionHero(case, target, cs)
        planes[case] = result
    return planes

def findMoment(ma, force, axis):
    moment_about_joint = cross(ma, force)
    moment_about_axis = dot(axis, moment_about_joint)
    return moment_about_axis




###</THURS28FEB4PM>

#run once:
#run each frame:
xyzUnits = updateCoordinateSystem(proxy_cs)
muscle_CB = makeVector(sel['C'],sel['B'])
joint_muscle_BA = makeVector(sel['B'],sel['A'])
armDecomposer(muscle_CB, joint_muscle_BA, xyzUnits)
