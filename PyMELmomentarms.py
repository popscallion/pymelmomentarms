### calculate muscle moment arms w/r/t a defined joint center
### JOINT CENTER MUST BE POINT+AIM CONSTRAINED, PARENTING IS DEATH. OR FREEZE TRANSFORMATIONS
from pymel.all import *
import pymel.core.datatypes as dt

def getMuscleName():
    if window('muscleWindow',exists=1) == True:
        deleteUI('muscleWindow')
    winHeight=160
    winWidth=320
    mainWindow = window('muscleWindow', title='Create Muscle', menuBar=1,
                        maximizeButton=0, minimizeButton=1, sizeable=1,resizeToFitChildren=0)
    frameLayout(label='Muscle Name Input',height=winHeight,width=winWidth, mw=5, mh=5)
    showWindow(mainWindow)




#def getMuscleName():
    #result = promptDialog(
                    #title='Create Muscle',
                    #message='Enter name of muscle',
                    #button=['OK'],
                    #defaultButton='OK',
                    #cancelButton='Cancel',
                    #dismissString='Cancel')
   # assert(result == 'OK'), "Muscle needs a name!"
   # muscname = promptDialog(query=True, text=True)
   # return muscname

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
    assert(0<len(sel)<=3),"Please select (in order): 1. joint center, 2. muscle marker 1, 3. muscle marker 2"
    return sel

### make muscle vector with selected locators
def makeVector(A,B):   
    posA = dt.Vector(xform(A,q=1,translation=1,worldSpace=1))
    posB = dt.Vector(xform(B,q=1,translation=1,worldSpace=1))   
    #posA = A.translate.get() OLD: broken by parent constraints
    #posB = B.translate.get() OLD: broken by parent constraints
    vectAB = posB - posA  
    # make measure tool to visualize muscle
    ## distanceDimension(sp=posA,ep=posB)
    # check to see if the vector was correctly calculated
    ## distanceDimension(sp=posB,ep=(VectAB[0]+posB[0],VectAB[1]+posB[1],VectAB[2]+posB[2]))
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
    pointConstraint(sel[1],mus_name)
    aimConstraint(sel[2],mus_name,aimVector=[0,1,0],worldUpType="vector")
    createNode("distanceBetween", n=mus_length_name)
    connectAttr(sel[1]+".translate",mus_length_name+'.point1',f=1)
    connectAttr(sel[2]+".translate",mus_length_name+'.point2',f=1)
    createNode("multiplyDivide", n=mus_mult_name)
    connectAttr(mus_length_name+".distance",mus_mult_name+".input1Y",f=1)
    connectAttr(mus_mult_name+".outputY",mus_name+".scaleY",f=1)
    setAttr(mus_mult_name+".input2Y",0.5)
    

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
    addAttr(MA_muscle, shortName='lMA', longName='MomentArmLength', defaultValue=0.0)
    addAttr(MA_muscle, shortName='iMD', longName='IntermarkerDistance', defaultValue=0.0)
    addAttr(MA_muscle, shortName='xMA', longName='xMomentArm', defaultValue=0.0)
    addAttr(MA_muscle, shortName='yMA', longName='yMomentArm', defaultValue=0.0)
    addAttr(MA_muscle, shortName='zMA', longName='zMomentArm', defaultValue=0.0)
    matchTransform(MA_muscle,sel[0],rot=1, pos=1)
    parent(MA_muscle, sel[0])
    print("made "+muscleName+"'s moment arm locator!")
    return(MA_muscle)
    
### get and store moment arm vector in local joint space
def keyMA(sel,muscleName,MA_muscle):
    #run calcMAVector to get projectionP
    result = calcMAVector(sel)
    xform(MA_muscle,t=result['projectionP'],ws=1)
    #get translation of MA locator   
    MA_xyz = xform(MA_muscle,q=1,t=1,objectSpace=1)
    #calculate projection of MA into planes of action
    xMA = sqrt(MA_xyz[1]**2+MA_xyz[2]**2)
    yMA = sqrt(MA_xyz[0]**2+MA_xyz[2]**2)
    zMA = sqrt(MA_xyz[0]**2+MA_xyz[1]**2)
    dict = {'xMA':xMA,'yMA':yMA,'zMA':zMA}
    #store MA projections as attributes in MA locator, then key
    setAttr(MA_muscle.xMA, xMA)
    setAttr(MA_muscle.yMA, yMA)
    setAttr(MA_muscle.zMA, zMA)
    setAttr(MA_muscle.lMA, result['distanceResult'])
    setAttr(MA_muscle.iMD, result['interMarkerDistance'])
    setKeyframe(MA_muscle, v=xMA, at='xMA')
    setKeyframe(MA_muscle, v=yMA, at='yMA')
    setKeyframe(MA_muscle, v=zMA, at='zMA')
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

muscleName = getMuscleName()
sel = getSelectionSet()
makeMuscleCyl(sel,muscleName)
keyFrameHelper(sel,muscleName)


    
    