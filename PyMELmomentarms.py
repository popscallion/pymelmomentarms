### calculate muscle moment arms w/r/t a defined joint center
### JOINT CENTER MUST BE POINT+AIM CONSTRAINED, PARENTING IS DEATH. OR FREEZE TRANSFORMATIONS
from pymel.all import *
import pymel.core.datatypes as dt

def getMuscleName():
    result = promptDialog(
                    title='Create Muscle',
                    message='Enter name of muscle',
                    button=['OK'],
                    defaultButton='OK',
                    cancelButton='Cancel',
                    dismissString='Cancel')
    assert(result == 'OK'), "Muscle needs a name!"
    muscname = promptDialog(query=True, text=True)
    return muscname

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
    posA = A.translate.get()
    posB = B.translate.get()
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
    
### make projection point for visualization
def makeProjectionBall(sel, muscleName):
    proj_point = "proj_"+muscleName
    polySphere(r=.1, sx=8, sy=8, n=proj_point)
    assignMuscleColor(proj_point)

### calculate orthogonal distance between joint center and muscle vector
def calcMomentArm(sel, muscleName):
    A, B, C = sel[0], sel[1], sel[2]
    vectBA = makeVector(B,A)
    vectBC = makeVector(B,C)
    unitBC = vectBC/vectBC.length()
    distT = vectBA * unitBC #get distance T of point B to projection P of point A on vector BC
    projectionP = B.translate.get() + distT * unitBC #get projection P as sum of position B and distance T along vector BC
    vectAP = projectionP-A.translate.get()
    distanceResult = vectAP.length()
    dict = {'vectAP': vectAP, 'distanceResult': distanceResult, 'distT':distT, 'projectionP':projectionP}
    return(dict)

### go through and key each frame of animation, inspiration from David Baier's outputRelMotion shelf tool
#def keyframeHelper(sel):
    #transformList = ls(tr=1)
    #frame=findKeyframe(transformList, hi="both",which="first")
   # lastframe=findKeyframe(transformList, hi="both",which="last")
    #assert(frame-lastframe != 0), "No animation found!"
   # previousframe = lastframe+1
   # progressWindow(title="Grabbing moment arms...", min=frame, max=lastframe, progress=frame, status = "Crunching frame: "+str(frame), isInterruptable=True)
     #   while frame<=lastframe:
    

### transform vector to local coordinate system
    
### LAST: once projection point and measure tools are made, constrain so they follow the shit around
    #spaceLocator000, then translate

muscleName = getMuscleName()
testsel = getSelectionSet()
makeMuscleCyl(testsel,muscleName)
calcMomentArm(testsel,muscleName)

distanceDimension(sp=testsel[0].translate.get(),ep=result['projectionP'])
makeProjectionBall(testsel, muscleName)
    
    