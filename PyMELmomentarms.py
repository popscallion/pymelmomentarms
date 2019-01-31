### calculate muscle moment arms w/r/t a defined joint center
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

### check to see if muscle material exists. if yes, do nothing. if no, make one.    
def createMuscleColor():
    if objExists("muscleMat"):
        print("Muscle material already exists!")
        muscleMaterial = "muscleMat"
    else:
        muscleMaterial = shadingNode("lambert", asShader=True, name="muscleMat")
        muscleMaterial.color.set(0.604, 0.036, 0.014)
        muscleMaterial.transparency.set(0.5, 0.5, 0.5)
    return(muscleMaterial)

### assign muscle material to selected object (passed as argument)
def assignMuscleColor(target):
    sets(createMuscleColor()+'SG', edit=True, forceElement=target)

### get currently selected objects in order: 
### 1. joint center, 2. muscle marker #1, 3. muscle marker #2
def getSelectionSet():
    sel = ls(sl=1)
    assert(0<len(sel)<=3),"Please select (in order): 1. joint center, 2. muscle marker 1, 3. muscle marker 2"
    return sel

### make muscle vector with selected locators
def makeMuscleVector(sel):
    posMuscle1 = sel[1].translate.get()
    posMuscle2 = sel[2].translate.get()
    muscleVect = posMuscle1 - posMuscle2
    # make measure tool to visualize muscle
    ## distanceDimension(sp=posMuscle1,ep=posMuscle2)
    # check to see if the vector was correctly calculated
    ## distanceDimension(sp=posMuscle2,ep=(muscleVect[0]+posMuscle2[0],muscleVect[1]+posMuscle2[1],muscleVect[2]+posMuscle2[2]))
    return muscleVect

### make muscle cylinder to visualize
def makeMuscleCyl(sel,muscleName):
    mus_name = "mus_"+muscleName
    polyCylinder(r=0.05,h=2, sx=8, ax=[0,1,0], n=mus_name)
    assignMuscleColor(mus_name)

    