from pymel.all import *
import pymel.core.datatypes as dt


def makeMuscle(muscle_name):
    sel = getSelectionSet()
    muscle = makeMuscleCyl(sel,muscle_name)
    return(muscle)


### return currently selected objects as dictionary
    ## order: 1. neural joint center, 2. proximal muscle marker, 3. distal muscle marker
def getSelectionSet():
    sel = ls(sl=1)
    dict = {}
    dict['B'], dict['C'] = sel[0], sel[1]
    return dict

### make muscle cylinder for visualization
def makeMuscleCyl(sel,muscleName):
    mus_name = "mus_"+muscleName
    mus_length_name = "length_"+mus_name
    mus_mult_name = "multiply"+mus_name
    polyCylinder(r=0.05,h=2, sx=8, ax=[0,1,0], n=mus_name)
    assignColor(mus_name, 'red', 0.75)
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


### check to see if muscle material exists. if yes, do nothing. if not, make one.
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
