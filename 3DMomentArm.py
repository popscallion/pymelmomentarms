###
### This is a test script for calculating the perpendicular distance between a point and a line (measure tool) in 3D
###
import maya.cmds as cmds #loads in native Maya MEL command library
import maya.api.OpenMaya as om #loads in OpenMaya API (Maya C++ via Python wrapper)

## Function: Returns set of selected locator names so they can be passed as variables
def defineSelectionSet():
    activeSelection = cmds.ls(sl=1, fl=1) #sl=1 lists currently selected objects, fl=1 flattens selection to identify individual components
    return activeSelection

## Function: Makes a new locator and stores its coordinates in testLocator_xyz
def makeTestLocator(x0,y0,z0,name):
    testLocator=cmds.spaceLocator(name=name,position=[0,0,0])
    cmds.move(x0,y0,z0,testLocator)
    testLocator_xyz=om.MVector(cmds.xform(testLocator, query=True, translation=True, worldSpace=True))
    print "test locator's coordinates are: "+name+":"+str(testLocator_xyz)
    return testLocator_xyz
    
## Function: Makes a new measure tool and stores its locators in newLocators
def makeMeasureTool(musclename,x1,y1,z1,loc1name,x2,y2,z2,loc2name,noNewLocators=True):
    scene_before = cmds.ls(l=True, transforms=True)
    cmds.distanceDimension(sp=[x1,y1,z1],ep=[x2,y2,z2])
    scene_after = cmds.ls(l=True, transforms=True)    #gets 'after' list of scene objects 
    newMeasureObjects = list(set(scene_after).difference(scene_before))    #gets new objects in scene (measure tool and locators)
    #print "New Measure Tool objects are: "+str(newMeasureObjects)
    newMeasureObjects[0]=cmds.rename(newMeasureObjects[0],musclename)
    
    if noNewLocators: #doesn't try to make new locators if optional flag noNewLocators is true
        #print "newMeasureObjects are:"+str(newMeasureObjects)
        return newMeasureObjects[0]

    else:
        newLocators = [newMeasureObjects[-1],newMeasureObjects[-2]]    #grabs locators from list
        print "newLocators are:"+str(newLocators)
        newLocators[0]=cmds.rename(newLocators[0],loc1name)
        newLocators[1]=cmds.rename(newLocators[1],loc2name)
        return newLocators
        
    
## Function: Makes a new measure tool from 2 existing locators
def makeMeasureToolFromExisting(musclename,loc1,loc2):
    if type(loc1) is list: #checks to see if locators are in list form vs vector 
        makeMeasureTool(musclename,loc1xyz[0],loc1xyz[1],loc1xyz[2],'null1',loc2xyz[0],loc2xyz[1],loc2xyz[2],'null2',noNewLocators=True) #new locators can have name "null" since Maya will not make duplicate locators at existing coordinates

    else:
        loc1xyz = cmds.xform(loc1, query=True, translation=True, worldSpace=True)
        print loc1xyz
        loc2xyz = cmds.xform(loc2, query=True, translation=True, worldSpace=True)
        print loc2xyz
        makeMeasureTool(musclename,loc1xyz[0],loc1xyz[1],loc1xyz[2],'null1',loc2xyz[0],loc2xyz[1],loc2xyz[2],'null2',noNewLocators=True) #new locators can have name "null" since Maya will not make duplicate locators at existing coordinates


## Function: Returns the xyz coordinates of the measure tool's locators
def measureToolQuery(selection):
    measureToolCoordinates = []    #create empty list
    measureToolPosVectors = []
    for i in selection:
        print i
        testLocator_xyz=cmds.xform(i, query=True, translation=True)
        #print "measure tool "+str(i[1:])+"'s coordinates are: "+str(testLocator_xyz)
        measureToolCoordinates.append(testLocator_xyz)    #add new coordinates to empty list created earlier
        measureLocator_posVector=om.MVector(testLocator_xyz)    #converts coordinates from list of floats to position vector using MVector class in OpenMaya API
        measureToolPosVectors.append(measureLocator_posVector)
    return measureToolPosVectors    #multiple values are returned from this function, and may be accessed by calling the function on the same number of variables (in this case, 2)

##Function: Calculates the location of projected point P from point A on line BC
def projectionOnLine(A,B,C):
    vectBA=A-B    #vector math is supported by the OpenMaya MVector class
    vectBC=C-B
    unitBC=vectBC/om.MVector.length(vectBC)    #get unit vector for BC by dividing BC vector by its scalar magnitude
    distT=vectBA*unitBC    #get distance T from point B to projection P of point A on vector BC
    projectionP=B+distT*unitBC    #get projection P of point A on vector BC as sum of postion B and distance T
    return list(projectionP)
    
##Function: Calculates the perpendicular distance from a point A to a line between points B and C
def perpendicularDistance(A,B,C,name):
    vectBA=A-B    #vector math is supported by the OpenMaya MVector class
    vectBC=C-B
    unitBC=vectBC/om.MVector.length(vectBC)    #get unit vector for BC by dividing BC vector by its scalar magnitude
    distT=vectBA*unitBC    #get distance T from point B to projection P of point A on vector BC
    projectionP=B+distT*unitBC    #get projection P of point A on vector BC as sum of postion B and distance T
    vectAP=projectionP-A
    distanceResult=om.MVector.length(vectAP)
    print "the perpendicular distance is: "+str(distanceResult)
    #projectionPpos=list(projectionP)
    makeMeasureTool("null", A[0],A[1],A[2],"momentArm_"+name,projectionP[0],projectionP[1],projectionP[2],"projectionP_"+name,noNewLocators=False)
    return distanceResult

##Function: 3D Moment arm given joint center and two muscle markers
def calculateMomentArm(selection,muscleName):
    #get xyz of jointcenter -> 
    jointCenter_xyz = om.MVector(cmds.xform(selection[0], query=True, translation=True, worldSpace=True))
    loc1_xyz = om.MVector(cmds.xform(selection[1], query=True, translation=True, worldSpace=True))
    loc2_xyz = om.MVector(cmds.xform(selection[2], query=True, translation=True, worldSpace=True))
    #make muscle measuretool ->
    makeMeasureToolFromExisting(muscleName,selection[1],selection[2])
    #make perp measuretool
    perpendicularDistance(jointCenter_xyz,loc1_xyz,loc2_xyz,muscleName)

##makes test muscle    
#calculateMomentArm(defineSelectionSet(),'testMuscle')    

##test case 1: coplanar, perpendicular to grid
#A_xyz=makeTestLocator(0,10,0,"A")
#newLocators=makeMeasureTool(-10,0,0,"B",10,0,0,"C")
#B_xyz,C_xyz=measureToolQuery()
#distanceResult=perpendicularDistance(A_xyz,B_xyz,C_xyz)

##test case 2: collinear on arbitrary line
#A_xyz=makeTestLocator(7,8.5,15.5,"A")
#newLocators=makeMeasureTool(1,2,11,"B",13,15,20,"C")
#B_xyz,C_xyz=measureToolQuery()
#distanceResult=perpendicularDistance(A_xyz,B_xyz,C_xyz)

##test case 3: non-collinear on arbitrary plane
#A_xyz=makeTestLocator(8,9,12,"A")
#newLocators=makeMeasureTool("MuscleName",1,12,11,"B",13,15,20,"C")
#B_xyz,C_xyz=measureToolQuery()
#distanceResult=perpendicularDistance(A_xyz,B_xyz,C_xyz,"MuscleNameMomentArm")

##stuff to make this a moment arm thing. TEST_glenohumeralcenter is at joint center
#cmds.xform("TEST_glenohumeralcenter", query=True, translation=True, worldSpace=True)    #worldSpace flag uses world's frame of reference, not parent's frame

#makeMeasureToolFromExisting("fakemuscle",defineSelectionSet())
#glenohumeralCenter=makeTestLocator(7,8.5,15.5,"A")

#calculateMomentArm(defineSelectionSet(),"testMuscle")

###next steps: handle keyframes
    #get all frames, get current frame
    #check to see if locators are keyed at current frame
    #if yes, go back to frame 1 and for each keyed frame, recalculate projection P, move projection P and make new key
    #return to current frame
def keyFrameHelper(selection): #assumes jointcenter, muscle marker 1, muscle marker 2,projectionP selection order
    initialFrame=cmds.currentTime(query=True) #get current frame 
    selection0TXkeyCount=cmds.keyframe(str(selection[0])+'.translateX', query=True,keyframeCount=True)    #get keyframe count for current selection
    if selection0TXkeyCount > 0:
        cmds.currentTime(1)
        for i in range(selection0TXkeyCount):
            cmds.currentTime(i)
            jointcenterVect=om.MVector(cmds.xform(selection[0], query=True, translation=True, worldSpace=True))
            muscle1Vect=om.MVector(cmds.xform(selection[1], query=True, translation=True, worldSpace=True))
            muscle2Vect=om.MVector(cmds.xform(selection[2], query=True, translation=True, worldSpace=True))
            projectionP_xyz=cmds.xform(selection[3], query=True, translation=True, worldSpace=True)
            projectionP=projectionOnLine(jointcenterVect,muscle1Vect,muscle2Vect)
            cmds.move(projectionP[0],projectionP[1],projectionP[2],selection[3])
            cmds.setKeyframe(selection[3],attribute='translate')
            cmds.refresh()###still doesn't refresh when playing after the fact
            ###!!!need way to handle first frame, when projectionP doesnt exist yet
            ###!!!measure tool only updates on viewport refresh
            
            

keyFrameHelper(defineSelectionSet())
