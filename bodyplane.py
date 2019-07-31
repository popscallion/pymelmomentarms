from pymel.all import *
import pymel.core.datatypes as dt

class bodyPlaneMaker():
    def __init__(self):
        if window('bodyplane_window',exists=1) == True:
                deleteUI('bodyplane_window')
        mainWindow = window('bodyplane_window', title='Create Body Plane',rtf=1, w=300, h=160)
        frameLayout(label='Input')
        columnLayout(columnOffset=('both',20))
        text(label='')
        text(label='Select (in order):')
        text(label=' 1) dorsal midline marker')
        text(label=' 2) ventral midline marker (cranial)')
        text(label=' 3) ventral midline marker (caudal)')
        text(label='')
        button(label='Create Body Plane',command=self.run)
        text(label='')
        showWindow(mainWindow)


    def updatePlane(self):
        vec12 = self.makeVector(self.loc1,self.loc2)
        vec32 = self.makeVector(self.loc3,self.loc2)
        vec32_u = vec32/vec32.length()
        vec12scalproj32 = dot(vec12,vec32)/vec32.length()
        vec12vecproj32 = vec12scalproj32*(vec32/vec32.length())
        newpos = xform(self.loc1,q=1,t=1,ws=1)+vec12vecproj32
        xform(self.loc4, t=newpos)
        setKeyframe(self.loc4)
        vec42 = self.makeVector(self.loc4,self.loc2)
        vec42_u = vec42/vec42.length()
        vec_orthogonal_u = cross(vec42_u,vec32_u)
        mat_jcs = dt.Matrix(
                            vec32_u.x, vec32_u.y, vec32_u.z, 0,
                            -vec_orthogonal_u.x, -vec_orthogonal_u.y, -vec_orthogonal_u.z, 0,
                            vec42_u.x, vec42_u.y, vec42_u.z, 0,
                            self.loc2.attr('translateX').get(),self.loc2.attr('translateY').get(),self.loc2.attr('translateZ').get(),1)
        xform(self.plane1,m=mat_jcs)
        setKeyframe(self.plane1)
        loc3_coords = xform(self.loc3,q=1,t=1,ws=1)
        move(loc3_coords[0],loc3_coords[1],loc3_coords[2],self.vtx2)
        loc4_coords = xform(self.loc4,q=1,t=1,ws=1)
        move(loc4_coords[0],loc4_coords[1],loc4_coords[2],self.vtx3)
        setKeyframe(self.vtx1,self.vtx2,self.vtx3)

    def makeVector(self,locA,locB):
        posA = dt.Vector(xform(locA,q=1,t=1,ws=1))
        posB = dt.Vector(xform(locB,q=1,t=1,ws=1))
        vectAB = posB - posA
        return vectAB

    def run(self,sel):
        #### setup: select locators (dorsal, ventral cranial, ventral caudal)
        self.loc1,self.loc2,self.loc3 = ls(sl=1)
        self.loc4 = spaceLocator(n="virtual_locator")
        #### setup: make a triangular plane and assign each vertex a name
        self.plane1 = polyPlane(sx=1, sy=1, w=1, h=1, n="bodyplane")
        delete(self.plane1[0].vtx[0])
        self.vtx1,self.vtx2,self.vtx3 = self.plane1[0].vtx[0], self.plane1[0].vtx[1], self.plane1[0].vtx[2]
        xform(self.vtx1, t=[0,0,0])
        xform(self.vtx2, t=[0,0,-1])
        xform(self.vtx3, t=[-1,0,0])
        transformList = ls(tr=1)
        frame=findKeyframe(transformList, hi="both",which="first")
        lastframe=findKeyframe(transformList, hi="both",which="last")
        assert(frame-lastframe != 0), "No animation found!"
        previousframe = lastframe+1
        currentTime(frame, update=1, edit=1)
        progressWindow(title="keying...", min=frame, max=lastframe, progress=frame, status = "Crunching frame: "+str(frame), isInterruptable=True)
        while frame<=lastframe:
            frame=findKeyframe(transformList, hi="both",which="next")
            if progressWindow(q=1, isCancelled=1):
                break
            if (frame-previousframe) == 0:
                break
            previousframe = frame
            self.updatePlane()

            progressWindow(edit=1, progress=frame, status=('Processing frame: '+str(frame)))
            currentTime(frame, update=1, edit=1)
        progressWindow(endProgress=1)
        deleteUI(mainWindow)
