from pymel.all import *
import pymel.core.datatypes as dt


### all together now
def doTheThing(muscle_name):
    #0: getting input
    muscle_name = textField('muscle_name_field', q=1,text=1)
    init_frame = textField('frame_number_field', q=1,text=1)
    #1: setting up
    currentTime(init_frame)
    sel = getSelectionSet()   
    muscle_vec = makeArrow(sel['C'], sel['B'], 'mus_'+muscle_name, 'yellow')
    addAttr(muscle_vec, shortName='Xma', longName='xMomentArm', defaultValue=0.0, keyable=1)
    addAttr(muscle_vec, shortName='Yma', longName='yMomentArm', defaultValue=0.0, keyable=1)
    addAttr(muscle_vec, shortName='Zma', longName='zMomentArm', defaultValue=0.0, keyable=1)
    proxy = makeJointProxy(sel['A'], muscle_name)
    momentarm_vecs = armDecomposer(sel, proxy)
    momentarm_objs = initializeMAArrows(muscle_name, momentarm_vecs, proxy)
    #2: update every frame
    keyFrameHelper(sel, proxy, muscle_vec, momentarm_objs)


### get muscle name from user input
def getMuscleNameUI():
    if window('muscle_window',exists=1) == True:
        deleteUI('muscle_window')
    mainWindow = window('muscle_window', title='Create Muscle',rtf=1, w=300, h=250)
    frameLayout(label='Muscle Name Input')
    columnLayout(columnOffset=('both',20))
    text(label=' ')
    text(label='Select (in order):')
    text(label=' 1) Joint Coordinate System (JCS)')
    text(label=' 2) Muscle Marker 1 (proximal)')
    text(label=' 3) Muscle Marker 2 (distal)')
    text(label='')
    text(label='Muscle Name')
    textField('muscle_name_field', text='Muscle Name')
    text(label='')
    text(label='Pick any frame with real data to initialize:')
    textField('frame_number_field', text='Frame')
    text(label='')
    button(label='Create Muscle',command=doTheThing)
    showWindow(mainWindow)


### return currently selected objects as dictionary
    ## order: 1. neural joint center, 2. proximal muscle marker, 3. distal muscle marker
def getSelectionSet():
    sel = ls(sl=1)
    assert(0<len(sel)<=3),"Please select (in order): 1. neutral joint center, 2. proximal muscle marker, 3. distal muscle marker"
    dict = {}
    dict['A'], dict['B'], dict['C'] = sel[0], sel[1], sel[2]
    return dict
    

### make a vector from two locators
def makeVector(locA,locB):
    posA = dt.Vector(xform(locA,q=1,t=1,ws=1))
    posB = dt.Vector(xform(locB,q=1,t=1,ws=1))
    vectAB = posB - posA
    return vectAB


### make vector arrow for visualization
def makeArrow(sp, ep, obj_name, tint='yellow', alpha=0.75):
    ## sp: start point as dt.Vector, 
    ## ep: end point as dt.Vector,
    ## name: name of vector object,
    ## mode: mode to run in ('vec' for vector or 'mus' for muscle)  
    # set up variables   
    size = makeVector(sp,ep).length()   
    obj_length_name = "length_"+obj_name
    obj_scale_name = "sf_"+obj_name
    obj_mult_name = "multiply_"+obj_name
    cyl_height = size*0.8
    cone_height = size*0.2
    cyl_radius = size*0.02
    cone_radius = size*0.04
    # make primitives and combine into arrow
    cyl_primitive = polyCylinder(r=cyl_radius,h=cyl_height, ax=[1,0,0], n='cyl_'+obj_name)
    cone_primitive = polyCone(r=cone_radius,h=cone_height, ax=[1,0,0], n='cone_'+obj_name)
    xform(cone_primitive, piv=[cone_height/2,0,0], t=[(cyl_height+cone_height)/2,0,0] )
    arrow = polyUnite(cyl_primitive, cone_primitive, n=obj_name, ch=0)
    assignColor(arrow, tint, alpha)
    xform(arrow, piv=[(cyl_height*-1)/2,0,0])
    # constrain arrow to follow ep
    pointConstraint(sp,arrow)
    aimConstraint(ep,arrow,aimVector=[1,0,0],worldUpType="vector")
    hide(listRelatives(arrow)[-2:])
    # scale arrow by distance between sp and ep,
    # parentMatrix distanceBetween node always gets worldspace distances, regardless of parenting
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
    # store vector magnitude as "Vector Length" attribute
    #addAttr(arrow[0], shortName='lv', longName='VectorLength', defaultValue=0.0, keyable=1)
    #connectAttr(obj_length_name+".distance", arrow[0].lv, f=1)
    return arrow
    
    
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


### make proxy coordinate system with unit axis locators
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
    return proxy


### initialize XYZ moment arm arrows
def initializeMAArrows(muscle_name, momentarm_vecs, proxy):
    result = {}
    for case in ['x','y','z']:       
        if case == 'x':
            tint = 'red'
        elif case == 'y':
            tint = 'green'
        elif case == 'z':
            tint = 'blue'
        locator_name = 'lc_'+ case
        arrow_name = 'ma_'+ case
        which_plane = case + 'Plane'
        #lc = spaceLocator(n = locator_name + '_' + muscle_name)
        #pointConstraint(proxy['joint'], lc)
        #orientConstraint(proxy['joint'], lc)
        #xform(lc, t=momentarm_vecs[which_plane]['ma_scaled'], r=1, ws=1)
        lc = spaceLocator(n = locator_name + '_' + muscle_name)
        matchTransform(lc, proxy['joint'], pos=1, rot=1)
        parent(lc, proxy['joint'])
        xform(lc, t=momentarm_vecs[which_plane]['ma_unscaled'], r=1, ws=1)
        ma_arrow = makeArrow(proxy['joint'], lc, arrow_name+'_'+muscle_name, tint, 0.75) 
        result[locator_name]=lc
        result[arrow_name]=ma_arrow
        hide(listRelatives(lc)[-2:])
        setKeyframe(lc)    
    return result


### update moment arms and store values as custom attributes
def frameUpdate(sel, proxy, muscle_vec, momentarm_objs, momentarm_vecs_old):
    loc = {}
    loc['x'], loc['y'], loc['z'] = momentarm_objs['lc_x'], momentarm_objs['lc_y'], momentarm_objs['lc_z']
    # shift moment arm locators to match new frame
    momentarm_vecs_new = armDecomposer(sel, proxy)
    xtrans = momentarm_vecs_new['xPlane']['ma_unscaled'] - momentarm_vecs_old['xPlane']['ma_unscaled']
    ytrans = momentarm_vecs_new['yPlane']['ma_unscaled'] - momentarm_vecs_old['yPlane']['ma_unscaled']
    ztrans = momentarm_vecs_new['zPlane']['ma_unscaled'] - momentarm_vecs_old['zPlane']['ma_unscaled']
    xform(loc['x'], t=xtrans, r=1, ws=1)
    xform(loc['y'], t=ytrans, r=1, ws=1)
    xform(loc['z'], t=ztrans, r=1, ws=1)
    setKeyframe(loc['x'],loc['y'],loc['z']) 
    # key moment arm attributes of muscle object
    setAttr(muscle_vec.Xma, momentarm_vecs_new['xPlane']['ma_actual'])
    setAttr(muscle_vec.Yma, momentarm_vecs_new['yPlane']['ma_actual'])
    setAttr(muscle_vec.Zma, momentarm_vecs_new['zPlane']['ma_actual'])
    setKeyframe(muscle_vec.Xma, v=momentarm_vecs_new['xPlane']['ma_actual'], at='Xma')
    setKeyframe(muscle_vec.Yma, v=momentarm_vecs_new['yPlane']['ma_actual'], at='Yma')
    setKeyframe(muscle_vec.Zma, v=momentarm_vecs_new['zPlane']['ma_actual'], at='Zma')
    return momentarm_vecs_new


### step through frames, inspiration from David Baier's outputRelMotion shelf tool
def keyFrameHelper(sel, proxy, muscle_vec, momentarm_objs):
    transformList = ls(tr=1)
    frame=findKeyframe(transformList, hi="both",which="first")
    lastframe=findKeyframe(transformList, hi="both",which="last")
    assert(frame-lastframe != 0), "No animation found!"
    previousframe = lastframe+1
    currentTime(frame, update=1, edit=1)
    progressWindow(title="Grabbing moment arms...", min=frame, max=lastframe, progress=frame, status = "Crunching frame: "+str(frame), isInterruptable=True)
    momentarm_vecs_old = armDecomposer(sel, proxy)
    while frame<=lastframe:
        momentarm_vecs_new = frameUpdate(sel, proxy, muscle_vec[0], momentarm_objs, momentarm_vecs_old)
        frame=findKeyframe(transformList, hi="both",which="next")
        if progressWindow(q=1, isCancelled=1):
            break
        if (frame-previousframe) == 0:
            break
        previousframe = frame
        momentarm_vecs_old = momentarm_vecs_new
        progressWindow(edit=1, progress=frame, status=('Processing frame: '+str(frame)))
        currentTime(frame, update=1, edit=1)
    progressWindow(endProgress=1)

    
### recalculate world space vectors for joint proxy
def updateCoordinateSystem(proxy):
    xUnit = makeVector(proxy['joint'],proxy['x'])
    yUnit = makeVector(proxy['joint'],proxy['y'])
    zUnit = makeVector(proxy['joint'],proxy['z'])
    xyzUnits = {'xUnit':xUnit,'yUnit':yUnit,'zUnit':zUnit}
    return(xyzUnits)


### dot muscle vector with each axis to get component in plane
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
    return result[mode]
    
    
### cross vectors to find area of parallelogram as magnitude of orthogonal vector
def armDecomposer(sel, proxy):
    cs = updateCoordinateSystem(proxy)
    muscle_CB = makeVector(sel['C'],sel['B'])
    u_muscle_CB = muscle_CB/muscle_CB.length()
    joint_muscle_BA = makeVector(sel['B'],sel['A'])
    result = {}
    for case in ['xPlane','yPlane','zPlane']:
        if case == 'xPlane':
            axis = 'xUnit'
        elif case == 'yPlane':
            axis = 'yUnit'
        elif case == 'zPlane':
            axis = 'zUnit'
        result[case]={}
        muscle_CB_proj = projectionHero(case, muscle_CB, cs)
        u_muscle_CB_proj = muscle_CB_proj/muscle_CB_proj.length()
        joint_muscle_BA_proj = projectionHero(case, joint_muscle_BA, cs)
        dist_T_proj = dot(joint_muscle_BA_proj, u_muscle_CB_proj)
        pos_P_xyz = dt.Vector(xform(sel['B'], q=1, t=1, ws=1)+(dist_T_proj*u_muscle_CB_proj))
        ma_unprojected = dt.Vector(xform(proxy['joint'], q=1, t=1, ws=1))-pos_P_xyz
        ma_unscaled = (-1*projectionHero(case, ma_unprojected, cs)) 
        #print('unscaled MA for '+str(case)+' is '+str(ma_unscaled.length()))
        #<LEGACY PARALLELOGRAM METHOD, RETURNS SCALAR>
        #joint_muscle_BA_proj_para = projectionHero(case, joint_muscle_BA, cs)
        #parallelogram = cross(muscle_CB_proj,joint_muscle_BA_proj)
        #ma_unscaled_para = round(parallelogram.length()/muscle_CB_proj.length(),5)
        #</LEGACY PARALLELOGRAM METHOD>
        scale_factor = muscle_CB_proj.length()/muscle_CB.length()
        ma_scaled = ma_unscaled*scale_factor
        moment_about_joint = cross(ma_scaled, muscle_CB_proj)
        moment_about_axis = dot(cs[axis], moment_about_joint)
        getSign = lambda a: (a>0) - (a<0)
        ma_sign = getSign(moment_about_axis)
        ma_actual = ma_scaled.length()*ma_sign
        #print('actual MA for '+str(case)+' is '+str(ma_actual))
        result[case]['ma_unscaled'] = ma_unscaled
        result[case]['ma_scaled'] = ma_scaled
        result[case]['scale_factor'] = scale_factor
        result[case]['ma_actual'] = ma_actual
    return result
