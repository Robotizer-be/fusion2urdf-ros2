# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: syuntoku
"""

import adsk, re
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils
from adsk.fusion import Component, JointGeometry
from adsk.core import Matrix3D
class Joint:
    def __init__(self, name, xyz, rpy, axis, parent, child, joint_type, upper_limit, lower_limit):
        """
        Attributes
        ----------
            name: str
                name of the joint
        type: str
            type of the joint(ex: rev)
        xyz: [x, y, z]
            coordinate of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.rpy = rpy
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'
        
    def make_joint_xml(self):
        """
        Generate the joint_xml and hold it by self.joint_xml
        """
        joint = Element('joint')
        joint.attrib = {'name': "${prefix}" + self.name, 'type':self.type}
        
        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy': ' '.join([str(_) for _ in self.rpy])}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link': "${prefix}" + self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link': "${prefix}" + self.child}
        if self.type == 'revolute' or self.type == 'continuous' or self.type == 'prismatic':        
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        if self.type == 'revolute' or self.type == 'prismatic':
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': '100', 'velocity': '100'}
            
        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_transmission_xml(self):
        """
        Generate the tran_xml and hold it by self.tran_xml
        
        
        Notes
        -----------
        mechanicalTransmission: 1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface        
        """        
        
        tran = Element('transmission')
        tran.attrib = {'name': "${prefix}" + self.name + '_tran'}
        
        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'
        
        joint = SubElement(tran, 'joint')
        joint.attrib = {'name':self.name}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'hardware_interface/EffortJointInterface'
        
        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name':self.name + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'hardware_interface/EffortJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'
        
        self.tran_xml = "\n".join(utils.prettify(tran).split("\n")[1:])

def make_joint_dict(joint, links_dict, msg, all_joints):
    joint_type_list = [
    'fixed', 'revolute', 'prismatic', 'Cylinderical',
    'PinSlot', 'Planner', 'Ball']  # these are the names in urdf
    joint_dict = {}
    joint_type = joint_type_list[joint.jointMotion.jointType]
    joint_dict['type'] = joint_type
    
    # swhich by the type of the joint
    joint_dict['axis'] = [0, 0, 0]
    joint_dict['upper_limit'] = 0.0
    joint_dict['lower_limit'] = 0.0
    
    # support  "Revolute", "Rigid" and "Slider"
    if joint_type == 'revolute':
        joint_dict['axis'] = [round(i, 6) for i in \
            joint.jointMotion.rotationAxisVector.asArray()] ## In Fusion, exported axis is normalized.
        max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
        min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled            
        if max_enabled and min_enabled:  
            joint_dict['upper_limit'] = round(joint.jointMotion.rotationLimits.maximumValue, 6)
            joint_dict['lower_limit'] = round(joint.jointMotion.rotationLimits.minimumValue, 6)
        elif max_enabled and not min_enabled:
            msg = joint.name + 'is not set its lower limit. Please set it and try again.'
            return None, None, msg
        elif not max_enabled and min_enabled:
            msg = joint.name + 'is not set its upper limit. Please set it and try again.'
            return None, None, msg
        else:  # if there is no angle limit
            joint_dict['type'] = 'continuous'
            
    elif joint_type == 'prismatic':
        joint_dict['axis'] = [round(i, 6) for i in \
            joint.jointMotion.slideDirectionVector.asArray()]  # Also normalized
        max_enabled = joint.jointMotion.slideLimits.isMaximumValueEnabled
        min_enabled = joint.jointMotion.slideLimits.isMinimumValueEnabled            
        if max_enabled and min_enabled:  
            joint_dict['upper_limit'] = round(joint.jointMotion.slideLimits.maximumValue/100, 6)
            joint_dict['lower_limit'] = round(joint.jointMotion.slideLimits.minimumValue/100, 6)
        elif max_enabled and not min_enabled:
            msg = joint.name + 'is not set its lower limit. Please set it and try again.'
            return None, None, msg
        elif not max_enabled and min_enabled:
            msg = joint.name + 'is not set its upper limit. Please set it and try again.'
            return None, None, msg
    # elif joint_type == 'fixed':
    #     pass
    
    if joint.occurrenceTwo is None: #.component.name == 'base_link':
        joint_dict['parent'] = 'base_link'
    else:
        joint_dict['parent'] = re.sub('[ :()]', '_', joint.occurrenceTwo.name.split(" ", 1)[0].split(":", 1)[0])
    joint_dict['child'] = re.sub('[ :()]', '_', joint.occurrenceOne.name.split(" ", 1)[0].split(":", 1)[0])

    # There seem to be a problem with geometryOrOriginTwo. To calcualte the correct orogin of the generated stl files following approach was used.
    # https://forums.autodesk.com/t5/fusion-360-api-and-scripts/difference-of-geometryororiginone-and-geometryororiginonetwo/m-p/9837767
    # Thanks to Masaki Yamamoto!

    # Coordinate transformation by matrix
    # M: 4x4 transformation matrix
    # a: 3D vector
    def trans(M, a):
        ex = [M[0],M[4],M[8]]
        ey = [M[1],M[5],M[9]]
        ez = [M[2],M[6],M[10]]
        oo = [M[3],M[7],M[11]]
        b = [0, 0, 0]
        for i in range(3):
            b[i] = a[0]*ex[i]+a[1]*ey[i]+a[2]*ez[i]+oo[i]
        return(b)


    # Returns True if two arrays are element-wise equal within a tolerance
    def allclose(v1, v2, tol=1e-6):
        return( max([abs(a-b) for a,b in zip(v1, v2)]) < tol )

    # try:
    #     # Basic information
    #     xyz_from_one_to_joint = joint.geometryOrOriginOne.origin.asArray() # Relative Joint pos
    #     xyz_from_two_to_joint = joint.geometryOrOriginTwo.origin.asArray() # Relative Joint pos
    #     xyz_of_one            = joint.occurrenceOne.transform.translation.asArray() # Link origin
    #     xyz_of_two            = joint.occurrenceTwo.transform.translation.asArray() # Link origin
    #     M_two = joint.occurrenceTwo.transform.asArray() # Matrix as a 16 element array.
    #     M_one = joint.occurrenceOne.transform.asArray() # Matrix as a 16 element array.
    #     if joint_type == 'revolute':
    #         # Compose joint position
    #         case1 = allclose(xyz_from_two_to_joint, xyz_from_one_to_joint)
    #         case2 = allclose(xyz_from_two_to_joint, xyz_of_one)
    #         if case1 or case2:
    #             xyz_of_joint = xyz_from_two_to_joint
    #         else:
    #             xyz_of_joint = trans(M_two, xyz_from_two_to_joint)

    #         joint_dict['xyz'] = [round(i / 100.0, 6) for i in xyz_of_joint]  # converted to meter
    #     # elif joint_type == 'prismatic' or joint_type == "fixed":
    #     else:
    #         joint_dict['xyz'] = [round(i / 100.0, 6) for i in xyz_from_one_to_joint]  # converted to meter
    #     # else:
    #     #     joint_dict['xyz'] = [round((one - two) / 100.0, 6) for one, two in zip(xyz_from_one_to_joint ,xyz_from_two_to_joint)]  # converted to meter
            

    # except Exception as e:
    #     # try:
    #     #     if type(joint.geometryOrOriginTwo)==adsk.fusion.JointOrigin:
    #     #         data = joint.geometryOrOriginTwo.geometry.origin.asArray()
    #     #     else:
    #     #         data = joint.geometryOrOriginTwo.origin.asArray()
    #     #     joint_dict['xyz'] = [round(i / 100.0, 6) for i in data]  # converted to meter
    #     # except:
    #         msg = joint.name + " doesn't have joint origin. Please set it and run again." + str(e)
    #         break
    
    
    try:
        xyz_from_one_to_joint = joint.geometryOrOriginOne.origin.asArray() # Relative Joint pos
        xyz_from_two_to_joint = joint.geometryOrOriginTwo.origin.asArray() # Relative Joint pos
        xyz_of_one            = joint.occurrenceOne.transform.translation.asArray() # Link origin
        xyz_of_two            = joint.occurrenceTwo.transform.translation.asArray() # Link origin
        M_two = joint.occurrenceTwo.transform.asArray() # Matrix as a 16 element array.
        

        # if joint_dict['child'] == 'divider':
        #     msg = f'{xyz_from_one_to_joint}, {xyz_from_two_to_joint}, {xyz_of_one}, {xyz_of_two}, {M_two}'
        #     break
        # xyz_of_joint = [one - two for one, two in zip(xyz_from_one_to_joint ,xyz_from_two_to_joint)] 
        
        # roll, pitch, yaw = utils.get_rpy_from_matrix(M_two)
    # Compose joint position
        # case1 = allclose(xyz_from_two_to_joint, xyz_from_one_to_joint)
        # case2 = allclose(xyz_from_two_to_joint, xyz_of_one)
        # case3 = allclose(xyz_of_one, xyz_of_two)
        # if case1 and case2 and case3:
        #     xyz_of_joint = [0,0,0]
        # # elif case1 or case2:
        # #     xyz_of_joint = xyz_from_two_to_joint
        # else:
        #     xyz_of_joint = trans(M_two, xyz_of_one)
        # # xyz_of_joint = trans(M_two, xyz_of_one)
        # # xyz_of_joint =trans(M_two, xyz_of_one) xyz_from_two_to_joint
        def get_joint_geometry_matrix(geo_or_origin):
            """
            Returns a Matrix3D (transform) for a JointGeometryOrOrigin.
            Works for JointOrigin or JointGeometry.
            """
        
            if geo_or_origin is None or not geo_or_origin.isValid:
                return None, "Invalid JointGeometryOrOrigin"

            # Case 1: JointOrigin → has coordinateSystem
            if geo_or_origin.objectType == adsk.fusion.JointOrigin.classType():
                return geo_or_origin.getAsCoordinateSystem(), f"{str(geo_or_origin.getAsCoordinateSystem())}" #coordinateSystem.copy(), None

            # Case 2: JointGeometry → get entity
            if geo_or_origin.objectType == adsk.fusion.JointGeometry.classType() and geo_or_origin.isValid:
                ent = geo_or_origin.entityTwo if geo_or_origin.entityTwo is not None else geo_or_origin.entityOne

                # ---- Points ----
                if ent.classType() in [
                    adsk.fusion.BRepVertex.classType(),
                    adsk.fusion.SketchPoint.classType(),
                    adsk.fusion.ConstructionPoint.classType()
                ]:
                    pt = ent.geometry
                    mat = adsk.core.Matrix3D.create()
                    mat.translation = pt.asVector()
                    return mat, "construction_point"  # Special flag for points

                # ---- Edges / Axes ----
                elif ent.classType() in [
                    adsk.fusion.BRepEdge.classType(),
                    adsk.fusion.SketchLine.classType(),
                    adsk.fusion.ConstructionAxis.classType()
                ]:
                    geom = ent.geometry
                    mat = adsk.core.Matrix3D.create()
                    
                    
                    # Handle different geometry types
                    if ent.classType() == adsk.fusion.BRepEdge.classType():
                        # For BRepEdge, we need to get the local coordinates, not global
                        try:
                            # First try to use the joint origin from JointGeometry if available
                            if hasattr(geo_or_origin, 'origin'):
                                joint_origin = geo_or_origin.origin
                                mat.translation = joint_origin.asVector()
                                return mat, None
                            
                            # If no joint origin, we need to transform the edge to local coordinates
                            curve = geom
                            
                            # Check if this is a Line3D geometry
                            if curve.curveType == adsk.core.Curve3DTypes.Line3DCurveType:
                                # Get the component transform to convert from global to local coordinates
                                # The edge coordinates are in global space, we need them in component space
                                
                                # For now, use the evaluator method which should give better local coordinates
                                if hasattr(curve, 'evaluator'):
                                    evaluator = curve.evaluator
                                    param_range = evaluator.parameterRange
                                    mid_param = (param_range[0] + param_range[1]) / 2.0
                                    success, midpoint = evaluator.getPointAtParameter(mid_param)
                                    if success:
                                        # Get direction at midpoint for orientation
                                        success2, direction = evaluator.getTangent(mid_param)
                                        if success2 and direction.length > 1e-6:
                                            direction.normalize()
                                            
                                            # Create orthogonal coordinate system with direction as X-axis
                                            z_axis = adsk.core.Vector3D.create(0, 0, 1)
                                            y_axis = direction.crossProduct(z_axis)
                                            if y_axis.length < 1e-6:  # direction is parallel to Z
                                                y_axis = adsk.core.Vector3D.create(0, 1, 0)
                                                z_axis = direction.crossProduct(y_axis)
                                            y_axis.normalize()
                                            z_axis.normalize()
                                            
                                            mat.setWithCoordinateSystem(midpoint, direction, y_axis, z_axis)
                                            return mat, None
                                        else:
                                            # Just use the midpoint without rotation
                                            mat.translation = midpoint.asVector()
                                            return mat, None
                            
                            # Final fallback: try any available geometry center
                            if hasattr(curve, 'evaluator'):
                                evaluator = curve.evaluator
                                param_range = evaluator.parameterRange
                                mid_param = (param_range[0] + param_range[1]) / 2.0
                                success, point = evaluator.getPointAtParameter(mid_param)
                                if success:
                                    mat.translation = point.asVector()
                                    return mat, None
                            
                            # Last resort: identity matrix
                            return mat, None
                                
                        except Exception as e:
                            app = adsk.core.Application.get()
                            ui = app.userInterface
                            ui.messageBox(f"Error processing BRepEdge: {str(e)}")
                            # Create identity matrix at origin as last resort
                            return mat, None
                    
                    elif ent.classType() == adsk.fusion.ConstructionAxis.classType():
                        # For construction axis, get origin and direction directly
                        if hasattr(geom, 'origin') and hasattr(geom, 'direction'):
                            origin = geom.origin
                            direction = geom.direction.copy()
                            direction.normalize()
                            
                            # Create orthogonal vectors
                            z_axis = adsk.core.Vector3D.create(0, 0, 1)
                            y_axis = direction.crossProduct(z_axis)
                            if y_axis.length < 1e-6:  # direction is parallel to Z
                                y_axis = adsk.core.Vector3D.create(0, 1, 0)
                                z_axis = direction.crossProduct(y_axis)
                            y_axis.normalize()
                            z_axis.normalize()
                            
                            mat.setWithCoordinateSystem(origin, direction, y_axis, z_axis)
                            return mat, None
                    
                    elif ent.classType() == adsk.fusion.SketchLine.classType():
                        # For sketch lines, use start point and direction
                        if hasattr(geom, 'startPoint') and hasattr(geom, 'endPoint'):
                            start_pt = geom.startPoint
                            end_pt = geom.endPoint
                            direction = start_pt.vectorTo(end_pt)
                            direction.normalize()
                            
                            # Create orthogonal vectors
                            z_axis = adsk.core.Vector3D.create(0, 0, 1)
                            y_axis = direction.crossProduct(z_axis)
                            if y_axis.length < 1e-6:  # direction is parallel to Z
                                y_axis = adsk.core.Vector3D.create(0, 1, 0)
                                z_axis = direction.crossProduct(y_axis)
                            y_axis.normalize()
                            z_axis.normalize()
                            
                            mat.setWithCoordinateSystem(start_pt, direction, y_axis, z_axis)
                            return mat, None
                    
                    # Fallback: create identity matrix at origin
                    return mat, None

                # ---- Faces / Planes ----
                elif ent.classType() in [
                    adsk.fusion.BRepFace.classType(),
                    adsk.fusion.ConstructionPlane.classType()
                ]:
                    geom = ent.geometry
                    mat = adsk.core.Matrix3D.create()
                    
                    # Handle different geometry types
                    if ent.classType() == adsk.fusion.BRepFace.classType():
                        # For BRepFace, use centroid and surface evaluator
                        if hasattr(geom, 'centroid'):
                            pt = geom.centroid
                            # Try to get normal using evaluator
                            try:
                                evaluator = geom.evaluator
                                success, normal = evaluator.getNormalAtPoint(pt)
                                if success and normal:
                                    normal.normalize()
                                    x_axis = adsk.core.Vector3D.create(1, 0, 0)
                                    y_axis = normal.crossProduct(x_axis)
                                    if y_axis.length < 1e-6:  # normal is parallel to X
                                        x_axis = adsk.core.Vector3D.create(0, 1, 0)
                                        y_axis = normal.crossProduct(x_axis)
                                    x_axis = y_axis.crossProduct(normal)
                                    x_axis.normalize()
                                    y_axis.normalize()
                                    
                                    mat.setWithCoordinateSystem(pt, x_axis, y_axis, normal)
                                    return mat, None
                            except:
                                pass
                        
                        # Fallback for BRepFace: use origin if available
                        if hasattr(geom, 'origin'):
                            pt = geom.origin
                            normal = adsk.core.Vector3D.create(0, 0, 1)  # Default normal
                            mat.translation = pt.asVector()
                            return mat, None
                    
                    elif ent.classType() == adsk.fusion.ConstructionPlane.classType():
                        # For construction plane, handle both Plane and other geometry types
                        if hasattr(geom, 'origin') and hasattr(geom, 'normal'):
                            # This is a Plane geometry
                            origin = geom.origin
                            normal = geom.normal.copy()
                            normal.normalize()
                            
                            # Create orthogonal vectors with normal as Z-axis
                            x_axis = adsk.core.Vector3D.create(1, 0, 0)
                            y_axis = normal.crossProduct(x_axis)
                            if y_axis.length < 1e-6:  # normal is parallel to X
                                x_axis = adsk.core.Vector3D.create(0, 1, 0)
                                y_axis = normal.crossProduct(x_axis)
                            x_axis = y_axis.crossProduct(normal)
                            x_axis.normalize()
                            y_axis.normalize()
                            
                            mat.setWithCoordinateSystem(origin, x_axis, y_axis, normal)
                            return mat, None
                        else:
                            # Fallback: try to get any available position
                            if hasattr(geom, 'origin'):
                                mat.translation = geom.origin.asVector()
                            return mat, None
                    
                    # Final fallback: identity matrix
                    return mat, None
                else:
                    return None, f"Unsupported JointGeometry entity type: {ent.classType()}"

            return None, None
        try:
            t1 = None
            
            # if joint_type == 'fixed':
            # t1 = utils.list_to_matrix(joint.occurrenceOne.transform.asArray()) # origin of child
            # else:
            # t1 = utils.list_to_matrix(joint.geometryOneTransform.asArray())
            # t1 = utils.list_to_matrix(joint.occurrenceOne.transform.asArray()) # origin of child
            
            # msg = f"Transform for {joint_dict['child']}: {str(.transform.asArray())}"
            # break
            # transform = Matrix3D.create()
            # transform.translation = adsk.core.Vector3D.create(joint.geometryOrOriginTwo.origin.x, joint.geometryOrOriginTwo.origin.y, joint.geometryOrOriginTwo.origin.z)
            # t2 = utils.list_to_matrix(joint.geometryOrOriginTwo.origin.asArray()) # origin of parent
            if joint.geometryOrOriginOne:
                transform2 = Matrix3D.create()
                if joint_dict['parent'] in links_dict:
                    transform2.setWithArray(links_dict[joint_dict['parent']]['transform_array'])
                else:
                    # first create the joint where there is a child (OccurenceOne) with this name
                    parent_joint = next((j for j in all_joints if re.sub('[ :()]', '_', j.occurrenceOne.name.split(" ", 1)[0].split(":", 1)[0]) == joint_dict['parent']), None)
                    if parent_joint:
                        pp_joint, links_dict, msg = make_joint_dict(parent_joint, links_dict, msg, all_joints)
                        if pp_joint is None or links_dict is None:
                            return None, None, msg
                    # now it should be in the dict
                    if joint_dict['parent'] not in links_dict:
                        if parent_joint and not parent_joint.occurrenceTwo:
                            pass # handle base_link joint
                        else:
                            msg = f"Cannot find parent link {joint_dict['parent']} for joint {joint.name}. Please check the joint connections. {parent_joint.name if parent_joint else ' NO PARENT'} {parent_joint.occurrenceTwo}"
                            return None, None, msg
                    else:
                        transform2.setWithArray(links_dict[joint_dict['parent']]['transform_array'])
                transform1, geometry_type = get_joint_geometry_matrix(joint.geometryOrOriginOne)
                
                # Special handling for construction points - they should have zero rotation
                if geometry_type == "construction_point":
                    # For construction points, only use translation difference, no rotation
                    # The position should be relative to parent, but rotation should be zero
                    
                    # Get the absolute positions
                    child_translation = [transform1.translation.x, transform1.translation.y, transform1.translation.z]
                    
                    if joint_dict['parent'] == 'base_link':
                        # Parent is base_link (at origin), so just use child position directly
                        parent_translation = [0, 0, 0]
                    else:
                        # Parent has a transform, get its translation
                        parent_translation = [transform2.translation.x, transform2.translation.y, transform2.translation.z]
                    
                    # Create identity rotation matrix with translation difference
                    # This ensures RPY will be [0, 0, 0] for construction points
                    transform_matrix = [
                        [1, 0, 0, (child_translation[0] - parent_translation[0])],
                        [0, 1, 0, (child_translation[1] - parent_translation[1])],
                        [0, 0, 1, (child_translation[2] - parent_translation[2])],
                        [0, 0, 0, 1]
                    ]
                else:
                    # Calculate relative transform matrix preserving all rotation information
                    transform_matrix = utils.relative_transform(utils.list_to_matrix(transform2.asArray()), utils.list_to_matrix(transform1.asArray()))
                
                # Create Matrix3D object from the calculated matrix for component creation
                transform = Matrix3D.create()
                transform.setWithArray(utils.matrix_to_list(transform_matrix))
            else:
                transform = Matrix3D.create()
                transform_matrix = utils.list_to_matrix(transform.asArray())
            if transform is None or (geometry_type is not None and geometry_type not in ["construction_point", ""]):
                if geometry_type is None or geometry_type == '':
                    msg = f"Cannot get transform for {joint_dict['child']}. Please set joint origin and try again."
                elif geometry_type != "construction_point":
                    msg = geometry_type  # This contains the error message
                return None, None, msg
            # transform = t2 #utils.relative_transform(t2, t1)
            xyz_of_joint = [transform_matrix[0][3], transform_matrix[1][3], transform_matrix[2][3]]
            roll, pitch, yaw = utils.get_rpy_from_matrix(utils.matrix_to_list(transform_matrix))
        except Exception as e:
            msg = f"1. Error creating child transform for {joint_dict['child']}: {str(e)}"
            return None, None, msg
        try:
            joint_dict['transform'] = transform_matrix
            joint_dict['xyz'] = [round(i / 100.0, 6) for i in xyz_of_joint]  # converted to meter
            joint_dict['rpy'] = [roll, pitch, yaw]  # roll pitch yaw
        
            # Store transform as raw array data to completely avoid proxy issues
            # For component creation, we may need to sanitize problematic BRepFace/BRepEdge transforms
            transform_array = transform.asArray()
            
            # Check if this comes from a problematic source and create sanitized version for component creation
            source_type = 'unknown'
            if joint.geometryOrOriginOne:
                if geometry_type == "construction_point":
                    source_type = "ConstructionPoint"
                elif joint.geometryOrOriginOne.objectType == adsk.fusion.JointGeometry.classType():
                    ent = joint.geometryOrOriginOne.entityTwo if joint.geometryOrOriginOne.entityTwo is not None else joint.geometryOrOriginOne.entityOne
                    if ent:
                        source_type = ent.classType()
                else:
                    source_type = joint.geometryOrOriginOne.objectType
            
            # def validate_and_fix_transform(matrix_array):
            #     """
            #     Validate and fix a transform matrix for component creation.
            #     Returns a clean matrix that Fusion 360 will accept.
            #     """
            #     import math
                
            #     # Convert to 4x4 matrix
            #     if len(matrix_array) == 16:
            #         M = [matrix_array[i*4:(i+1)*4] for i in range(4)]
            #     else:
            #         return None
                
            #     # Check for invalid values
            #     for row in M:
            #         for val in row:
            #             if not math.isfinite(val):
            #                 return None
                
            #     # Extract rotation part (3x3 upper-left)
            #     R = [[M[i][j] for j in range(3)] for i in range(3)]
                
            #     # Validate rotation matrix
            #     def normalize_vector(v):
            #         length = math.sqrt(sum(x*x for x in v))
            #         if length < 1e-10:
            #             return None
            #         return [x/length for x in v]
                
            #     def cross_product(a, b):
            #         return [
            #             a[1]*b[2] - a[2]*b[1],
            #             a[2]*b[0] - a[0]*b[2], 
            #             a[0]*b[1] - a[1]*b[0]
            #         ]
                
            #     def dot_product(a, b):
            #         return sum(a[i]*b[i] for i in range(3))
                
            #     # Extract and normalize columns (as vectors)
            #     x_axis = normalize_vector([R[i][0] for i in range(3)])
            #     y_axis = normalize_vector([R[i][1] for i in range(3)])
            #     z_axis = normalize_vector([R[i][2] for i in range(3)])
                
            #     if not x_axis or not y_axis or not z_axis:
            #         # Rotation matrix is degenerate, use identity
            #         x_axis, y_axis, z_axis = [1,0,0], [0,1,0], [0,0,1]
            #     else:
            #         # Check orthogonality and handedness
            #         xy_dot = abs(dot_product(x_axis, y_axis))
            #         xz_dot = abs(dot_product(x_axis, z_axis))
            #         yz_dot = abs(dot_product(y_axis, z_axis))
                    
            #         if xy_dot > 0.1 or xz_dot > 0.1 or yz_dot > 0.1:
            #             # Not orthogonal enough, reconstruct
            #             # Keep x_axis, make y_axis orthogonal to x, then z = x × y
            #             y_axis = normalize_vector([y_axis[i] - dot_product(y_axis, x_axis)*x_axis[i] for i in range(3)])
            #             if not y_axis:
            #                 y_axis = [0, 1, 0] if abs(x_axis[1]) < 0.9 else [0, 0, 1]
            #                 y_axis = normalize_vector([y_axis[i] - dot_product(y_axis, x_axis)*x_axis[i] for i in range(3)])
            #             z_axis = normalize_vector(cross_product(x_axis, y_axis))
                
            #     # Construct clean matrix
            #     clean_matrix = [
            #         [x_axis[0], y_axis[0], z_axis[0], M[0][3]],
            #         [x_axis[1], y_axis[1], z_axis[1], M[1][3]], 
            #         [x_axis[2], y_axis[2], z_axis[2], M[2][3]],
            #         [0, 0, 0, 1]
            #     ]
                
            #     # Convert back to flat array
            #     return [clean_matrix[i][j] for i in range(4) for j in range(4)]
            

            transform_child, _ = get_joint_geometry_matrix(joint.geometryOrOriginOne) if joint.geometryOrOriginOne else (None, None)
            if transform_child:
                links_dict[joint_dict['child']] = {
                    'transform_array': transform_child.asArray(),  # Use the validated transform with rotation
                    'source_type': source_type
                }
        except Exception as e:
            msg = f"2. Error creating child transform for {joint_dict['child']}: {str(e)}"
            return None, None, msg

    except Exception as e:
        
        try:
            if type(joint.geometryOrOriginTwo)==adsk.fusion.JointOrigin:
                data = joint.geometryOrOriginTwo.geometry.origin.asArray()
            else:
                data = joint.geometryOrOriginTwo.origin.asArray()
            joint_dict['xyz'] = [round(i / 100.0, 6) for i in data]  # converted to meter
            joint_dict['rpy'] = [0, 0, 0]  # roll pitch yaw
        except:
            msg = joint.name + " doesn't have joint origin. Please set it and run again. " + str(e)
            return None, None, msg
    if msg == None:
        msg = 'Successfully create URDF file'
    
    return joint_dict, links_dict, msg
def make_joints_dict(root: Component, msg):
    """
    joints_dict holds parent, axis and xyz informatino of the joints
    
    
    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status
        
    Returns
    ----------
    joints_dict: 
        {name: {type, axis, upper_limit, lower_limit, parent, child, xyz}}
    msg: str
        Tell the status
    """



    joints_dict = {}
    links_dict = {}
    
    for joint in root.joints:
        joint_dict, links_dict, msg = make_joint_dict(joint, links_dict, msg, root.joints) # joint_dict
        if joint_dict is None or links_dict is None:
            break
        joints_dict[joint.name] = joint_dict
    return joints_dict, links_dict, msg