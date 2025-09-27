# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
from adsk.fusion import Component, Occurrence
import os.path, re
from xml.etree import ElementTree
from xml.dom import minidom
from shutil import copytree
import fileinput
import sys

def copy_occs(root: Component, links_xyz_dict):
    """
    duplicate all the components
    """
    def copy_body(allOccs, occs: Occurrence):
        """
        copy the old occs to new component
        """

        bodies = occs.bRepBodies
        # origin = occs.transform.translation
        # transform = adsk.core.Matrix3D.create()
        name = re.sub('[ :()]', '_', occs.name.split(" ", 1)[0].split(":", 1)[0])
        # Transform to origin position of body
        transform = occs.transform.copy() # adsk.core.Vector3D.create(links_xyz_dict[name][0] * 100, links_xyz_dict[name][1]* 100, links_xyz_dict[name][2]* 100)
        
        # Create new components from occs
        # This support even when a component has some occses.
        
        # add a new component with the same origin as

        new_occs = allOccs.addNewComponent(transform)  # this create new occs
        if occs.component.name == 'base_link':
            occs.component.name = 'old_component'
            new_occs.component.name = 'base_link'
        else:
            new_occs.component.name = re.sub('[ :()]', '_', occs.name.split(" ", 1)[0].split(":", 1)[0])
        new_occs = allOccs.item((allOccs.count-1))
        for i in range(bodies.count):
            body = bodies.item(i)
            body.copyToComponent(new_occs)
        

    allOccs = root.occurrences
    oldOccs = []
    coppy_list = [occs for occs in allOccs]
    for occs in coppy_list:
        if occs.bRepBodies.count > 0:
            copy_body(allOccs, occs)
            oldOccs.append(occs)

    for occs in oldOccs:
        occs.component.name = 'old_component'


def export_stl(design, save_dir, components):
    """
    export stl files into "sace_dir/"


    Parameters
    ----------
    design: adsk.fusion.Design.cast(product)
    save_dir: str
        directory path to save
    components: design.allComponents
    """

    # create a single exportManager instance
    exportMgr = design.exportManager
    # get the script location
    try: os.mkdir(save_dir + '/meshes')
    except: pass
    scriptDir = save_dir + '/meshes'
    # export the occurrence one by one in the component to a specified file
    for component in components:
        allOccus = component.allOccurrences
        for occ in allOccus:
            if 'old_component' not in occ.component.name:
                try:
                    print(occ.component.name)
                    name = re.sub('[ :()]', '_', occ.component.name.split(" ", 1)[0].split(":", 1)[0])
                    fileName = scriptDir + "/" + name # occ.component.name
                    # create stl exportOptions
                    stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
                    stlExportOptions.sendToPrintUtility = False
                    stlExportOptions.isBinaryFormat = True
                    # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
                    stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                    exportMgr.execute(stlExportOptions)
                except:
                    print('Component ' + occ.component.name + 'has something wrong.')


def file_dialog(ui):
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog'

    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into
    that about center of mass coordinate


    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]


    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                         -x*y, -y*z, -x*z]
    return [ round(i - mass*t, 6) for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element


    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def create_package(save_dir, package_dir):
    try: os.mkdir(save_dir + '/launch')
    except: pass

    try: os.mkdir(save_dir + '/urdf')
    except: pass

    try: os.mkdir(save_dir + '/config')
    except: pass

    try: os.mkdir(save_dir + '/worlds')
    except: pass
    copytree(package_dir, save_dir, dirs_exist_ok=True)

def update_cmakelists(save_dir, package_name):
    file_name = save_dir + '/CMakeLists.txt'

    for line in fileinput.input(file_name, inplace=True):
        if 'project(fusion2urdf)' in line:
            sys.stdout.write("project(" + package_name + ")\n")
        else:
            sys.stdout.write(line)

def update_package_xml(save_dir, package_name):
    file_name = save_dir + '/package.xml'

    for line in fileinput.input(file_name, inplace=True):
        if '<name>' in line:
            sys.stdout.write("  <name>" + package_name + "</name>\n")
        elif '<description>' in line:
            sys.stdout.write("<description>The " + package_name + " package</description>\n")
        else:
            sys.stdout.write(line)
    
def get_rpy_from_matrix(M):
    """
    Get roll, pitch, yaw from a 4x4 transformation matrix

    Parameters
    ----------
    M: list of float
        16 elements of a 4x4 transformation matrix in row-major order

    Returns
    -------
    roll, pitch, yaw: float
        rotation in radian
    """
    import math
    if abs(M[10]) != 1:
        pitch = -math.asin(M[2])
        roll = math.atan2(M[6]/math.cos(pitch), M[10]/math.cos(pitch))
        yaw = math.atan2(M[1]/math.cos(pitch), M[0]/math.cos(pitch))
    else:
        yaw = 0 # can set to anything
        if M[10] == -1:
            pitch = math.pi/2
            roll = yaw + math.atan2(M[4], M[8])
        else:
            pitch = -math.pi/2
            roll = -yaw + math.atan2(-M[4], -M[8])
    return [round(roll, 6), round(pitch, 6), round(yaw, 6)]

def mat_mult(A, B):
    """Multiply two 4x4 matrices A and B"""
    result = [[0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            result[i][j] = sum(A[i][k] * B[k][j] for k in range(4))
    return result

def invert_transform(T):
    """Invert a 4x4 homogeneous transform (rigid body only)"""
    # Extract rotation (R) and translation (t)
    R = [row[:3] for row in T[:3]]
    t = [T[i][3] for i in range(3)]

    # Transpose rotation
    R_T = [[R[j][i] for j in range(3)] for i in range(3)]

    # Compute -R^T * t
    t_inv = [-sum(R_T[i][k] * t[k] for k in range(3)) for i in range(3)]

    # Build inverse matrix
    T_inv = [[0]*4 for _ in range(4)]
    for i in range(3):
        for j in range(3):
            T_inv[i][j] = R_T[i][j]
        T_inv[i][3] = t_inv[i]
    T_inv[3][3] = 1
    return T_inv

def relative_transform(T1, T2):
    """Compute T1^-1 * T2 without numpy"""
    return mat_mult(invert_transform(T1), T2)

def list_to_matrix(M_list):
    """Convert a list of 16 elements to a 4x4 matrix"""
    return [M_list[i*4:(i+1)*4] for i in range(4)]

def matrix_to_list(M):
    """Convert a 4x4 matrix to a list of 16 elements"""
    return [M[i][j] for i in range(4) for j in range(4)]