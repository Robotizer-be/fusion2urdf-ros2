# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:11:28 2019

@author: syuntoku
"""

import adsk, re
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils
import json


class Link:

    def __init__(
        self,
        name,
        xyz,
        center_of_mass,
        repo,
        mass,
        inertia_tensor,
        material_name="silver",
    ):
        """
        Parameters
        ----------
        name: str
            name of the link
        xyz: [x, y, z]
            coordinate for the visual and collision
        center_of_mass: [x, y, z]
            coordinate for the center of mass
        link_xml: str
            generated xml describing about the link
        repo: str
            the name of the repository to save the xml file
        mass: float
            mass of the link
        inertia_tensor: [ixx, iyy, izz, ixy, iyz, ixz]
            tensor of the inertia
        """
        self.name = name
        # xyz for visual
        self.xyz = [0, 0, 0]  # [-_ for _ in xyz]  # reverse the sign of xyz
        # xyz for center of mass
        self.center_of_mass = center_of_mass
        self.link_xml = None
        self.repo = repo
        self.pkg_name = repo.split("/")[0]
        self.remain_repo_addr = repo[len(self.pkg_name) :]
        self.mass = mass
        self.inertia_tensor = inertia_tensor
        self.material_name = material_name

    def make_link_xml(self):
        """
        Generate the link_xml and hold it by self.link_xml
        """

        link = Element("link")
        link.attrib = {"name": "${prefix}" + self.name}

        if self.name != "base_link":
            # inertial
            inertial = SubElement(link, "inertial")
            origin_i = SubElement(inertial, "origin")
            origin_i.attrib = {
                "xyz": " ".join([str(_) for _ in self.center_of_mass]),
                "rpy": "0 0 0",
            }
            mass = SubElement(inertial, "mass")
            mass.attrib = {"value": str(self.mass)}
            inertia = SubElement(inertial, "inertia")
            inertia.attrib = {
                "ixx": str(self.inertia_tensor[0]),
                "iyy": str(self.inertia_tensor[1]),
                "izz": str(self.inertia_tensor[2]),
                "ixy": str(self.inertia_tensor[3]),
                "iyz": str(self.inertia_tensor[4]),
                "ixz": str(self.inertia_tensor[5]),
            }

            # visual
            visual = SubElement(link, "visual")
            origin_v = SubElement(visual, "origin")
            origin_v.attrib = {
                "xyz": " ".join([str(_) for _ in self.xyz]),
                "rpy": "0 0 0",
            }
            geometry_v = SubElement(visual, "geometry")
            mesh_v = SubElement(geometry_v, "mesh")
            # mesh_v.attrib = {'filename':'file://' + '$(find %s)' % self.pkg_name + self.remain_repo_addr + self.name + '.stl','scale':'0.001 0.001 0.001'}
            mesh_v.attrib = {
                "filename": "package://%s" % self.pkg_name
                + self.remain_repo_addr
                + self.name
                + ".stl",
                "scale": "0.001 0.001 0.001",
            }
            material = SubElement(visual, "material")
            material.attrib = {"name": "%s" % self.material_name}

            # collision
            collision = SubElement(link, "collision")
            origin_c = SubElement(collision, "origin")
            origin_c.attrib = {
                "xyz": " ".join([str(_) for _ in self.xyz]),
                "rpy": "0 0 0",
            }
            geometry_c = SubElement(collision, "geometry")
            mesh_c = SubElement(geometry_c, "mesh")
            mesh_c.attrib = {
                "filename": "package://%s" % self.pkg_name
                + self.remain_repo_addr
                + self.name
                + ".stl",
                "scale": "0.001 0.001 0.001",
            }

        # print("\n".join(utils.prettify(link).split("\n")[1:]))
        self.link_xml = "\n".join(utils.prettify(link).split("\n")[1:])


def make_inertial_dict(root, msg, colors_dict, links_colors_dict, ui, design):
    """
    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status

    Returns
    ----------
    inertial_dict: {name:{mass, inertia, center_of_mass}}

    msg: str
        Tell the status
    """
    # Get component properties.
    allOccs = root.occurrences
    inertial_dict = {}

    for occs in allOccs:
        # Skip the root component.
        occs_dict = {}
        prop = occs.getPhysicalProperties(
            adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy
        )

        occs_dict["name"] = re.sub(
            "[ :()]", "_", occs.name.split(" ", 1)[0].split(":", 1)[0]
        )

        mass = prop.mass  # kg
        occs_dict["mass"] = mass
        center_of_mass = [_ / 100.0 for _ in prop.centerOfMass.asArray()]  ## cm to m
        occs_dict["center_of_mass"] = center_of_mass

        # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-ce341ee6-4490-11e5-b25b-f8b156d7cd97
        (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
        moment_inertia_world = [
            _ / 10000.0 for _ in [xx, yy, zz, xy, yz, xz]
        ]  ## kg / cm^2 -> kg/m^2
        occs_dict["inertia"] = utils.origin2center_of_mass(
            moment_inertia_world, center_of_mass, mass
        )

        if occs.component.name == "base_link":
            inertial_dict["base_link"] = occs_dict
        else:
            inertial_dict[
                re.sub("[ :()]", "_", occs.name.split(" ", 1)[0].split(":", 1)[0])
            ] = occs_dict
            if occs.bRepBodies.count > 0:
                appearance = occs.bRepBodies[0].appearance
                for body in occs.bRepBodies:
                    color = body.appearance.appearanceProperties.itemByName("Color")
                    if color and (
                        color.value.red != 255
                        or color.value.green != 255
                        or color.value.blue != 255
                    ):
                        colorValue = f"{str(round(color.value.red/255,3))} {str(round(color.value.green/255,3))} {str(round(color.value.blue/255,3))} {str(round((color.value.opacity if color.value.opacity > 10 else 25)/255,3))}"
                        try:
                            existing_color = [
                                a[0]
                                for a in colors_dict.items()
                                if a[1] and a[1] == colorValue
                            ][0]
                            if existing_color:
                                links_colors_dict[
                                    re.sub(
                                        "[ :()]",
                                        "_",
                                        occs.name.split(" ", 1)[0].split(":", 1)[0],
                                    )
                                ] = existing_color
                        except:
                            # generate random name
                            new_color_name = "color" + str(len(colors_dict))
                            colors_dict[new_color_name] = colorValue
                            links_colors_dict[
                                re.sub(
                                    "[ :()]",
                                    "_",
                                    occs.name.split(" ", 1)[0].split(":", 1)[0],
                                )
                            ] = new_color_name
                        break
                # color = None
                # try:
                #     globalColor = design.appearances.itemByName(appearance.name)
                # except:
                #     globalColor = None
                # if globalColor is None:
                #     color = occs.bRepBodies[0].appearance.appearanceProperties.itemByName('Color')
                # else:
                #     color = globalColor.appearanceProperties.itemByName('Color')
                # if color and (color.value.red != 255 or color.value.green != 255 or color.value.blue != 255):
                #     colorValue = f"{str(round(color.value.red/255,3))} {str(round(color.value.green/255,3))} {str(round(color.value.blue/255,3))} {str(round(color.value.opacity/255,3))}"
                #     try:
                #         existing_color = [a[0] for a in colors_dict.items() if a[1] and a[1] == colorValue][0]
                #         if existing_color:
                #             links_colors_dict[re.sub('[ :()]', '_', occs.name.split(" ", 1)[0].split(":", 1)[0])] = existing_color
                #     except:
                #         # generate random name
                #         new_color_name = 'color' + str(len(colors_dict))
                #         colors_dict[new_color_name] = colorValue
                #         links_colors_dict[re.sub('[ :()]', '_', occs.name.split(" ", 1)[0].split(":", 1)[0])] = new_color_name
                #     # ui.messageBox("r = {0}, g = {1}, b = {2}".format(color.value.red, color.value.green, color.value.blue))

    return inertial_dict, msg, colors_dict, links_colors_dict
