#! /usr/bin/env python
# Thanks for https://github.com/mikaelarguedas/gazebo_models
from __future__ import print_function

import argparse
from xml.dom.minidom import parse
import os
import sys

parser = argparse.ArgumentParser(
    usage='generate gazebo models for AR tags',
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)
parser.add_argument(
    '-i', '--images-dir',
    default="$HOME/gazebo_models/ar_tags/images",
    help='directory where the marker images are located')
parser.add_argument(
    '-g', '--gazebodir',
    default="$HOME/.gazebo/models",
    help='Gazebo models directory')
parser.add_argument(
    '-s', '--size',
    default=500, type=int,
    help='marker size in mm')
parser.add_argument(
    '-v', '--verbose',
    dest='verbose', action='store_true',
    help='verbose mode')

parser.add_argument(
    '-w', '--white-contour-size-mm',
    default=0, type=int,
    help='Add white contour around images, default to no contour')
parser.set_defaults(verbose=False)

args = parser.parse_args()

args.gazebodir = os.path.expandvars(args.gazebodir)
args.images_dir = os.path.expandvars(args.images_dir)
script_path = os.path.dirname(os.path.realpath(__file__))
model_dir = os.path.join(os.path.dirname(script_path), 'model')
ORIGINAL_MARKER_SIZE_MM = 500
ORIGINAL_IMAGE_SIZE_PX = 170
white_contour_px = \
    args.white_contour_size_mm * ORIGINAL_IMAGE_SIZE_PX / args.size

if not os.path.isdir(args.images_dir):
    print("provided image directory '%s' is not a directory" % args.images_dir)
    sys.exit()

# Open every collada file
if args.verbose:
    print(args.images_dir)

# Copy marker0 directory into gazebo model directory
cp_marker0_cmd = "cp -r " + os.path.join(model_dir, 'marker0') + \
             " " + os.path.join(args.gazebodir, "marker0")
if args.verbose:
    print(cp_marker0_cmd)
os.system(cp_marker0_cmd)

file_list = sorted(os.listdir(args.images_dir))
for image_file in file_list:
    if not image_file.endswith('.png'):
        continue
    image_file_path = os.path.join(args.images_dir, image_file)
    filename_without_ext = image_file[0:image_file.rfind('.')]
    # ignore marker0 as it has already been copied above
    if not filename_without_ext.lower() == 'marker0':
        cmd = "cp -r " + os.path.join(args.gazebodir, "marker0") + \
              " " + os.path.join(args.gazebodir, filename_without_ext.lower())
        if args.verbose:
            print(cmd)
        os.system(cmd)

    cmd = "rm " + os.path.join(
        args.gazebodir, filename_without_ext.lower(),
        "materials", "textures", "Marker0.png")
    if args.verbose:
        print(cmd)
    os.system(cmd)
    image_dest_path = os.path.join(
        args.gazebodir,
        filename_without_ext.lower(),
        "materials", "textures", image_file)
    cmd = "cp " + image_file_path + " " + \
          image_dest_path
    if args.verbose:
        print(cmd)
    os.system(cmd)

    # add white contour if applicable:
    if white_contour_px > 0:
        convert_cmd = "convert %s -bordercolor white -border %dx%d %s" % (
            image_dest_path, white_contour_px,
            white_contour_px, image_dest_path)
        if args.verbose:
            print(convert_cmd)
        os.system(convert_cmd)

    model_config_path = os.path.join(
        args.gazebodir, filename_without_ext.lower(), "model.config")
    if args.verbose:
        print(model_config_path)
    dom = parse(model_config_path)
    # modify model.config
    for node in dom.getElementsByTagName('name'):
        node.firstChild.nodeValue = filename_without_ext
        if args.verbose:
            print(node.firstChild.nodeValue)
        break
    f = open(os.path.join(
        args.gazebodir, filename_without_ext.lower(), "model.config"), 'w+')
    # Write the modified xml file
    f.write(dom.toxml())
    f.close()

    # modify model.sdf
    model_noversion_sdf_path = os.path.join(
        args.gazebodir, filename_without_ext.lower(), "model.sdf")
    if args.verbose:
        print("open model.sdf")
        print(model_noversion_sdf_path)
    dom = parse(model_noversion_sdf_path)
    for node in dom.getElementsByTagName('model'):
        node.attributes["name"].value = filename_without_ext
        if args.verbose:
            print("model tag found")
        break

    scaleModified = False
    scale = (args.size + 2 * args.white_contour_size_mm) / \
        float(ORIGINAL_MARKER_SIZE_MM)
    for node in dom.getElementsByTagName('mesh'):
        for child in node.childNodes:
            if child.nodeName == "scale":
                child.firstChild.nodeValue = \
                    "{} {} {}".format(scale, scale, scale)
                scaleModified = True
            if child.nodeName == "uri":
                child.firstChild.nodeValue = "model://" + os.path.join(
                    filename_without_ext.lower(), "meshes",
                    filename_without_ext + ".dae")
                if args.verbose:
                    print("uri tag found")
                    print(node.firstChild.nodeValue)
        if not scaleModified:
            if args.verbose:
                print("creating scale tag")
            x = dom.createElement("scale")
            y = dom.createTextNode("{} {} {}".format(scale, scale, scale))
            x.appendChild(y)
            node.appendChild(x)

    f = open(model_noversion_sdf_path, 'w+')
    # Write the modified xml file
    f.write(dom.toxml())
    f.close()

    # modify model-1_4.sdf
    model_sdf_path = os.path.join(
        args.gazebodir, filename_without_ext.lower(), "model-1_4.sdf")
    cmd = "cp " + model_noversion_sdf_path + " " + model_sdf_path
    if args.verbose:
        print(cmd)
    os.system(cmd)

    if args.verbose:
        print("open model-1_4.sdf")
        print(model_sdf_path)
    dom = parse(model_sdf_path)
    for node in dom.getElementsByTagName('sdf'):
        node.attributes["version"].value = "1.4"
        break
    f = open(model_sdf_path, 'w+')
    # Write the modified xml file
    f.write(dom.toxml())
    f.close()

    # modify model-1_5.sdf
    model_sdf_path = os.path.join(
        args.gazebodir, filename_without_ext.lower(), "model-1_5.sdf")
    cmd = "cp " + model_noversion_sdf_path + \
          " " + model_sdf_path
    if args.verbose:
        print(cmd)
    os.system(cmd)

    if args.verbose:
        print("open model-1_5.sdf")
        print(model_sdf_path)
    dom = parse(model_sdf_path)
    for node in dom.getElementsByTagName('sdf'):
        node.attributes["version"].value = "1.5"
        break
    f = open(model_sdf_path, 'w+')
    # Write the modified xml file
    f.write(dom.toxml())
    f.close()

    meshes_dir = os.path.join(
        args.gazebodir, filename_without_ext.lower(), "meshes")
    if args.verbose:
        print(os.path.join(meshes_dir, "Marker0.dae") +
              "  newname " + os.path.join(
                  meshes_dir, filename_without_ext + ".dae"))
    os.rename(
        os.path.join(meshes_dir, "Marker0.dae"),
        os.path.join(meshes_dir, filename_without_ext + ".dae"))

    # modify ModelX.dae
    if args.verbose:
        print("open ModelX.dae")
        print(os.path.join(meshes_dir, filename_without_ext + ".dae"))
    dom = parse(os.path.join(meshes_dir, filename_without_ext + ".dae"))
    for node in dom.getElementsByTagName('init_from'):
        node.firstChild.nodeValue = image_file
        if args.verbose:
            print("init_from tag found")
        break

    f = open(os.path.join(
        meshes_dir, filename_without_ext + ".dae"), 'w+')
    # Write the modified xml file
    f.write(dom.toxml())
    f.close()