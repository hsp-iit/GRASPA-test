#!/usr/bin/python

import os
import glob

directory = "/home/fbottarel/workspace/docker-shared-workspace/simox/GRASPA-test/experiment_data/panda"
planner = "dexnet"
layout = "layout_2"

working_dir = os.path.join(directory, planner, layout)



xml_filelist = glob.glob(working_dir + "/*.xml")

for xml_filename in xml_filelist:

    filename = os.path.splitext(os.path.basename(xml_filename))
    file_dir = os.path.dirname(xml_filename)
    splitted_filename = filename[0].split("_")

    object_name_camelcase = []
    for i in splitted_filename:
        object_name_camelcase.append(i.capitalize())

    new_filename = "Ycb" + "".join(object_name_camelcase[1:]) + "_grasp.xml"
    new_filename = os.path.join(file_dir, new_filename)

    # os.rename(xml_filename, new_filename)

    with open(xml_filename, "r") as input_file:
        with open(new_filename, "w") as output_file:
            for line in input_file:
                if "grasp_data" not in line:
                    output_file.write(line)
