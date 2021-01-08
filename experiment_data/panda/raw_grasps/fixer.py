import xml.etree.ElementTree as et
import os
import argparse
from xml.dom.minidom import Node
from xml.dom import minidom

def remove_blanks(node):
    for x in node.childNodes:
        if x.nodeType == Node.TEXT_NODE:
            if x.nodeValue:
                x.nodeValue = x.nodeValue.strip()
        elif x.nodeType == Node.ELEMENT_NODE:
            remove_blanks(x)

def parse_object_file(filename):

    if not os.path.isfile(filename):

        print("File {} does not exist".format(filename))
        return -1

    tree = et.parse(filename)
    grasp_data = tree.getroot()
    manip_object_field = grasp_data.find('ManipulationObject')
    graspset_field = manip_object_field.find('GraspSet')
    # current_grasp_idx = len(list(graspset_field))

    for grasp in graspset_field:

        matrix=grasp.find('Transform').find('Matrix4x4')

        for row in matrix.getchildren():

            row.attrib['c1'] = str(float(row.attrib['c1']) / 1000)
            row.attrib['c2'] = str(float(row.attrib['c2']) / 1000)
            row.attrib['c3'] = str(float(row.attrib['c3']) / 1000)

    domstring = minidom.parseString(et.tostring(grasp_data))
    remove_blanks(domstring)
    domstring.normalize()
    with open(filename, 'w') as handle:
        handle.write(domstring.toprettyxml(indent = '  '))

    # grasped_field = grasp_data.find('Grasped')

    # stability_field = grasp_data.find('GraspStability')

    # avoidance_field = grasp_data.find('ObstacleAvoidance')

    # print("\tObject: {}".format(manip_object_field.attrib['name']))
    #
    # overall_success = 0
    # overall_stability = 0.0
    #
    # for success, stability in zip(grasped_field.getchildren(), stability_field.getchildren()):
    #
    #     print("{} : success {} | stability {}".format(success.get('name'), success.get('quality'), stability.get('quality')))
    #
    #     overall_success += int(success.get('quality'))
    #     overall_stability += float(stability.get('quality'))
    #
    # print('Overall: \n\tsuccess {}/{}\n\tstability {}/{}.\n'.format(overall_success, len(grasped_field.getchildren()), overall_stability, float(len(grasped_field.getchildren()))))

    return 0


if __name__ == "__main__":

    arg_parser = argparse.ArgumentParser()

    # Add an argument for layout and one for planner
    arg_parser.add_argument('-p', dest='planner', default="")
    arg_parser.add_argument('-l', dest='layout', default="")
    arg_parser.add_argument('-o', dest='manip_object', default="")

    args = arg_parser.parse_args()

    cwd = os.path.abspath(os.getcwd())
    planners = [d for d in os.listdir(cwd) if (os.path.isdir(d) and '.' not in d)]

    if args.planner:
        planners = [args.planner]

    # Decide what planner and layout to use

    for p in planners:

        print("========================")
        print("\nPlanner {}\n".format(p))
        print("========================")

        planner_dir = os.path.join(cwd, p)

        if args.layout:
            layouts = [args.layout]
        else:
            layouts = [l for l in os.listdir(planner_dir) if os.path.isdir(os.path.join(planner_dir, l))]

        for l in layouts:

            print("========================")
            print("Parsing layout {}\n".format(l))

            layout_dir = os.path.join(planner_dir, l)

            if args.manip_object:
                objects = ['grasp_' + args.manip_object + '.xml']
            else:
                objects = [o for o in os.listdir(layout_dir) if '.xml' in o]

            for o in objects:

                object_filename = os.path.join(layout_dir, o)

                print ("Parsing {}".format(object_filename))
                parse_object_file(object_filename)
