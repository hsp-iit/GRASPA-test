/******************************************************************************
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation; either version 2 of the License, or (at your option) any later
* version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.                                                                     *
 ******************************************************************************/

/**
 * @file idl.thirft
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

 /**
 * Bottle
 *
 * IDL structure to set/show advanced parameters.
 */
 struct Vector
 {
 } (
    yarp.name = "yarp::sig::Vector"
    yarp.includefile="yarp/sig/Vector.h"
 )

 service GraspAndStability_IDL
 {
     bool get_grasp(1: string &arm, 2: string &obj);

	 bool grasp();

     bool generate_trajectory();

     bool save_grasp_data(1: double graspable, 2: double grasped, 3: double grasp_stability, 4: double obstacles);

     bool execute_trajectory();

     bool set_layout_name(1: string &name);

     bool home();

 }
