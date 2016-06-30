/*---------------------------------------------------------------------------
 *
 * Copyright 2016 by Kitware, Inc. All Rights Reserved. Please refer to
 *
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 *
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 *
 * Author : Chengjiang Long <chengjiang.long@kitware.com>
 *
 * Description : Simulated-kinect
 *
 *
 * Created : <2016-06-27>
 *
 *-------------------------------------------------------------------------*/

This software named "simulated-kinect" is implemented to visualize the sdf (http://sdformat.org/) format world (created by Gazebo http://gazebosim.org/) in VTK and to take RGBD images.

-----------------------
Prerequisites
-----------------------
1.Install SDFormat according to this page (URL: https://bitbucket.org/osrf/sdformat).
2.Install OGRE 3D according to this page (URL: https://github.com/ogre3d/clean-project).
3.VTK 7.0 

-----------------------
Before 
-----------------------
Modify the CMakeLists.txt to specify the pathes according your computers.


-----------------------
Testing examples
-----------------------
Enter the director "build", type:
cmake ..
make

For the beerstable.world, you can run:
./VisualizeSDFWorld ../worlds/beerstable.world 0  (only visualize the world)
./VisualizeSDFWorld ../worlds/beerstable.world 1  (take a RGBD image)

For the caffebeers.world, you can run:
./VisualizeSDFWorld ../worlds/caffebeers.world 0  (only visualize the world)
./VisualizeSDFWorld ../worlds/caffebeers.world 1  (take a RGBD image)

For the realscene.world, you can run:
./VisualizeSDFWorld ../worlds/realscene.world 0  (only visualize the world)
./VisualizeSDFWorld ../worlds/realscene.world 1  (take a RGBD image)


