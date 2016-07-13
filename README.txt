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

This software named "simulated-kinect" is implemented to visualize the sdf (http://sdformat.org/) format 
world (created by Gazebo http://gazebosim.org/) in VTK and to take RGBD images.

-----------------------
Prerequisites
-----------------------
1.Install SDFormat according to this page (URL: https://bitbucket.org/osrf/sdformat).
2.Install OGRE 3D according to this page (URL: https://github.com/ogre3d/clean-project).
3.VTK 7.0 


----------------------
Coversion tool
----------------------
We generate the conversion tool called "collada2obj" to conduct the coversion from *.dae into *.obj and
*.obj.mtl (See the dirctory "collada2obj" https://github.com/chengjianglong/simulated-kinect/tree/master/collada2obj)

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
./VisualizeSDFWorld ../worlds/beerstable.world   (only visualize the world)
./SimulatedKinect ../worlds/beerstable.world   (take a RGBD image)

For the caffebeers.world, you can run:
./VisualizeSDFWorld ../worlds/caffebeers.world   (only visualize the world)
./SimulatedKinect ../worlds/caffebeers.world   (take a RGBD image)

For the realscene.world, you can run:
./VisualizeSDFWorld ../worlds/realscene.world   (only visualize the world)
./SimulatedKinect ../worlds/realscene.world   (take a RGBD image)

For SimulatedKinect, you can press the 4 key arrows on the keybord to control the moving of the robot. If you want to take a RGBD pictures, just press the ENTER/RETURN key, and then you will get the "capture_rgb.png" and "capture_depth.bmp" immediately.

BTW, we haven't implemented the collission-check part for the moving of the robot in the current version.

