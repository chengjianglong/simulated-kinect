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
* Description : Collada2obj is a Ruby program to convert Collada models to obj format.
*
*
* Created : <2016-06-30>
*
*-------------------------------------------------------------------------*/


-----------------------
Prerequisites
-----------------------
Install Ruby. 


-----------------------
Usage
-----------------------

Just call the program with the Collada model to convert as the first argument.

./collada2obj.rb model.dae

To address the scale issue appearing from the coversion from Gazebo's 3D objects, I would suggest to call the program with the following command:

./collada2obj.rb -b model.dae

Note verify.rb is writen to verify the correctness of collada2orb.rb. You can check with the follow command:

./verfy.rb


----------------------
Testing example
----------------------
We provide two testing *.dae data (i.e., cafe.dae and cafe_table.dae) under the directory "./testdae/meshes/". You can run the examples like this:

./collada2obj.rb ./testdae/meshes/cafe_table.dae  (or ./collada2obj.rb -b ./testdae/meshes/cafe_table.dae)

And then you will find the new folder called "cafe_table" created. Enter the folder, you can view the genterated cafe_table.obj and cafe_table.obj.mtl with the Meshlab tools (https://sourceforge.net/projects/meshlab/).

Note to integrate the generated folder containing *.obj and *.obj.mtl into our "Simulated-kinect" projects, we suggest you to copy the folder to the director "../worlds/models/".  Also, please change the pathes of images mentioned in the *.obj.mtl file so that our program VisualizeSDFWorld can read the images.

For the current setting, the VisualizeSDFWorld program can find the images with the pathname like this "/../worlds/models/cafe/__atuo_1.jpg" (See ../world/models/cafe/cafe.obj.mtl).

-----------------------
Others
-----------------------
This software developed for the coversion from *.dae into *.obj and *.obj.mtl. However, the online conversion tools like onlineconv (http://www.greentoken.de/onlineconv/) and are not reliable. 

We also notice there is a Ruby program called "collada-to-obj"(https://github.com/chumpage/collada-to-obj), which can do the basic conversions. To extend it to our complex requirement, we make a modification and rename our modification version as "collada2obj", in which we addressed the know issues (https://github.com/chumpage/collada-to-obj#known-issues) pointed out in the current collada-to-obj.

In our collada2obj, we support the valid normal information rather than setting it as (0, 0, 0) for each vertex. Also, we support the <specular>, <emission>, <d>, <Ns> and <illumination> material properties besides <ambient> and <diffuse> material properties.

