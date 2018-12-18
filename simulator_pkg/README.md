This package is for launching gazebo mith a model camera and block, then publishing the images with the related pose to a new topic.

The main code is in src/image_saver.cpp

To run the program:

First, launch the gazebo camera and block model:

``roslaunch simulator_pkg simple_camera_simu.launch``


Then, launch the opencv node with the display:

``roslaunch simulator_pkg image_saver.launch``

After that, disable gravity in camera model in gazebo.

Finally, run the image saver node:

 ``rosrun simulator_pkg image_saver``
