This package is for launching gazebo mith a model camera and block, then publishing the images with the related pose to a new topic.

The main code is in src/image_saver.cpp

To run the program:

First launch the gazebo camera and block model:
''roslaunch simulator_pkg simple_camera_simu.launch''


launch the opencv node with the display:
``roslaunch simulator_pkg image_saver.launch``

disable gravity in camera model

 ``rosrun simulator_pkg image_saver``
