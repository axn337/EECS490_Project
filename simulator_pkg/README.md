Launch the gazebo camera and block model:
``roslaunch simulator_pkg simple_camera_simu.launch``
``roslaunch simulator_pkg map_camera_simu.launch``

launch the opencv node with the display:
``roslaunch simulator_pkg image_saver.launch``

disable gravity in camera model

 ``rosrun simulator_pkg image_saver``
 ``rosrun simulator_pkg map_saver``
------------------------------
publishing a new pose to the camera
``rostopic pub -r 2000 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: simple_camera_model, pose: { position: { x: 0.2, y: 1, z: 1 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'``

publishing a new pose to the camera
``rostopic pub -r 20 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: block, pose: { position: { x: 0.1, y: 0, z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'``

rostopic info /gazebo/set_model_state 
Type: gazebo_msgs/ModelState

Publishers: None

Subscribers: 
 * /gazebo (http://129.22.149.233:35041/)

------------------------------

subscribe to the topic /simple_camera/image_raw. 

rostopic info /simple_camera/image_rawType: sensor_msgs/Image

Publishers: 
 * /gazebo (http://129.22.149.233:35041/)

Subscribers: 
 * /find_red_pixels (http://129.22.149.233:35981/)
 * /image_view (http://129.22.149.233:44779/)

-0.000 -> -0.0234 = -0.234
-0.540 -> 0.418 = +0.958
-1.5707 -> 0.0152 = +1.586
