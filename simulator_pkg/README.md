Launch the gazebo camera and block model:
``roslaunch simulator_pkg simple_camera_simu.launch``

launch the opencv node with the display:
``roslaunch simulator_pkg find_red_pixels.launch``

------------------------------
publishing a new pose to the camera
``rostopic pub -r 2000 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: simple_camera, pose: { position: { x: 0.1, y: 0, z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'``

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


