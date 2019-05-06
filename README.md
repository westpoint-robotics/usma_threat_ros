Setting up the package
=======

	git init
	git remote add gh git@github.com:westpoint-robotics/usma_ardrone.git

Weights files can be downloaded from here: 

	linkto.google.drive/weights
	mv pistol-tiny_300000.weights <catkin_ws>/src/usma_threat_ros/yolo

open <catkin_ws>/src/usma_threat_ros/yolo/pistol.data and change names to 

	names = /../<catkin_ws>/src/usma_threat_ros/yolo/pistol.names

---
Required repos:
=======

	Darknet yolo: linkto.alexey.darknet

---
Configurations:
=======
These changes must be made for the package to work:

	echo "DARKNET_PATH=/path/to/your/darknet/installation" >> ~/.bashrc

---
Testing a configuration:
=======

	roslaunch usma_threat_ros logitech.launch
	roslaunch usma_threat_ros alexey_darknet.launch

