## Getting Started


1. Copy the src-folder to your repository and build it
2. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fi3YWNfndrPTpXXkCT9sKvM9/
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. Run a test:
  a.) roslaunch simulation simulation.launch
  b.) rosrun controller_pkg traj_publisher
  c.) rosrun controller_pkg controller_node
  
The 1st drone will take off and fly in a circle. Use 'rqt_graph' to understand the structure of the project. 

The 1st drone receives commands on port 12346 and sends data from port 9998. The second drone receives commands on port 9997 and sends data from port 12345. 



# Tips

Here are a couple of hints regarding the implementation. The hints are just suggestions; you are free so solve the task differently:
- Generating point cloud from depth image: use depth_image_proc in http://wiki.ros.org/depth_
image_proc.
- Generating occupancy Grid: use Octomap in http://wiki.ros.org/octomap.
- Please ping us in case you have any questions or if you get stuck in some subtasks.
- Use a global map as your voxel grid representation. Use a smart resolution for your voxel grid representation (e.g. 1m).

