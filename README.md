# Ros workspace:
## Description:
Laboratories of legged robots course

## Requirements:
- Ros Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
- Pinocchio (https://stack-of-tasks.github.io/pinocchio/download.html)
- Pandas (>=0.24)

## Config .bashrc file
Open .basrhc file
<pre><code>$ gedit ~/.bashrc </code></pre>
Add these lines:
<pre><code># ROS
source /opt/ros/noetic/setup.bash 
# ROS WORKSPACE (don't modify)
export work_space="${HOME}/catkin_ws/labs_ws"
source $work_space/devel/setup.bash
</code></pre>    

## Config workspace:
Create the workspace
<pre><code>$ mkdir -p ~/catkin_ws/labs_ws/src 
$ cd ~/catkin_ws/
</code></pre>

Clone repository
<pre><code>$ git clone https://github.com/JhonPool4/legged_robots_labs_ws.git 
</code></pre>

Create necessary files
<pre><code>$ cd ~/catkin_ws/labs_ws/
$ catkin_make
</code></pre>

## Install library with useful functions:
Move to library location
<pre><code>$ roscd; cd ../src/labpythonlib
</code></pre>
Build library
<pre><code>$ python3 setup.py bdist_wheel
</code></pre>
Install library
<pre><code>$ pip3 install dist/labpythonlib-3.1.9-py3-none-any.whl
</code></pre>
