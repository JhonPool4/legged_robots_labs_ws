Ros workspace:
-------------

Info:
----
    - Laboratories of legged robots 

Requirements:
-------------
    - Ros Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
    - Pinocchio (review how_to_install_pinocchio)
    - Pandas (>=2.24)

Config .bashrc:
----------------


Config workspace:
----------------
    - mkdir -p ~/catkin_ws/labs_ws/src
    - cd ~/catkin_ws/
    - git clone https://github.com/JhonPool4/legged_robots_labs_ws.git
    - cd ~/catkin_ws/labs_ws/ 
    - catkin_make

Install library:
---------------
    # move to library location
    - roscd; cd ../src/labpythonlib
    # build library
    - python3 setup.py bdist_wheel
    # install library
    - pip3 install labpythonlib-0.0.1-py3-none-any.whl