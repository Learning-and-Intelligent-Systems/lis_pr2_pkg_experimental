Prior to running tests make sure robot is in a nice start state. Move arms up at the side and don't have them twisted all crazy and stuff.
Often motion planning fails because the start state is terrible so don't do that.

---------


# Launch Code

this will launch the code with rviz and object dector launched:

    roslaunch ari_moveit_tests all.launch rviz:=true vision:=true


What you should see after running:
* robot
* table detector
* bounding boxes over objects
* interactive moveable arms -- play around with the motion planning panel to make it work



----------


Test 1: lift arm up a couple of inches

    rosrun ari_moveit_tests basic.py


* look at rviz when you are running - there will be a trajectory robot in purple that will loop through the desired motion plan. Type y to physcially move the robot


-----------

Test 2: pick up an object

**Setup**

1. Manually open the right hand of the robot
2. put an object in front of the robot on a table
3. Make sure robot in nice starting state and looking at object
4. There should be abounding box around the object -- if it is purple and says 0 it was dected by the object detector.  There should be another box called 'bb' . 'bb' is the actual box that will be sent to the planner.  When running the code make sure bb is where you want it to be.  If not, wait a while and make sure robot arms are not in the way.

Run the code with:
    
    rosrun ari_moveit_tests pick_up.py

###ALWAYS VERIFY PURPLE TRAJECTRY BEFORE HITTING ENTER !!!!

Hit enter for the different queries.  The robot should approach the object in order to grasp it.  I don't have gripper closing and picking implemented.  
Hit enter to go back to home (verify trajectory first) and it will loop through the process again.







