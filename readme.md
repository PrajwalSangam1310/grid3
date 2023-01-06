# Flipkart Grid 3.0 robotics challenge
## Team IIT Bhubaneswar

### Ros Scripts


- Brain.py
- controller_ros.py
- trajectory_planner_ros.py
- bot_identifier_ros.py


### Gazebo Related Scripts

- fake_pose_publisher.py
- gazebo_controller.py

### Utility scripts
    
- gridIdentifier.py 

### Launch Files

- **processing.launch**
    - Nodes corresponding to processing.
        - Brain.py
            - checks when to plan trajectory.
            - checks when to initiate dropping command.
            - checks start moving.
            - publishes the robot goal locations
                - induction points if the robot has dropped the package.
                - to the destination if the robot is at induction.

        - trajectory_planner_ros.py
            - publishes the planned trajectories.
            - publishes the planner_status
                - planner status will indicate if the robot has valid path, has reached the end of trajectory or it has to wait.
            - subscribes the robot goals
            - subscribes the robot poses

        - controller_ros.py
            - subscribes the current positions and current trajectories.
            - publishes the controller output accordingly.
            - encodes the command into 2 bytes to send them to the correponding bots.
        
    
- **visualize.launch**
    - To visualize the process
        - visualize_robots.py
            - subscribes to robot poses and displays them as marker array.
            - uses cube markers
        - visualize_rviz.py
            - shows the arena, ppublishes markers to represent the arena.
            - uses the cube marker.
        - display_trajectory.py
            - subscribes to the planned trajectory to show them in the rviz.
            - uses line strip marker type.

- **arduino_communication.launch**
    - runs the rosserial_arduino serial_node.py to connect to the connected arduino nrf transmitters.

- **bot_identifier_ros.py**
    - connects to the web cam for bot_identification.
    - publishes the robot poses.

- **Gazebo scripts**
    - fake_pose_publisher.py
        - reads the robot_states topic and publishes the robot poses in the form of image cordinates to mimic the bot identification(Used for testing)
    - gazebo_controller.py
        - subscribes the commands published by the controller_ros.py and converts them in the form of twist msgs to use /cmd_vel topic.
        - publishes on /robot1/cmd_vel, /robot2/cmd_vel, /robot3/cmd_vel, /robot4/cmd_vel.

- **Util Scripts.**
    - gridIdentifier.py
        - contains functions to convert the image pixels i,j to x,y and r,c and other functions.   

### Brief on communication architecture.

- **Transmission.**
    - arduino nano connected to pc using USB cable.
    - the arduino uses the ros.h library.
    - It also uses the custom msgs in order to subscribe to the command topics in ros.
    - it takes the commands published by the ros and codes them to 2 byte command.
    - these two byte commands are send through the nrf transmitter.

- **Command architecture.**
    - the first 2 bits represents the botID
    - the next three bits represent the Instruction.
        - Move forwart - 100
        - Halt - 000
        - Rotate clockwise - 101
        - Rotate Anti clockwise - 111
        - Drop - 001

    - next five bits represents the rotationalSpeed.
    - last five bits for linear speeds.
        - number of levels 32 using five bits.
        - maximum voltage level for the motors is 255.
        - so one level represents 8 units

- **Receiving.**
    - All the robots receive their commmands using the nrf module.
    - the received data is 2 bytes.
    - The arduino decodes to get the corresponding instructions and speeds.

- **Brief working steps.**
    - brain.py starts the process when the packages are loaded and robots are triggered.
    - the corresponding trajectory planning starts for that particular robot.
    - when the trajectory is planned the trajectory planner updates its status.
    - the controller looks at the planner status and trajectory to compute the instructions, forward speeds and rotational speeds.
    - controller publishes the corresponding commands on the topics
    - the arduino collects the topics using rosserial_arduino serial_node.py command.
    - the transmitter arduino sends it the robots.


### DifferentialDriveRobot

- This is the workspace use to launch the gazebo environment for the robots.
- The myrobot_description contains the xacro files of the robot.
- The myrobot_gazebo contains the stage and textures and world files.
