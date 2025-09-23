# Computation Robotics Warmup Project 
### By: Andrew Kurtz and Sam Wisnoski

# 1. Introduction

In this project we aimed to develop a foundational understanding of ROS2 and mobile robotics by developing a set of behaviors that range in complexity and sensor use. The first behaviors we developed were Teleoperation and Driving a Shape, by default a square. As we got more comfortable, we added wall following using proportional control and lidar, as well as Letterbox, which is a word drawing program that draws each letter by going to points sequentially. We then integrated three behaviors together using a finite-state-controller. Below, we go into detail about the design decisions for each of these behaviors, as well as reflect on the overall project.

#### Running the Code

In order to run the code, please follow the setup for ROS2 + Gazebo enviornments, found on the [2025 CompRobo Website](https://comprobo25.github.io/How%20to/setup_your_environment). Once you have properly set up your environment, you can clone this repository to your ros2_ws. Remember to run `colcon build --symlink-install` and `source ~/ros2_ws/install/setup.bash` after creating the new package found in our repo. 

Lastly, to run the code, initialize the gazebo environment with: 
`ros2 launch neato2_gazebo neato_gauntlet_world.py`
And run one of behavior nodes with: 
`ros2 run -package-name -node-name`


# 2. Behaviors + Finite State Machine 

## 2a. Teleoperation

#### Overview

Our Teleoperation behavior is a simple node that locally reads input from a laptop and publishes a Twist to the 'cmd_vel' topic in response to the input. While running the node, the following keystrokes are taken as input: 
* [W] - Move Forward 
* [A] - Rotate Left 
* [S] - Move Backward 
* [D] - Rotate Right
* Any Other Key - Stop Moving
* [CTRL-C] - Stop the Node

#### Code Design

There are three main functions which help us operate teleop. In our main run_loop, we repeatedly call `self.get_key()` to recieve the most recent keystroke and `self.steer()` to determine what to do with that keystroke. If one of the "movement" keystrokes are entered, we call a generic `drive()` function, which publishes a given linear and angular velocity to the `cmd_vel` topic.


## 2b. Drive Shape

#### Overview

To drive in a shape, we designed the node to drive an arbitrary `n` sided regular polygon where `n >= 3`. The number of sides (`num_sides`) and side lengths (`side_len`) are defined using dynamic ROS parameters, so they can be adjusted when starting the node or mid run. The turn angle per a `num_sides` polygon is calculated geometrically with the math below. If new parameters are set mid-shape, they robot will start the new shape where it is.

```Python
sum_of_angles = (self.num_sides - 2) * math.radians(180)
self.turn_angle = math.radians(180) - sum_of_angles / self.num_sides
```

#### Code Design

The node is implemented using a standard single threaded approach. It uses a `run_loop` method that runs at 10hz and publishes a Twist message to `cmd_vel` with the linear/angular velocity depending on where Neato is within shape. 

To add the ROS parameters to dynamically change the shape through `num_sides` and `side_len`, they are defined using the built in Node methods. A parameter callback method is also defined. When the parameters are updated and the callback is called, it recalculates the turn angle and resets the internal state of the location within shape.

## 2c. Wall Following

#### Overview

The wall follower node is designed to drive parallel along a wall that is within one meter of the Neato. To accomplish, the program must first must detect which side the wall is on, then the Neato's orientation relative to the wall, and finally choose an angular velocity commamnd to correct the direction. 

#### Code Design

To detect the wall side, we chose the simple heuristic that the side with the wall will have more lidar pings. To calculate this, we simply loop through all the lidar points and tally the points between 10 cm and 1 meter on the left and right side, then compare.

To calculate the orientation relative to the wall, we boil the problem down to finding a way measure the error between the current and target directions. This will be used as the input to the proportional controller in the last step. Because we can scale it by a constant in the controller, we only need a value that correlates to the error, not an exact angle difference. This simplifies the problem.

To find this error, we can compare the lidar distances of corresponding angles that mirror across the Neato's x-axis as shown in the diagram below. When the Neato is parallel to the wall, all the corresponding distances (shown in same color) will be equal. If the wall is to the right of the Neato and the Neato is facing towards the wall, the distances in the first quadrant will be greater than the distances in the 4th quadrant. If it is facing away from the wall, the bottom distances will be greater than the top distances. It is the opposite if the wall is to left. 

<img src="assets/wall_error.png" width="400"/>

Finally, the controller can simply set the neato to maintain a constant forward velocity and set the angular velocity proportional to the error as shown the code below. Since counterclockwise is negative angular velocity, we need to negate the result to match our setup.

```Python
msg.angular.z = -self.kp * float(self.error)
```

#### Improvements

There are a number of improvements that could be made to this behavior. If we had more time, we would have added better wall detection logic so that it doesn't need to assume there will always be a wall. This could be done using RANSAC which excels at noisy data. Additionally, we could have implemented turning logic so that once it reaches a corner, it could turn to follow the next wall.

## 2d. Letterbox 

#### Overview

Our last behavior is letterbox. Letterbox uses a keyboard listen that specifically waits for the user to press the "enter" key, at which time it will prompt the user to enter a string of text. The neato will then navigate to the original world frame (0,0) and trace out the provided string, returning to (0,0) between each point. 

<img src="assets/letter_tracing.gif" alt="Neato Traces ABC in Gazebo" width="400" height="500">

#### Code Design

Letterbox has a single subcription to the `\odom` topic, which allows it to read the Neato's odometry data (it's x, y, and z positioning). It also has a single publisher to the `cmd_vel` topic, allowing us to control the robot and draw letters. 

Unlike our other behaviors, letterbox is multi-threaded, allowing for two additional threads running alongside the main ROS loop: a keyboard listener and a letter-drawing loop. The keyboard listener waits for the Enter key, at which point the drawing loop is activated. It prompts the user for a word, then uses the Hershey font library to convert each character into "stroke" data. These "strokes" represent each character as a sequence of line segments, and those segments are broken down into (x,y) coordinate points. If we convert these points into the world frame we can then trace the letter. Because of this multi-threaded approach, the robot to trace letters interactively without blocking on user input.

The core movement logic is handled in `go_to_point()`, which uses simple trigonometry to drive to the (x,y) segmented coordinate points. The function calculates the target angle with atan2, rotates the robot until aligned, then drives forward the required distance. The `\odom` topic is used to track the robot’s current pose and update orientation, with a simple function converting quaternion data converted into yaw. Ither supporting functions include `collect_input()` for user prompts, `_draw_letter()` for tracing each stroke and resetting to the origin after every character, and `drive()` for publishing velocity commands based on linear and angular velocity. 

Our finite state controller is designed to switch between 

# 3. Conclusion

## 3a. Challenges

## 3b. Improvements 

## 3c. Key Takeaways 


