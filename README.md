# Computation Robotics Warmup
### By: Andrew Kurtz and Sam Wisnoski

In this project we aimed to develop a foundational understanding of ROS2 and mobile robotics by developing a set of behaviors that range in complexity and sensor use. The first behaviors we developed were Teleoperation and Driving a Shape, by default a square. As we got more comfortable, we added wall following using proportional control and lidar, as well as Letterbox, which is a word drawing program that draws each letter by going to points sequentially. We then integrated three behaviors together using a finite-state-controller.

## Running the Code

## Teleoperation

## Drive Shape

### Overview

To drive in a shape, we designed the node to drive an arbitrary `n` sided regular polygon where `n >= 3`. The number of sides (`num_sides`) and side lengths (`side_len`) are defined using dynamic ROS parameters, so they can be adjusted when starting the node or mid run. The turn angle per a `num_sides` polygon is calculated geometrically with the math below. If new parameters are set mid-shape, they robot will start the new shape where it is.

```Python
sum_of_angles = (self.num_sides - 2) * math.radians(180)
self.turn_angle = math.radians(180) - sum_of_angles / self.num_sides
```

### Code Design

The node is implemented using a standard single threaded approach. It uses a `run_loop` method that runs at 10hz and publishes a Twist message to `cmd_vel` with the linear/angular velocity depending on where Neato is within shape. 

To add the ROS parameters to dynamically change the shape through `num_sides` and `side_len`, they are defined using the built in Node methods. A parameter callback method is also defined. When the parameters are updated and the callback is called, it recalculates the turn angle and resets the internal state of the location within shape.

## Wall Following

## Letterbox (idk if this is what you want to name it in report)

## Finite State Controller

## Conclusion




