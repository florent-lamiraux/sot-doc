# Control of a UR10 robot {#tutorial_ur10}

\section tutorial_ur10_introduction Introduction

This tutorial explains how to control the stack of tasks running in roscontrol
on a UR10 robot.

It is assumed that:

1. the stack of tasks is installed either from binary packages or from source,
2. \c sot-universal-robot is installed (no binary package yet available).

\section tutorial_ur10_starting_ros_control Starting the SoT within roscontrol

To start simulating \c roscontrol running in the UR10 robot in Gazebo, type the following command in a terminal:

    roslaunch agimus_demos ur10_pointing_simulation.launch

You should see the following window.

\image html roscontrol-ur10-gazebo.png "Simulation of roscontrol in Gazebo" width=100%

In this application, gazebo simulates UR10 robot controlled by \c roscontrol. The next step consists in loading the SoT controller into \c roscontrol. To do so, type in another terminal:

    roslaunch sot_universal_robot controller.launch simulation:=true

You should see 2 lines like

    [INFO] [1658753202.754842, 16.418000]: Controller Spawner: Loaded controllers: sot_controller
    [INFO] [1658753202.786559, 16.439000]: Started controllers: sot_controller

meaning that the Stack of Task has been loaded by \c roscontrol and that the controller has been started. Every control period, \c roscontrol asks the \c sot_controller for a reference position of the 6 axes of the robot and controls the robot around this reference configuration.

\section tutorial_ur10_controlling_the_robot Controlling the robot with the Stack of Tasks

At startup, \c sot_controller simply reads the encoder values and sends this constant position as a reference configuration. Hence, the robot remains static.

After calling ROS service \c /start_dynamic_graph, \c sot_controller reads input signal \c control of the \c device, integrates this velocity to compute the reference configuration.

If we call \c /start_dynamic_graph before initializing \c control signal, \c sot_controller will raise an exception and gazebo will stop.

\image html device.svg "The input signal \c control of entity \c device is the desired velocity. This velocity is integrated into output signal \c device.state and sent as the reference configuration of the robot.

\subsection tutorial_ur10_controlling_the_robot_python_communication Communicating with the Stack of Tasks via Python

To control the stack of tasks via python, type in a bash terminal the following command:

    rosrun dynamic_graph_bridge run_command

This will open a python channel with the python interpreter embedded in the Stack of Tasks. All the commands typed in this terminal are sent to the remote interpreter and the result is received and displayed in the terminal.

In the terminal, type the following commands:

    import numpy
    u = numpy.zeros(6)
    u[0] = .01
    robot.device.control.value = u

Then in another bash terminal, type

    rosservice call /start_dynamic_graph

The latter command asks the \c sot_controller to start taking into account
signal \c robot.device.control to compute the reference configuration of the
robot.

As a consequence, you should see the robot moving the first axis in Gazebo.

To stop the motion, type the following commands in the remote python terminal:

    u = numpy.zeros(6)
    robot.device.control.value = u

\subsection tutorial_ur10_controlling_the_robot_simple_computation_graph A simple computation graph

In this section, we will create a simple computation graph, that is a graph
that computes the value of the control signal by evaluating an error depending
on the state of the robot. Let us assume we want to reach configuration

    q_goal = [pi/6, -pi/2, pi/2, 0, 0, 0,]

We will first create an entity that will substract this value to the robot state
signal to compute the error:

    from math import pi
    from dynamic_graph import plug
    from dynamic_graph.sot.core.operator import Substract_of_vector
    error = Substract_of_vector('error')
    plug(robot.device.state, error.sin1)
    error.sin2.value = numpy.array([pi/6, -pi/2, pi/2, 0, 0, 0,])

Then we will multiply this error by a negative gain that we will plug into
the device control signal:

    from dynamic_graph.sot.core.operator import Multiply_double_vector
    control = Multiply_double_vector('control')
    control.sin1.value = -0.01
    plug(error.sout, control.sin2)
    plug(control.sout, robot.device.control)

After typing the last line, the robot starts moving to the desired
configuration. To display the current configuration, type:

    robot.device.state.value

To display the current error, type:

    error.sout.value

To make the robot converge faster, you can change the gain:

    control.sin1.value = -0.1

It is also possible to display the graph in format dot:

    from dynamic_graph import writeGraph
    writeGraph('/tmp/graph.dot')

Then in a bash terminal:

    cd /tmp
    dot -T svg -o graph.svg graph.dot

This will produce a svg file representing the control graph as below

\image html simple-control-graph.svg "A Simple control graph"
