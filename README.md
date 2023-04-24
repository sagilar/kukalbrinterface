# kukainterface
Kuka - Python to Java interface for Kuka LBR iiwa 7 robotic arms.

## Python module
This module enables the client connection to a Kuka LBR iiwa 7 robotic arm running a TCP connection.

The class ```RobotConnection``` creates an object with moving, closing, and connecting methods.

See the example code in ```test_kukainterface.py```

## Java module
The folder ```JavaServer``` contains the Java code to be installed in the Kuka controller through the Sunrise Workbench application.
The application enables a non-blocking Java server, which receives external commands via, e.g., the kukainterface Python module.