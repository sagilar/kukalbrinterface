# kukalbrinterface
KukaLBR - Python to Java interface for Kuka LBR iiwa 7 robotic arms.

## Python module
The module ```kukalbrinterface``` enables the client connection to a Kuka LBR iiwa 7 robotic arm running a TCP connection.

The class ```RobotConnection``` creates an object with moving, connecting, and closing methods.
Additionally, it stores the data coming from the robot to a CSV file and publishes joint positions using ZeroMQ.

See the example code in ```test_kukalbrinterface.py```

## Java module
The folder ```JavaServer``` contains the Java code to be installed in the Kuka controller through the Sunrise Workbench application.
The application enables a non-blocking Java server, which receives external commands via, e.g., the kukalbrinterface Python module.

Note the Python module won't work if the Java application is not properly installed in the Kuka controller.
