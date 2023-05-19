from kukalbrinterface import RobotConnection
import numpy as np

# Initializing the connection
kuka_robot = RobotConnection("localhost",enabled_zmq=True,port_zmq="5557")

# Sending a move ptp radian command (in degrees)
q = np.array([59.203307588872796,56.77165839581051,27.735234167284126,-78.85274368563117,-29.762410944525595,51.64901929708859,94.81149283526317])
kuka_robot.move_ptp_rad(q=q)

# Closing the connection
kuka_robot.close()