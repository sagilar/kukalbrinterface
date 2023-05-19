import tcpnonblock
import os
import zmq
import numpy as np

# Requires Java server connection running in the Kuka Robot controller
class RobotConnection:
    def __init__(self,ip_addr,enabled_zmq=False,port_zmq="5557",threaded=True,filename="joint_position_data.csv"):
        self.ip_addr = ip_addr
        self.threaded = threaded
        self.port_zmq = port_zmq
        self.filename = filename
        self.enabled_zmq = enabled_zmq
        if self.enabled_zmq:
            self.context = zmq.Context()
            self.socket_zmq = self.context.socket(zmq.PUB)
            self.socket_zmq.bind("tcp://*:" + port_zmq)
        self.client = tcpnonblock.TCPSocketClient(threaded=True)

        @self.client.on_open
        def on_open():
            print("Connected to Robot Server")
            self.client.send("Client connected\n")
        
        @self.client.on_close
        def on_close():
            print("Disconnected from Robot Server")

        @self.client.on_message
        def on_message(msg):
            if os.path.isfile(filename):
                pass
            else:
                with open(filename,"w") as file:
                    header = "timestamp,actual_q_0,actual_q_1,actual_q_2,actual_q_3,actual_q_4,actual_q_5,actual_q_6,target_q_0,target_q_1,target_q_2,target_q_3,target_q_4,target_q_5,target_q_6,"
                    header += "measured_tq_0,measured_tq_1,measured_tq_2,measured_tq_3,measured_tq_4,measured_tq_5,measured_tq_6,external_tq_0,external_tq_1,external_tq_2,external_tq_3,external_tq_4,external_tq_5,external_tq_6,"
                    header += "force_X,force_Y,force_Z\n"
                    file.write(header)
                    file.close()

            try:
                with open(filename,"a") as file:
                    file.write(msg)
                try:
                    splt_msg = str(msg).split(",")
                    if len(splt_msg) > 5:
                        positions = splt_msg[1:8]
                        for i in range(len(positions)):
                            if self.enabled_zmq:
                                self.socket_zmq.send_string(f"actual_q_{i} {positions[i]}")
                except Exception as e:
                    pass
            except Exception as exc:
                pass
        
        self._connect()

    def _connect(self):
        self.client.connect(self.ip_addr, 30001)
    
    def _close(self):
        self.client.send("Client disconnected\n")
        try:
            self.client.close()
        except:
            print("Exception on closing client thread")
            self.client.close()
        if self.enabled_zmq:
            self.socket_zmq.send_string("stop stop")
            self.socket_zmq.close()

    def close(self):
        self._close()

    def move_ptp_rad(self,q=np.zeros(7)):
        q_string = np.array2string(q, precision=6, separator=',').replace(" ", "").replace("[","").replace("]","")
        if (".," in q_string):
            q_string = q_string.replace(".,",",")
        if(q_string[-1]=="."):
            q_string = q_string.replace(".","")
        self.client.send("moveptprad(" + q_string + ")\n")

    def move_ptp_cart(self,q=np.zeros(6)):
        q_string = np.array2string(q, precision=6, separator=',').replace(" ", "").replace("[","").replace("]","")
        if (".," in q_string):
            q_string = q_string.replace(".,",",")
        if(q_string[-1]=="."):
            q_string = q_string.replace(".","")
        self.client.send("moveptpcart(" + q_string + ")\n")

    def move_l(self,q=np.zeros(6)):
        q_string = np.array2string(q, precision=6, separator=',').replace(" ", "").replace("[","").replace("]","")
        if (".," in q_string):
            q_string = q_string.replace(".,",",")
        if(q_string[-1]=="."):
            q_string = q_string.replace(".","")
        self.client.send("movelin(" + q_string + ")\n")

    def move_l_rel(self,q=np.zeros(6)):
        q_string = np.array2string(q, precision=6, separator=',').replace(" ", "").replace("[","").replace("]","")
        if (".," in q_string):
            q_string = q_string.replace(".,",",")
        if(q_string[-1]=="."):
            q_string = q_string.replace(".","")
        self.client.send("movelrel(" + q_string + ")\n")

    def move_circ(self,q=np.zeros(6)):
        q_string = np.array2string(q, precision=6, separator=',').replace(" ", "").replace("[","").replace("]","")
        if (".," in q_string):
            q_string = q_string.replace(".,",",")
        if(q_string[-1]=="."):
            q_string = q_string.replace(".","")
        self.client.send("movecirc(" + q_string + ")\n")

        
