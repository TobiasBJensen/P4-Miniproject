#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node

# Import our data type for the carrier
from plc_interface.msg import Carrier, Time, EstTime, NodeCheck, NodeBool
import socket
import re

# Import xml data file library
from xml.dom import minidom
import time


# We start the node the connection with the PLC
class PLCNode(Node):

    def __init__(self):
        super().__init__("plc_node_script")
        # Start up parameters for the node with a stardard IP
        self.declare_parameter('ip_addr', '192.168.0.106')

        # We publish to the topic NodeCheck and subscripe to the topic NodeBool
        self.node_check = self.create_publisher(NodeCheck, "check", 10)
        # When we recieve anything on our subscription, the wait_node function will run
        self.node_bool = self.create_subscription(
            NodeBool, "bool", self.wait_nodes, 10)

        # We publish to the topic Carrier and subscripe to the topic EstTime
        self.carrier = self.create_publisher(Carrier, "carrier", 10)
        self.est_time = self.create_subscription(
            EstTime, "est_time", self.tcp_send, 10)
        self.end_time = self.create_publisher(Time, "end_time", 10)

        # We create some starting parameters for checking whether the other nodes have started yet
        self.node = "control"

        # From the start the other nodes are set to not turned on, until we recieve information that they are turned on
        self.control = False
        self.display = False
        self.conn = None
        self.data = None

        # if tcp_up is set to False then the program is in simulation mode
        # if tcp_up is set to True then the tcp server is set up
        self.tcp_up = False

        # If in the launch xml file have a new IP, change the IP to the IP from the xml file and log it
        ip = self.get_parameter('ip_addr').get_parameter_value().string_value
        self.get_logger().info(str(ip))

        # Set up the TCP network to communicate with the PLC
        if self.tcp_up:
            HOST = ip
            PORT = 20000
            # Create a socket object
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Bind the socket to a specific address and port
            s.bind((HOST, PORT))
            # Listen for incoming connections
            s.listen(1)
            # Wait for a client to connect
            self.get_logger().info("Waiting for client")
            conn, addr = s.accept()
            self.get_logger().info("Client connected")


        # We create a timer that every 1 milisec checks the function pub_check to see if the other nodes are running
        # We log the messege "Any out there?" as an indication of that we are looking to see if our nodes are running
        self.timer = self.create_timer(1, self.pub_check)
        self.get_logger().info("Any out there?")
            

    # The function for checking whether the other nodes are running
    def pub_check(self):
        # We set the msg to be a NodeCheck message type
        msg = NodeCheck()
        msg.check = self.node
        # We send a msg saying "control" to the other nodes and
        self.node_check.publish(msg)
    # then nothing else happens than the code sending "control" to the node, until we recieve something back on the subscription of the nodes

    # When we have recieved something on the NodeBool subscription we run this code
    def wait_nodes(self, msg: NodeBool):
        # If we recieve the message that is from the control node, change the message send by the  timer to "display" and set the control node as running
        if msg.control == True:
            self.node = "display"
            self.control = True

        # If we recieve the message that is from the display node, change the message send by the timer to "" and set the display node as running
        if msg.display == True:
            self.node = ""
            self.display = True

        # If both nodes are running destroy the timer and log that all nodes are ready and run the function carrier
        if self.control and self.display:
            self.destroy_timer(self.timer)
            self.get_logger().info("All nodes ready")
            self.tcp_recv_carrier()



    def tcp_recv_carrier(self):
        if not self.tcp_up:
            # We sleep to simulate waiting for the message
            time.sleep(5)

            # Simulation code, we fake PLC data
            # We set the data to be byte instead of a string (we turn this into bytes to simulate the PLC since the PLC gives data in bytes)
            xml_msg = "<carrier><id>8</id><Station>2</Station><time>DT#time</time></carrier>\x00"
            self.data = bytes(xml_msg, 'utf-8')

        if self.tcp_up:
            # PLC Code
            # Receive data from the client
            self.data = self.conn.recv(1024)

        # When we have the data that we totally recieved from the PLC we send it to processing
        self.xmlCarrier()


    def xmlCarrier(self):
        # We make msg of the datatype of the Carrier topic
        msg = Carrier()

        # Convert byte data to string
        re_msg = str(self.data)

        # Isolates the part of the string that starts with <carrier> and ends with </carrier>
        # Find the part of the xml file that contains ID, station and StartT information for the carrier and saves that part as a "string"
        xmlString = re.findall('<carrier>.*</carrier>', re_msg)
        #print(xmlString[0]) (Debugging)

        # Analyses the structure of the xml string (so what other things are inside the carrier?) and turn in into data?
        # We take the first xmlString, since the findall can potentially find more than one
        docIn = minidom.parseString(xmlString[0])
        # We produce the data 'carrier' that contrains data and 'id' 'station' 'time'

        # We save the carrier data as carrier
        carrier = docIn.getElementsByTagName('carrier')

        # Saves the first data id (there can technically be more than 1) as ID and converts the ID into a int
        ID = carrier[0].getElementsByTagName('id')
        msg.id = int(ID[0].firstChild._get_data())
        #print("Carrier ID:", IDData)

        # Saves the first data station (there can technically be more than 1) as station and converts the staion number into a int
        station = carrier[0].getElementsByTagName('Station')
        msg.station = int(station[0].firstChild._get_data())
        #print("Station ID:", stationData)

        # Locate time element and reads the data in it
        time = carrier[0].getElementsByTagName('time')
        msg.start_time = time[0].firstChild._get_data()
        
        # We publish the recieved xml file in the topic carrier and log the message that we send
        self.carrier.publish(msg)
        self.get_logger().info('Publishing Id: "%s"' % msg.id)
        self.get_logger().info('Publishing Station: "%s"' % msg.station)
        self.get_logger().info('Publishing Start Time: "%s"' % msg.start_time)

    # Now nothing happens until we recieve information from the estTime subscription

    # We have recieved a message on the topic estTime and we run this function
    def tcp_send(self, msg: EstTime):

        # Converts string to bytes (the PLC likes bytes)
        newData = bytes(f'<time>{msg.est_time}</time>', 'utf-8')

        if not self.tcp_up:
            # We simulated waiting on the PLC recieving the message and work the job
            time.sleep(2)

        if self.tcp_up:
            # Code for sending answer to client(plc)
            self.conn.send(newData)

        # We log the estimate time
        self.get_logger().info('Recive Est Time: "%s"' % msg.est_time)

        # We run the function to recieve the end time
        self.tcp_recv_time()

    # Function for the recieval of the end time
    def tcp_recv_time(self):
        # We set the message type to that of the Time topic
        msg = Time()

        if not self.tcp_up:
            # Code for simulating the recieval of data from the PLC (which we turn into bytes because the PLC sends data in bytes)
            xml_msg = "<time>DT#time</time>\x00"
            self.data = bytes(xml_msg, 'utf-8')
        
        if self.tcp_up:
            # Receive data from the client
            self.data = self.conn.recv(1024)

        # We convert the data from the PLC into string
        re_msg = str(self.data)

        # Isolates the part of the string that starts with <time> and ends with </time>
        xmlString = str(re.findall('<time>.*</time>', re_msg)[0])

        # We find the data in this part of the string (Hard coded since it will always be the same place)
        msg.end_time = xmlString[6:-7]

        # We publish the msg endtime to the topic end_time
        self.end_time.publish(msg)
        self.get_logger().info('Publishing End Time: "%s"' % msg.end_time)
        
        # We run the code for recieving data from the PLC for the next carrier
        self.tcp_recv_carrier()


def main(args=None):
    rclpy.init(args=args)
    node = PLCNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()