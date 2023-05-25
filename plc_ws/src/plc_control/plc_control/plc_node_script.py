#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from plc_interface.msg import Carrier, Time, EstTime, NodeCheck, NodeBool
import socket
import re
from xml.dom import minidom
import time


class PLCNode(Node):

    def __init__(self):
        super().__init__("plc_node_script")
        self.declare_parameter('ip_addr', '192.168.0.106')

        self.node_check = self.create_publisher(NodeCheck, "check", 10)
        self.node_bool = self.create_subscription(
            NodeBool, "bool", self.wait_nodes, 10)

        self.carrier = self.create_publisher(Carrier, "carrier", 10)
        self.est_time = self.create_subscription(
            EstTime, "est_time", self.tcp_send, 10)
        self.end_time = self.create_publisher(Time, "end_time", 10)
        self.node = "control"
        self.control = False
        self.display = False
        self.conn = None
        self.data = None

        time.sleep(2)

        ip = self.get_parameter('ip_addr').get_parameter_value().string_value
        self.get_logger().info(str(ip))
        
        """
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
        """

        self.timer = self.create_timer(1, self.pub_check)
        self.get_logger().info("Any out there?")
            


    def pub_check(self):
        msg = NodeCheck()
        msg.check = self.node
        self.node_check.publish(msg)


    def wait_nodes(self, msg: NodeBool):
        if msg.control == True:
            self.node = "display"
            self.control = True
        if msg.display == True:
            self.node = ""
            self.display = True
        if self.control and self.display:
            self.destroy_timer(self.timer)
            self.get_logger().info("All nodes ready")
            self.tcp_recv_carrier()



    def tcp_recv_carrier(self):
        time.sleep(5)

        xml_msg = "<carrier><id>8</id><Station>2</Station><time>DT#time</time></carrier>\x00"
        self.data = bytes(xml_msg, 'utf-8')

        # Receive data from the client
        #data = self.conn.recv(1024)

        # Process the data
        self.xmlCarrier()


    def xmlCarrier(self):
        msg = Carrier()

        # Convert byte data to string
        re_msg = str(self.data)

        # Isolates the part of the string that starts with <carrier> and ends with </carrier>
        xmlString = re.findall('<carrier>.*</carrier>', re_msg)
        #print(xmlString[0])

        # Parses the xml string
        docIn = minidom.parseString(xmlString[0])

        # Locate carrier element
        carrier = docIn.getElementsByTagName('carrier')

        # Locate ID element and reads the data in it
        ID = carrier[0].getElementsByTagName('id')
        msg.id = int(ID[0].firstChild._get_data())
        #print("Carrier ID:", IDData)

        # Locate station element and reads the data in it
        station = carrier[0].getElementsByTagName('Station')
        msg.station = int(station[0].firstChild._get_data())
        #print("Station ID:", stationData)

        # Locate time element and reads the data in it
        time = carrier[0].getElementsByTagName('time')
        msg.start_time = time[0].firstChild._get_data()
        
        self.carrier.publish(msg)
        self.get_logger().info('Publishing Id: "%s"' % msg.id)
        self.get_logger().info('Publishing Station: "%s"' % msg.station)
        self.get_logger().info('Publishing Start Time: "%s"' % msg.start_time)


    def tcp_send(self, msg: EstTime):
        # Converts string to bytes
        newData = bytes(f'<time>{msg.est_time}</time>', 'utf-8')

        # Sends answer to client(plc)
        #conn.send(newData)

        self.get_logger().info('Recive Est Time: "%s"' % msg.est_time)

        time.sleep(1)
        self.tcp_recv_time()


    def tcp_recv_time(self):
        msg = Time()

        xml_msg = "<time>DT#time</time>\x00"
        self.data = bytes(xml_msg, 'utf-8')
        
        # Receive data from the client
        #data = self.conn.recv(1024)

        re_msg = str(self.data)

        # Isolates the part of the string that starts with <carrier> and ends with </carrier>
        xmlString = str(re.findall('<time>.*</time>', re_msg)[0])
        msg.end_time = xmlString[6:-7]
        self.end_time.publish(msg)
        self.get_logger().info('Publishing End Time: "%s"' % msg.end_time)
        
        self.tcp_recv_carrier()


def main(args=None):
    rclpy.init(args=args)
    node = PLCNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()