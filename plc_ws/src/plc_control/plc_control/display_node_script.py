#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from plc_interface.msg import Carrier, Time, NodeCheck, NodeBool
import cv2
import numpy as np


class DisplayNode(Node):

    def __init__(self):
        super().__init__("display_node_script")

        # We subscripe to the topic NodeCheck and publish to the topic NodeBool
        self.node_check = self.create_subscription(
            NodeCheck, "check", self.check_node, 10)
        self.node_bool = self.create_publisher(NodeBool, "bool", 10)

        # We subscripe to the Carrier topic and the end_time topic 
        self.carrier = self.create_subscription(
            Carrier, "carrier", self.display_carrier, 10)
        self.end_time = self.create_subscription(
            Time, "end_time", self.display_end_time, 10)
        # We create a timer to refresh the display
        self.timer = self.create_timer(0.1, self.display)
        
        # Opens a window for display
        self.display = np.zeros((600,1600), dtype=np.uint8)
        cv2.namedWindow("Display", cv2.WINDOW_AUTOSIZE)

        self.count = 0
        self.get_logger().info("Ready")


    # When it receieves a message on the topic CheckNode it runs the following code
    def check_node(self, msg_check: NodeCheck):
        # if the message recieves if "display" publish msg_bool to the topic node_bool, which will tell the control node that the display node is running
        if msg_check.check == "display":
            msg_bool = NodeBool()
            msg_bool.display = True
            self.node_bool.publish(msg_bool)
            self.destroy_subscription(self.node_check)
            self.get_logger().info("Displays here :)")

    # When the node gets a msg on the topic Carrier it runs the following code
    def display_carrier(self, msg: Carrier):
        self.count += 1

        # Creates a display that displayes the amount of carriers that has been processed
        self.display = np.zeros((600,1600), dtype=np.uint8)
        self.display = cv2.putText(
            img = self.display,
            text = f"Carriers Processed: {self.count}",
            org = (50, 100),
            fontFace = cv2.FONT_HERSHEY_DUPLEX,
            fontScale = 2.0,
            color = 255,
            thickness = 3)
        
        # Displays the id of the carrier
        self.display = cv2.putText(
            img = self.display,
            text = f"Carrier Id: {msg.id}",
            org = (50, 200),
            fontFace = cv2.FONT_HERSHEY_DUPLEX,
            fontScale = 2.0,
            color = 255,
            thickness = 3)
        
        # Displays the station id
        self.display = cv2.putText(
            img = self.display,
            text = f"Station Id: {msg.station}",
            org = (50, 270),
            fontFace = cv2.FONT_HERSHEY_DUPLEX,
            fontScale = 2.0,
            color = 255,
            thickness = 3)
        
        # Displays the starting time
        self.display = cv2.putText(
            img = self.display,
            text = f"Start Time: {msg.start_time}",
            org = (50, 370),
            fontFace = cv2.FONT_HERSHEY_DUPLEX,
            fontScale = 2.0,
            color = 255,
            thickness = 3)

        # Logs the id, station and start time
        self.get_logger().info('Number of Carriers: "%s"' % self.count)
        self.get_logger().info('Recive: "%s"' % msg.id)
        self.get_logger().info('Recive: "%s"' % msg.station)
        self.get_logger().info('Recive: "%s"' % msg.start_time)

    # When the node recieves a msg on the topic end_time the following function is run
    def display_end_time(self, msg: Time):
        # We display in the previously made display the endtime
        self.display = cv2.putText(
            img = self.display,
            text = f"End Time: {msg.end_time}",
            org = (50, 450),
            fontFace = cv2.FONT_HERSHEY_DUPLEX,
            fontScale = 2.0,
            color = 255,
            thickness = 3)
        
        # ???
        self.display

        # We log the endtime
        self.get_logger().info('Recive: "%s"' % msg.end_time)

    # Function for refreshing the display
    def display(self):
        cv2.imshow("Display", self.display)
        cv2.waitKey(1)
        #self.get_logger().info("hej")


def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()