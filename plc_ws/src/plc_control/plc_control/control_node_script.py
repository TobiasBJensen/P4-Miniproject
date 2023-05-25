#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from plc_interface.msg import Carrier, Time, EstTime, NodeCheck, NodeBool
import pandas as pd
import time


class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node_script")
        self.node_check = self.create_subscription(
            NodeCheck, "check", self.check_node, 10)
        self.node_bool = self.create_publisher(NodeBool, "bool", 10)

        self.carrier = self.create_subscription(
            Carrier, "carrier", self.check_list, 10)
        
        self.est_time = self.create_publisher(EstTime, "est_time", 10)
        self.end_time = self.create_subscription(
            Time, "end_time", self.update_data, 10)
            
        self.msg_carrier = None

        self.get_logger().info('Accessing Data Files...')
        self.pre_time_list = pd.read_csv('/home/tobias/plc_ws/src/plc_control/plc_control/data_files/preTimeList.csv', sep=';', index_col=0)
        self.carrier_data = pd.read_csv('/home/tobias/plc_ws/src/plc_control/plc_control/data_files/carrierData.csv', sep=';', index_col=0)

        self.get_logger().info('Ready')


    def check_node(self, msg_check: NodeCheck):
        if msg_check.check == "control":
            msg_bool = NodeBool()
            msg_bool.control = True
            self.node_bool.publish(msg_bool)
            self.destroy_subscription(self.node_check)
            self.get_logger().info("Controls here :)")

    
    def check_list(self, msg_sub: Carrier):
        msg_pub = EstTime()

        self.msg_carrier = msg_sub

        self.get_logger().info('Recive Id: "%s"' % msg_sub.id)
        self.get_logger().info('Recive Station: "%s"' % msg_sub.station)
        self.get_logger().info('Recive Start Time: "%s"' % msg_sub.start_time)

        msg_pub.est_time = int(self.pre_time_list.values[int(msg_sub.id)-1, int(msg_sub.station)-1])
        self.est_time.publish(msg_pub)
        self.get_logger().info('Publishing Est Time: "%s"' % msg_pub.est_time)

    
    def update_data(self, msg_time: Time):
        self.get_logger().info('Recive End Time: "%s"' % msg_time.end_time)

        self.carrier_data.values[int(self.msg_carrier.id)-1, 
                                 int(self.msg_carrier.station)-1] = f'StartTime:{self.msg_carrier.start_time},EndTime:{msg_time._end_time}'
        #self.get_logger().info(str(self.carrier_data.values[int(msg_carrier.id)-1, int(msg_carrier.station)-1]))
        #self.get_logger().info(str(f'StartTime:{msg_carrier.start_time},EndTime:{msg_time._end_time}'))
        
        self.carrier_data.to_csv('/home/tobias/plc_ws/src/plc_control/plc_control/data_files/carrierData.csv', sep=';')
        self.carrier_data.to_excel('/home/tobias/plc_ws/src/plc_control/plc_control/data_files/carrierDataX.xlsx') 
        


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
