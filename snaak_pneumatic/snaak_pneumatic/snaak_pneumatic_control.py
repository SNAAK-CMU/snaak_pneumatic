import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from example_interfaces.srv import SetBool
import serial
import time

class VacuumControlNode(Node):
    def __init__(self):
        super().__init__('vacuum_control')
        self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        # self.serial_conn = serial.Serial('COM12', 115200, timeout=1)

        self.enable_srv = self.create_service(Trigger, 'enable_vacuum', self.enable_callback)
        self.disable_srv = self.create_service(Trigger, 'disable_vacuum', self.disable_callback)
        self.eject_srv = self.create_service(SetBool, 'eject_vacuum', self.eject_callback)

    def send_command(self, command):
        self.serial_conn.write((command + '\n').encode('utf-8'))
        time.sleep(0.1)  # Small delay to ensure command is processed

    def enable_callback(self, request, response):
        self.send_command("enable")
        response.success = True
        response.message = "Vacuum enabled"
        return response

    def disable_callback(self, request, response):
        self.send_command("disable")
        response.success = True
        response.message = "Vacuum disabled"
        return response

    def eject_callback(self, request, response):
        # duration = max(100, min(5000, int(request.data)))  # Clamp duration between 100ms and 5000ms
        duration = 1000
        self.send_command(f"eject {duration}")
        response.success = True
        response.message = f"Vacuum ejected for {duration} ms"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VacuumControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
