import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from example_interfaces.srv import SetBool
import serial
import time

class VacuumControlNode(Node):
    def __init__(self):
        super().__init__('vacuum_control')
        self.serial_conn = self.wait_for_serial('/dev/clearcore', 115200, timeout=10)

        self.enable_srv = self.create_service(Trigger, 'snaak_pneumatic/enable_vacuum', self.enable_callback)
        self.disable_srv = self.create_service(Trigger, 'snaak_pneumatic/disable_vacuum', self.disable_callback)
        self.eject_srv = self.create_service(SetBool, 'snaak_pneumatic/eject_vacuum', self.eject_callback)

        try:
            self.send_command("disable")
        except Exception as e:
            self.get_logger().error(f"Failed to send initial disable command: {e}")

    def wait_for_serial(self, port, baudrate, timeout=1):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Trying to connect to serial port: {port}")
                conn = serial.Serial(port, baudrate, timeout=timeout)
                self.get_logger().info("Serial connection established.")
                return conn
            except serial.SerialException as e:
                self.get_logger().warn(f"Serial connection failed: {e}. Retrying in 2 seconds...")
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f"Unexpected error during serial connection: {e}")
                time.sleep(2)
        raise RuntimeError("Shutting down before serial connection was established.")

    def send_command(self, command):
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.get_logger().info(f"Sending command: {command}")
                self.serial_conn.write((command + '\n').encode('utf-8'))
                time.sleep(0.1)  # Delay to allow the command to process
            else:
                raise serial.SerialException("Serial connection is not open.")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
            raise
        except Exception as e:
            self.get_logger().error(f"Unexpected error while sending command: {e}")
            raise

    def enable_callback(self, request, response):
        try:
            self.send_command("enable")
            response.success = True
            response.message = "Vacuum enabled"
        except Exception as e:
            response.success = False
            response.message = f"Failed to enable vacuum: {e}"
        return response

    def disable_callback(self, request, response):
        try:
            self.send_command("disable")
            response.success = True
            response.message = "Vacuum disabled"
        except Exception as e:
            response.success = False
            response.message = f"Failed to disable vacuum: {e}"
        return response

    def eject_callback(self, request, response):
        duration = 1000  # Could be parameterized
        try:
            self.send_command(f"eject {duration}")
            response.success = True
            response.message = f"Vacuum ejected for {duration} ms"
        except Exception as e:
            response.success = False
            response.message = f"Failed to eject vacuum: {e}"
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VacuumControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger('vacuum_control').error(f"Unhandled exception: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
