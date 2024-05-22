import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
from serial.serialutil import SerialException  # SerialException을 임포트합니다.

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')
        
        # 로거 설정
        self.get_logger().info('Initializing SerialReader node...')
        
        self.publisher_ = self.create_publisher(Float32, 'angle_topic', 10)

        try:
            self.serial_port = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
            self.get_logger().info('Serial port /dev/ttyUSB2 opened successfully')
        except SerialException as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            rclpy.shutdown()
            return

        self.timer = self.create_timer(0.1, self.read_serial_data)


    def read_serial_data(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f'Received line: {line}')  # 디버깅 메시지 추가
                if line.startswith("angle:"):
                    angle = float(line.split(":")[1])
                    msg = Float32()
                    msg.data = angle
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published angle: {angle}')
            except Exception as e:
                self.get_logger().error(f'Error reading serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialReader()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

