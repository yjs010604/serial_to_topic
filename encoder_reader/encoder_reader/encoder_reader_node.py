import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
from serial.serialutil import SerialException

class EncoderReaderNode(Node):
    def __init__(self):
        super().__init__('encoder_reader_node')
        
        # 로거 설정
        self.get_logger().info('Initializing EncoderReader node...')
        
        # 퍼블리셔 설정
        self.speed_publisher = self.create_publisher(Float32, 'speed_topic', 10)
        self.angle_publisher = self.create_publisher(Float32, 'angle_topic', 10)

        # 시리얼 포트 설정
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('Serial port /dev/ttyACM0 opened successfully')
        except SerialException as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            rclpy.shutdown()
            return

        # 타이머 설정
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                #self.get_logger().info(f'Received line: {line}')  # 디버깅 메시지 추가
                
                # 데이터 형식이 "speed:0.00  s_angle:408.00"인 경우 처리
                if "speed:" in line and "s_angle:" in line:
                    parts = line.split()
                    if len(parts) == 2:
                        try:
                            speed_val = float(parts[0].split(":")[1])
                            angle_val = float(parts[1].split(":")[1])
                            
                            # speed 값을 퍼블리시
                            speed_msg = Float32()
                            speed_msg.data = speed_val
                            self.speed_publisher.publish(speed_msg)
                            self.get_logger().info(f'Published speed: {speed_val}')
                            
                            # s_angle 값을 퍼블리시
                            angle_msg = Float32()
                            angle_msg.data = angle_val
                            self.angle_publisher.publish(angle_msg)
                            self.get_logger().info(f'Published angle: {angle_val}')
                            
                        except ValueError as e:
                            self.get_logger().error(f'Error converting to float: {e}')
                    else:
                        self.get_logger().error(f'Unexpected data format: {line}')
                else:
                    self.get_logger().error(f'Unexpected data received: {line}')
                    
            except Exception as e:
                self.get_logger().error(f'Error reading serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
