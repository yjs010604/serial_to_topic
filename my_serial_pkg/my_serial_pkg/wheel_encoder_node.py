import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
from serial.serialutil import SerialException

class WheelEncoderNode(Node):
    def __init__(self):
        super().__init__('wheel_encoder_node')
        
        # 로거 설정
        self.get_logger().info('Initializing WheelEncoder node...')
        
        # 속도 데이터를 게시하기 위한 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Float32, 'speed_topic', 10)

        try:
            # 시리얼 포트 설정
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)  # 매우 짧은 시간 초과 설정
            self.get_logger().info('Serial port /dev/ttyACM0 opened successfully')
        except SerialException as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            rclpy.shutdown()
            return

        # 타이머 설정: 1초마다 read_serial_data 함수를 호출합니다.
        self.timer = self.create_timer(1, self.read_serial_data)

    def read_serial_data(self):
        while self.serial_port.in_waiting > 0:  # 시리얼 포트에 데이터가 있는 동안 계속 읽음
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f'Received line: {line}')  # 디버깅 메시지 추가
                if line.startswith("speed:"):
                    parts = line.split('speed:')
                    for part in parts:
                        if part.strip():
                            try:
                                speed_val = float(part.strip())
                                msg = Float32()
                                msg.data = speed_val
                                self.publisher_.publish(msg)
                                self.get_logger().info(f'Published speed: {speed_val}')
                            except ValueError as e:
                                self.get_logger().error(f'Error parsing speed value: {e}')
            except Exception as e:
                self.get_logger().error(f'Error reading serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoderNode()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
