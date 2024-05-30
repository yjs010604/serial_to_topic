import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
from serial.serialutil import SerialException

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')
        
        # 로거 설정
        self.get_logger().info('Initializing SerialReader node...')
        
        self.publisher_ = self.create_publisher(Float32, 'angle_topic', 10)

        try:
            # 시리얼 포트 설정
            self.serial_port = serial.Serial('/dev/ttyUSB2', 115200, timeout=0.01)  # 매우 짧은 시간 초과 설정
            self.get_logger().info('Serial port /dev/ttyUSB2 opened successfully')
        except SerialException as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            rclpy.shutdown()
            return

        # 타이머 설정: 0.5초마다 read_serial_data 함수를 호출합니다.
        self.timer = self.create_timer(0.5, self.read_serial_data)

    def read_serial_data(self):
        while self.serial_port.in_waiting > 0:  # 시리얼 포트에 데이터가 있는 동안 계속 읽음
            try:
                # 시리얼 포트에서 한 줄을 읽고, UTF-8로 디코드하여 공백을 제거합니다.
                line = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f'Received line: {line}')  # 디버깅 메시지 추가
                if line.startswith("angle:"):
                    # "angle:" 뒤의 값을 부동 소수점 숫자로 변환합니다.
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
