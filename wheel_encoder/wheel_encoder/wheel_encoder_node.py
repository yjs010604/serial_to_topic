import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
from serial.serialutil import SerialException
import os

class WheelEncoderNode(Node):
    def __init__(self):
        super().__init__('wheel_encoder_node')
        
        # 로거 설정
        self.get_logger().info('Initializing WheelEncoder node...')
        # 속도 데이터를 게시하기 위한 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Float32, 'speed_topic', 10)

        # 시리얼 포트 설정
        try:
            # 시리얼 포트 '/dev/ttyACM0'를 115200 보드레이트로 엽니다.
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('Serial port /dev/ttyACM0 opened successfully')
        except SerialException as e:
            # 시리얼 포트를 열지 못할 경우 오류 메시지를 출력하고 노드를 종료합니다.
            self.get_logger().error(f'Could not open serial port: {e}')
            rclpy.shutdown()
            return

        # 타이머 설정: 0.1초마다 read_serial_data 함수를 호출합니다.
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        # 시리얼 포트에 대기 중인 데이터가 있는지 확인합니다.
        if self.serial_port.in_waiting > 0:
            try:
                # 시리얼 포트에서 한 줄을 읽고, UTF-8로 디코드하여 공백을 제거합니다.
                line = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f'Received line: {line}')  # 디버깅 메시지 추가
                # 수신된 데이터가 "speed:"로 시작하는지 확인합니다.
                if line.startswith("speed:"):
                    # "speed:" 뒤의 값을 부동 소수점 숫자로 변환합니다.
                    speed_val = float(line.split(":")[1].strip())
                    # Float32 메시지를 생성하고 데이터를 설정합니다.
                    msg = Float32()
                    msg.data = speed_val
                    # 퍼블리셔를 통해 메시지를 게시합니다.
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published speed: {speed_val}')
            except Exception as e:
                # 데이터를 읽는 동안 오류가 발생하면 오류 메시지를 출력합니다.
                self.get_logger().error(f'Error reading serial data: {e}')

def main(args=None):
    # ROS 2 클라이언트를 초기화합니다.
    rclpy.init(args=args)
    node = None
    try:
        # WheelEncoderNode 인스턴스를 생성하고, 스핀하여 노드를 실행합니다.
        node = WheelEncoderNode()
        rclpy.spin(node)
    except Exception as e:
        # 메인 함수에서 오류가 발생하면 오류 메시지를 출력합니다.
        rclpy.logging.get_logger('wheel_encoder_node').error(f'Error in main: {e}')
    finally:
        # 노드를 안전하게 종료합니다.
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
