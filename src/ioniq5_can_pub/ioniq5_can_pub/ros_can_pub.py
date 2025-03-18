import rclpy
from rclpy.node import Node
from std_msgs.msg import Header  
import can
import cantools
import threading
import queue
import asyncio
from autoware_vehicle_msgs.msg import *
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor

# 기어 변환 딕셔너리
Gear_DISP_dict = { 
    0 : 22,
    7 : 20,
    6 : 1,
    5 : 2,
}

class CanReceiver(Node):
    def __init__(self):
        super().__init__('can_receiver_node')

        # ROS 2 퍼블리셔 생성
        self.pub_GearReport = self.create_publisher(GearReport, '/vehicle/status/GearReport', 10)
        self.pub_SteeringReport = self.create_publisher(SteeringReport, '/vehicle/status/SteeringReport', 10)
        self.pub_VelocityReport = self.create_publisher(VelocityReport, '/vehicle/status/VelocityReport', 10)

        # CAN 인터페이스 설정 (ThreadSafeBus 사용)
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0')

        # DBC 파일 로드 및 메시지 캐싱 (속도 향상)
        self.dbc = cantools.database.load_file('/home/kiapi/KIAPI_ioniq5_can_pub/KIAPI.dbc')
        self.dbc_messages = {msg.frame_id: msg for msg in self.dbc.messages}

        # CAN 메시지 수신 큐 (멀티스레드, 최대 크기 제한)
        self.msg_queue = queue.Queue(maxsize=1000)

        # CAN 데이터 수신을 비동기 스레드에서 실행
        self.can_thread = threading.Thread(target=self.can_receive_thread, daemon=True)
        self.can_thread.start()

        # 데이터 수신 및 퍼블리시 (5ms 주기)
        self.create_timer(0.002, self.timer_callback)  # 2ms 주기

    def can_receive_thread(self):
        """CAN 메시지를 별도 스레드에서 수신하여 큐에 저장"""
        while rclpy.ok():
            try:
                message = self.bus.recv(timeout=0.0001)  # 타임아웃 최소화
                if message:
                    if not self.msg_queue.full():  # 큐가 가득 찬 경우 데이터 삭제 방지
                        self.msg_queue.put_nowait(message)
            except queue.Full:
                pass  # 큐가 가득 차면 버퍼 오버플로우 방지

    def timer_callback(self):
        """메인 스레드에서 CAN 데이터를 가져와 퍼블리시"""
        while not self.msg_queue.empty():
            try:
                message = self.msg_queue.get_nowait()
            except queue.Empty:
                return  # 큐가 비어 있으면 처리 안 함

            decoded_signals = self.decode_message(message)
            if not decoded_signals:
                continue  # 디코딩 실패 시 스킵

            # 공통 헤더 설정
            header = Header()
            header.frame_id = 'base_link'
            header.stamp = Clock().now().to_msg()

            # 메시지 처리
            if message.arbitration_id == 0x710:
                SteeringReport_msg = SteeringReport()
                SteeringReport_msg.steering_tire_angle = decoded_signals.get('StrAng', 0)
                SteeringReport_msg.stamp = header.stamp
                self.pub_SteeringReport.publish(SteeringReport_msg)

            elif message.arbitration_id == 0x711:
                GearReport_msg = GearReport()
                gear_disp = decoded_signals.get('Gear_DISP', 0)
                GearReport_msg.report = Gear_DISP_dict.get(gear_disp, 0)
                GearReport_msg.stamp = header.stamp
                self.pub_GearReport.publish(GearReport_msg)

            elif message.arbitration_id == 0x713:
                VelocityReport_msg = VelocityReport()
                VelocityReport_msg.longitudinal_velocity = decoded_signals.get('Long_Accel', 0)
                VelocityReport_msg.lateral_velocity = decoded_signals.get('Lat_Accel', 0)
                VelocityReport_msg.heading_rate = decoded_signals.get('Yaw_Rate', 0)
                VelocityReport_msg.header = header
                self.pub_VelocityReport.publish(VelocityReport_msg)

    def decode_message(self, message):
        """캐싱된 DBC 메시지를 사용하여 디코딩"""
        try:
            can_message = self.dbc_messages.get(message.arbitration_id)
            if can_message:
                return can_message.decode(message.data)
        except Exception as e:
            self.get_logger().error(f"Failed to decode message: {e}")
        return {}

def main(args=None):
    rclpy.init(args=args)

    # 다중 스레드 실행을 위한 Executor 설정
    executor = MultiThreadedExecutor()
    can_receiver_node = CanReceiver()
    executor.add_node(can_receiver_node)

    # 병렬 실행 시작
    try:
        executor.spin()
    finally:
        can_receiver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
