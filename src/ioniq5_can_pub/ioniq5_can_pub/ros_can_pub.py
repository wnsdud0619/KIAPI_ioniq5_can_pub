import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header  
import can
from can.matrix import dbc
from autoware_vehicle_msgs.msg import *
import time
import rospy

# CAN - 0:P, 7:R, N:6, D:5 , Autoware - 22:P, 20:R, 1:N, 2:D
Gear_DISP_dict = { 
    0 : 22,
    7 : 20,
    6 : 1,
    5 : 2,
}

class can710():
    def __init__(self, _EPS_control_state, _EPS_Enable_FB, _Control_SW_FB, _Override_state, _StrAng, _Str_Tq_1, _Str_Tq_2, _EPS_Alive_Cnt):
        self.EPS_control_state = _EPS_control_state
        self.EPS_Enable_FB = _EPS_Enable_FB
        self.Control_SW_FB = _Control_SW_FB
        self.Override_state = _Override_state
        self.StrAng = _StrAng
        self.Str_Tq_1 = _Str_Tq_1
        self.Str_Tq_2 = _Str_Tq_2
        self.EPS_Alive_Cnt = _EPS_Alive_Cnt
	
class can711():
    def __init__(self, _ACC_Control_state, _ACC_En_FB, _AEB_State, _Override_ACC_state, _Override_BRK_state, _Vehicle_speed, _Gear_DISP):
        self.ACC_Control_state = _ACC_Control_state
        self.ACC_En_FB = _ACC_En_FB
        self.AEB_State = _AEB_State
        self.Override_ACC_state = _Override_ACC_state
        self.Override_BRK_state = _Override_BRK_state
        self.Vehicle_speed = _Vehicle_speed
        self.Gear_DISP = _Gear_DISP
    
class can713():
    def __init__(self, _Long_Accel, _Lat_Accel, _Yaw_Rate):
        self.Long_Accel = _Long_Accel
        self.Lat_Accel = _Lat_Accel
        self.Yaw_Rate = _Yaw_Rate
    
class can715():
    def __init__(self, _vehicle_speed, _steering_angle, _brake_position, _accel_position, _turn_signal_right, _hazard, _turn_signal_left):
        self.vehicle_speed = _vehicle_speed
        self.steering_angle = _steering_angle
        self.brake_position = _brake_position
        self.accel_position = _accel_position
        self.turn_signal_right = _turn_signal_right
        self.hazard = _hazard
        self.turn_signal_left = _turn_signal_left
   

class CanReceiver(Node):
    def __init__(self):
        super().__init__('can_receiver_node')

        self.can710 = can710(0, 0, 0, 0, 0, 0, 0, 0)
        self.can711 = can711(0, 0, 0, 0, 0, 0, 0)
        self.can713 = can713(0, 0, 0)
        self.can715 = can715(0, 0, 0, 0, 0, 0, 0, 0)

        # ROS 2 퍼블리셔 생성
        self.pub_ControlModeReport = self.create_publisher(ControlModeReport, '/vehicle/status/ControlModeReport', 10)
        self.pub_GearReport = self.create_publisher(GearReport, '/vehicle/status/GearReport', 10)
        self.pub_SteerReport = self.create_publisher(SteerReport, '/vehicle/status/SteerReport', 10)
        self.pub_VelocityReport = self.create_publisher(VelocityReport, '/vehicle/status/VelocityReport', 10)
        
        # CAN 인터페이스 설정
        self.bus = can.Bus(interface='socketcan', channel='can0')  # 'vcan0'는 예시입니다. 실제 CAN 인터페이스 이름을 사용하세요.

        # DBC 파일 로드 (사용할 DBC 파일 경로를 지정)
        self.dbc = canmatrix.loadp('KIAPI.dbc')  # DBC 파일 경로를 지정해야 합니다

        # 데이터 수신 및 퍼블리시
        self.create_timer(0.01, self.timer_callback)  # 0.01초마다 콜백 함수 호출

    def timer_callback(self):
        # CAN 메시지 수신
        message = self.bus.recv()

        if message is not None and message.arbitration_id == 0x710:  # CAN ID 0x710만 처리
            
            SteerReport_msg = SteerReport()
            # 메시지 ID가 0x156인 경우
            self.get_logger().info(f"Received CAN message with ID: {hex(message.arbitration_id)}")
            #binary_data = ' '.join(f'{byte:08b}' for byte in message.data)  # 각 바이트를 2진법으로 변환

            # CAN 메시지에서 신호 디코딩 (DBC 파일에서 해당 메시지를 찾음)
            decoded_signals = self.decode_message(message)

            # 원하는 신호만 선택하여 문자열로 변환
            selected_signals = {key: decoded_signals[key] for key in ['EPS_control_state', 'EPS_Enable_FB', 'Control_SW_FB', 'Override_state', 'StrAng'] if key in decoded_signals}
            self.can710.EPS_control_state = selected_signals['EPS_control_state']
            self.can710.EPS_Enable_FB = selected_signals['EPS_Enable_FB']
            self.can710.Control_SW_FB = selected_signals['Control_SW_FB']
            self.can710.Override_state = selected_signals['Override_state']
            self.can710.StrAng = selected_signals['StrAng']
            SteerReport_msg.steering_tire_angle = self.can710.StrAng

        if message is not None and message.arbitration_id == 0x711:  # CAN ID 0x711만 처리

            GearReport_msg = GearReport()
            # 메시지 ID가 0x156인 경우
            self.get_logger().info(f"Received CAN message with ID: {hex(message.arbitration_id)}")
            #binary_data = ' '.join(f'{byte:08b}' for byte in message.data)  # 각 바이트를 2진법으로 변환

            # CAN 메시지에서 신호 디코딩 (DBC 파일에서 해당 메시지를 찾음)
            decoded_signals = self.decode_message(message)

            # 원하는 신호만 선택하여 문자열로 변환
            selected_signals = {key: decoded_signals[key] for key in ['ACC_Control_state', 'ACC_En_FB', 'AEB_State', 'Gear_DISP'] if key in decoded_signals}
            self.can711.ACC_Control_state = selected_signals['ACC_Control_state']
            self.can711.ACC_En_FB = selected_signals['ACC_En_FB']
            self.can711.AEB_State = selected_signals['AEB_State']
            self.can711.Gear_DISP = selected_signals['Gear_DISP']

            GearReport_msg.report = Gear_DISP_dict.get(self.can711.Gear_DISP)

        if message is not None and message.arbitration_id == 0x713:  # CAN ID 0x713만 처리
            #ros msg 생성            
            VelocityReport_msg = VelocityReport()
            # 메시지 ID가 0x156인 경우
            self.get_logger().info(f"Received CAN message with ID: {hex(message.arbitration_id)}")
            #binary_data = ' '.join(f'{byte:08b}' for byte in message.data)  # 각 바이트를 2진법으로 변환

            # CAN 메시지에서 신호 디코딩 (DBC 파일에서 해당 메시지를 찾음)
            decoded_signals = self.decode_message(message)

            # 원하는 신호만 선택하여 문자열로 변환
            selected_signals = {key: decoded_signals[key] for key in ['Long_Accel', 'Lat_Accel', 'Yaw_Rate'] if key in decoded_signals}

            self.can713.Long_Accel = selected_signals['Long_Accel']
            self.can713.Lat_Accel = selected_signals['Lat_Accel']
            self.can713.Yaw_Rate = selected_signals['Yaw_Rate']

            VelocityReport_msg.longitudinal_velocity = self.can713.Long_Accel
            VelocityReport_msg.lateral_velocity = self.can713.Lat_Accel
            VelocityReport_msg.heading_rate = self.can713.Yaw_Rate
            
        if message is not None and message.arbitration_id == 0x715:  # CAN ID 0x715만 처리
            # 메시지 ID가 0x156인 경우
            self.get_logger().info(f"Received CAN message with ID: {hex(message.arbitration_id)}")
            #binary_data = ' '.join(f'{byte:08b}' for byte in message.data)  # 각 바이트를 2진법으로 변환

            # CAN 메시지에서 신호 디코딩 (DBC 파일에서 해당 메시지를 찾음)
            decoded_signals = self.decode_message(message)

            # 원하는 신호만 선택하여 문자열로 변환
            selected_signals = {key: decoded_signals[key] for key in ['Long_Accel', 'Lat_Accel', 'Yaw_Rate'] if key in decoded_signals}
           
        header = Header()
        header.frame_id = 'base_link'
        header.stamp = rospy.Time.now()

        #ControlModeReport_msg = ControlModeReport()
        
        VelocityReport_msg.header = header
        GearReport_msg.stamp = header.stamp
        SteerReport_msg.stamp = header.stamp

        self.pub_ControlModeReport.publish(VelocityReport_msg)
        self.pub_GearReport.publish(GearReport_msg)
        self.pub_SteerReport.publish(SteerReport_msg)
        #self.get_logger().info(f"Published selected CAN data: {msg.data}")

    def decode_message(self, message):
        decoded_data = {}
        # CAN 메시지에서 신호들을 디코딩하여 변수명과 값을 딕셔너리로 반환
        try:
            can_message = self.dbc.get_message_by_id(message.arbitration_id)
            for signal in can_message.signals:
                signal_value = signal.decode(message.data)
                decoded_data[signal.name] = signal_value
        except Exception as e:
            self.get_logger().error(f"Failed to decode message: {e}")
        return decoded_data


def main(args=None):
    rclpy.init(args=args)

    can_receiver_node = CanReceiver()

    rclpy.spin(can_receiver_node)

    can_receiver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
