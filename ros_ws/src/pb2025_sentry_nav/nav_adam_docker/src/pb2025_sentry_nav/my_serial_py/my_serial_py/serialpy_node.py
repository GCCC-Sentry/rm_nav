# import rclpy  #ros2的python接口
# from rclpy.node import Node  # 导入Node类,用于创建节点
# from referee_msg.msg import Referee # 导入自定义消息类型，这个是自己写的裁判系统消息类型
# from geometry_msgs.msg import Twist # 导入Twist消息类型，用于控制机器人运动
# import serial  # 导入串口模块
# import json  # 导入json模块
# import struct # 导入struct模块,用于打包数据
# import threading  # 导入线程模块
# from std_msgs.msg import Int8  # 状态消息类型
# class SerialNode(Node):
#     def __init__(self):
#         super().__init__('serial_node')

#         # 设置串口参数
#         self.serial_port = '/dev/ttyACM1'  # 使用实际存在的串口路径
#         self.baud_rate = 115200
#         self.get_logger().info(f'Serial port set to: {self.serial_port}')
#         self.Status_nav2 = 0
#         # 初始化串口
#         self.serial_conn = None
#         try:
#             self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
#             self.get_logger().info(f'Connected to {self.serial_port} at {self.baud_rate} baud rate.')
#         except serial.SerialException as e:
#             self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
#             # 错误处理
#             self.destroy_node()
#             rclpy.shutdown()
#         # 创建订阅者，订阅导航数据话题，把计算好的数据发给单片机
#         self.subscription = self.create_subscription(Twist, '/cmd_vel', self.SendtoSTM32_callback, 10)
#         self.subscription_1 = self.create_subscription(Int8, 'nav2_status',self.Nav2Stat_callback,10)
#         # 创建发布者,将接受到的来自单片机的数据发布到/stm32_ros2_data话题
#         self.publisher_ = self.create_publisher(Referee, 'stm32_ros2_data', 10)

#         # 创建定时器，定期读取串口数据
#         self.timer = self.create_timer(0.1, self.read_serial_data)

#     def read_serial_data(self):
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
#                 data = self.serial_conn.readline().decode('utf-8',errors='ignore').strip()
#                 if data:
#                     try:
#                         # 尝试解析JSON数据
#                         parsed_data = json.loads(data)
#                         self.process_data(parsed_data)
#                     except (json.JSONDecodeError, ValueError, TypeError) as e:
#                         self.get_logger().error(f'Failed to parse JSON: {e}')
#                         return
#             except serial.SerialException as e:
#                 self.get_logger().error(f'Error reading serial data: {e}')
#         else:
#             self.get_logger().warning('Serial connection is not open.')

#     def process_data(self, data):
#         # 处理解析后的数据，根据实际需求进行相应操作
#         msg = Referee()
#         msg.game_type = int(data.get('game_type'))#比赛类型
#         msg.game_progress = int(data.get('game_progress'))#比赛阶段——4 比赛进行中
#         msg.remain_hp = int(data.get('remain_hp'))#机器人当前血量
#         msg.max_hp = int(data.get('max_hp'))#。。。
#         msg.stage_remain_time = int(data.get('stage_remain_time'))#当前阶段剩余时间，                     
#         msg.bullet_remaining_num_17mm = int(data.get('bullet_remaining_num_17mm'))#剩余发弹量
#         msg.red_outpost_hp = int(data.get('red_outpost_hp'))    
#         msg.red_base_hp = int(data.get('red_base_hp'))
#         msg.blue_outpost_hp = int(data.get('blue_outpost_hp'))
#         msg.blue_base_hp = int(data.get('blue_base_hp'))
#         msg.rfid_status = int(data.get('rfid_status'))#rfid状态
#         # 发布消息
#         self.publisher_.publish(msg)
#     def Nav2Stat_callback(self,msg):
#          self.Status_nav2 = msg.data
#     def SendtoSTM32_callback(self, msg):
#         # 接收来自ROS2的指令，并发送给单片机
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
#                 # 数据字段定义
#                 header = 0xAA
#                 checksum = 19
#                 x_speed = -msg.linear.x *0.6
#                 y_speed = -msg.linear.y *0.6
#                 rotate = msg.angular.z *0
#                 yaw_speed = msg.angular.z *0
#                 # yaw_speed = 10
#                 running_state = 0x00
#                 data_frame = struct.pack(
#                     '<BBffffB',  # 格式化字符串：<表示小端，B表示uint8_t，f表示float
#                     header,         # uint8_t
#                     checksum,       # uint8_t
#                     x_speed,        # float
#                     y_speed,        # float
#                     rotate,         # float
#                     yaw_speed,      # float
#                     running_state   # uint8_t
#                 )
#                 # 发送数据
#                 self.serial_conn.write(data_frame)
#                 self.get_logger().info('Sent data to STM32')
#             except serial.SerialException as e:
#                 self.get_logger().error(f'Error sending data to STM32: {e}')
#         else:
#             self.get_logger().warning('Serial connection is not open.')

#     def __del__(self):
#         if self.serial_conn and self.serial_conn.is_open:
#             self.serial_conn.close()
#             self.get_logger().info(f'Serial connection to {self.serial_port} closed.')

# def ros_spin_thread(node):
#     rclpy.spin(node)

# def main(args=None):
#     rclpy.init(args=args)
#     serial_node = SerialNode()
#     spin_thread = threading.Thread(target=ros_spin_thread, args=(serial_node,))
#     spin_thread.start()
#     spin_thread.join()
#     serial_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()










# nengyong1




# import rclpy  # ros2的python接口
# from rclpy.node import Node  # 导入Node类,用于创建节点
# import serial  # 导入串口模块
# import json  # 导入json模块
# import struct  # 导入struct模块,用于打包数据
# import threading  # 导入线程模块
# from std_msgs.msg import Int8  # 状态消息类型
# from geometry_msgs.msg import Twist  # 导入Twist消息类型，用于控制机器人运动

# # 导入派大星消息类型
# # 删除: from referee_msg.msg import Referee
# # 导入 pb_rm_interfaces 消息
# try:
#     from pb_rm_interfaces.msg import (
#         RobotStatus, GameStatus, GameRobotHP, RfidStatus, Buff, EventData
#     )
# except ImportError:
    
#     exit(1)


# class SerialNode(Node):
#     def __init__(self):
#         super().__init__('serial_node')

#         # 设置串口参数
#         self.serial_port = '/dev/ttyUSB0'  
#         self.baud_rate = 115200
#         self.get_logger().info(f'Serial port set to: {self.serial_port}')
#         self.Status_nav2 = 0
#         self.chassis_mode = 0  #底盘模式
#         # 初始化串口
#         self.serial_conn = None
#         try:
#             self.serial_conn = serial.Serial(
#                 self.serial_port, self.baud_rate, timeout=0.1)
#             self.get_logger().info(
#                 f'Connected to {self.serial_port} at {self.baud_rate} baud rate.')
#         except serial.SerialException as e:
#             self.get_logger().error(
#                 f'Failed to connect to {self.serial_port}: {e}')
#             # 错误处理
#             self.destroy_node()
#             rclpy.shutdown()
        
#         # 创建订阅者，订阅导航数据话题，把计算好的数据发给单片机
#         # 行为树会发布 /cmd_vel
#         self.subscription = self.create_subscription(
#             Twist, '/cmd_vel', self.SendtoSTM32_callback, 10)
#         self.subscription_1 = self.create_subscription(
#             Int8, 'nav2_status', self.Nav2Stat_callback, 10)
#         self.mode_sub = self.create_subscription(
#             Int8, '/cmd_chassis_mode', self.chassis_mode_callback, 10)

        
#         # 话题名称和类型必须与 pb2025_sentry_behavior_server.cpp 中的订阅完全一致
#         self.publishers_ = {
#             'game_status': self.create_publisher(GameStatus, 'referee/game_status', 10),
#             'robot_status': self.create_publisher(RobotStatus, 'referee/robot_status', 10),
#             'all_robot_hp': self.create_publisher(GameRobotHP, 'referee/all_robot_hp', 10),
#             'rfid_status': self.create_publisher(RfidStatus, 'referee/rfid_status', 10),
#             # 以下是 C++ 节点还订阅了的，但JSON中目前没有数据
#             # 如果JSON未来会包含它们，请取消注释
#             # 'buff': self.create_publisher(Buff, 'referee/buff', 10),
#             # 'event_data': self.create_publisher(EventData, 'referee/event_data', 10),
#         }
#         self.get_logger().info('已创建行为树的多个发布者。')
        

#         # 创建定时器，定期读取串口数据
#         self.timer = self.create_timer(0.1, self.read_serial_data)

#     def read_serial_data(self):
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
#                 data = self.serial_conn.readline().decode(
#                     'utf-8', errors='ignore').strip()
#                 if data:
#                     try:
#                         # 尝试解析JSON数据
#                         parsed_data = json.loads(data)
#                         # 调用 *新的* process_data 函数
#                         self.process_data(parsed_data)
#                     except (json.JSONDecodeError, ValueError, TypeError) as e:
#                         self.get_logger().error(f'Failed to parse JSON: {e}')
#                         return
#             except serial.SerialException as e:
#                 self.get_logger().error(f'Error reading serial data: {e}')
#         else:
#             self.get_logger().warning('Serial connection is not open.')

#     # 修改 3: 重写 process_data 来分发数据
#     def process_data(self, data):
#         # 处理解析后的数据，并分发到不同的 ROS 话题

#         # 1. 发布 GameStatus (比赛状态)
#         try:
#             msg_game_status = GameStatus()
#             msg_game_status.game_progress = int(data.get('game_progress', 0))
#             msg_game_status.stage_remain_time = int(
#                 data.get('stage_remain_time', 0))
#             self.publishers_['game_status'].publish(msg_game_status)
#         except Exception as e:
#             self.get_logger().warn(f'Failed to publish GameStatus: {e}')

#         # 2. 发布 RobotStatus (机器人状态)
#         try:
#             msg_robot_status = RobotStatus()
#             # 将JSON字段映射到 pb_rm_interfaces 字段
#             msg_robot_status.current_hp = int(data.get('remain_hp', 0))
#             msg_robot_status.maximum_hp = int(data.get('max_hp', 0))
#             msg_robot_status.projectile_allowance_17mm = int(
#                 data.get('bullet_remaining_num_17mm', 0))


#             # IsAttacked 节点需要 'armor_id' 和 'hp_deduction_reason'
#             # JSON中没有这些！必须让STM32发送这些数据。
#             # 否则 IsAttacked 节点将永远不会工作。
#             msg_robot_status.armor_id = int(data.get('armor_id', 0)) # 假设您在JSON中添加了 'armor_id'
#             msg_robot_status.hp_deduction_reason = int(data.get('hp_deduction_reason', 0)) # 假设您在JSON中添加了 'hp_deduction_reason'
            
#             # (示例：如果被击打，is_hp_deduced 应该为 true)
#             # last_hp = ... (您需要自己实现这个逻辑)
#             # if (last_hp - msg_robot_status.current_hp > 0):
#             #    msg_robot_status.is_hp_deduced = True
            
#             self.publishers_['robot_status'].publish(msg_robot_status)
#         except Exception as e:
#             self.get_logger().warn(f'Failed to publish RobotStatus: {e}')

#         # 3. 发布 GameRobotHP (双方基地/前哨站血量)
#         try:
#             msg_all_hp = GameRobotHP()
#             msg_all_hp.red_outpost_hp = int(data.get('red_outpost_hp', 0))
#             msg_all_hp.red_base_hp = int(data.get('red_base_hp', 0))
#             msg_all_hp.blue_outpost_hp = int(data.get('blue_outpost_hp', 0))
#             msg_all_hp.blue_base_hp = int(data.get('blue_base_hp', 0))
#             self.publishers_['all_robot_hp'].publish(msg_all_hp)
#         except Exception as e:
#             self.get_logger().warn(f'Failed to publish GameRobotHP: {e}')

#         # 4. 发布 RfidStatus (RFID 状态)
#         try:
#             msg_rfid = RfidStatus()
#             rfid_int = int(data.get('rfid_status', 0))

#             # IsRfidDetected 节点需要一个bitmask(位掩码)
            
#             msg_rfid.friendly_fortress_gain_point = True if (rfid_int & (1 << 0)) else False  # 假设第0位是己方堡垒
#             msg_rfid.friendly_supply_zone_non_exchange = True if (rfid_int & (1 << 1)) else False # 假设第1位是己方补给区(非兑换)
#             msg_rfid.friendly_supply_zone_exchange = True if (rfid_int & (1 << 2)) else False # 假设第2位是己方补给区(兑换)
#             msg_rfid.center_gain_point = True if (rfid_int & (1 << 3)) else False  # 假设第3位是中心增益点

#             self.publishers_['rfid_status'].publish(msg_rfid)
#         except Exception as e:
#             self.get_logger().warn(f'Failed to publish RfidStatus: {e}')


#     def Nav2Stat_callback(self, msg):
#         self.Status_nav2 = msg.data
    
#     #模式切换回调函数
#     def chassis_mode_callback(self, msg):
#         self.chassis_mode = msg.data
#         self.get_logger().info(f'收到底盘模式切换指令: {self.chassis_mode}')

#     def SendtoSTM32_callback(self, msg):
#         # 接收来自ROS2的指令(例如/cmd_vel)，并发送给单片机
#         # 定义了和STM32之间的私有通信协议
#         # 行为树的 PubTwist 节点会发布到 /cmd_vel，这个回调会正确接收它。
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
#                 # 数据字段定义
#                 header = 0xAA
#                 checksum = 19
#                 x_speed = -msg.linear.x * 0.8
#                 y_speed = -msg.linear.y * 0.8
#                 rotate = msg.angular.z * 57.3
#                 yaw_speed = msg.angular.z * 57.3
#                 # yaw_speed = 10
#                 # running_state = 0x00
#                 running_state = self.chassis_mode
#                 data_frame = struct.pack(
#                     '<BBffffB',  # 格式化字符串：<表示小端，B表示uint8_t，f表示float
#                     header,      # uint8_t
#                     checksum,    # uint8_t
#                     x_speed,     # float
#                     y_speed,     # float
#                     rotate,      # float
#                     yaw_speed,   # float
#                     running_state  # uint8_t
#                 )
#                 # 发送数据
#                 self.serial_conn.write(data_frame)
#                 self.get_logger().debug('Sent data to STM32')
#             except serial.SerialException as e:
#                 self.get_logger().error(f'Error sending data to STM32: {e}')
#         else:
#             self.get_logger().warning('Serial connection is not open.')

#     def __del__(self):
#         if self.serial_conn and self.serial_conn.is_open:
#             self.serial_conn.close()
#             self.get_logger().info(
#                 f'Serial connection to {self.serial_port} closed.')


# def ros_spin_thread(node):
#     rclpy.spin(node)


# def main(args=None):
#     rclpy.init(args=args)
#     serial_node = SerialNode()
    

#     spin_thread = threading.Thread(target=ros_spin_thread, args=(serial_node,))
#     spin_thread.start()
#     spin_thread.join()
    
#     serial_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




# import rclpy  # ros2的python接口
# from rclpy.node import Node  # 导入Node类,用于创建节点
# import serial  # 导入串口模块
# import json  # 导入json模块
# import struct  # 导入struct模块,用于打包数据
# import threading  # 导入线程模块
# import libscrc  # 导入 libscrc 用于CRC计算
# from std_msgs.msg import Int8  # 状态消息类型
# from geometry_msgs.msg import Twist  # 导入Twist消息类型，用于控制机器人运动

# # 导入 pb_rm_interfaces 消息
# try:
#     from pb_rm_interfaces.msg import (
#         RobotStatus, GameStatus, GameRobotHP, RfidStatus, Buff, EventData
#     )
# except ImportError:
#     exit(1)


# class SerialNode(Node):
#     def __init__(self):
#         super().__init__('serial_node')

#         # 设置串口参数
#         self.serial_port = '/dev/ttyACM0'  
#         self.baud_rate = 115200
#         self.get_logger().info(f'Serial port set to: {self.serial_port}')
#         self.Status_nav2 = 0
#         self.chassis_mode = 0  #底盘模式
#         # 初始化串口
#         self.serial_conn = None
#         try:
#             self.serial_conn = serial.Serial(
#                 self.serial_port, self.baud_rate, timeout=1)
#             self.get_logger().info(
#                 f'Connected to {self.serial_port} at {self.baud_rate} baud rate.')
#         except serial.SerialException as e:
#             self.get_logger().error(
#                 f'Failed to connect to {self.serial_port}: {e}')
#             # 错误处理
#             self.destroy_node()
#             rclpy.shutdown()
        
#         # 创建订阅者
#         self.subscription = self.create_subscription(
#             Twist, '/cmd_vel', self.SendtoSTM32_callback, 10)
#         self.subscription_1 = self.create_subscription(
#             Int8, 'nav2_status', self.Nav2Stat_callback, 10)
#         self.mode_sub = self.create_subscription(
#             Int8, '/cmd_chassis_mode', self.chassis_mode_callback, 10)

#         # 发布者配置
#         self.publishers_ = {
#             'game_status': self.create_publisher(GameStatus, 'referee/game_status', 10),
#             'robot_status': self.create_publisher(RobotStatus, 'referee/robot_status', 10),
#             'all_robot_hp': self.create_publisher(GameRobotHP, 'referee/all_robot_hp', 10),
#             'rfid_status': self.create_publisher(RfidStatus, 'referee/rfid_status', 10),
#         }
#         self.get_logger().info('已创建行为树的多个发布者。')

#         # 创建定时器，定期读取串口数据
#         self.timer = self.create_timer(0.1, self.read_serial_data)

#     def read_serial_data(self):
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
#                 data = self.serial_conn.readline().decode(
#                     'utf-8', errors='ignore').strip()
#                 if data:
#                     try:
#                         # 尝试解析JSON数据
#                         parsed_data = json.loads(data)
#                         self.process_data(parsed_data)
#                     except (json.JSONDecodeError, ValueError, TypeError) as e:
#                         # JSON解析偶尔出错
#                         # self.get_logger().error(f'Failed to parse JSON: {e}')
#                         pass
#             except serial.SerialException as e:
#                 self.get_logger().error(f'Error reading serial data: {e}')
#         else:
#             self.get_logger().warning('Serial connection is not open.')

#     def process_data(self, data):
#         # 1. 发布 GameStatus
#         try:
#             msg_game_status = GameStatus()
#             msg_game_status.game_progress = int(data.get('game_progress', 0))
#             msg_game_status.stage_remain_time = int(data.get('stage_remain_time', 0))
#             self.publishers_['game_status'].publish(msg_game_status)
#         except Exception as e:
#             pass

#         # 2. 发布 RobotStatus
#         try:
#             msg_robot_status = RobotStatus()
#             msg_robot_status.current_hp = int(data.get('remain_hp', 0))
#             msg_robot_status.maximum_hp = int(data.get('max_hp', 0))
#             msg_robot_status.projectile_allowance_17mm = int(data.get('bullet_remaining_num_17mm', 0))
#             msg_robot_status.armor_id = int(data.get('armor_id', 0)) 
#             msg_robot_status.hp_deduction_reason = int(data.get('hp_deduction_reason', 0)) 
#             self.publishers_['robot_status'].publish(msg_robot_status)
#         except Exception as e:
#             pass

#         # 3. 发布 GameRobotHP
#         try:
#             msg_all_hp = GameRobotHP()
#             msg_all_hp.red_outpost_hp = int(data.get('red_outpost_hp', 0))
#             msg_all_hp.red_base_hp = int(data.get('red_base_hp', 0))
#             msg_all_hp.blue_outpost_hp = int(data.get('blue_outpost_hp', 0))
#             msg_all_hp.blue_base_hp = int(data.get('blue_base_hp', 0))
#             self.publishers_['all_robot_hp'].publish(msg_all_hp)
#         except Exception as e:
#             pass

#         # 4. 发布 RfidStatus
#         try:
#             msg_rfid = RfidStatus()
#             rfid_int = int(data.get('rfid_status', 0))
#             msg_rfid.friendly_fortress_gain_point = True if (rfid_int & (1 << 0)) else False
#             msg_rfid.friendly_supply_zone_non_exchange = True if (rfid_int & (1 << 1)) else False
#             msg_rfid.friendly_supply_zone_exchange = True if (rfid_int & (1 << 2)) else False
#             msg_rfid.center_gain_point = True if (rfid_int & (1 << 3)) else False 
#             self.publishers_['rfid_status'].publish(msg_rfid)
#         except Exception as e:
#             pass

#     def Nav2Stat_callback(self, msg):
#         self.Status_nav2 = msg.data
    
#     def chassis_mode_callback(self, msg):
#         self.chassis_mode = msg.data
#         self.get_logger().info(f'收到底盘模式切换指令: {self.chassis_mode}')

 
#     def SendtoSTM32_callback(self, msg):
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
               
#                 header = 0xAA
#                 x_speed = -msg.linear.x * 0.3
#                 y_speed = -msg.linear.y * 0.3
#                 rotate = msg.angular.z * 1
#                 yaw_speed = msg.angular.z * 1
#                 running_state = self.chassis_mode
  
#                 # 格式: Header(1) + X(4) + Y(4) + Rot(4) + Yaw(4) + State(1) = 18 bytes
#                 data_to_send = struct.pack(
#                     '<BffffB',       # < 小端, B u8, f float
#                     header,
#                     x_speed,
#                     y_speed,
#                     rotate,
#                     yaw_speed,
#                     running_state
#                 )

#                 # 3. 使用 libscrc 计算 CRC16 (Modbus模式)
#                 # 计算范围是前面所有的 18 个字节
#                 crc_val = libscrc.modbus(data_to_send)

#                 # 4. 将 CRC 附加到数据包末尾
#                 # <H 表示 unsigned short (2字节), 小端
#                 final_packet = data_to_send + struct.pack('<H', crc_val)

#                 # 5. 发送 (总长度 20 字节)
#                 self.serial_conn.write(final_packet)
#                 # self.get_logger().debug(f'Sent frame with CRC: {final_packet.hex()}')

#             except serial.SerialException as e:
#                 self.get_logger().error(f'Error sending data to STM32: {e}')
#         else:
#             self.get_logger().warning('Serial connection is not open.')

#     def __del__(self):
#         if self.serial_conn and self.serial_conn.is_open:
#             self.serial_conn.close()
#             self.get_logger().info(
#                 f'Serial connection to {self.serial_port} closed.')


# def ros_spin_thread(node):
#     rclpy.spin(node)


# def main(args=None):
#     rclpy.init(args=args)
#     serial_node = SerialNode()
    
#     spin_thread = threading.Thread(target=ros_spin_thread, args=(serial_node,))
#     spin_thread.start()
#     spin_thread.join()
    
#     serial_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




#crc2








# import rclpy
# from rclpy.node import Node
# import serial
# import json
# import struct
# import threading
# import time
# from std_msgs.msg import Int8
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import JointState
# from example_interfaces.msg import UInt8, Float64, Float32
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# # ==================== 1. 导入 SMBU 官方消息类型 ====================
# try:
#     from pb_rm_interfaces.msg import (
#         RobotStatus, GameStatus, GameRobotHP, RfidStatus, Buff, EventData
#     )
# except ImportError:
#     print("错误: 无法导入 'pb_rm_interfaces'。")
#     print("请确保您已经将 pb_rm_interfaces 包放到了您的 ros2_ws/src 目录下并执行了 colcon build。")
#     exit(1)
# # ====================================================================

# class SerialNode(Node):
#     def __init__(self):
#         super().__init__('serial_node')

#         # --- 串口设置 ---
#         self.serial_port = '/dev/ttyACM1'  # 请确认您的串口号 (ttyACM0 或 ttyACM1)
#         self.baud_rate = 115200
#         self.serial_conn = None
#         self.Status_nav2 = 0

#         # --- [新增] 控制指令缓存 (用于聚合发送) ---
#         self.latest_twist = Twist()         # 底盘速度
#         self.latest_gimbal = JointState()   # 云台角度
#         self.latest_shoot = UInt8()         # 发射指令
#         self.latest_spin = Float32()        # 摩擦轮/小陀螺

#         # --- 初始化串口 ---
#         try:
#             self.serial_conn = serial.Serial(
#                 self.serial_port, self.baud_rate, timeout=1)
#             self.get_logger().info(
#                 f'Connected to {self.serial_port} at {self.baud_rate} baud rate.')
#         except serial.SerialException as e:
#             self.get_logger().error(f'Failed to connect: {e}')
#             self.destroy_node()
#             rclpy.shutdown()
        
#         # --- [修改] 创建订阅者 (接收行为树的所有指令) ---
#         # 1. 底盘速度
#         self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
#         # 2. [新增] 云台角度 (行为树发布 JointState)
#         self.create_subscription(JointState, '/cmd_gimbal_joint', self.cmd_gimbal_callback, 10)
#         # 3. [新增] 发射指令
#         self.create_subscription(UInt8, '/cmd_shoot', self.cmd_shoot_callback, 10)
#         # 4. [新增] 小陀螺/摩擦轮
#         self.create_subscription(Float32, '/cmd_spin', self.cmd_spin_callback, 10)
        
#         self.subscription_1 = self.create_subscription(
#             Int8, 'nav2_status', self.Nav2Stat_callback, 10)

#         # --- [优化] 创建发布者 (使用 Best Effort QoS 提高兼容性) ---
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.RELIABLE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         self.publishers_ = {
#             'game_status': self.create_publisher(GameStatus, 'referee/game_status', qos_profile),
#             'robot_status': self.create_publisher(RobotStatus, 'referee/robot_status', qos_profile),
#             'all_robot_hp': self.create_publisher(GameRobotHP, 'referee/all_robot_hp', qos_profile),
#             'rfid_status': self.create_publisher(RfidStatus, 'referee/rfid_status', qos_profile),
#         }
#         self.get_logger().info('已创建兼容 SMBU 行为树的发布者。')

#         # --- 启动线程 ---
#         # 1. 接收线程 (STM32 -> ROS)
#         self.receive_thread = threading.Thread(target=self.read_serial_data, daemon=True)
#         self.receive_thread.start()

#         # 2. [新增] 发送线程 (ROS -> STM32) - 独立循环发送
#         self.send_thread = threading.Thread(target=self.send_data_loop, daemon=True)
#         self.send_thread.start()

#     # ==================== 接收逻辑 (保持您现有的逻辑) ====================
#     def read_serial_data(self):
#         while rclpy.ok() and self.serial_conn and self.serial_conn.is_open:
#             try:
#                 data = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
#                 if data:
#                     try:
#                         parsed_data = json.loads(data)
#                         self.process_data(parsed_data)
#                     except (json.JSONDecodeError, ValueError, TypeError):
#                         # 偶尔的数据错误忽略即可
#                         pass
#             except serial.SerialException as e:
#                 self.get_logger().error(f'Error reading serial data: {e}')
#                 break

#     def process_data(self, data):
#         # 1. GameStatus
#         try:
#             msg = GameStatus()
#             msg.game_progress = int(data.get('game_progress', 0))
#             msg.stage_remain_time = int(data.get('stage_remain_time', 0))
#             self.publishers_['game_status'].publish(msg)
#         except Exception: pass

#         # 2. RobotStatus
#         try:
#             msg = RobotStatus()
#             msg.current_hp = int(data.get('remain_hp', 0))
#             msg.maximum_hp = int(data.get('max_hp', 0))
#             msg.projectile_allowance_17mm = int(data.get('bullet_remaining_num_17mm', 0))
#             msg.armor_id = int(data.get('armor_id', 0))
#             msg.hp_deduction_reason = int(data.get('hp_deduction_reason', 0))
#             # 如果下位机发了热量就读，没发就默认为0
#             msg.shooter_17mm_1_barrel_heat = int(data.get('heat', 0))
#             self.publishers_['robot_status'].publish(msg)
#         except Exception: pass

#         # 3. GameRobotHP
#         try:
#             msg = GameRobotHP()
#             msg.red_outpost_hp = int(data.get('red_outpost_hp', 0))
#             msg.red_base_hp = int(data.get('red_base_hp', 0))
#             msg.blue_outpost_hp = int(data.get('blue_outpost_hp', 0))
#             msg.blue_base_hp = int(data.get('blue_base_hp', 0))
#             self.publishers_['all_robot_hp'].publish(msg)
#         except Exception: pass

#         # 4. RfidStatus (您已经修复了 bool 问题，这里保持正确)
#         try:
#             msg = RfidStatus()
#             rfid = int(data.get('rfid_status', 0))
#             msg.friendly_fortress_gain_point = True if (rfid & (1 << 0)) else False
#             msg.friendly_supply_zone_non_exchange = True if (rfid & (1 << 1)) else False
#             msg.friendly_supply_zone_exchange = True if (rfid & (1 << 2)) else False
#             msg.center_gain_point = True if (rfid & (1 << 3)) else False
#             self.publishers_['rfid_status'].publish(msg)
#         except Exception: pass

#     # ==================== [新增] 回调函数 (只更新变量，不发送) ====================
#     def Nav2Stat_callback(self, msg):
#         self.Status_nav2 = msg.data

#     def cmd_vel_callback(self, msg):
#         self.latest_twist = msg

#     def cmd_gimbal_callback(self, msg):
#         self.latest_gimbal = msg

#     def cmd_shoot_callback(self, msg):
#         self.latest_shoot = msg

#     def cmd_spin_callback(self, msg):
#         self.latest_spin = msg

#     # ==================== [核心修改] 发送线程 (ROS -> 下位机) ====================
#     def send_data_loop(self):
#         """
#         独立线程：每 10ms 将所有最新指令打包发送给 STM32
#         """
#         while rclpy.ok() and self.serial_conn and self.serial_conn.is_open:
#             try:
#                 # 1. 准备底盘数据
#                 x_speed = -self.latest_twist.linear.x * 0.3
#                 y_speed = -self.latest_twist.linear.y * 0.3
#                 rot_speed = self.latest_twist.angular.z
#                 # yaw_speed = rot_speed # 如果您的下位机需要两个旋转量，可根据情况赋值

#                 # 2. 准备云台数据 (从 JointState 解析 Pitch/Yaw)
#                 pitch = 0.0
#                 yaw = 0.0
#                 if self.latest_gimbal.name:
#                     try:
#                         if "gimbal_pitch_joint" in self.latest_gimbal.name:
#                             idx = self.latest_gimbal.name.index("gimbal_pitch_joint")
#                             pitch = self.latest_gimbal.position[idx]
#                         if "gimbal_yaw_joint" in self.latest_gimbal.name:
#                             idx = self.latest_gimbal.name.index("gimbal_yaw_joint")
#                             yaw = self.latest_gimbal.position[idx]
#                     except ValueError: pass

#                 # 3. 准备功能数据
#                 # 摩擦轮 (cmd_spin > 0 则开启)
#                 fric_on = 1 if self.latest_spin.data > 0 else 0
#                 # 发射指令 (0:停, 1:单发, 2:连发)
#                 fire_cmd = self.latest_shoot.data
                
#                 running_state = 0x00

#                 # 4. 打包数据 (扩充协议)
#                 # 格式: 头(1B) + 校验(1B) + 底盘(4B*3) + 云台(4B*2) + 功能(1B*3) = 25 字节
#                 # 您需要修改下位机接收代码来匹配这个 Struct 长度和顺序
#                 header = 0xAA
#                 checksum = 0x00 # 简易占位，可自行计算

#                 data_frame = struct.pack(
#                     '<BBfffffBBB', # Little-endian
#                     header,     # uint8
#                     checksum,   # uint8
#                     x_speed,    # float
#                     y_speed,    # float
#                     rot_speed,  # float
#                     pitch,      # float (新增: 云台P)
#                     yaw,        # float (新增: 云台Y)
#                     fric_on,    # uint8 (新增: 摩擦轮)
#                     fire_cmd,   # uint8 (新增: 开火)
#                     running_state # uint8
#                 )

#                 # 5. 发送
#                 self.serial_conn.write(data_frame)
                
#                 # 6. 控制发送频率 (10ms = 100Hz)
#                 time.sleep(0.01)

#             except Exception as e:
#                 self.get_logger().error(f'发送线程异常: {e}')
#                 time.sleep(1)

#     def __del__(self):
#         if self.serial_conn and self.serial_conn.is_open:
#             self.serial_conn.close()
#             self.get_logger().info(
#                 f'Serial connection to {self.serial_port} closed.')

# def main(args=None):
#     rclpy.init(args=args)
#     serial_node = SerialNode()
#     # 因为我们在 __init__ 里启动了独立的读写线程，主线程只需 spin 即可
#     rclpy.spin(serial_node)
#     serial_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#以下是20260225代码可用版
# import rclpy
# from rclpy.node import Node
# import serial
# import struct
# import threading
# import time
# import libscrc
# from std_msgs.msg import Int8
# from geometry_msgs.msg import Twist


# try:
#     from pb_rm_interfaces.msg import (
#         RobotStatus, GameStatus, GameRobotHP, RfidStatus, Buff, EventData
#     )
# except ImportError:
#     print("Error: pb_rm_interfaces not found. Please ensure custom messages are compiled.")
#     exit(1)

# class SerialNode(Node):
#     def __init__(self):
#         super().__init__('serial_node')


#         self.serial_port = '/dev/ttyUSB0'
#         self.baud_rate = 115200
        

#         self.Status_nav2 = 0
#         self.chassis_mode = 0
#         self.running = True  # 控制线程运行标志

#         # --- 协议定义 ---
#         self.HEADER_BYTE = 0xA5
#         # 结构体: H(u16), B(u8), I(u32) -
#         # 注意: 这里的 I 对应 4字节 RFID，如果下位机是 u16，请改为 H
#         self.STRUCT_FMT = '<HHBBHHHHHHI' 
#         self.PAYLOAD_SIZE = struct.calcsize(self.STRUCT_FMT)
#         self.PACKET_SIZE = 1 + self.PAYLOAD_SIZE + 2  # Header + Data + CRC
        
#         self.get_logger().info(f'Protocol: Packet Size = {self.PACKET_SIZE} bytes')

#         # 初始化串口对象 (暂不打开，由线程管理) 
#         self.serial_conn = None

#         # ROS 通信接口 
#         self.create_subscribers()
#         self.create_publishers()

#         # 独立的串口接收线程

#         self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
#         self.read_thread.start()

#     def create_subscribers(self):
#         self.subscription = self.create_subscription(
#             Twist, '/cmd_vel', self.send_to_stm32_callback, 10)
#         self.subscription_1 = self.create_subscription(
#             Int8, 'nav2_status', self.nav2_stat_callback, 10)
#         self.mode_sub = self.create_subscription(
#             Int8, '/cmd_chassis_mode', self.chassis_mode_callback, 10)

#     def create_publishers(self):
#         self.pubs = {
#             'game_status': self.create_publisher(GameStatus, 'referee/game_status', 10),
#             'robot_status': self.create_publisher(RobotStatus, 'referee/robot_status', 10),
#             'all_robot_hp': self.create_publisher(GameRobotHP, 'referee/all_robot_hp', 10),
#             'rfid_status': self.create_publisher(RfidStatus, 'referee/rfid_status', 10),
#         }

#     # ================= 串口连接管理 =================
#     def try_connect_serial(self):
#         """尝试连接串口，失败则返回 False"""
#         try:
#             if self.serial_conn and self.serial_conn.is_open:
#                 self.serial_conn.close()
            
#             self.serial_conn = serial.Serial(
#                 self.serial_port, self.baud_rate, timeout=0.1, write_timeout=0.1
#             )
#             self.get_logger().info(f'Serial connected: {self.serial_port}')
#             return True
#         except serial.SerialException as e:
#             self.get_logger().warn(f'Waiting for serial {self.serial_port}... ({e})')
#             return False

#     # 接收线程
#     def serial_read_loop(self):
#         """独立的串口读取线程，包含重连机制和流缓冲区处理"""
#         buffer = bytearray()
        
#         while self.running:
#             # check conncet
#             if not self.serial_conn or not self.serial_conn.is_open:
#                 if not self.try_connect_serial():
#                     time.sleep(1.0)  # 连接失败等待1秒重试
#                     continue

#             # 读取数据
#             try:
#                 # 读取缓冲区所有数据
#                 waiting = self.serial_conn.in_waiting
#                 if waiting > 0:
#                     buffer.extend(self.serial_conn.read(waiting))
                
#                 # 解析缓冲区
#                 while len(buffer) >= self.PACKET_SIZE:
#                     # 检查帧头
#                     if buffer[0] == self.HEADER_BYTE:
#                         # 提取完整包
#                         packet = buffer[:self.PACKET_SIZE]
                        
#                         # 校验与解析
#                         if self.check_crc_and_parse(packet):
#                             # 解析成功，从缓冲区移除该包
#                             del buffer[:self.PACKET_SIZE]
#                         else:
#                             # 校验失败，可能是假头，移除1字节继续寻找
#                             del buffer[0]
#                     else:
#                         # 帧头不对，移除1字节，滑动窗口寻找 0xA5
#                         del buffer[0]
                
#                 # 避免空转
#                 if waiting == 0:
#                     time.sleep(0.005)

#             except serial.SerialException:
#                 self.get_logger().error('Serial connection lost!')
#                 if self.serial_conn:
#                     self.serial_conn.close()
#             except Exception as e:
#                 self.get_logger().error(f'Unexpected error in read loop: {e}')
#                 time.sleep(1)

#     # 解析与发布
#     def check_crc_and_parse(self, packet):
#         try:
#             # 分离数据与CRC
#             payload_with_header = packet[:-2]
#             received_crc_bytes = packet[-2:]
            
#             received_crc = struct.unpack('<H', received_crc_bytes)[0]
#             # calculated_crc = libscrc.modbus(payload_with_header)
#             calculated_crc = libscrc.ccitt(payload_with_header)

#             if received_crc != calculated_crc:
#                 self.get_logger().debug(f'CRC Fail: Recv {received_crc:04X} != Calc {calculated_crc:04X}')
#                 return False

#             # 解析 Payload (去掉 Header 0xA5)
#             payload = payload_with_header[1:]
#             data = struct.unpack(self.STRUCT_FMT, payload)

#             self.get_logger().info(f"【收到数据】血量:{data[0]} 时间:{data[4]} RFID:{data[10]}")

#             # 对应 C++ 结构体
#             (
#                 remain_hp, max_hp, game_type, game_progress, 
#                 stage_remain_time, bullet_17mm, 
#                 red_outpost, red_base, blue_outpost, blue_base, 
#                 rfid_status_int  # 修正变量名
#             ) = data

#             # --- 发布 ROS 消息 ---
            
#             # GameStatus
#             gs_msg = GameStatus()
#             gs_msg.game_progress = int(game_progress)
#             gs_msg.stage_remain_time = int(stage_remain_time)
#             self.pubs['game_status'].publish(gs_msg)

#             # RobotStatus
#             rs_msg = RobotStatus()
#             rs_msg.current_hp = int(remain_hp)
#             rs_msg.maximum_hp = int(max_hp)
#             rs_msg.projectile_allowance_17mm = int(bullet_17mm)
#             self.pubs['robot_status'].publish(rs_msg)

#             # HP
#             hp_msg = GameRobotHP()
#             hp_msg.red_outpost_hp = int(red_outpost)
#             hp_msg.red_base_hp = int(red_base)
#             hp_msg.blue_outpost_hp = int(blue_outpost)
#             hp_msg.blue_base_hp = int(blue_base)
#             self.pubs['all_robot_hp'].publish(hp_msg)

#             # RFID (修复了原代码变量名错误)
#             rfid_msg = RfidStatus()
#             rfid_msg.friendly_fortress_gain_point = bool(rfid_status_int & (1 << 0))
#             rfid_msg.friendly_supply_zone_non_exchange = bool(rfid_status_int & (1 << 1))
#             rfid_msg.friendly_supply_zone_exchange = bool(rfid_status_int & (1 << 2))
#             rfid_msg.center_gain_point = bool(rfid_status_int & (1 << 3))
#             self.pubs['rfid_status'].publish(rfid_msg)

#             return True

#         except Exception as e:
#             self.get_logger().error(f'Parse error: {e}')
#             return False

#     # ================= 回调函数 =================
#     def nav2_stat_callback(self, msg):
#         self.Status_nav2 = msg.data

#     def chassis_mode_callback(self, msg):
#         self.chassis_mode = msg.data
#         self.get_logger().info(f'Chassis mode set to: {self.chassis_mode}')

#     def send_to_stm32_callback(self, msg):
#         """发送速度控制指令到下位机"""
#         if not (self.serial_conn and self.serial_conn.is_open):
#             return # 串口未连接时不执行，防止报错

#         try:
#             header = 0xAA
#             # 增加安全限幅，防止异常值
#             x_speed = max(min(-msg.linear.x * 0.4, 3.0), -3.0)
#             y_speed = max(min(-msg.linear.y * 0.4, 3.0), -3.0)
#             rotate = msg.angular.z * 1.0
#             yaw_speed = msg.angular.z * 1.0
#             running_state = self.chassis_mode

#             data_to_pack = (header, x_speed, y_speed, rotate, yaw_speed, running_state)
            
#             # Header(1) + 4 floats(16) + State(1) = 18 bytes payload
#             payload = struct.pack('<BffffB', *data_to_pack)
            
#             # 计算 CRC
#             crc_val = libscrc.modbus(payload)
#             final_packet = payload + struct.pack('<H', crc_val)

#             self.serial_conn.write(final_packet)

#         except serial.SerialTimeoutException:
#             self.get_logger().warn('Serial write timeout')
#         except Exception as e:
#             self.get_logger().error(f'Send error: {e}')

#     #  资源清理 
#     def destroy_node(self):
#         self.running = False  # 停止线程循环
#         if self.read_thread.is_alive():
#             self.read_thread.join(timeout=1)
        
#         if self.serial_conn:
#             self.serial_conn.close()
            
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


































# import rclpy  #ros2的python接口
# from rclpy.node import Node  # 导入Node类,用于创建节点
# from referee_msg.msg import Referee # 导入自定义消息类型，这个是自己写的裁判系统消息类型
# from geometry_msgs.msg import Twist # 导入Twist消息类型，用于控制机器人运动
# import serial  # 导入串口模块
# import json  # 导入json模块
# import struct # 导入struct模块,用于打包数据
# import threading  # 导入线程模块
# from std_msgs.msg import Int8  # 状态消息类型
# class SerialNode(Node):
#     def __init__(self):
#         super().__init__('serial_node')

#         # 设置串口参数
#         self.serial_port = '/dev/ttyACM0'  # 使用实际存在的串口路径
#         self.baud_rate = 115200
#         self.get_logger().info(f'Serial port set to: {self.serial_port}')
#         self.Status_nav2 = 0
#         # 初始化串口
#         self.serial_conn = None
#         try:
#             self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
#             self.get_logger().info(f'Connected to {self.serial_port} at {self.baud_rate} baud rate.')
#         except serial.SerialException as e:
#             self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
#             # 错误处理
#             self.destroy_node()
#             rclpy.shutdown()
#         # 创建订阅者，订阅导航数据话题，把计算好的数据发给单片机
#         self.subscription = self.create_subscription(Twist, '/cmd_vel', self.SendtoSTM32_callback, 10)
#         self.subscription_1 = self.create_subscription(Int8, 'nav2_status',self.Nav2Stat_callback,10)
#         # 创建发布者,将接受到的来自单片机的数据发布到/stm32_ros2_data话题
#         self.publisher_ = self.create_publisher(Referee, 'stm32_ros2_data', 10)

#         # 创建定时器，定期读取串口数据
#         self.timer = self.create_timer(0.1, self.read_serial_data)

#     def read_serial_data(self):
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
#                 data = self.serial_conn.readline().decode('utf-8',errors='ignore').strip()
#                 if data:
#                     try:
#                         # 尝试解析JSON数据
#                         parsed_data = json.loads(data)
#                         self.process_data(parsed_data)
#                     except (json.JSONDecodeError, ValueError, TypeError) as e:
#                         self.get_logger().error(f'Failed to parse JSON: {e}')
#                         return
#             except serial.SerialException as e:
#                 self.get_logger().error(f'Error reading serial data: {e}')
#         else:
#             self.get_logger().warning('Serial connection is not open.')

#     def process_data(self, data):
#         # 处理解析后的数据，根据实际需求进行相应操作
#         msg = Referee()
#         msg.game_type = int(data.get('game_type'))#比赛类型
#         msg.game_progress = int(data.get('game_progress'))#比赛阶段——4 比赛进行中
#         msg.remain_hp = int(data.get('remain_hp'))#机器人当前血量
#         msg.max_hp = int(data.get('max_hp'))#。。。
#         msg.stage_remain_time = int(data.get('stage_remain_time'))#当前阶段剩余时间，                     
#         msg.bullet_remaining_num_17mm = int(data.get('bullet_remaining_num_17mm'))#剩余发弹量
#         msg.red_outpost_hp = int(data.get('red_outpost_hp'))    
#         msg.red_base_hp = int(data.get('red_base_hp'))
#         msg.blue_outpost_hp = int(data.get('blue_outpost_hp'))
#         msg.blue_base_hp = int(data.get('blue_base_hp'))
#         msg.rfid_status = int(data.get('rfid_status'))#rfid状态
#         # 发布消息
#         self.publisher_.publish(msg)
#     def Nav2Stat_callback(self,msg):
#          self.Status_nav2 = msg.data
#     def SendtoSTM32_callback(self, msg):
#         # 接收来自ROS2的指令，并发送给单片机
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
#                 # 数据字段定义
#                 header = 0xAA
#                 checksum = 19
#                 x_speed = -msg.linear.x *0.3
#                 y_speed = -msg.linear.y *0.3
#                 rotate = msg.angular.z *0
#                 yaw_speed = msg.angular.z *0
#                 # yaw_speed = 10
#                 running_state = 0x00
#                 data_frame = struct.pack(
#                     '<BBffffB',  # 格式化字符串：<表示小端，B表示uint8_t，f表示float
#                     header,         # uint8_t
#                     checksum,       # uint8_t
#                     x_speed,        # float
#                     y_speed,        # float
#                     rotate,         # float
#                     yaw_speed,      # float
#                     running_state   # uint8_t
#                 )
#                 # 发送数据
#                 self.serial_conn.write(data_frame)
#                 self.get_logger().info('Sent data to STM32')
#             except serial.SerialException as e:
#                 self.get_logger().error(f'Error sending data to STM32: {e}')
#         else:
#             self.get_logger().warning('Serial connection is not open.')

#     def __del__(self):
#         if self.serial_conn and self.serial_conn.is_open:
#             self.serial_conn.close()
#             self.get_logger().info(f'Serial connection to {self.serial_port} closed.')

# def ros_spin_thread(node):
#     rclpy.spin(node)

# def main(args=None):
#     rclpy.init(args=args)
#     serial_node = SerialNode()
#     spin_thread = threading.Thread(target=ros_spin_thread, args=(serial_node,))
#     spin_thread.start()
#     spin_thread.join()
#     serial_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()






# import rclpy  # 导入rclpy库
# from rclpy.node import Node  # 导入Node类
# from referee_msg.msg import Referee  # 导入自定义的裁判系统消息
# from geometry_msgs.msg import Twist  # 导入Twist消息（在此文件中已不直接使用）
# import serial  # 导入串口通信库
# import json  # 导入JSON库
# import struct  # 导入struct库用于数据打包
# import threading  # 导入线程库（在此修改版中main函数已简化，不再需要）
# from std_msgs.msg import Int8  # 导入Int8消息（在此文件中已不直接使用）

# class SerialNode(Node):
#     def __init__(self):
#         """
#         节点初始化
#         """
#         super().__init__('serial_node_debug')

#         # 设置串口参数
#         self.serial_port = '/dev/ttyUSB0'  # 请确保使用实际存在的串口路径
#         self.baud_rate = 115200
#         self.get_logger().info(f'串口设置为: {self.serial_port}')

#         # 初始化串口
#         self.serial_conn = None
#         try:
#             self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
#             self.get_logger().info(f'成功连接到 {self.serial_port}，波特率 {self.baud_rate}。')
#         except serial.SerialException as e:
#             self.get_logger().error(f'无法连接到串口 {self.serial_port}: {e}')
#             # 如果串口连接失败，则销毁节点并关闭rclpy
#             self.destroy_node()
#             rclpy.shutdown()
#             return

#         # 【修改】: 注释掉订阅者，不再需要回调来触发发送
#         # self.subscription = self.create_subscription(Twist, '/cmd_vel', self.SendtoSTM32_callback, 10)
#         # self.subscription_1 = self.create_subscription(Int8, 'nav2_status',self.Nav2Stat_callback,10)

#         # 创建发布者，用于将从单片机收到的数据发布出去
#         self.publisher_ = self.create_publisher(Referee, 'stm32_ros2_data', 10)
        
#         # 【新增】: 创建一个定时器，周期性地直接发送调试数据
#         self.send_timer = self.create_timer(0.5, self.send_debug_data)  # 每0.5秒发送一次

#         # 创建定时器，周期性地直接读取串口数据
#         self.read_timer = self.create_timer(0.1, self.read_serial_data)

#     def send_debug_data(self):
#         """
#         直接发送调试数据到下位机，所有数据字段都置为1。
#         此函数由定时器周期性调用。
#         """
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
#                 # 定义所有要发送的数据字段，并全部置1
#                 header = 0xAA          # 帧头 (uint8_t)
#                 checksum = 19       # 校验和 (uint8_t)
#                 x_speed = 1.0       # x轴速度 (float)
#                 y_speed = 1.0       # y轴速度 (float)
#                 rotate = 1.0        # 旋转 (float)
#                 yaw_speed = 1.0     # yaw轴速度 (float)
#                 running_state = 1   # 运行状态 (uint8_t)

#                 # 使用struct将数据打包成二进制格式
#                 # '<' 表示小端模式
#                 # 'B' 表示无符号字符 (1字节), 'f' 表示浮点数 (4字节)
#                 data_frame = struct.pack(
#                     '<BBffffB',
#                     header,
#                     checksum,
#                     x_speed,
#                     y_speed,
#                     rotate,
#                     yaw_speed,
#                     running_state
#                 )
                
#                 # 将打包好的数据帧写入串口
#                 self.serial_conn.write(data_frame)
#                 # 打印发送的数据的十六进制形式，方便调试
#                 self.get_logger().info(f'已发送调试数据帧 (全1): {data_frame.hex()}')

#             except serial.SerialException as e:
#                 self.get_logger().error(f'发送数据时发生错误: {e}')
#         else:
#             self.get_logger().warning('无法发送数据，串口未连接。')

#     def read_serial_data(self):
#         """
#         直接从串口读取数据。
#         此函数由定时器周期性调用。
#         """
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
#                 # 读取一行数据
#                 data = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
#                 if data:
#                     self.get_logger().info(f"接收到原始数据: {data}")
#                     try:
#                         # 尝试将数据解析为JSON
#                         parsed_data = json.loads(data)
#                         self.process_data(parsed_data)
#                     except (json.JSONDecodeError, ValueError, TypeError) as e:
#                         self.get_logger().error(f'JSON解析失败: {e}')
#                         return
#             except serial.SerialException as e:
#                 self.get_logger().error(f'读取串口数据时发生错误: {e}')
#         else:
#             self.get_logger().warning('串口未连接。')

#     def process_data(self, data):
#         """
#         处理从下位机接收并解析后的数据，然后发布为ROS2消息。
#         """
#         msg = Referee()
#         try:
#             msg.game_type = int(data.get('game_type', 0))
#             msg.game_progress = int(data.get('game_progress', 0))
#             msg.remain_hp = int(data.get('remain_hp', 0))
#             msg.max_hp = int(data.get('max_hp', 0))
#             msg.stage_remain_time = int(data.get('stage_remain_time', 0))
#             msg.bullet_remaining_num_17mm = int(data.get('bullet_remaining_num_17mm', 0))
#             msg.red_outpost_hp = int(data.get('red_outpost_hp', 0))
#             msg.red_base_hp = int(data.get('red_base_hp', 0))
#             msg.blue_outpost_hp = int(data.get('blue_outpost_hp', 0))
#             msg.blue_base_hp = int(data.get('blue_base_hp', 0))
#             msg.rfid_status = int(data.get('rfid_status', 0))
#             # 发布消息
#             self.publisher_.publish(msg)
#             self.get_logger().info('已发布从STM32接收到的数据。')
#         except (ValueError, TypeError) as e:
#             self.get_logger().error(f"处理接收到的数据时出错: {e}")

#     def __del__(self):
#         """
#         节点销毁时的析构函数，用于关闭串口连接。
#         """
#         if self.serial_conn and self.serial_conn.is_open:
#             self.serial_conn.close()
#             self.get_logger().info(f'串口 {self.serial_port} 已关闭。')

# def main(args=None):
#     rclpy.init(args=args)
#     serial_node = SerialNode()
    
#     # 检查节点是否成功初始化
#     if rclpy.ok():
#         try:
#             # rclpy.spin() 会阻塞主线程，使节点保持活动状态以处理定时器等事件
#             rclpy.spin(serial_node)
#         except KeyboardInterrupt:
#             # 允许通过 Ctrl+C 来停止节点
#             serial_node.get_logger().info('检测到键盘中断，正在关闭节点...')
#         finally:
#             # 确保节点被正确销毁
#             serial_node.destroy_node()
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
import serial
import struct
import threading
import time
import libscrc
from std_msgs.msg import Int8, Float32
from geometry_msgs.msg import Twist

# 导入自定义接口
try:
    from pb_rm_interfaces.msg import (
        RobotStatus, GameStatus, GameRobotHP, RfidStatus
    )
except ImportError:
    print("Error: pb_rm_interfaces not found. 请确保工作空间已编译。")
    exit(1)

# 姿态 ID → 下位机 running_state 映射
# 注意: 1/2/3 专用于姿态切换, 5 专用于颠簸区域底盘对齐 (由 region_monitor 发送)
# 两套逻辑互不干扰: 姿态走 /cmd_stance, 颠簸区域走 /cmd_chassis_mode
STANCE_TO_RUNNING_STATE = {
    0: 1,   # 移动姿态  (running_state=1)
    1: 2,   # 进攻姿态  (running_state=2)
    2: 3,   # 防御姿态  (running_state=3)
}

# ================= RoboMaster 官方 CRC16 查表算法 =================
W_CRC_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
]

def get_rm_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = ((crc >> 8) ^ W_CRC_TABLE[(crc ^ b) & 0xff]) & 0xffff
    return crc

# =============================================================

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200
        self.HEADER_BYTE = 0xA5
        
        # 【重要：数据顺序修正】
        # 根据 minicom 观察：A5(头) + 04(Type) + 04(Progress) + 64 00(HP=100) ...
        # 格式：B(Type), B(Progress), H(remain_hp), H(max_hp), H(time), H(bullet), H(outpost), H(base), I(rfid)
        self.STRUCT_FMT = '<BBHHHHHHI' 
        self.PAYLOAD_SIZE = struct.calcsize(self.STRUCT_FMT) # 18
        self.PACKET_SIZE = 1 + self.PAYLOAD_SIZE + 2  # 总计 21 字节

        self.serial_conn = None
        self.running = True
        self.chassis_mode = 0
        self.stance_running_state = 1  # 默认移动姿态 (running_state=1)
        self.yaw_angle = 0.0  # 来自 /cmd_yaw_angle 的独立 yaw 角度
        self.latest_x = 0.0   # 缓存最新 cmd_vel
        self.latest_y = 0.0

        self.create_subscribers()
        self.create_publishers()

        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

        # 定时发送定时器: 50Hz 合并 cmd_vel + yaw_angle 发送给下位机
        self.send_timer = self.create_timer(0.02, self.periodic_send_to_stm32)

        self.get_logger().info(f'✅ 串口节点已启动。当前解析顺序: Type -> Progress -> HP')

    def create_subscribers(self):
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.mode_sub = self.create_subscription(Int8, '/cmd_chassis_mode', self.chassis_mode_callback, 10)
        self.stance_sub = self.create_subscription(Float32, '/cmd_stance', self.stance_callback, 10)
        self.yaw_angle_sub = self.create_subscription(Float32, '/cmd_yaw_angle', self.yaw_angle_callback, 10)

    def create_publishers(self):
        # 使用绝对路径发布话题，供行为树使用
        self.pubs = {
            'game_status': self.create_publisher(GameStatus, '/referee/game_status', 10),
            'robot_status': self.create_publisher(RobotStatus, '/referee/robot_status', 10),
            'all_robot_hp': self.create_publisher(GameRobotHP, '/referee/all_robot_hp', 10),
            'rfid_status': self.create_publisher(RfidStatus, '/referee/rfid_status', 10),
        }

    def try_connect_serial(self):
        try:
            if self.serial_conn: self.serial_conn.close()
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            return True
        except Exception as e:
            self.get_logger().warn(f'❌ 串口连接失败: {e}')
            return False

    def serial_read_loop(self):
        buffer = bytearray()
        while self.running:
            if not self.serial_conn or not self.serial_conn.is_open:
                if not self.try_connect_serial():
                    time.sleep(1.0); continue

            try:
                waiting = self.serial_conn.in_waiting
                if waiting > 0:
                    buffer.extend(self.serial_conn.read(waiting))
                
                while len(buffer) >= self.PACKET_SIZE:
                    if buffer[0] == self.HEADER_BYTE:
                        packet = buffer[:self.PACKET_SIZE]
                        if self.check_crc_and_parse(packet):
                            del buffer[:self.PACKET_SIZE]
                        else:
                            del buffer[0]
                    else:
                        del buffer[0]
                time.sleep(0.005)
            except Exception:
                self.serial_conn = None

    def check_crc_and_parse(self, packet):
        try:
            # 1. 验证 CRC (DJI 官方算法)
            data_to_check = bytes(packet[:-2]) 
            received_crc = struct.unpack('<H', packet[-2:])[0]
            calculated_crc = get_rm_crc16(data_to_check)

            if received_crc != calculated_crc:
                return False

            # 2. 按照修正后的顺序解包
            payload = packet[1:-2]
            res = struct.unpack(self.STRUCT_FMT, payload)

            (game_type, game_progress, remain_hp, max_hp, 
             stage_remain_time, bullet_17mm, 
             outpost_hp, base_hp, rfid_status_int) = res

            # --- 发布数据到 ROS 话题 ---
            
            # 1. RobotStatus
            rs_msg = RobotStatus()
            rs_msg.current_hp = int(remain_hp)
            rs_msg.maximum_hp = int(max_hp)
            rs_msg.projectile_allowance_17mm = int(bullet_17mm)
            self.pubs['robot_status'].publish(rs_msg)

            # 2. GameStatus (包含修正后的 game_type)
            gs_msg = GameStatus()
            gs_msg.game_type = int(game_type)
            gs_msg.game_progress = int(game_progress)
            gs_msg.stage_remain_time = int(stage_remain_time)
            self.pubs['game_status'].publish(gs_msg)

            # 3. GameRobotHP
            hp_msg = GameRobotHP()
            hp_msg.red_outpost_hp = int(outpost_hp)
            hp_msg.red_base_hp = int(base_hp)
            self.pubs['all_robot_hp'].publish(hp_msg)

            # 4. RFID
            rfid_msg = RfidStatus()
            rfid_msg.friendly_fortress_gain_point = bool(rfid_status_int & (1 << 0))
            rfid_msg.center_gain_point = bool(rfid_status_int & (1 << 3))
            self.pubs['rfid_status'].publish(rfid_msg)

            # 终端日志实时确认数据顺序
            self.get_logger().info(f"✅ 类型:{game_type} | 进度:{game_progress} | 血量:{remain_hp}")
            return True

        except Exception as e:
            self.get_logger().error(f"❌ 解析/发布错误: {e}")
            return False

    def chassis_mode_callback(self, msg):
        self.chassis_mode = msg.data

    def yaw_angle_callback(self, msg):
        """接收 region_monitor 发来的独立 yaw 角度 (rad)。"""
        self.yaw_angle = msg.data
        self.get_logger().debug(f'[yaw_angle] 收到: {msg.data:.3f}')

    def cmd_vel_callback(self, msg):
        """缓存最新的 cmd_vel xy 速度"""
        self.latest_x = msg.linear.x
        self.latest_y = msg.linear.y

    def stance_callback(self, msg):
        """接收行为树姿态指令, 映射为下位机 running_state"""
        stance_id = int(msg.data)
        new_state = STANCE_TO_RUNNING_STATE.get(stance_id, 1)
        if new_state != self.stance_running_state:
            self.stance_running_state = new_state
            stance_names = {1: '移动', 2: '进攻', 3: '防御'}
            self.get_logger().info(
                f'姿态切换: stance_id={stance_id} -> running_state={new_state} '
                f'({stance_names.get(new_state, "未知")})')

    def periodic_send_to_stm32(self):
        """定时合并发送: xy来自 cmd_vel缓存, yaw_angle来自 /cmd_yaw_angle"""
        if not (self.serial_conn and self.serial_conn.is_open): return
        try:
            header = 0xAA
            running_state = self.stance_running_state if self.chassis_mode == 0 else self.chassis_mode
            x_val = float(-self.latest_x * 1.0)
            y_val = float(-self.latest_y * 1.0)
            yaw_val = float(self.yaw_angle)
            # 保持下位机原有 4 个 float 的报文结构不变，只把原 yaw 速度改成 yaw 角度。
            payload = struct.pack('<BffffB', header,
                                  x_val, y_val,
                                  yaw_val, yaw_val,
                                  int(running_state))
            crc_val = libscrc.modbus(payload)
            final_packet = payload + struct.pack('<H', crc_val)
            self.serial_conn.write(final_packet)
        except Exception as e:
            self.get_logger().error(f'[TX] 发送异常: {e}')

    def destroy_node(self):
        self.running = False
        if self.serial_conn: self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
