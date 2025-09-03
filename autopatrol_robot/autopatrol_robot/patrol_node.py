from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer #订阅坐标变换
from tf_transformations import euler_from_quaternion,quaternion_from_euler #四元数转欧拉角,欧拉角转四元数
import math
from autopatrol_interfaces.srv import SpeechText
import time
 

class PatrolNode(BasicNavigator):
    def __init__(self,node_name='patrol_node'):
        super().__init__(node_name)
        #声明参数
        self.declare_parameter('initial_point',[0.0,0.0,0.0]) #初始位置x,y,theta
        self.declare_parameter('target_points',[0.0,0.0,0.0, 1.0,1.0,1.57]) #目标位置x,y,theta
        self.initial_point_ = self.get_parameter('initial_point').value #获取参数值
        self.target_points_ = self.get_parameter('target_points').value #获取参数值
        self.buffer = Buffer() #坐标变换缓存
        self.listener = TransformListener(self.buffer, self) #订阅坐标变换
        self.speech_client_ = self.create_client(SpeechText, 'speech_text') #创建语音播报服务客户端

    def get_pose_by_xyyaw(self,x,y,yaw): 
        #根据x,y,yaw创建PoseStamped

        pose = PoseStamped() #创建目标位置
        pose.header.frame_id = 'map' #坐标系
        pose.pose.position.x = x #位置x
        pose.pose.position.y = y #位置y
        #返回顺序为(x,y,z,w)
        quat = quaternion_from_euler(0,0,yaw) #欧拉角转四元数
        pose.pose.orientation.x = quat[0] #四元数x
        pose.pose.orientation.y = quat[1] #四元数y
        pose.pose.orientation.z = quat[2] #四元数z
        pose.pose.orientation.w = quat[3] #四元数w
        return pose

        
    def init_robot_pose(self):
        #初始化机器人位姿
        self.initial_point_ = self.get_parameter('initial_point').value #获取参数值
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0],self.initial_point_[1],self.initial_point_[2]) #根据x,y,yaw创建PoseStamped
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()

        
    def get_target_points(self):
        #获取目标点列表
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x, y, yaw])
            self.get_logger().info(f'获取到目标点: {index}->({x},{y},{yaw})')
        return points

    def nav_to_pose(self,target_point):
        #导航到指定位置
        self.waitUntilNav2Active()
        self.goToPose(target_point)
        while not self.isTaskComplete():
                feedback = self.getFeedback()
                self.get_logger().info(f'剩余距离: {feedback.distance_remaining}')      
        result = self.getResult()
        self.get_logger().info(f'导航结果: {result}')

    def get_current_pose(self):
        #获取机器人当前位置
        
        while rclpy.ok():
            try:
                tf = self.buffer_.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
                transform = tf.transform
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ])
                self.get_logger().info(
                    f'平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')
                

    def speech_text(self,text):
        #调用语音播报服务
        while not self.speech_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('语音播报服务不可用，等待中...')
        request = SpeechText.Request()
        request.text = text
        future = self.speech_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.result == True:
                self.get_logger().info(f'语音播报成功: "{text}"')
            else:
                self.get_logger().error('语音播报失败')
        else:
            self.get_logger().error(f'服务调用失败')



def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speech_text("准备初始化位置")
    patrol.init_robot_pose() #初始化机器人位置
    patrol.speech_text("初始化位置完成，开始巡逻")


    while rclpy.ok():
        points = patrol.get_target_points() #获取目标点列表
        for point in points:
            x,y,yaw = point[0],point[1],point[2]
            target_pose = patrol.get_pose_by_xyyaw(x,y,yaw) #根据x,y,yaw创建PoseStamped
            patrol.speech_text(f"准备前往下一个巡逻点,位置x:{x}米,y:{y}米,角度{math.degrees(yaw)}度")
            patrol.nav_to_pose(target_pose) #导航到指定位置
    
    rclpy.shutdown()