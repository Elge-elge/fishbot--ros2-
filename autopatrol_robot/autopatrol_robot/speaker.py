import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeechText
import espeakng

class Speaker(Node):
    def __init__(self):
        super().__init__('speaker')
        self.speech_service = self.create_service(SpeechText, 'speech_text', self.speak_callback)
        self.speaker_ = espeakng.Speaker()
        self.speaker_.voice = 'zh'

    def speak_callback(self, request, response):
        self.get_logger().info(f'准备朗读: "{request.text}"')
        self.speaker_.say(request.text)
        self.speaker_.wait()
        response.result = True
        return response
        
def main(args=None):
    rclpy.init()
    node = Speaker()
    rclpy.spin(node)
    rclpy.shutdown()