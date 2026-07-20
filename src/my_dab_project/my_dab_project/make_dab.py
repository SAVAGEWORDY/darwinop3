import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import time
import sys
import tty
import termios

class DabMotionNode(Node):
    def __init__(self):
        super().__init__('dab_motion_node')
        
        self.module_pub = self.create_publisher(String, '/robotis/enable_ctrl_module', 10)
        
        self.page_pub = self.create_publisher(Int32, '/robotis/action/page_num', 10)
        
        self.RIGHT_DAB_PAGE = 20
        self.LEFT_DAB_PAGE = 21
        self.BALERINA = 18 
        
        self.motion_delay = 2.5
        self.init_robot()

    def init_robot(self):
        time.sleep(0.5)
        msg = String()
        msg.data = 'action_module'
        self.module_pub.publish(msg)
    
    def play_page(self, page_num):
        msg = Int32()
        msg.data = page_num
        self.page_pub.publish(msg)

    def run_dance_cycle(self):
        #self.get_logger().info('right')
        self.play_page(self.RIGHT_DAB_PAGE)
        time.sleep(self.motion_delay)

        #self.get_logger().info('left')
        self.play_page(self.LEFT_DAB_PAGE)
        time.sleep(self.motion_delay)
        
        self.play_page(self.BALERINA)
        self.get_logger().info('probel nazmi')
        

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main(args=None):
    rclpy.init(args=args)
    node = DabMotionNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1) 
            
            key = get_key()
            if key == ' ': 
                node.run_dance_cycle()
            elif key == '\x03': 
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
