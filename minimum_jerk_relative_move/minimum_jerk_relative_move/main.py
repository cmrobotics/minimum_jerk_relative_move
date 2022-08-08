from .minimum_jerk_relative_move import MinimumJerkRelativeMove
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor 

def main(args=None):
    rclpy.init(args=args)
 
    exe = MultiThreadedExecutor()
    exe.add_node(MinimumJerkRelativeMove())
    
    while (rclpy.ok()):
        exe.spin_once(timeout_sec=1.0)


if __name__ == '__main__':
    main()