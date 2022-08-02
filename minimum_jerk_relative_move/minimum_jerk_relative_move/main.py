from .minimum_jerk_relative_move import MinimumJerkRelativeMove
import rclpy

def main(args=None):
    rclpy.init(args=args)

    minimum_jerk_relative_move = MinimumJerkRelativeMove()

    rclpy.spin(minimum_jerk_relative_move)


if __name__ == '__main__':
    main()