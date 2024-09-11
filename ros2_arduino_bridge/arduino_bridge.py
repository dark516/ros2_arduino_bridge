from time import sleep
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from serial import Serial

from ros2_arduino_bridge.connection import ArduinoConnection


class MotorController(Node):
    MAX_MOTOR_SPEED_TICKS = 13
    """Максимальное значение скорости для моторов"""

    @classmethod
    def clamp_motor_speed(cls, motor_speed):
        return max(min(int(motor_speed * cls.MAX_MOTOR_SPEED_TICKS), cls.MAX_MOTOR_SPEED_TICKS),
                   -cls.MAX_MOTOR_SPEED_TICKS)

    def __init__(self, connect: ArduinoConnection):
        super().__init__('motor_controller')

        self._connect = connect

        # Подписка на топик /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10)

        self.last_left_motor_speed = None
        self.last_right_motor_speed = None

    def cmd_vel_callback(self, msg):
        # Извлечение линейной и угловой скорости из сообщения Twist
        linear = msg.linear.x
        angular = msg.angular.z

        # Рассчитываем скорость для каждого мотора
        left_motor_speed = linear - angular
        right_motor_speed = linear + angular

        # Ограничиваем скорость в пределах [-13, 13]
        left_motor_speed = self.clamp_motor_speed(left_motor_speed)
        right_motor_speed = self.clamp_motor_speed(right_motor_speed)

        if left_motor_speed == self.last_left_motor_speed:
            return

        if right_motor_speed == self.last_right_motor_speed:
            return

        self._connect.setSpeeds(left_motor_speed, right_motor_speed)

    def shutdown(self) -> None:
        self._connect.close()
        super().shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    motor_controller = MotorController(ArduinoConnection(Serial("/dev/ttyACM0", 115200)))
    sleep(2)  # TODO

    rclpy.spin(motor_controller)

    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
