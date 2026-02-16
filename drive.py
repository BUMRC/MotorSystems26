#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from enum import Enum
import numpy as np
import signal
import sys

import adafruit_pca9685
import adafruit_extended_bus


class MotorState(Enum):
    STOPPED = 0
    RUNNING = 1
    ERROR = 2

class Side(Enum):
    LEFT = 0
    RIGHT = 1


class MotorNode(Node):
    def __init__(self, motor_id, config, shield):
        super().__init__(f'motor_node_{motor_id}') # Maybe add side to the name??

        # Motor configuration
        self.motor_id = motor_id
        self.side = Side(motor_id % 2)
        self.config = config
        self.shield = shield

        # Motor state
        self.state = MotorState.STOPPED
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.last_command_time = self.get_clock().now()

        # pub sub stuff
        self.velocity_subscriber = self.create_subscription(
            Float64,
            f'/motor_{self.motor_id}/target_velocity',
            self.velocity_callback,
            10
        )

        self.status_publisher = self.create_publisher(
            Float64,
            f'/motor_{self.motor_id}/current_velocity',
            10
        )

        # loop 50 times per second
        self.control_loop_timer = self.create_timer(
            1.0/config['control_frequency'],
            self.control_loop
        )

        self.init_hardware()

        self.get_logger().info(f"Motor {motor_id} ({self.side}) initialized")

    def init_hardware(self):
        try:
            self.forward_pin = self.shield.channels[self.config['forward_pin_id']]
            self.reverse_pin = self.shield.channels[self.config['reverse_pin_id']]
            self.get_logger().info(f"Forward pin: {self.config['forward_pin_id']}, Reverse pin: {self.config['reverse_pin_id']} for motor {self.motor_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motor hardware: {str(e)}")
            self.state = MotorState.ERROR

    def velocity_callback(self, msg: Float64):
        self.target_velocity = msg.data
        self.get_logger().info(f"Target velocity: {self.target_velocity} for motor {self.motor_id}")
        self.last_command_time = self.get_clock().now()

    def ramp_velocity(self, current, target):
        max_acceleration = self.config['motor']['max_acceleration']
        dt = 1.0/self.config['control_frequency']

        max_velocity_change = max_acceleration * dt
        velocity_error = target - current

        if abs(velocity_error) <= max_velocity_change:
            return target
        else:
            return current + np.sign(velocity_error) * max_velocity_change

    def zero_minimum_output(self, output):
        # im not sure if this is the best way to do this, but stop any output less than 20%
        minimum_output = self.config['motor']['minimum_output']
        if abs(output) < minimum_output:
            return 0
        return output

    def set_motor_output(self, output):
        try:
            # if self.side == Side.RIGHT:
            #     output = -output # reversed since motors are connected backward...

            output = self.zero_minimum_output(output)
            output = int(np.clip(output, -self.config['motor']['max_output'], self.config['motor']['max_output']))
            if output > 0:
                self.get_logger().info(f"Setting motor {self.motor_id} to {output}")
                self.forward_pin.duty_cycle = output
                self.reverse_pin.duty_cycle = 0
            elif output < 0:
                self.get_logger().info(f"Setting motor {self.motor_id} to {output}")
                self.forward_pin.duty_cycle = 0
                self.reverse_pin.duty_cycle = -output
            else:
                # self.get_logger().info(f"Setting motor {self.motor_id} to 0")
                self.forward_pin.duty_cycle = 0
                self.reverse_pin.duty_cycle = 0

            self.state = MotorState.RUNNING if output != 0 else MotorState.STOPPED

        except Exception as e:
            self.get_logger().error(f"Failed to set motor output: {str(e)}")
            self.state = MotorState.ERROR

    def control_loop(self):
        time_since_last_command = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        if time_since_last_command > self.config['timeout']:
            self.target_velocity = 0.0
            self.get_logger().warn("Too long since last command - stopping motor")

        ramped_target = self.ramp_velocity(self.current_velocity, self.target_velocity)

        # Bypass PID — feed ramp directly to output until encoder feedback is available
        self.set_motor_output(ramped_target)

        self.current_velocity = ramped_target # TODO: get encoder values

        status_msg = Float64()
        status_msg.data = self.current_velocity
        self.status_publisher.publish(status_msg) #TODO: actually set this up for correct data

    # on shutdown, set all motors to 0
    def on_shutdown(self):
        self.target_velocity = 0.0
        self.current_velocity = 0.0
        self.set_motor_output(0)


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription
        self.motor_publishers = [self.create_publisher(
            Float64,
            f'/motor_{i}/target_velocity',
            10) for i in range(6)]
        self.get_logger().info("MotorController node started")

        self.motor_velocities = [0, 0, 0, 0, 0, 0]

    def set_motor(self, motor_index, velocity):
        self.motor_velocities[motor_index] = velocity
        msg = Float64()
        msg.data = float(velocity)
        self.motor_publishers[motor_index].publish(msg)
        self.get_logger().info(f"Motor {motor_index} set to {velocity}")

    def cmd_vel_callback(self, msg: Twist):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        print(f"linear_speed: {linear_speed}, angular_speed: {angular_speed}")

        # Differential mixing — left motors: 0,2,4  right motors: 1,3,5
        left = np.clip(linear_speed + angular_speed, -1.0, 1.0) * 65535
        right = np.clip(linear_speed - angular_speed, -1.0, 1.0) * 65535

        for i in range(6):
            vel = left if i % 2 == 0 else right
            self.set_motor(i, int(vel))


def main():
    rclpy.init()

    i2c = adafruit_extended_bus.ExtendedI2C(15)
    shield = adafruit_pca9685.PCA9685(i2c, address=0x40)
    shield.frequency = 250

    commander = MotorController()
    motor_config = {
        0: (1, 0), 
        1: (2, 3), 
        2: (4, 5), 
        3: (7, 6), 
        4: (9, 8), 
        5: (10, 11), 
    }
    motors = []
    for motor_id in range(6):
        config = {
            'motor': {
                'max_output': 65535 * 0.5,
                'min_output': -65535 * 0.5,
                'minimum_output': 65535 * 0.10,  # prevent motor damage?
                'max_acceleration': 65535 * 0.1
            },
            'forward_pin_id': motor_id * 2 + (motor_id % 2),
            'reverse_pin_id': motor_id * 2 + 1 - (motor_id % 2),
            'control_frequency': 50,
            'timeout': 0.5
        }
        motor_node = MotorNode(motor_id, config, shield)
        motors.append(motor_node)

    executor = MultiThreadedExecutor()
    executor.add_node(commander)

    for motor in motors:
        executor.add_node(motor)

    def shutdown(*_):
        for motor in motors:
            motor.on_shutdown()
        for motor_id in range(6):
            forward_pin = motor_id * 2 + (motor_id % 2)
            reverse_pin = motor_id * 2 + 1 - (motor_id % 2)
            shield.channels[forward_pin].duty_cycle = 0
            shield.channels[reverse_pin].duty_cycle = 0
        shield.deinit()
        for motor in motors:
            motor.destroy_node()
        commander.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()

if __name__ == '__main__':
    main()
