#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO
import time

class SnowbotNode(Node):
    def __init__(self):
        super().__init__('snowbot_node')

        self.kit = ServoKit(channels=16, address=0x40)
        self.kit._pca.frequency = 50

        for ch in [0, 1, 2]:
            self.kit.servo[ch].actuation_range = 240
            self.kit.servo[ch].set_pulse_width_range(500, 2500)

        self.kit.servo[3].set_pulse_width_range(1000, 2000)

        self.left_stop = 23
        self.right_stop = 24
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.left_stop, self.right_stop], GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.raised_ch0, self.raised_ch1 = 120, 200
        self.lowered_ch0, self.lowered_ch1 = 230, 70
        self.chute_stop = 120
        self.chute_max = 30

        self.current_chute_target = self.chute_stop
        self.last_chute_command = 0.0

        self.current_auger_speed = 0.0
        self.target_auger_speed = 0.0
        self.auger_ramp_rate = 0.02
        self.auger_command_timeout = 999.0  # Changed was 10.0, now won't timeout
        self.last_auger_command_time = 0.0

        self.kit._pca.channels[2].duty_cycle = 0
        time.sleep(0.5)
        self.kit.servo[2].angle = self.chute_stop
        self.get_logger().info(f'Chute servo set to stop position: {self.chute_stop}')
        self.kit.servo[3].angle = 90

        self.create_subscription(Float64MultiArray, '/servo_cmd', self.servo_cb, 10)
        self.create_subscription(Float64, '/auger_speed', self.auger_cb, 10)
        self.create_subscription(Float64, '/led_brightness', self.led_cb, 10)  # NEW: LED subscriber

        self.create_timer(0.05, self.check_endstops)
        self.create_timer(0.02, self.update_auger)

        self.get_logger().info('Lift + Continuous Chute + Auger ESC + LEDs with soft start READY')

    def servo_cb(self, msg):
        if len(msg.data) < 2:
            return
        
        lift_raw = msg.data[0]
        chute_raw = msg.data[1]
        
        if lift_raw != -999:
            lift = max(0.0, min(1.0, lift_raw))
            ch0 = self.raised_ch0 + lift * (self.lowered_ch0 - self.raised_ch0)
            ch1 = self.raised_ch1 + lift * (self.lowered_ch1 - self.raised_ch1)
            self.kit.servo[0].angle = ch0
            self.kit.servo[1].angle = ch1
            self.get_logger().info(f'Lift set to: {lift:.2f}')

        if chute_raw != -999:
            self.last_chute_command = max(-1.0, min(1.0, chute_raw))
            self.get_logger().info(f'Chute command received: {self.last_chute_command:.2f}')
            self.update_chute()

    def check_endstops(self):
        left_hit = GPIO.input(self.left_stop) == GPIO.HIGH
        right_hit = GPIO.input(self.right_stop) == GPIO.HIGH
        
        if self.last_chute_command > 0.05 and left_hit:
            if self.current_chute_target != self.chute_stop:
                self.get_logger().warn('LEFT ENDSTOP HIT - stopping chute')
                self.current_chute_target = self.chute_stop
                self.kit.servo[2].angle = self.chute_stop
                self.last_chute_command = 0.0
        
        elif self.last_chute_command < -0.05 and right_hit:
            if self.current_chute_target != self.chute_stop:
                self.get_logger().warn('RIGHT ENDSTOP HIT - stopping chute')
                self.current_chute_target = self.chute_stop
                self.kit.servo[2].angle = self.chute_stop
                self.last_chute_command = 0.0

    def update_chute(self):
        left_hit = GPIO.input(self.left_stop) == GPIO.HIGH
        right_hit = GPIO.input(self.right_stop) == GPIO.HIGH
        
        if self.last_chute_command > 0.05 and left_hit:
            target = self.chute_stop
            self.get_logger().warn('Cannot move left - endstop active')
        elif self.last_chute_command < -0.05 and right_hit:
            target = self.chute_stop
            self.get_logger().warn('Cannot move right - endstop active')
        elif abs(self.last_chute_command) > 0.05:
            target = self.chute_stop + (self.last_chute_command * self.chute_max)
            self.get_logger().info(f'Moving chute to: {target:.1f}')
        else:
            target = self.chute_stop

        self.current_chute_target = target
        self.kit.servo[2].angle = target

    def auger_cb(self, msg):
        self.last_auger_command_time = time.time()
        self.target_auger_speed = max(0.0, min(1.0, msg.data))
        self.get_logger().info(f'Auger target set to: {self.target_auger_speed:.2f}')

    def update_auger(self):
        current_time = time.time()
        
        if current_time - self.last_auger_command_time > self.auger_command_timeout:
            self.target_auger_speed = 0.0
        
        if abs(self.target_auger_speed - self.current_auger_speed) > 0.001:
            if self.target_auger_speed > self.current_auger_speed:
                self.current_auger_speed = min(
                    self.current_auger_speed + self.auger_ramp_rate,
                    self.target_auger_speed
                )
            else:
                self.current_auger_speed = max(
                    self.current_auger_speed - self.auger_ramp_rate,
                    self.target_auger_speed
                )
            
            # CHANGED: Reversed auger direction (was 90 + ...)
            angle = 90 - self.current_auger_speed * 90
            self.kit.servo[3].angle = angle

    # NEW: LED control callback
    def led_cb(self, msg):
        brightness = max(0.0, min(1.0, msg.data))
        angle = brightness * 180  # 0 = off, 180 = full brightness
        self.kit.servo[4].angle = angle
        self.get_logger().info(f'LED brightness set to: {brightness:.2f}')

def main():
    rclpy.init()
    node = SnowbotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.kit._pca.channels[2].duty_cycle = 0
        node.kit.servo[3].angle = 90
        node.kit.servo[4].angle = 0  # NEW: Turn off LEDs on shutdown
        GPIO.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
