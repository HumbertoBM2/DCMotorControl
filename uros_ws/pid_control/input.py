#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
import math

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

class Input(Node):
    def __init__(self):
        super().__init__('input')
        
        # 1. Declarar parámetros
        self.declare_parameter('signal_type', 'step')  # "step", "square", "sine", "manual"
        self.declare_parameter('amplitude', 1.0)
        self.declare_parameter('omega', 1.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('step_time', 1.0)
        self.declare_parameter('speed_command', 0.0)   # en [-1, 1] si signal_type == "manual"

        # 2. Obtener valores iniciales de parámetros
        self.signal_type   = self.get_parameter('signal_type').value
        self.amplitude     = self.get_parameter('amplitude').value
        self.omega         = self.get_parameter('omega').value
        self.publish_rate  = self.get_parameter('publish_rate').value
        self.step_time     = self.get_parameter('step_time').value
        self.speed_command = clamp(self.get_parameter('speed_command').value, -1.0, 1.0)

        # Registrar callback de parámetros dinámicos
        self.add_on_set_parameters_callback(self.param_callback)

        # 3. Publisher al tópico /set_point
        self.set_point_pub = self.create_publisher(Float64, '/set_point', 10)

        # 4. Suscriptor al tópico /motor_output (velocidad actual del motor)
        self.motor_output_sub = self.create_subscription(
            Float64,
            '/motor_output',
            self.motor_output_callback,
            10
        )
        self.motor_output_sub  # evitar warning de variable sin usar

        # Variable para almacenar la velocidad medida
        self.current_motor_speed = 0.0

        # 5. Timer para publicar setpoint periódicamente
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Contador de tiempo
        self.current_time = 0.0

        self.get_logger().info('Input node started with the following initial parameters:')
        self.log_current_params()

    def param_callback(self, params):
        for param in params:
            if param.name == 'signal_type':
                self.signal_type = param.value
            elif param.name == 'amplitude':
                self.amplitude = param.value
            elif param.name == 'omega':
                self.omega = param.value
            elif param.name == 'publish_rate':
                self.publish_rate = param.value
                new_period = 1.0 / max(self.publish_rate, 1e-6)
                self.timer.cancel()
                self.timer = self.create_timer(new_period, self.timer_callback)
            elif param.name == 'step_time':
                self.step_time = param.value
            elif param.name == 'speed_command':
                self.speed_command = clamp(param.value, -1.0, 1.0)
                self.get_logger().info(f"Clamped speed_command to {self.speed_command}")
        
        self.log_current_params()
        return SetParametersResult(successful=True)

    def timer_callback(self):
        # 1) Aumentar el tiempo
        self.current_time += 1.0 / self.publish_rate
        # 2) Calcular el setpoint según el tipo de señal
        setpoint_value = self.compute_signal(self.current_time)
        # 3) Publicar en /set_point
        msg = Float64()
        msg.data = setpoint_value
        self.set_point_pub.publish(msg)

        # 4) Loguear setpoint y velocidad medida para ver cuánto tarda en llegar
        self.get_logger().info(f"t={self.current_time:.2f}s | setpoint={setpoint_value:.3f} | motor_speed={self.current_motor_speed:.3f}")

    def compute_signal(self, t):
        if self.signal_type == 'manual':
            return clamp(self.speed_command, -1.0, 1.0)
        elif self.signal_type == 'step':
            val = self.amplitude if t >= self.step_time else 0.0
            return clamp(val, -1.0, 1.0)
        elif self.signal_type == 'square':
            if abs(self.omega) < 1e-9:
                val = self.amplitude
            else:
                period = (2.0 * math.pi) / self.omega
                time_in_period = t % period
                half_period = period / 2.0
                val = self.amplitude if time_in_period < half_period else -self.amplitude
            return clamp(val, -1.0, 1.0)
        elif self.signal_type == 'sine':
            val = self.amplitude * math.sin(self.omega * t)
            return clamp(val, -1.0, 1.0)
        else:
            self.get_logger().warn(f"Unknown signal_type '{self.signal_type}'. Returning 0.")
            return 0.0

    def motor_output_callback(self, msg):
        """Se llama cada vez que llega un mensaje de /motor_output."""
        self.current_motor_speed = msg.data
        # Si quieres loguearlo en cada mensaje, descomenta:
        # self.get_logger().info(f"Recibida velocidad motor: {self.current_motor_speed}")

    def log_current_params(self):
        self.get_logger().info(
            f"  signal_type={self.signal_type}, amplitude={self.amplitude}, "
            f"omega={self.omega}, publish_rate={self.publish_rate}, step_time={self.step_time}, "
            f"speed_command={self.speed_command}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Input()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Input node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
