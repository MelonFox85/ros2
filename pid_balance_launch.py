#!/usr/bin/env python3
"""
pid_balance.launch.py
---------------------
Запускает стек робота с PID-регулятором для балансировки на месте.

Эта версия launch-файла адаптирована под новую архитектуру узлов, в которой
низкоуровневая работа с энкодерами делегирована ядру Linux, а узел одометрии
публикует стандартные сообщения типа nav_msgs/msg/Odometry.

Дата обновления: 2025-07-09
Автор: Айрат (адаптация с помощью Copilot)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Собирает и возвращает полное описание запуска для ROS 2.
    Определяет все узлы, их параметры и взаимосвязи.
    """

    # --- 1. Аргументы запуска ---
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Если true — узлы берут время из /clock (для симуляции).")

    # --- 2. Конфигурация узлов ---

    # --- Группа IMU (Драйвер + Фильтр) ---
    mpu6050_driver_node = Node(
        package="mpu6050_cpp_pkg",
        executable="mpu6050_node",
        name="mpu6050_node",
        output="screen",
        emulate_tty=True,
        parameters=[{'frame_id': 'imu_link_uncalibrated'}]
    )

    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'gain_acc': 0.02},
            {'use_mag': False},
            {'publish_tf': False},
            {'do_bias_estimation': True},
            {'bias_alpha': 0.01},
            {'qos_overrides./imu/data_raw.subscription.reliability': 'best_effort'}
        ],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/data', '/imu/data')
        ]
    )

    # --- Узел Одометрии ---
    # ВАЖНО: Этот узел заменяет старый 'encoder_node_exe'.
    # Он не работает с GPIO напрямую, а читает данные из файлов устройств,
    # созданных драйвером ядра Linux.
    odom_publisher_node = Node(
        package='encoders_cpp_pkg',
        executable='odom_publisher_node',
        name='odom_publisher_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # Пути к файлам устройств энкодеров.
            # Рекомендуется использовать символические ссылки из /dev/input/by-path/
            # для стабильной работы, так как они не меняются после перезагрузки.
            'left_encoder_device': '/dev/input/by-path/platform-rotary@18-event',
            'right_encoder_device': '/dev/input/by-path/platform-rotary@8-event',
            
        }]
    )

    # --- Группа высокоуровневого управления (PID) ---
    pid_params = {
        'balance.kp': 20.0, 'balance.ki': 0.0, 'balance.kd': 1.0,
        'yaw.kp': 0.0, 'yaw.kd': 0.0,
        'max_voltage': 12.0
    }

    high_ctrl_group = [
        Node(
            package="cmd_publisher_cpp",
            executable="cmd_publisher_zeros",
            name="cmd_publisher_zeros"
        ),
        Node(
            package="feedback_cpp_pkg",
            executable="neg_feedback",
            name="neg_feedback",
            parameters=[{
                # Размер окна для фильтра скользящего среднего внутри узла.
                # Сглаживает "сырые" данные о скорости из /odom.
                'filter_window_size': 100
            }]
        ),
        Node(
            package="pid_controller_cpp",
            executable="pid_controller",
            name="pid_controller_cpp",
            parameters=[pid_params]
        ),
    ]

    # --- Драйвер моторов ---
    motor_driver_node = Node(
        package="motor_controller_cpp",
        executable="motor_controller",
        name="motor_controller",
        parameters=[
            {"critical_angle_deg": 45.0, "V_clip": 12.0, "min_duty_pct": 0.0}
        ],
        output="screen",
        emulate_tty=True
    )

    # --- 3. Сборка и возврат LaunchDescription ---
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(mpu6050_driver_node)
    ld.add_action(imu_filter_node)
    ld.add_action(odom_publisher_node) # Добавляем новый узел одометрии
    for node in high_ctrl_group:
        ld.add_action(node)
    ld.add_action(motor_driver_node)
    
    return ld
