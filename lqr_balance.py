#!/usr/bin/env python3
"""
robot_bringup_lqr.launch.py
---------------------------
Запускает полный стек двуколёсного балансирующего робота с LQR-контроллером.

Эта версия launch-файла адаптирована под новую архитектуру узлов, в которой
низкоуровневая работа с энкодерами делегирована ядру Linux, а узел одометрии
публикует стандартные сообщения типа nav_msgs/msg/Odometry.

Дата обновления: 2025-07-09
Автор: Айрат (адаптация с помощью Copilot)

Новая, улучшенная схема потоков данных:

                                            ┌───────────────────────────┐
/mpu6050_node ── /imu/data_raw (сырые) ▶───┤ imu_complementary_filter  ├─▶ /imu/data (отфильтрованные)
                                            └───────────────────────────┘         │
                                                                                  │
/odom_publisher_node ── /odom (сырая одометрия)                                   │
                                │                                                 │
                                ▼                                                 ▼
/cmd_publisher_zeros ── /cmd ───▶────────── /neg_feedback (фильтрует /odom) ◀─────┘
                                                 │
                                                 ▼
                                             /feedback
                                                 │
                                                 ▼
                                         /lqr_controller_cpp
                                                 │
                                                 ▼
                                     /motor_cmd/left, /motor_cmd/right
                                                 │
                                                 ▼
                                          /motor_controller (ШИМ)
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

    # --- 1. Декларация аргументов запуска ---
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Если true — все узлы используют время из топика /clock.")

    # --- 2. Конфигурация узлов ---

    # --- Группа IMU (Драйвер + Стандартный Фильтр) ---
    mpu6050_driver_node = Node(
        package="mpu6050_cpp_pkg",
        executable="mpu6050_node",
        name="mpu6050_node",
        output="screen",
        emulate_tty=True
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
            {'do_adaptive_gain': True},
            {'publish_debug_topics': True},
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
            'left_encoder_device': '/dev/input/by-path/platform-rotary@18-event',
            'right_encoder_device': '/dev/input/by-path/platform-rotary@8-event',
            # Частота публикации одометрии.
            'loop_rate': 400
        }]
    )

    # --- Группа высокоуровневого управления (LQR) ---
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
                'filter_window_size': 15
            }]
        ),
        Node(
            package="lqr_controller_cpp",
            executable="lqr_controller",
            name="lqr_controller_cpp"
        ),
    ]

    # --- Драйвер моторов ---
    motor_driver_node = Node(
        package="motor_controller_cpp",
        executable="motor_controller",
        name="motor_controller",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "critical_angle_deg": 60.0,
            "V_clip": 12.0,
            "min_duty_pct": 7.0,
        }]
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
