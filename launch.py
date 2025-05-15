from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stockfish_path', default_value='/usr/bin/stockfish'),
        DeclareLaunchArgument('lichess_token', default_value='your_bot_token_here'),

        Node(
            package='lichess_chess_bot',
            executable='lichess_game_manager_node',
            name='lichess_game_manager',
            output='screen',
            parameters=[
                {'lichess_token': LaunchConfiguration('lichess_token')}
            ]
        ),
        Node(
            package='lichess_chess_bot',
            executable='chess_engine_node',
            name='chess_engine',
            output='screen',
            parameters=[
                {'stockfish_path': LaunchConfiguration('stockfish_path')}
            ]
        ),
        Node(
            package='lichess_chess_bot',
            executable='lichess_move_sender_node',
            name='lichess_move_sender',
            output='screen',
            parameters=[
                {'lichess_token': LaunchConfiguration('lichess_token')}
            ]
        ),
        Node(
            package='lichess_chess_bot',
            executable='chess_state_node',
            name='chess_state',
            output='screen'
        )
    ])