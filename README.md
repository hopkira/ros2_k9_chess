# k9_ros_chess

ROS2 Chess Nodes

This guide sets up and launches the ROS 2 nodes for a chess-playing robot that integrates with Lichess and Stockfish, using launch-time parameters.


---

## 🛠️ Add Entry Points to `setup.py`

```python
entry_points={
    'console_scripts': [
        'lichess_game_manager_node = lichess_chess_bot.lichess_game_manager_node:main',
        'chess_engine_node = lichess_chess_bot.chess_engine_node:main',
        'lichess_move_sender_node = lichess_chess_bot.lichess_move_sender_node:main',
        'chess_state_node = lichess_chess_bot.chess_state_node:main',
    ],
},
```

---

## 📦 Build Your Package

```bash
colcon build --packages-select lichess_chess_bot
source install/setup.bash
```

---

## 🚀 Launch the Chess Bot

You can now pass parameters to the launch file at runtime:

```bash
ros2 launch lichess_chess_bot chess_bot_launch.py stockfish_path:=/path/to/stockfish lichess_token:=your_real_token
```

---
