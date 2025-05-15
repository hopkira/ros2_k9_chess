import rclpy
from rclpy.node import Node
from chess_interfaces.msg import ChessMove
import chess

# Maintains the current game state in ROS2
# and could offer services to query the game board.

class ChessStateNode(Node):
    def __init__(self):
        super().__init__('chess_state_node')
        self.board = chess.Board()
        self.create_subscription(ChessMove, '/lichess/opponent_move', self.on_move, 10)
        self.create_subscription(ChessMove, '/lichess/bot_move', self.on_move, 10)

    def on_move(self, msg):
        self.board.push_uci(msg.uci)
        self.get_logger().info(f"Board FEN: {self.board.fen()}")

def main():
    rclpy.init()
    node = ChessStateNode()
    rclpy.spin(node)
