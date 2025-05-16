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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from chess_interfaces.msg import ChessMove, GameOutcome

import chess
import os
import chess.engine

class ChessStateNode(Node):
    def __init__(self):
        super().__init__('chess_state_node')

        self.board = chess.Board()

        # Publishers
        self.board_state_pub = self.create_publisher(String, '/chess/board_state', 10)
        self.legal_moves_pub = self.create_publisher(String, '/chess/legal_moves', 10)
        self.bot_move_request_pub = self.create_publisher(String, '/chess/request_bot_move', 10)
        self.game_outcome_pub = self.create_publisher(GameOutcome, '/chess/game_outcome', 10)

        # Subscribers
        self.move_sub = self.create_subscription(
            ChessMove,
            '/chess/move',
            self.handle_move,
            10
        )

        # Engine for scoring
        stockfish_path = os.getenv("STOCKFISH_PATH", "/usr/bin/stockfish")
        try:
            self.engine = chess.engine.SimpleEngine.popen_uci(stockfish_path)
        except Exception as e:
            self.get_logger().error(f"Failed to start Stockfish: {e}")
            self.engine = None

        self.get_logger().info("Chess State Node initialized.")

    def handle_move(self, msg: ChessMove):
        try:
            move = chess.Move.from_uci(msg.uci)
            if move in self.board.legal_moves:
                self.board.push(move)
                self.publish_board_state()

                # Evaluate game status
                if self.board.is_game_over():
                    self.publish_game_outcome()
                else:
                    if self.board.turn == chess.BLACK:
                        self.request_bot_move()
            else:
                self.get_logger().warn(f"Illegal move attempted: {msg.uci}")
        except Exception as e:
            self.get_logger().error(f"Failed to process move {msg.uci}: {e}")

    def publish_board_state(self):
        fen = self.board.fen()
        legal_moves = ' '.join([m.uci() for m in self.board.legal_moves])

        self.board_state_pub.publish(String(data=fen))
        self.legal_moves_pub.publish(String(data=legal_moves))

        self.get_logger().info(f"Board updated. FEN: {fen}")
        self.get_logger().info(f"Legal moves: {legal_moves}")

    def request_bot_move(self):
        self.bot_move_request_pub.publish(String(data="request"))
        self.get_logger().info("Requested bot move.")

    def publish_game_outcome(self):
        result = GameOutcome()

        if self.board.is_checkmate():
            if self.board.turn == chess.WHITE:
                result.result = "black_win"
                result.description = "Checkmate! Black wins."
            else:
                result.result = "white_win"
                result.description = "Checkmate! White wins."
        elif self.board.is_stalemate():
            result.result = "draw"
            result.description = "Draw by stalemate."
        elif self.board.is_insufficient_material():
            result.result = "draw"
            result.description = "Draw due to insufficient material."
        elif self.board.is_seventyfive_moves():
            result.result = "draw"
            result.description = "Draw by 75-move rule."
        elif self.board.is_fivefold_repetition():
            result.result = "draw"
            result.description = "Draw by fivefold repetition."
        else:
            result.result = "unknown"
            result.description = "Game over by unknown rule."

        self.game_outcome_pub.publish(result)
        self.get_logger().info(f"Game ended: {result.description}")

        # Stop engine
        if self.engine:
            self.engine.quit()

        # Reset board after short delay if needed (for new game)
        # self.board.reset()


def main(args=None):
    rclpy.init(args=args)
    node = ChessStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
