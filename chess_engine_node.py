import rclpy
from rclpy.node import Node
from chess_interfaces.msg import ChessMove
import chess
import chess.engine

# When notified of an opponent's move.
# Compute best move
# Publish bot move

class ChessEngineNode(Node):
    def __init__(self):
        super().__init__('chess_engine_node')
        self.subscription = self.create_subscription(ChessMove, '/lichess/opponent_move', self.on_opponent_move, 10)
        self.publisher = self.create_publisher(ChessMove, '/lichess/bot_move', 10)
        self.board = chess.Board()
        self.engine = chess.engine.SimpleEngine.popen_uci('/usr/bin/stockfish')

    def on_opponent_move(self, msg):
        self.board.push_uci(msg.uci)
        result = self.engine.play(self.board, chess.engine.Limit(time=0.5))
        move = result.move

        self.board.push(move)

        reply = ChessMove(
            from_square=move.uci()[:2],
            to_square=move.uci()[2:],
            uci=move.uci(),
            san=self.board.san(move)
        )
        self.publisher.publish(reply)

    def destroy_node(self):
        self.engine.quit()
        super().destroy_node()

def main():
    rclpy.init()
    node = ChessEngineNode()
    rclpy.spin(node)
    node.destroy_node()
