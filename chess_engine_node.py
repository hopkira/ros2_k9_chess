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


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lichess_interfaces.msg import BoardState, BotMoveRequest, BotMove

import chess
import chess.engine
import os

class ChessEngineNode(Node):
    def __init__(self):
        super().__init__('chess_engine_node')

        self.board = chess.Board()
        self.engine_path = os.getenv('STOCKFISH_PATH', '/usr/games/stockfish')
        self.engine = chess.engine.SimpleEngine.popen_uci(self.engine_path)

        self.subscription = self.create_subscription(
            BotMoveRequest,
            'chess/bot_move_request',
            self.handle_move_request,
            10
        )
        self.state_subscription = self.create_subscription(
            BoardState,
            'chess/board_state',
            self.update_board_state,
            10
        )
        self.move_publisher = self.create_publisher(BotMove, 'chess/bot_move', 10)

        self.get_logger().info('Chess Engine Node has started.')

    def update_board_state(self, msg: BoardState):
        self.board = chess.Board()
        for move_uci in msg.moves:
            move = chess.Move.from_uci(move_uci)
            if self.board.is_legal(move):
                self.board.push(move)
            else:
                self.get_logger().warn(f"Illegal move in board state: {move_uci}")

    def handle_move_request(self, msg: BotMoveRequest):
        if not self.board.is_game_over():
            self.get_logger().info('Computing move...')
            try:
                result = self.engine.play(self.board, chess.engine.Limit(time=1.0))
                move = result.move

                move_msg = BotMove()
                move_msg.game_id = msg.game_id
                move_msg.move = move.uci()
                self.move_publisher.publish(move_msg)

                self.get_logger().info(f"Published bot move: {move.uci()}")
            except Exception as e:
                self.get_logger().error(f"Engine error: {str(e)}")
        else:
            self.get_logger().info("Game is already over. No move computed.")

    def destroy_node(self):
        self.engine.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChessEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
