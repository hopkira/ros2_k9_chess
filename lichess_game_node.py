import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from chess_interfaces.msg import ChessMove, GameResult
import asyncio

# Connects to Lichess
# Accepts challenges
# Monitors game state
# Publishes updates

class LichessGameManager(Node):
    def __init__(self):
        super().__init__('lichess_game_manager')
        self.publisher_game_started = self.create_publisher(Bool, '/lichess/game_started', 10)
        self.publisher_opponent_move = self.create_publisher(ChessMove, '/lichess/opponent_move', 10)
        self.publisher_game_over = self.create_publisher(GameResult, '/lichess/game_over', 10)

        asyncio.create_task(self.lichess_event_loop())

    async def lichess_event_loop(self):
        while rclpy.ok():
            # Connect to Lichess streaming events
            # Example pseudo-code:
            async for event in lichess_stream():
                if event['type'] == 'gameStart':
                    self.publisher_game_started.publish(Bool(data=True))
                elif event['type'] == 'gameState':
                    move = ChessMove(
                        from_square='e2', to_square='e4', uci='e2e4', san='e4'
                    )
                    self.publisher_opponent_move.publish(move)
                elif event['type'] == 'gameFinish':
                    result = GameResult(winner='white', reason='checkmate')
                    self.publisher_game_over.publish(result)

def main():
    rclpy.init()
    node = LichessGameManager()
    rclpy.spin(node)


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import json
import time
import chess
from lichess import LichessAPI

class LichessGameManager(Node):
    def __init__(self):
        super().__init__('lichess_game_manager')

        self.bot_token = os.getenv("LICHESS_BOT_TOKEN")
        self.username = os.getenv("LICHESS_USERNAME")
        lichess_url = "https://lichess.org/api/"
        self.li = LichessAPI(token=self.bot_token, url=lichess_url)

        self.board_pub = self.create_publisher(String, 'chess/board_state', 10)
        self.bot_move_pub = self.create_publisher(String, 'chess/bot_move_request', 10)

        self.move_sub = self.create_subscription(String, 'chess/bot_move', self.bot_move_callback, 10)

        self.board = chess.Board()
        self.game_id = None
        self.player_color = None

        self.timer = self.create_timer(3.0, self.poll_lichess)
        self.stream = None
        self.lines = None
        self.first_update = True

    def poll_lichess(self):
        if not self.game_id:
            response = self.li.get_event_stream()
            self.lines = response.iter_lines()
            for line in self.lines:
                if line:
                    event = json.loads(line.decode("utf-8"))
                    if event["type"] == "challenge":
                        self.li.accept_challenge(event["challenge"]["id"])
                    elif event["type"] == "gameStart":
                        self.game_id = event["game"]["id"]
                        self.stream = self.li.get_stream(self.game_id)
                        self.lines = self.stream.iter_lines()
                        self.first_update = True
                        break
        else:
            try:
                line = next(self.lines)
                if line:
                    data = json.loads(line.decode("utf-8"))
                    if data["type"] == "gameFull" and self.first_update:
                        white = data["white"]["name"].lower()
                        self.player_color = "white" if white == self.username.lower() else "black"
                        self.first_update = False

                    if data["type"] == "gameState":
                        moves = data["moves"].split()
                        self.board = chess.Board()
                        for move in moves:
                            self.board.push_uci(move)

                        if self.board.is_game_over():
                            self.get_logger().info("Game over")
                            return

                        board_turn = "white" if self.board.turn == chess.WHITE else "black"
                        if board_turn == self.player_color:
                            self.bot_move_pub.publish(String(data=self.board.fen()))

                        self.board_pub.publish(String(data=self.board.fen()))
            except StopIteration:
                self.get_logger().info("Game stream ended")
                self.game_id = None

    def bot_move_callback(self, msg):
        move_uci = msg.data
        if self.game_id:
            self.li.make_move(self.game_id, move_uci)


def main(args=None):
    rclpy.init(args=args)
    node = LichessGameManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()