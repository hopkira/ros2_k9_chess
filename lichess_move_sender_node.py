import rclpy
from rclpy.node import Node
from chess_interfaces.msg import ChessMove
import asyncio

# Listens for bot moves and sends them to Lichess via API.

class LichessMoveSenderNode(Node):
    def __init__(self):
        super().__init__('lichess_move_sender')
        self.subscription = self.create_subscription(ChessMove, '/lichess/bot_move', self.on_bot_move, 10)

    def on_bot_move(self, msg):
        asyncio.create_task(self.send_move_to_lichess(msg))

    async def send_move_to_lichess(self, move):
        # Send move to Lichess using your bot token
        # Replace with actual HTTP call
        await lichess_api.send_move(move.uci)

def main():
    rclpy.init()
    node = LichessMoveSenderNode()
    rclpy.spin(node)
