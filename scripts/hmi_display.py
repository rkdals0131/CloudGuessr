#!/usr/bin/env python3
"""
CloudGuessr HMI Display
Rich 라이브러리를 사용한 터미널 기반 게임 정보 표시
"""

import json
from collections import deque
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.layout import Layout


class HMIDisplay(Node):
    def __init__(self):
        super().__init__('hmi_display')

        self.console = Console()

        # Game state
        self.current_round = 0
        self.total_rounds = 0
        self.state = "IDLE"
        self.difficulty = ""

        # Last result
        self.last_score = 0
        self.last_status = ""

        # Log messages (최근 15개)
        self.log_messages = deque(maxlen=15)

        # Subscribers
        self.status_sub = self.create_subscription(
            String, '/cloudguessr/status', self.on_status, 10)
        self.result_sub = self.create_subscription(
            String, '/cloudguessr/result', self.on_result, 10)
        self.hmi_log_sub = self.create_subscription(
            String, '/cloudguessr/hmi_log', self.on_hmi_log, 10)

    def on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.current_round = data.get('round_idx', 0) + 1
            self.total_rounds = data.get('total_rounds', 0)
            self.state = data.get('state_str', 'UNKNOWN')
            self.difficulty = data.get('difficulty', '')
        except json.JSONDecodeError:
            pass

    def on_result(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.last_score = data.get('score', 0)
            self.last_status = data.get('status', '')
        except json.JSONDecodeError:
            pass

    def on_hmi_log(self, msg: String):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_messages.append(f"[dim]{timestamp}[/] {msg.data}")

    def make_layout(self) -> Layout:
        layout = Layout()
        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="status", size=5),
            Layout(name="logs", ratio=1),
        )
        return layout

    def render_header(self) -> Panel:
        title = Text()
        title.append("  CloudGuessr  ", style="bold white on blue")
        title.append("  Point Cloud Geo-Guesser Game  ", style="dim")
        return Panel(title, border_style="blue")

    def render_status(self) -> Panel:
        table = Table(show_header=False, box=None, expand=True)
        table.add_column(justify="center", ratio=1)
        table.add_column(justify="center", ratio=1)
        table.add_column(justify="center", ratio=1)
        table.add_column(justify="center", ratio=1)

        # State color
        state_style = {
            "IDLE": "dim",
            "LOADING": "yellow",
            "WAITING_CLICK": "green bold",
            "SCORING": "cyan bold",
            "SHOWING_RESULT": "magenta bold",
        }.get(self.state, "white")

        # Difficulty 한국어
        diff_kr = self.difficulty
        if diff_kr == "easy":
            diff_kr = "쉬움"
        elif diff_kr == "medium":
            diff_kr = "보통"
        elif diff_kr == "hard":
            diff_kr = "어려움"

        # Score style
        if self.last_status == "OK":
            if self.last_score >= 4000:
                score_style = "bold green"
            elif self.last_score >= 2000:
                score_style = "bold yellow"
            else:
                score_style = "bold red"
        elif self.last_status == "FAIL":
            score_style = "bold red"
        else:
            score_style = "dim"

        table.add_row(
            f"[bold]ROUND[/]\n[white]{self.current_round}/{self.total_rounds}[/]",
            f"[bold]STATE[/]\n[{state_style}]{self.state}[/]",
            f"[bold]DIFF[/]\n[white]{diff_kr or '-'}[/]",
            f"[bold]SCORE[/]\n[{score_style}]{self.last_score}[/]",
        )

        return Panel(table, title="[bold]Game Status[/]", border_style="green")

    def render_logs(self) -> Panel:
        if not self.log_messages:
            content = Text("대기 중...", style="dim")
        else:
            lines = list(self.log_messages)
            content = Text.from_markup("\n".join(lines))

        return Panel(
            content,
            title="[bold]Log[/]",
            border_style="cyan",
        )

    def render(self) -> Layout:
        layout = self.make_layout()
        layout["header"].update(self.render_header())
        layout["status"].update(self.render_status())
        layout["logs"].update(self.render_logs())
        return layout


def main():
    rclpy.init()
    node = HMIDisplay()

    console = Console()
    console.clear()

    try:
        with Live(node.render(), console=console, refresh_per_second=4, screen=True) as live:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                live.update(node.render())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
