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
        self.last_dist_error = None
        self.last_score_tier = None
        self.last_elapsed_ms = None
        self.last_comment = "-"

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
            self.last_dist_error = data.get('dist_error_m')
            self.last_score_tier = data.get('score_tier')
            self.last_elapsed_ms = data.get('elapsed_ms')

            comment = str(data.get('comment', '') or '').strip()
            if not comment:
                comment = '-'
            self.last_comment = comment
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
            Layout(name="result", size=8),
            Layout(name="logs", ratio=1),
        )
        return layout

    def render_header(self) -> Panel:
        title = Text()
        title.append("  CloudGuessr  ", style="bold white on blue")
        title.append("  Point Cloud Geo-Guesser Game  ", style="dim")
        return Panel(title, border_style="blue")

    def _score_style(self) -> str:
        if self.last_status == "FAIL":
            return "bold red"
        if self.last_score_tier == "S":
            return "bold green"
        if self.last_score_tier == "A":
            return "bold yellow"
        if self.last_score_tier == "B":
            return "bold #ffb173"
        if self.last_score_tier == "C":
            return "bold #ff8c8c"
        if self.last_score >= 4500:
            return "bold green"
        if self.last_score >= 3000:
            return "bold yellow"
        if self.last_score >= 1500:
            return "bold #ffb173"
        return "dim"

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
            "ALIGNING": "bright_cyan bold",
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

        score_style = self._score_style()

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

    def render_result(self) -> Panel:
        table = Table(show_header=False, box=None, expand=True)
        table.add_column(justify="left", ratio=1)
        table.add_column(justify="left", ratio=1)

        score_style = self._score_style()

        dist_text = "-"
        if self.last_dist_error is not None:
            dist_text = f"{float(self.last_dist_error):.2f} m"

        elapsed_text = "-"
        if self.last_elapsed_ms is not None:
            elapsed_text = f"{float(self.last_elapsed_ms):.0f} ms"

        table.add_row(
            f"[bold]점수[/]\n[{score_style}]{self.last_score}[/]",
            f"[bold]거리 오차[/]\n[white]{dist_text}[/]",
        )
        table.add_row(
            f"[bold]처리 시간[/]\n[white]{elapsed_text}[/]",
            "",
        )
        table.add_row(
            f"[bold]멘트[/]\n[white]{self.last_comment}[/]",
            "",
        )

        return Panel(table, title="[bold]Last Result[/]", border_style="magenta")

    def render(self) -> Layout:
        layout = self.make_layout()
        layout["header"].update(self.render_header())
        layout["status"].update(self.render_status())
        layout["result"].update(self.render_result())
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
