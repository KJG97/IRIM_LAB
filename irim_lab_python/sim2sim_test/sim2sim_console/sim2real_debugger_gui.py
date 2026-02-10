import logging
import os
import sys

from PySide6.QtWidgets import QApplication

from sim2real_debugger.dashboard import Sim2RealDebugger

_DEBUG = os.environ.get("SIM2REAL_DEBUG", "0") == "1"
logging.basicConfig(
    level=logging.DEBUG if _DEBUG else logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
)


def show_debugger() -> None:
    """Isaac Sim 등 기존 앱 내부에서 디버거 창만 띄울 때 사용. app.exec()/sys.exit 호출 없음."""
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    window = Sim2RealDebugger()
    window.show()


def main() -> None:
    app = QApplication(sys.argv)
    window = Sim2RealDebugger()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
