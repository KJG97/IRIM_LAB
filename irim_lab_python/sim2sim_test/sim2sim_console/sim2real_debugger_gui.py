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


def main() -> None:
    app = QApplication(sys.argv)
    window = Sim2RealDebugger()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
