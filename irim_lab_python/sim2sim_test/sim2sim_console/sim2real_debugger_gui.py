import logging
import os
import sys

from PySide6.QtWidgets import QApplication

try:
    from .sim2real_debugger.dashboard import Sim2RealDebugger
except ImportError:
    from sim2real_debugger.dashboard import Sim2RealDebugger

_DEBUG = os.environ.get("SIM2REAL_DEBUG", "0") == "1"
logging.basicConfig(
    level=logging.DEBUG if _DEBUG else logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
)


def show_debugger(mode: str = "sim2real", obs_provider=None, physics_dt_provider=None) -> None:
    """Isaac Sim 등 기존 앱 내부에서 디버거 창만 띄울 때 사용. app.exec()/sys.exit 호출 없음.
    Sim2Sim 모드에서 obs_provider로 관측 채움, physics_dt_provider로 입력 주기(Hz/ms) 표시."""
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    window = Sim2RealDebugger(
        obs_provider=obs_provider,
        deploy_mode=mode,
        physics_dt_provider=physics_dt_provider,
    )
    window.show()

    # Isaac Sim 메인 루프가 Qt 이벤트를 처리하지 않으므로, 매 프레임 processEvents() 호출
    try:
        import omni.kit.app
        event_stream = omni.kit.app.get_app().get_update_event_stream()
        if event_stream:
            state = {"active": True, "window": window}

            def _pump_qt_events(_event):
                if not state.get("active"):
                    return
                try:
                    if not state["window"].isVisible():
                        state["active"] = False
                        return
                    QApplication.processEvents()
                except Exception:
                    state["active"] = False

            state["_sub"] = event_stream.create_subscription_to_pop(_pump_qt_events)
    except Exception:
        pass  # Isaac Sim 밖에서 실행 시 구독 생략


def main() -> None:
    app = QApplication(sys.argv)
    window = Sim2RealDebugger()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
