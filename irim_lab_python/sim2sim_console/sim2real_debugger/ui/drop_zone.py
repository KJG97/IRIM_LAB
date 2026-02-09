from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QFrame, QLabel, QVBoxLayout


class DropZone(QFrame):
    files_dropped = Signal(list)  # list[str]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAcceptDrops(True)
        self._set_idle_style()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        lbl = QLabel("Drag&Drop")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("font-size: 32px; font-weight: bold; color: #e0e0e0;")
        layout.addWidget(lbl)

    @staticmethod
    def _is_supported(path: str) -> bool:
        p = path.lower()
        return p.endswith((".pt", ".pth", ".npz"))

    def _set_idle_style(self):
        self.setStyleSheet("QFrame { background-color: #333; border: 2px dashed #777; border-radius: 10px; }")

    def _set_ready_style(self):
        self.setStyleSheet("QFrame { background-color: #2b5c2b; border: 2px dashed #00ff00; border-radius: 10px; }")

    def dragEnterEvent(self, event):
        if not event.mimeData().hasUrls():
            event.ignore()
            return
        paths = [u.toLocalFile() for u in event.mimeData().urls()]
        if any(self._is_supported(p) for p in paths):
            self._set_ready_style()
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragLeaveEvent(self, event):
        self._set_idle_style()
        event.accept()

    def dropEvent(self, event):
        self._set_idle_style()
        if not event.mimeData().hasUrls():
            return
        paths = [u.toLocalFile() for u in event.mimeData().urls()]
        paths = [p for p in paths if self._is_supported(p)]
        if paths:
            self.files_dropped.emit(paths)


__all__ = ["DropZone"]
