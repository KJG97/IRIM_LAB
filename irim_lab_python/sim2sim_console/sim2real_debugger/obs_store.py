import threading
from typing import Dict

import numpy as np

from .types import ObsDict


class ObservationStore:
    """ROS thread / UI thread 간 데이터 경합 방지용 store."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data: Dict[str, np.ndarray] = {}

    def set(self, key: str, value: np.ndarray) -> None:
        with self._lock:
            self._data[key] = value

    def pop(self, key: str) -> None:
        with self._lock:
            self._data.pop(key, None)

    def clear(self) -> None:
        with self._lock:
            self._data.clear()

    def snapshot(self) -> ObsDict:
        with self._lock:
            return dict(self._data)


__all__ = ["ObservationStore"]
