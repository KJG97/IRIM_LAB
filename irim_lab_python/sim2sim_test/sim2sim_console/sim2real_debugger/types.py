from typing import Callable, Dict

import numpy as np

ObsDict = Dict[str, np.ndarray]
ObsProvider = Callable[[], ObsDict]

__all__ = [
    "ObsDict",
    "ObsProvider",
]
