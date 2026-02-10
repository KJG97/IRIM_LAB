"""
쿼터니언 연산 [w, x, y, z].
"""

import numpy as np


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """단일 쿼터니언 [w,x,y,z]의 켤레 반환."""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=q.dtype)


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """두 단일 쿼터니언 [w,x,y,z]의 곱."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=q1.dtype)


def quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """쿼터니언 q로 3D 벡터 v 회전."""
    v_q = np.array([0.0, v[0], v[1], v[2]], dtype=q.dtype)
    return quat_mul(quat_mul(q, v_q), quat_conjugate(q))[1:]
