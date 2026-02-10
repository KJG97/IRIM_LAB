"""
Articulation / DOF 인덱스 해석.
"""


def resolve_dof_indices(articulation, joint_names: list) -> list:
    """조인트 이름 리스트로 DOF 인덱스 리스트 반환. 실패 시 빈 리스트."""
    indices = []
    for name in joint_names:
        idx = articulation.get_dof_index(name)
        if idx is None:
            return []
        indices.append(idx)
    return indices
