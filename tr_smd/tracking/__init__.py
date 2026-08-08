"""Reference timing and interpolation utilities."""

from .smd_reference import (
    SMD_REFERENCE_DT_S,
    SMD_SUPPORT_POINTS,
    SMD_TRAJECTORY_DURATION_S,
    HermiteReference,
    finite_difference_velocity,
    load_primary_reference,
)

__all__ = [
    "SMD_REFERENCE_DT_S",
    "SMD_SUPPORT_POINTS",
    "SMD_TRAJECTORY_DURATION_S",
    "HermiteReference",
    "finite_difference_velocity",
    "load_primary_reference",
]
