"""Rotate, reduce, and measure depth images without camera hardware."""

from typing import Optional, Tuple

import numpy as np


RANGE_MM = 2000
BLOCK_SIZE = 4
ZONE_NAMES = ("Left", "Centre", "Right")


def rotate_frames(depth: np.ndarray, confidence: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Rotate both images 180 degrees before other image operations."""
    return np.rot90(depth, 2), np.rot90(confidence, 2)


def downsample_depth(depth: np.ndarray, confidence: np.ndarray,
                     min_confidence: float = 30.0) -> np.ndarray:
    """Use the median valid depth in each 4 by 4 block."""
    if depth.ndim != 2 or depth.shape != confidence.shape:
        raise ValueError("Depth and confidence must have the same two-dimensional shape.")
    height, width = depth.shape
    if not height or not width or height % BLOCK_SIZE or width % BLOCK_SIZE:
        raise ValueError("Image dimensions must be positive multiples of four.")
    if not np.isfinite(min_confidence) or min_confidence < 0:
        raise ValueError("Minimum confidence must be finite and non-negative.")
    # Put each block in one contiguous row before filtering and counting.
    shape = (height // 4, 4, width // 4, 4)
    blocks = depth.reshape(shape).transpose(0, 2, 1, 3).reshape(-1, 16)
    quality = confidence.reshape(shape).transpose(0, 2, 1, 3).reshape(-1, 16)
    valid = (np.isfinite(blocks) & (blocks > 0) & (blocks <= RANGE_MM)
             & np.isfinite(quality) & (quality >= min_confidence))
    count = valid.sum(axis=-1, dtype=np.int16)
    blocks = np.where(valid, blocks, np.inf)
    blocks.sort(axis=-1)
    rows = np.arange(len(blocks))
    lower = np.maximum((count - 1) // 2, 0)
    median = (blocks[rows, lower] + blocks[rows, count // 2]) / 2
    reduced = np.rint(np.where(count > 0, median, 0)).astype(np.uint16)
    return reduced.reshape(height // 4, width // 4)


def _zones(depth: np.ndarray) -> list:
    if depth.ndim != 2 or depth.shape[0] < 1 or depth.shape[1] < 3:
        raise ValueError("Use an image with at least one row and three columns.")
    return np.array_split(depth, 3, axis=1)


def _valid_values(depth: np.ndarray) -> np.ndarray:
    return depth[np.isfinite(depth) & (depth > 0)]


def zone_distances(depth: np.ndarray) -> Tuple[Optional[float], ...]:
    """Find each zone's 10th percentile. Ten percent of valid values are at or below this value."""
    result = []
    for zone in _zones(depth):
        values = _valid_values(zone)
        result.append(float(np.percentile(values, 10)) if values.size else None)
    return tuple(result)


def centre_distance(depth: np.ndarray) -> Optional[float]:
    """Find the median valid depth in the centre third."""
    values = _valid_values(_zones(depth)[1])
    return float(np.median(values)) if values.size else None
