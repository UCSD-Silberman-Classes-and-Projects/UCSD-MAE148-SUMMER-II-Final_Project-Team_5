"""Shared validation helpers for the Mac vision bridge protocol."""

import math

import cv2


PROTOCOL_VERSION = 1
STOP_CLASS_ID = 11
STOP_LABEL = 'stop sign'
VALID_LANE_MODES = ('NORMAL', 'PIT', 'EXIT_PIT')


class ConfirmationFilter:
    """Require consecutive positive frames and clear on any failure."""

    def __init__(self, required_hits):
        """Configure how many uninterrupted positive frames are required."""
        if required_hits < 1:
            raise ValueError('required_hits must be positive')
        self.required_hits = int(required_hits)
        self.hits = 0
        self.confirmed = False

    def update(self, detected):
        """Add one result and return the current confirmed state."""
        if detected:
            self.hits += 1
            self.confirmed = self.hits >= self.required_hits
        else:
            self.clear()
        return self.confirmed

    def clear(self):
        """Discard all positive history."""
        self.hits = 0
        self.confirmed = False
        return False


def normalize_lane_mode(mode):
    """Match the existing detector's safe fallback to NORMAL."""
    normalized = str(mode).upper()
    return normalized if normalized in VALID_LANE_MODES else 'NORMAL'


def resize_for_transport(frame, max_dimension):
    """Downscale without upscaling while preserving aspect ratio."""
    height, width = frame.shape[:2]
    if height < 1 or width < 1:
        raise ValueError('camera frame has invalid dimensions')
    scale = min(1.0, float(max_dimension) / max(height, width))
    if scale == 1.0:
        return frame
    output_width = max(1, round(width * scale))
    output_height = max(1, round(height * scale))
    return cv2.resize(
        frame,
        (output_width, output_height),
        interpolation=cv2.INTER_AREA,
    )


def _finite_number(payload, name, minimum=None, maximum=None):
    value = payload.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'response {name} is not numeric')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'response {name} is not finite')
    if minimum is not None and value < minimum:
        raise ValueError(f'response {name} is below {minimum}')
    if maximum is not None and value > maximum:
        raise ValueError(f'response {name} is above {maximum}')
    return value


def validate_response(
    payload,
    expected_request_id,
    expected_lane_mode,
    confidence_threshold,
    min_box_height_ratio,
):
    """Validate an entire combined response before accepting either result."""
    if not isinstance(payload, dict):
        raise ValueError('response must be a JSON object')
    protocol_version = payload.get('protocol_version')
    if (
        type(protocol_version) is not int
        or protocol_version != PROTOCOL_VERSION
    ):
        raise ValueError('response protocol version mismatch')
    request_id = payload.get('request_id')
    if type(request_id) is not int or request_id != expected_request_id:
        raise ValueError('response request id mismatch')
    class_id = payload.get('class_id')
    if type(class_id) is not int or class_id != STOP_CLASS_ID:
        raise ValueError('response class id mismatch')
    if payload.get('label') != STOP_LABEL:
        raise ValueError('response class label mismatch')
    if type(payload.get('stop')) is not bool:
        raise ValueError('response stop value is not boolean')

    confidence = _finite_number(payload, 'confidence', 0.0, 1.0)
    box_ratio = _finite_number(
        payload,
        'box_height_ratio',
        0.0,
        1.0,
    )

    lane_mode = payload.get('lane_mode')
    if lane_mode != expected_lane_mode:
        raise ValueError('response lane mode mismatch')
    if type(payload.get('lane_acquired')) is not bool:
        raise ValueError('response lane_acquired value is not boolean')
    lane_acquired = payload['lane_acquired']
    expected_status = (
        f'{lane_mode}:ACQUIRED' if lane_acquired else f'{lane_mode}:LOST'
    )
    if payload.get('lane_status') != expected_status:
        raise ValueError('response lane status is inconsistent')

    centroid = payload.get('centroid')
    if centroid is not None:
        centroid = _finite_number(payload, 'centroid')
    elif lane_acquired:
        raise ValueError('acquired lane response has no centroid')

    stop = (
        payload['stop']
        and confidence >= confidence_threshold
        and box_ratio >= min_box_height_ratio
    )
    return {
        'stop': stop,
        'confidence': confidence,
        'box_height_ratio': box_ratio,
        'lane_mode': lane_mode,
        'lane_acquired': lane_acquired,
        'lane_status': expected_status,
        'centroid': centroid,
    }
