"""Explicit camera choice; Astra remains the default baseline camera."""
from dataclasses import dataclass
from .core import Settings


@dataclass(frozen=True)
class CameraProfile:
    topic: str
    status_topic: str
    settings: Settings


def camera_profile(camera='astra_s'):
    if camera == 'astra_s':
        return CameraProfile('/wheeltec_camera/rgbd', '/wheeltec_camera/status', Settings())
    if camera == 'd455':
        return CameraProfile('/wheeltec_camera/d455/rgbd', '/wheeltec_camera/d455/status',
                             Settings(expected_frame='wheeltec_d455_color_optical_frame', depth_max=6.))
    raise ValueError('Unsupported camera; select astra_s or d455')
