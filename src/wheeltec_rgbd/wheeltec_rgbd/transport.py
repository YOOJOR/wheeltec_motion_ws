"""Process-local Fast DDS transport sizing for VGA RGB-D publishers."""
import os
from pathlib import Path
from rclpy.utilities import get_rmw_implementation_identifier


def camera_transport_environment(profile):
    # A caller's explicit middleware/profile remains authoritative. Apply this
    # only to the camera driver and adapter, never to the user's global shell.
    if any(os.environ.get(key) for key in
           ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE')):
        return {}
    if get_rmw_implementation_identifier() not in ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp'):
        return {}
    profile = Path(profile)
    if not profile.is_file():
        raise FileNotFoundError('Camera Fast DDS profile missing: '+str(profile))
    return {'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile)}
