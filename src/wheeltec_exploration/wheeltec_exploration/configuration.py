from pathlib import Path
import numpy as np
import yaml
from wheeltec_rgbd.posed import transform, rigid
from wheeltec_rgbd.profiles import camera_profile
from .core import Config


def load_configuration(path, camera=None):
    values = yaml.safe_load(Path(path).read_text())
    selected = camera or values['camera']
    camera_profile(selected)  # Never silently fall back to another camera.
    camera_cfg = values['cameras'][selected]
    if camera_cfg['base_from_camera'] is None:
        raise ValueError('CAMERA_INSTALLATION_NOT_CONFIGURED: '+selected)
    r,p,y = np.asarray(values['body_from_base_rpy'],float)/2
    cr,sr,cp,sp,cy,sy = np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    q = [sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]
    values['body_from_base'] = transform(values['body_from_base_translation'], q)
    values['base_from_camera'] = rigid(camera_cfg['base_from_camera'])
    values['camera'] = selected
    values['extrinsics_source'] = camera_cfg['source']
    values['extrinsics_confirmed'] = camera_cfg['confirmed'] is True
    map_values = dict(values.get('map',{}))
    map_values['base_height'] = values.get('base_height',0.0)
    values['map_config'] = Config(**map_values)
    if not values['prefix'].startswith('/') or values['prefix'].rstrip('/') in ('','/cmd_vel','/wheeltec_motion'):
        raise ValueError('INVALID_PREVIEW_PREFIX')
    return values
