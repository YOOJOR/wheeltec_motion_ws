"""D455 contract regression: aligned device metadata, no Astra calibration overlay."""
import copy
from array import array
import numpy as np
import pytest
from wheeltec_rgbd.core import Settings, normalize, rectify, FrameError
from test_core import make_pair


def test_d455_aligned_mm_depth_extends_beyond_astra_range():
    pair = make_pair(delta=0)
    pair[1].data = array('B', np.array([[0,399,400],[3000,6000,6001]], dtype='<u2').tobytes())
    rgb, depth, fraction, delta = normalize(pair, Settings(depth_max=6.), 2_010_000_000)
    np.testing.assert_allclose(depth[[0,1,1],[2,0,1]], [.4,3.,6.])
    assert np.isnan(depth[[0,0,1],[0,1,2]]).all()
    assert fraction == .5 and delta == 0.
    _, astra_depth, _, _ = normalize(pair, Settings(), 2_010_000_000)
    assert np.isnan(astra_depth[1]).all()  # Astra defaults unchanged.


def test_d455_zero_distortion_device_projection_preserved():
    pair = make_pair(delta=0)
    for info in pair[2:]:
        info.distortion_model = 'plumb_bob'
        info.d = [0.]*5
    rgb, depth, _, _ = normalize(pair, Settings(depth_max=6.), 2_010_000_000)
    out_rgb, out_depth, ci, di = rectify(rgb, depth, *pair[2:])
    np.testing.assert_array_equal(out_rgb, rgb)
    np.testing.assert_array_equal(out_depth, depth)
    assert list(ci.k) == list(pair[2].k) == list(di.k)
    assert ci.header.frame_id == pair[0].header.frame_id


@pytest.mark.parametrize('fault', ['unregistered', 'wrong_intrinsics'])
def test_native_depth_cannot_masquerade_as_aligned_depth(fault):
    pair = list(copy.deepcopy(make_pair()))
    if fault == 'unregistered':
        pair[1].header.frame_id = pair[3].header.frame_id = 'wheeltec_camera_depth_optical_frame'
    else:
        pair[3].k[0] += 10.
    with pytest.raises(FrameError): normalize(pair, Settings(depth_max=6.), 2_010_000_000)


def test_camera_selection_preserves_astra_and_separates_d455():
    from wheeltec_rgbd.profiles import camera_profile
    astra, d455 = camera_profile(), camera_profile('d455')
    assert astra == camera_profile('astra_s')
    assert astra.settings == Settings()
    assert astra.topic != d455.topic and astra.status_topic != d455.status_topic
    assert astra.settings.expected_frame != d455.settings.expected_frame
    assert d455.settings.depth_max == 6.
    with pytest.raises(ValueError): camera_profile('automatic')
