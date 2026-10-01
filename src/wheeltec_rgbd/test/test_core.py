import copy
import numpy as np
import pytest
from sensor_msgs.msg import Image, CameraInfo
from wheeltec_rgbd.core import Settings, PairBuffer, FrameError, normalize, image_array


def make_pair(stamp=2_000_000_000, delta=1_000_000, padded=False, big_endian=False):
    images = []
    for kind, ts in [('color', stamp), ('depth', stamp+delta)]:
        msg = Image()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ts, 10**9)
        msg.header.frame_id = Settings().expected_frame
        msg.height, msg.width = 2, 3
        if kind == 'color':
            msg.encoding = 'rgb8'
            array = np.arange(18, dtype=np.uint8).reshape(2, 9)
        else:
            msg.encoding = '16UC1'
            msg.is_bigendian = int(big_endian)
            array = np.array([[0, 400, 1000], [2000, 2001, 65535]],
                             dtype='>u2' if big_endian else '<u2').view('u1').reshape(2, 6)
        rows = [row.tobytes()+(b'xx' if padded else b'') for row in array]
        msg.step = len(rows[0])
        msg.data = b''.join(rows)
        images.append(msg)
    infos = []
    for img in images:
        info = CameraInfo()
        info.header = copy.deepcopy(img.header)
        info.height, info.width = img.height, img.width
        info.k = [100., 0., 1., 0., 100., 0.5, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [100., 0., 1., 0., 0., 100., 0.5, 0., 0., 0., 1., 0.]
        infos.append(info)
    return tuple(images+infos)


@pytest.mark.parametrize('padded,big_endian', [(False, False), (True, False), (True, True)])
def test_layout_units_and_range(padded, big_endian):
    pair = make_pair(padded=padded, big_endian=big_endian)
    rgb, depth, fraction, delta = normalize(pair, Settings(), 2_010_000_000)
    np.testing.assert_array_equal(rgb, np.arange(18).reshape(2,3,3))
    np.testing.assert_allclose(depth[[0,0,1],[1,2,0]], [0.4,1.0,2.0])
    assert np.isnan(depth[[0,1,1],[0,1,2]]).all()
    assert fraction == 0.5 and delta == 0.001


def test_bgr_and_float_depth_without_double_conversion():
    pair = list(make_pair())
    pair[0].encoding = 'bgr8'
    pair[1].encoding = '32FC1'
    pair[1].step = 12
    pair[1].data = np.array([[np.nan, .4, 1.], [2., np.inf, -.1]], dtype='<f4').tobytes()
    rgb, depth, _, _ = normalize(pair, Settings(), 2_010_000_000)
    assert tuple(rgb[0,0]) == (2,1,0) and depth[0,2] == 1.


@pytest.mark.parametrize('fault,expected', [
    ('frame', 'FRAME_MISMATCH'), ('intrinsics', 'INVALID_INTRINSICS'),
    ('registered_k', 'REGISTERED_INTRINSICS'), ('info_stamp', 'CAMERA_INFO_MISMATCH'),
    ('encoding', 'UNSUPPORTED_ENCODING'), ('stride', 'INVALID_IMAGE_LAYOUT'),
    ('size', 'RGB_DEPTH_SIZE'), ('zero', 'NO_USABLE_DEPTH'),
])
def test_invalid_observation_is_rejected(fault, expected):
    pair = make_pair()
    if fault == 'frame': pair[1].header.frame_id = 'depth_optical'
    if fault == 'intrinsics': pair[2].k[0] = float('nan')
    if fault == 'registered_k': pair[3].k[0] = 120.
    if fault == 'info_stamp': pair[2].header.stamp.nanosec += 1
    if fault == 'encoding': pair[0].encoding = 'mono8'
    if fault == 'stride': pair[1].step = 1
    if fault == 'size': pair[1].width = 4
    if fault == 'zero': pair[1].data = b'\0'*12
    with pytest.raises(FrameError, match=expected):
        normalize(pair, Settings(), 2_010_000_000)


@pytest.mark.parametrize('now', [3_000_000_000, 1_000_000_000])
def test_stale_or_future_source_rejected(now):
    with pytest.raises(FrameError, match='STALE_OR_FUTURE'):
        normalize(make_pair(), Settings(), now)


def test_pairing_requires_exact_image_info_and_new_images():
    buf = PairBuffer(Settings())
    pair = make_pair()
    assert buf.add('color', pair[0], 2_010_000_000) is None
    assert buf.add('depth', pair[1], 2_010_000_000) is None
    assert buf.add('color_info', pair[2], 2_010_000_000) is None
    assert buf.add('depth_info', pair[3], 2_010_000_000) == pair
    with pytest.raises(FrameError, match='DUPLICATE'):
        buf.add('color', pair[0], 2_010_000_000)
    with pytest.raises(FrameError, match='OUT_OF_ORDER'):
        buf.add('depth', make_pair(stamp=1_999_000_000)[1], 2_010_000_000)
    newer = make_pair(stamp=2_020_000_000)
    for k, msg in zip(buf.kinds, newer):
        result = buf.add(k, msg, 2_030_000_000)
    assert result == newer


def test_unsynchronized_sources_not_paired_and_storage_bounded():
    buf = PairBuffer(Settings())
    pair = make_pair(delta=50_000_000)
    for k, msg in zip(buf.kinds, pair):
        assert buf.add(k, msg, 2_060_000_000) is None
    for i in range(10):
        buf.add('color', make_pair(stamp=2_100_000_000+i*1_000_000)[0], 2_200_000_000)
    assert len(buf.samples['color']) == 4
    buf.clear()
    assert all(not x for x in buf.samples.values())


def test_invalid_config():
    with pytest.raises(ValueError): Settings(depth_min=2., depth_max=1.)
    with pytest.raises(ValueError): Settings(frame_timeout=float('nan'))


def test_registered_rectification_uses_supplied_projection():
    from wheeltec_rgbd.core import rectify
    pair = make_pair()
    rgb, metres, _, _ = normalize(pair, Settings(), 2_010_000_000)
    for info in pair[2:]:
        info.distortion_model = 'plumb_bob'
        info.d = [.01, 0., 0., 0., 0.]
        info.p[0], info.p[5] = 110., 110.
    out, depth, ci, di = rectify(rgb, metres, *pair[2:])
    assert out.shape == rgb.shape and depth.shape == metres.shape
    assert ci.k[0] == 110. and di.k[4] == 110. and not any(ci.d)
    assert pair[2].k[0] == 100.
    pair[3].d[0] = .1
    with pytest.raises(FrameError, match='UNSUPPORTED_REGISTERED_CALIBRATION'):
        rectify(rgb,metres,*pair[2:])


def test_explicit_calibration_overlay_keeps_source_stamp_and_frame():
    from wheeltec_rgbd.calibration import overlay_calibration
    pair = make_pair()
    template = copy.deepcopy(pair[3])
    pair[3].k[0] = float('nan')
    fixed = overlay_calibration(pair, depth_template=template)
    assert fixed[3].k[0] == 100. and np.isnan(pair[3].k[0])
    assert fixed[3].header == pair[3].header
    normalize(fixed, Settings(), 2_010_000_000)
    pair[3].header.frame_id = 'wrong_frame'
    with pytest.raises(FrameError, match='FRAME_MISMATCH'):
        normalize(overlay_calibration(pair, depth_template=template), Settings(), 2_010_000_000)
    template.width = 4
    with pytest.raises(FrameError, match='RESOLUTION_MISMATCH'):
        overlay_calibration(pair, depth_template=template)


def test_local_calibration_loader_rejects_nonfinite_and_wrong_matrix(tmp_path):
    import yaml
    from wheeltec_rgbd.calibration import load_camera_info
    data = {'image_width':3, 'image_height':2, 'distortion_model':'plumb_bob',
            'camera_matrix': {'rows':3,'cols':3,'data':[float(x) for x in make_pair()[2].k]},
            'rectification_matrix': {'rows':3,'cols':3,'data':[float(x) for x in make_pair()[2].r]},
            'projection_matrix': {'rows':3,'cols':4,'data':[float(x) for x in make_pair()[2].p]},
            'distortion_coefficients': {'rows':1,'cols':5,'data':[0.]*5}}
    path = tmp_path/'calibration.yaml'
    path.write_text(yaml.safe_dump(data))
    assert load_camera_info(path.as_uri()).k[0] == 100.
    data['camera_matrix']['data'][0] = float('nan')
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='Nonfinite'): load_camera_info(path.as_uri())
    with pytest.raises(ValueError, match='local'): load_camera_info('https://example.com/calib.yaml')
    data['camera_matrix']['data'][0] = 100.
    data['camera_matrix']['rows'] = 2
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='matrix'): load_camera_info(path.as_uri())
