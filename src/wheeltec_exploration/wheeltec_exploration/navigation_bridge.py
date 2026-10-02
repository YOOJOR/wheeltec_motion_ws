"""Prepared adapter, NOT created by shadow runtime. Inject a client; arm explicitly.

No timers/replay. Software tests inject fake/isolated clients only. A future live
entry must obtain human acceptance first; this module provides no live CLI.
"""
import math
from wheeltec_motion_control.navigation_client import decision_velocity


class DecisionBridge:
    def __init__(self, client, *, extrinsics_confirmed=False, calibration_verified=False):
        self.client = client
        self.verified = extrinsics_confirmed is True and calibration_verified is True
        self.epoch = None
        self.last_stamp = 0

    def arm(self, epoch):
        if not self.verified: raise ValueError('CAMERA_GEOMETRY_NOT_VERIFIED')
        self.client.enable()
        self.epoch = epoch
        self.last_stamp = 0

    def stop(self):
        was_armed = self.epoch is not None
        self.epoch = None
        if was_armed: self.client.stop()

    def submit(self, decision, stamp_ns, epoch):
        if self.epoch is None: raise RuntimeError('EXPLICIT_ARM_REQUIRED')
        if epoch != self.epoch:
            self.stop()
            raise RuntimeError('LOCALIZATION_EPOCH_CHANGED')
        if decision.reason in ('NO_NEW_OBSERVATION','NO_TIME_MATCHED_POSE','WAITING_FOR_POSE'):
            self.client.poll()  # No cached decision or fresh timestamp is manufactured.
            return False
        if decision.reason != 'PLANNED' or decision.action not in ('forward','left','right'):
            self.stop()
            return False
        if type(stamp_ns) is not int or stamp_ns <= self.last_stamp:
            self.stop()
            raise ValueError('NEW_SOURCE_TIMESTAMP_REQUIRED')
        try:
            expected = decision_velocity(decision.action,self.client.forward_speed,self.client.turn_speed)
            actual = (decision.linear_x,decision.angular_z)
            if not all(math.isfinite(a) and math.isclose(a,b,abs_tol=1e-9)
                       for a,b in zip(actual,expected)):
                raise ValueError('CLIENT_PREVIEW_SPEED_MISMATCH')
            self.client.update(decision.action,stamp_ns)
            self.last_stamp = stamp_ns
        except Exception:
            self.stop()
            raise
        return True
