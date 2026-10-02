import pytest
from wheeltec_exploration.core import Decision
from wheeltec_exploration.navigation_bridge import DecisionBridge

class Fake:
    def __init__(self):
        self.calls=[]; self.fail=False
        self.forward_speed=.1; self.turn_speed=.25
    def enable(self): self.calls.append('enable')
    def stop(self): self.calls.append('stop')
    def poll(self): self.calls.append('poll')
    def update(self, action, stamp):
        if self.fail: raise RuntimeError('lost status')
        self.calls.append((action,stamp))


def test_unconfirmed_geometry_cannot_arm():
    client=Fake()
    with pytest.raises(ValueError): DecisionBridge(client).arm(1)
    assert not client.calls


def test_bridge_updates_new_decisions_without_replay_and_stops_on_fault():
    client=Fake()
    bridge=DecisionBridge(client,extrinsics_confirmed=True,calibration_verified=True)
    bridge.arm(1)
    assert bridge.submit(Decision('PLANNED','forward',.1,0),100,1)
    assert bridge.submit(Decision('PLANNED','left',0,.25),200,1)
    assert not bridge.submit(Decision('NO_NEW_OBSERVATION'),200,1)
    assert client.calls==['enable',('forward',100),('left',200),'poll']
    assert not bridge.submit(Decision('POSE_STALE'),300,1)
    assert client.calls[-1]=='stop'
    with pytest.raises(RuntimeError): bridge.submit(Decision('PLANNED','forward'),400,1)


@pytest.mark.parametrize('fault',['old','epoch','client'])
def test_bridge_failure_disarms_without_auto_enable(fault):
    client=Fake()
    bridge=DecisionBridge(client,extrinsics_confirmed=True,calibration_verified=True)
    bridge.arm(1)
    bridge.submit(Decision('PLANNED','forward',.1,0),100,1)
    if fault=='client': client.fail=True
    with pytest.raises((ValueError,RuntimeError)):
        bridge.submit(Decision('PLANNED','forward',.1,0),100 if fault=='old' else 200,2 if fault=='epoch' else 1)
    assert bridge.epoch is None and client.calls.count('enable')==1 and client.calls[-1]=='stop'


def test_bridge_rejects_speed_difference_instead_of_silently_changing_preview():
    client=Fake()
    bridge=DecisionBridge(client,extrinsics_confirmed=True,calibration_verified=True)
    bridge.arm(1)
    with pytest.raises(ValueError,match='CLIENT_PREVIEW_SPEED_MISMATCH'):
        bridge.submit(Decision('PLANNED','forward',.12,0),100,1)
    assert client.calls==['enable','stop'] and bridge.epoch is None
