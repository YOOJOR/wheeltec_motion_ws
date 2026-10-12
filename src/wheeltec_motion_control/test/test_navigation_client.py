"""Bounded RPC lifecycle; no ROS graph or physical motion is created."""
from types import SimpleNamespace

import pytest
from wheeltec_motion_control import navigation_client as interface


def test_discovery_at_deadline_still_allows_response_without_resending(monkeypatch):
    clock=[0.]
    monkeypatch.setattr(interface.time,'monotonic',lambda:clock[0])
    response=SimpleNamespace(success=True,message='stopped')
    future=SimpleNamespace(done=lambda:clock[0]>=2.03,result=lambda:response)
    sent=[]
    def discover(timeout_sec):
        clock[0]+=timeout_sec+.004
        return True
    def spin(node,request_future,timeout_sec):
        clock[0]+=min(timeout_sec,.03)
    monkeypatch.setattr(interface.rclpy,'spin_until_future_complete',spin)
    client=SimpleNamespace(wait_for_service=discover,
        call_async=lambda request:(sent.append(request) or future),
        remove_pending_request=lambda f:None)
    request=object()
    assert interface.NavigationClient.call(object(),client,request,2.) is response
    assert sent==[request]


def test_missing_service_sends_nothing_and_response_timeout_is_not_retried(monkeypatch):
    sent=[];removed=[]
    future=SimpleNamespace(done=lambda:False)
    client=SimpleNamespace(wait_for_service=lambda timeout_sec:False,
        call_async=lambda request:(sent.append(request) or future),
        remove_pending_request=removed.append)
    with pytest.raises(interface.ServiceUnavailable):
        interface.NavigationClient.call(object(),client,object(),2.)
    assert not sent
    client.wait_for_service=lambda timeout_sec:True
    waits=[]
    monkeypatch.setattr(interface.rclpy,'spin_until_future_complete',
                        lambda node,f,timeout_sec:waits.append(timeout_sec))
    with pytest.raises(RuntimeError,match='state is uncertain'):
        interface.NavigationClient.call(object(),client,object(),2.)
    assert len(sent)==1 and waits==[2.] and removed==[future]
