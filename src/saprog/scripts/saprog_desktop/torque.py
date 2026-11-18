"""Bounded asynchronous torque requests with explicit per-joint outcomes."""

import time


class TorqueBatch:
    """Issue a SetBool request per joint and retain every request until completion.

    Results are ``pending``, ``success``, ``failure``, ``timeout`` or ``not_sent``.
    Batch identity and per-request tokens make late callbacks harmless.
    """

    def __init__(self, clients, request_type, timeout=8.0, clock=time.monotonic):
        self.clients = dict(clients)
        self.request_type = request_type
        self.timeout, self.clock = timeout, clock
        self.outcomes = {}
        self._pending = {}
        self._requests = {}
        self._identity = object()
        self.closed = False

    def start(self, enabled, joints=None):
        if self.closed:
            raise RuntimeError("Torque adapter is closed")
        if self._pending:
            raise RuntimeError("A torque batch is already active")
        selected = tuple(self.clients if joints is None else joints)
        if not selected or len(set(selected)) != len(selected):
            raise ValueError("Torque batch needs unique joints")
        self.outcomes = {joint: "pending" for joint in selected}
        self._identity = object()
        for joint in selected:
            client = self.clients.get(joint)
            if client is None:
                self.outcomes[joint] = "not_sent: service unavailable"
                continue
            try:
                if not client.service_is_ready():
                    self.outcomes[joint] = "not_sent: service unavailable"
                    continue
                request = self.request_type.Request()
                request.data = bool(enabled)
                token = object()
                future = client.call_async(request)
                self._requests[joint] = request
                self._pending[joint] = (token, self.clock() + self.timeout, future)
                identity = self._identity
                future.add_done_callback(
                    lambda done, j=joint, t=token, i=identity: self._done(j, t, i, done)
                )
            except Exception as exc:
                self.outcomes[joint] = f"failure: {exc}"
        return self.outcomes

    def _done(self, joint, token, identity, future):
        item = self._pending.get(joint)
        if self.closed or identity is not self._identity or item is None or item[0] is not token:
            return
        self._pending.pop(joint)
        self._requests.pop(joint, None)
        try:
            response = future.result()
            self.outcomes[joint] = "success" if response.success else f"failure: {response.message}"
        except Exception as exc:
            self.outcomes[joint] = f"failure: {exc}"

    def poll(self):
        now = self.clock()
        for joint, (token, deadline, _future) in tuple(self._pending.items()):
            if now >= deadline:
                self._pending.pop(joint, None)
                self._requests.pop(joint, None)
                self.outcomes[joint] = "timeout: torque state unknown"
        return dict(self.outcomes)

    @property
    def done(self):
        self.poll()
        return not self._pending

    def close(self):
        self.closed = True
        self._identity = object()
        for joint, (_token, _deadline, future) in tuple(self._pending.items()):
            self.outcomes[joint] = "timeout: adapter closed; torque state unknown"
            try:
                future.cancel()
            except Exception:
                pass
        self._pending.clear()
        self._requests.clear()
