"""Task-owned resource leases shared by native SIL and USB HIL."""
import time

import iceoryx2 as iox2
from .messages import ActivityMessage, TraceContext

DOWN_CAMERA = 1
LANDING = 2


class ActivityLease:
    def __init__(self, node, *, rate_hz=15, prefix="Firefly"):
        if rate_hz not in (15, 30):
            raise ValueError("down-camera rate must be 15 or 30 FPS")
        service = node.service_builder(iox2.ServiceName.new(prefix + "/Activity")).publish_subscribe(ActivityMessage).user_header(TraceContext).subscriber_max_buffer_size(8).max_publishers(8).open_or_create()
        self.publisher = service.publisher_builder().create()
        self.owner, self.sequence, self.mask = time.time_ns(), 0, 0
        self.rate_hz, self.next_send = rate_hz, 0.

    def set(self, mask, stamp):
        if mask & ~(DOWN_CAMERA | LANDING):
            raise ValueError("unknown activity resource")
        self.mask, self.next_send = mask, 0.
        self.pump(stamp)

    def pump(self, stamp):
        now = time.monotonic()
        if now < self.next_send:
            return
        self.sequence += 1
        message = ActivityMessage(timestamp=stamp, owner=self.owner, sequence=self.sequence,
                                  mask=self.mask, rate_hz=self.rate_hz, lease_ms=2000 if self.mask else 0)
        self.publisher.loan_uninit().write_payload(message).send()
        self.next_send = now + .2
