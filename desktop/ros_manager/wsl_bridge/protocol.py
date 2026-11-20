"""Line-delimited JSON events and a non-blocking stdin command reader."""

import json
import queue
import sys
import threading


def emit(event, **values):
    print(json.dumps({"event": event, **values}, separators=(",", ":")), flush=True)


class CommandReader:
    """Read stdin in the background; signal EOF and cancel outstanding builds."""

    def __init__(self, shutdown_event):
        self.queue = queue.Queue()
        self.shutdown_event = shutdown_event
        self.thread = threading.Thread(
            target=self._read, name="saprog-command-reader", daemon=True,
        )

    def start(self):
        self.thread.start()

    def _read(self):
        try:
            for line in sys.stdin:
                self.queue.put(line)
        finally:
            self.shutdown_event.set()
            self.queue.put(None)
