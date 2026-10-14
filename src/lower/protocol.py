"""Bounded request/response transport; the robot polls its guards during I/O."""
import json
import struct
import time

MAGIC = b'SVSHAD01'  # Observation framing shared with the verified shadow bridge.


def send_exact(sock, data, guard=lambda: None, deadline=None):
    """Send each byte once, polling guards while TCP applies backpressure."""
    import socket
    if deadline is None:
        deadline = time.monotonic() + 5
    remaining_data = memoryview(data)
    original_timeout = sock.gettimeout()
    try:
        while remaining_data:
            guard()
            remaining_seconds = deadline - time.monotonic()
            if remaining_seconds <= 0:
                raise TimeoutError('Packet send deadline exceeded')
            sock.settimeout(min(.05, remaining_seconds))
            try:
                sent = sock.send(remaining_data)
            except socket.timeout:
                continue
            if sent == 0:
                raise ConnectionError('Inference peer disconnected during send')
            remaining_data = remaining_data[sent:]
    finally:
        sock.settimeout(original_timeout)


def recv_exact(sock, size, guard=lambda: None, deadline=None):
    parts = []
    while size:
        guard()
        if deadline is not None and time.monotonic() >= deadline:
            raise TimeoutError('Inference response deadline exceeded')
        import socket
        try:
            part = sock.recv(size)
        except socket.timeout:
            continue
        if not part:
            raise ConnectionError('Inference peer disconnected')
        parts.append(part)
        size -= len(part)
    return b''.join(parts)


def read_observation(sock):
    # The robot can execute a long retimed trajectory or wait for onsite
    # confirmation between requests. Bound packet reception after its first byte.
    first = recv_exact(sock, 1)
    deadline=time.monotonic()+10
    if first + recv_exact(sock, 7, deadline=deadline) != MAGIC:
        raise ValueError('Invalid observation magic')
    def part(limit):
        size = struct.unpack('!I', recv_exact(sock, 4, deadline=deadline))[0]
        if not 0 < size <= limit:
            raise ValueError('Invalid packet length')
        return recv_exact(sock, size, deadline=deadline)
    return json.loads(part(1_000_000)), [part(5_000_000) for _ in range(3)]


def send_response(sock, value):
    data = json.dumps(value, allow_nan=False).encode('utf-8')
    if len(data) > 100_000:
        raise ValueError('Response too large')
    send_exact(sock, struct.pack('!I', len(data)) + data)


def read_response(sock, guard, deadline):
    size = struct.unpack('!I', recv_exact(sock, 4, guard, deadline))[0]
    if not 0 < size <= 100_000:
        raise ValueError('Invalid inference response length')
    return json.loads(recv_exact(sock, size, guard, deadline))
