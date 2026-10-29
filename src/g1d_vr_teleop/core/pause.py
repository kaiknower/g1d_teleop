"""Public protocol-v3 pause RPC; no automatic retry or lifecycle startup."""

class SamplingError(RuntimeError):
    pass


def pause_once(host, port):
    import zmq
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SNDTIMEO, 1500)
    socket.setsockopt(zmq.RCVTIMEO, 1500)
    socket.connect(f"tcp://{host}:{port}")
    try:
        socket.send_json({"command": "pause_policy"})
        result = socket.recv_json()
        if not isinstance(result, dict) or result.get("success") is not True or result.get("protocol_version") != 3:
            raise SamplingError(f"pause_policy failed: {result}")
        return result
    finally:
        socket.close(0)
        context.term()
