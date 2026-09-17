#!/usr/bin/env python3
"""Read FSDS reference geometry for offline calibration only; no control RPCs."""
import argparse
import json
import socket
from pathlib import Path

import msgpack


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {}
    with socket.create_connection((args.host, 41451), timeout=5) as conn:
        unpacker = msgpack.Unpacker(raw=False)
        for i, (method, params) in enumerate([
            ('getRefereeState', []), ('simGetCameraInfo', ['cam_left', 'FSCar']),
        ]):
            conn.sendall(msgpack.packb([0, i, method, params], use_bin_type=True))
            response = None
            while response is None:
                block = conn.recv(65536)
                if not block:
                    raise ConnectionError('FSDS closed the RPC connection')
                unpacker.feed(block)
                response = next(unpacker, None)
            if response[0] != 1 or response[1] != i or response[2] is not None:
                raise RuntimeError(f'Failed reference RPC: {response[:3]}')
            result[method] = response[3]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(f'Reference geometry saved to {args.output}')


if __name__ == '__main__':
    main()
