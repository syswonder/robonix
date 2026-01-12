import os, sys, grpc

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # xfer/
sys.path.insert(0, os.path.join(BASE, "protos"))

import file_sink_pb2 as pb2
import file_sink_pb2_grpc as pb2_grpc

MAX_MSG = 50 * 1024 * 1024

def send_file(host: str, port: int, filepath: str, rename: str = None):
    with open(filepath, "rb") as f:
        data = f.read()
    filename = rename or os.path.basename(filepath)
    options = [("grpc.max_send_message_length", MAX_MSG),
               ("grpc.max_receive_message_length", MAX_MSG)]
    with grpc.insecure_channel(f"{host}:{port}", options=options) as ch:
        stub = pb2_grpc.FileSinkStub(ch)
        resp = stub.Upload(pb2.FileUpload(filename=filename, data=data))
        if not resp.status.startswith("ok"):
            raise RuntimeError(resp.status)
        print(f"[Sender] ok -> {host}:{port}/{filename} ({len(data)} bytes)")

if __name__ == "__main__":
    # 用法：python send_file_client.py ORIN_B_IP 50111 /opt/riva/test/asr_*.txt [optional_rename]
    host, port, fpath = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    rename = sys.argv[4] if len(sys.argv) > 4 else None
    send_file(host, port, fpath, rename)

