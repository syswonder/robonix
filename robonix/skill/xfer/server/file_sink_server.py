import os, sys, grpc
from concurrent import futures

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # xfer/
sys.path.insert(0, os.path.join(BASE, "protos"))

import file_sink_pb2 as pb2
import file_sink_pb2_grpc as pb2_grpc

SAVE_DIR = os.environ.get("ASR_INBOX", os.path.expanduser("~/asr_inbox"))
PORT = int(os.environ.get("FILESINK_PORT", "50111"))
MAX_MSG = 50 * 1024 * 1024

class FileSink(pb2_grpc.FileSinkServicer):
    def Upload(self, request, context):
        try:
            os.makedirs(SAVE_DIR, exist_ok=True)
            fname = request.filename or "asr_result.txt"
            path = os.path.join(SAVE_DIR, os.path.basename(fname))
            with open(path, "wb") as f:
                f.write(request.data)
            print(f"[FileSink] saved: {path} ({len(request.data)} bytes)")
            return pb2.Ack(status="ok")
        except Exception as e:
            return pb2.Ack(status=f"error: {e}")

def main():
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=4),
        options=[("grpc.max_send_message_length", MAX_MSG),
                 ("grpc.max_receive_message_length", MAX_MSG)]
    )
    pb2_grpc.add_FileSinkServicer_to_server(FileSink(), server)
    server.add_insecure_port(f"[::]:{PORT}")
    server.start()
    print(f"[FileSink] listening on {PORT}, saving to {SAVE_DIR}")
    server.wait_for_termination()

if __name__ == "__main__":
    main()

