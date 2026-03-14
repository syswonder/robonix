# SPDX-License-Identifier: MulanPSL-2.0
"""gRPC echo server for latency benchmark."""

import grpc
from concurrent import futures

from proto import echo_pb2, echo_pb2_grpc


class EchoServicer(echo_pb2_grpc.EchoServicer):
    def Echo(self, request, context):
        return echo_pb2.EchoResponse(data=request.data)


def main():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    echo_pb2_grpc.add_EchoServicer_to_server(EchoServicer(), server)
    server.add_insecure_port("[::]:50052")
    server.start()
    print("gRPC echo server listening on [::]:50052", flush=True)
    server.wait_for_termination()


if __name__ == "__main__":
    main()
