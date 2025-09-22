import grpc
import sys
import os
from concurrent import futures

sys.path.append(
    os.path.dirname(
        os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        )
    )
)
spatiallm_path = os.path.join(os.path.dirname(__file__), "spatiallm")
sys.path.insert(0, spatiallm_path)
protobuf_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "protos")
sys.path.insert(0, protobuf_path)

import robonix.skill.pointcloud.protos.skl_spatiallm_detect_pb2 as skl_spatiallm_detect_pb2
import robonix.skill.pointcloud.protos.skl_spatiallm_detect_pb2_grpc as skl_spatiallm_detect_pb2_grpc
import robonix.skill.pointcloud.api.api as api


class SklSpatialLMDetectService(skl_spatiallm_detect_pb2_grpc.SpatialLMDetectServicer):
    def Detect(self, request, context):
        ply_model = request.ply_model
        print(
            f"[skl_spatiallm_detect::server] detect method called, ply model size: {len(ply_model)} bytes"
        )

        # call the native skl_spatiallm_detect_local python function
        result = api.skl_spatiallm_detect_local(ply_model=ply_model)

        # convert native data to grpc data
        payload = {
            "txt": result["layout_string"],
            "rrd": b"123",
            "status": "ok",
        }

        return skl_spatiallm_detect_pb2.SpatialLMDetectReply(**payload)


DEFAULT_PORT = 50051


def serve(port: int = DEFAULT_PORT):
    try:
        print(f"starting skl_spatiallm_detect grpc server on port {port}")
        server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
        skl_spatiallm_detect_pb2_grpc.add_SpatialLMDetectServicer_to_server(
            SklSpatialLMDetectService(), server
        )
        server.add_insecure_port(f"[::]:{port}")
        server.start()
        print(f"skl_spatiallm_detect server started on port {port}")
        server.wait_for_termination()
    except Exception as e:
        print(f"error starting skl_spatiallm_detect grpc server: {e}")
        # quit with error code 1
        sys.exit(1)
    finally:
        server.stop(0)
        print(f"skl_spatiallm_detect server stopped on port {port}, exiting...")
        sys.exit(0)


if __name__ == "__main__":
    serve()
