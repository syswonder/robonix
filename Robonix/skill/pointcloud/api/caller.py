import grpc
import sys
import os
import traceback
import time
import random
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

import Robonix.skill.pointcloud.protos.skl_spatiallm_detect_pb2 as skl_spatiallm_detect_pb2
import Robonix.skill.pointcloud.protos.skl_spatiallm_detect_pb2_grpc as skl_spatiallm_detect_pb2_grpc
from Robonix.manager.eaios_decorators import eaios
from Robonix.uapi.graph.entity import Entity


@eaios.caller
def __rpc_skl_spatiallm_detect(
    # __rpc functions will always have two prepended args
    self_entity, # TODO: Entity should also be serialized and transferred across skill providers
    target_host: str,
    target_port: int,
    # the original input args
    ply_model: bytes,
) -> dict:
    """
    gRPC call function with automatic retry mechanism

    Args:
        target_host: Target host address
        target_port: Target port number
        ply_model: PLY model data

    Returns:
        gRPC response result

    Raises:
        Exception: Last exception after all retries exhausted
    """
    # Retry configuration
    max_retries = 10
    base_delay = 1.0
    max_delay = 60.0

    last_exception = None

    for attempt in range(max_retries + 1):  # +1 for initial attempt
        try:
            print(f"gRPC call attempt {attempt + 1}/{max_retries + 1}")

            with grpc.insecure_channel(f"{target_host}:{target_port}") as channel:
                stub = skl_spatiallm_detect_pb2_grpc.SpatialLMDetectStub(channel)
                request = skl_spatiallm_detect_pb2.SpatialLMDetectRequest(
                    ply_model=ply_model
                )
                response: dict = stub.Detect(request)
                print(f"gRPC call successful, attempt: {attempt + 1}")
                return response

        except Exception as e:
            last_exception = e
            print(f"gRPC call failed, attempt: {attempt + 1}, error: {e}")

            # If not the last attempt, wait and retry
            if attempt < max_retries:
                # Calculate delay time: exponential backoff + random jitter
                delay = min(base_delay * (2**attempt), max_delay)
                jitter = random.uniform(0, delay * 0.1)  # Add 10% random jitter
                total_delay = delay + jitter

                print(f"Waiting {total_delay:.2f} seconds before retry...")
                time.sleep(total_delay)
            else:
                print(f"gRPC call finally failed after {max_retries} retries")
                print(f"Detailed error info: {traceback.format_exc()}")

    # If all retries failed, raise the last exception
    raise last_exception


def test():
    print("testing __rpc_skl_spatiallm_detect")

    def get_demo_ply_model():
        with open(
            os.path.join(os.path.dirname(__file__), "..", "pointcloud.ply"), "rb"
        ) as f:
            return f.read()

    result = __rpc_skl_spatiallm_detect(
        target_host="localhost", target_port=50051, ply_model=get_demo_ply_model()
    )
    print(result)


if __name__ == "__main__":
    test()
