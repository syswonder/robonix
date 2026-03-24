#!/usr/bin/env python3
# pyright: reportArgumentType=false
"""VLM service: registers with robonix-server, serves chat completions over gRPC.

The agent discovers this service via the control plane, negotiates a channel,
and calls /vlm.Vlm/Chat with an OpenAI-compatible JSON request.

Required environment variables:
  VLM_API_KEY       API key for the VLM/LLM provider

Optional environment variables:
  VLM_BASE_URL      OpenAI-compatible API base URL (default: Qwen DashScope)
  VLM_MODEL         Model name (default: qwen-vl-plus-latest)
  ROBONIX_SERVER    Control-plane address (default: localhost:50051)
  VLM_BIND_ADDR     If set, try this host first when binding the data-plane (default: try 127.0.0.1 then 0.0.0.0).
                    The service picks a free TCP port (DeclareInterface listen_port) so it does not collide with host services on 50100+.

Control plane (must match robonix-agent discovery):
  Registers under `robonix/sys/model/vlm` with interface `chat` (abstract id
  `robonix/sys/model/vlm/chat`). The agent uses `QueryNodes.abstract_interface_id`
  by default; override with `ROBONIX_VLM_ABSTRACT_INTERFACE_ID`, or empty abstract
  + `ROBONIX_VLM_NAMESPACE_PREFIX` for legacy split queries. See `rust/docs/NAMESPACE.md`.
"""
import json
import os
import sys
from concurrent import futures
from pathlib import Path


def _ensure_proto_gen() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "proto_gen"
        if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


_ensure_proto_gen()
import grpc
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc
import robonix_msg_pb2
import vlm_pb2
import vlm_pb2_grpc

_HELP = """\
ERROR: VLM_API_KEY environment variable is not set.

vlm_service.py requires an OpenAI-compatible VLM/LLM API endpoint.
Set the following environment variables before running:

  export VLM_API_KEY="your-api-key"
  export VLM_BASE_URL="https://api-endpoint/v1"   # optional
  export VLM_MODEL="model-name"                    # optional

Examples for common providers:

  # Qwen (Alibaba DashScope) — default
  export VLM_API_KEY="sk-xxx"
  export VLM_BASE_URL="https://dashscope.aliyuncs.com/compatible-mode/v1"
  export VLM_MODEL="qwen-vl-plus-latest"

  # OpenAI GPT
  export VLM_API_KEY="sk-xxx"
  export VLM_BASE_URL="https://api.openai.com/v1"
  export VLM_MODEL="gpt-4o"

  # Google Gemini (OpenAI-compatible)
  export VLM_API_KEY="AIza..."
  export VLM_BASE_URL="https://generativelanguage.googleapis.com/v1beta/openai"
  export VLM_MODEL="gemini-2.0-flash"

  # DeepSeek
  export VLM_API_KEY="sk-xxx"
  export VLM_BASE_URL="https://api.deepseek.com/v1"
  export VLM_MODEL="deepseek-chat"
"""


def _load_skill_md() -> str:
    p = Path(__file__).with_name("SKILL.md")
    if p.is_file():
        return p.read_text(encoding="utf-8")
    return "# VLM\nVision-language chat service.\n"


def _iface_meta() -> str:
    return json.dumps(
        {
            "transport": "grpc",
            "contract": {
                "idl_type": "protobuf",
                "proto_file": "robonix-interfaces/robonix_proto/vlm.proto",
                "service": "robonix.vlm.VlmService",
                "rpc_method": "/robonix.vlm.VlmService/Chat",
                "request_type": "Chat_Request",
                "response_type": "Chat_Response",
            },
        }
    )


def main() -> None:
    if "VLM_API_KEY" not in os.environ:
        print(_HELP, file=sys.stderr)
        sys.exit(1)

    channel = grpc.insecure_channel(os.environ.get("ROBONIX_SERVER", "localhost:50051"))
    stub = pb_grpc.RobonixRuntimeStub(channel)

    stub.RegisterNode(
        pb.RegisterNodeRequest(
            node_id="com.robonix.services.vlm",
            namespace="robonix/sys/model/vlm",
            kind="service",
            skill_md=_load_skill_md(),
        )
    )

    from openai import OpenAI

    client = OpenAI(
        api_key=os.environ["VLM_API_KEY"],
        base_url=os.environ.get(
            "VLM_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1"
        ),
    )
    model = os.environ.get("VLM_MODEL", "qwen-vl-plus-latest")

    def _openai_chat(messages, tools=None, tool_choice=None, max_tokens=0):
        kwargs: dict = {"model": model, "messages": messages}
        if tools:
            kwargs["tools"] = tools
        if tool_choice:
            kwargs["tool_choice"] = tool_choice
        if max_tokens:
            kwargs["max_tokens"] = max_tokens
        out = client.chat.completions.create(**kwargs)
        return out.choices[0]

    def handle_chat(request, context):
        """OpenAI-compatible chat completions proxy — forwards tools for function calling."""
        req_messages = [{"role": m.role, "content": m.content} for m in request.messages]

        tools_list = None
        if request.tools:
            tools_list = []
            for t in request.tools:
                schema = json.loads(t.input_schema_json) if t.input_schema_json else {}
                tools_list.append({
                    "type": "function",
                    "function": {
                        "name": t.name,
                        "description": t.description,
                        "parameters": schema,
                    },
                })

        tc_mode = request.tool_choice if request.tool_choice else None

        choice = _openai_chat(
            req_messages,
            tools=tools_list,
            tool_choice=tc_mode,
            max_tokens=request.max_tokens,
        )
        result = vlm_pb2.Chat_Response(content=choice.message.content or "")
        if choice.message.tool_calls:
            for tc in choice.message.tool_calls:
                result.tool_calls.add(
                    id=tc.id,
                    name=tc.function.name,
                    arguments_json=tc.function.arguments,
                )
        return result

    def handle_describe(request, context):
        """Simple image description (legacy interface)."""
        prompt = request.prompt or "Describe the image."
        parts = [{"type": "text", "text": prompt}]
        if request.image_base64:
            parts.append(
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{request.image_base64}"},
                }
            )
        choice = _openai_chat([{"role": "user", "content": parts}], max_tokens=512)
        return vlm_pb2.Describe_Response(text=choice.message.content or "")

    class VlmHandler(vlm_pb2_grpc.VlmServiceServicer):
        def Chat(self, request, context):
            return handle_chat(request, context)

        def Describe(self, request, context):
            return handle_describe(request, context)

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    vlm_pb2_grpc.add_VlmServiceServicer_to_server(VlmHandler(), server)

    bind_candidates: list[str] = []
    if os.environ.get("VLM_BIND_ADDR", "").strip():
        bind_candidates.append(os.environ["VLM_BIND_ADDR"].strip())
    bind_candidates.extend(["127.0.0.1", "0.0.0.0"])

    bound_port: int | None = None
    last_err: BaseException | None = None
    for bind in bind_candidates:
        try:
            p = server.add_insecure_port(f"{bind}:0")
            if p > 0:
                bound_port = p
                break
        except RuntimeError as e:
            last_err = e

    if bound_port is None:
        raise RuntimeError(
            f"Failed to bind VLM gRPC data plane (tried {bind_candidates}): {last_err}"
        )

    resp = stub.DeclareInterface(
        pb.DeclareInterfaceRequest(
            node_id="com.robonix.services.vlm",
            name="chat",
            supported_transports=["grpc"],
            metadata_json=_iface_meta(),
            listen_port=bound_port,
        )
    )
    data_endpoint = resp.allocated_endpoint
    print(f"[vlm-service] using model={model}, data-plane at {data_endpoint}")

    _, ep_port_str = data_endpoint.rsplit(":", 1)
    if int(ep_port_str) != bound_port:
        print(
            f"[vlm-service] warning: control plane endpoint port {ep_port_str} != bound port {bound_port}",
            file=sys.stderr,
        )

    server.start()
    print(f"[vlm-service] gRPC listening on port {bound_port}")
    server.wait_for_termination()


if __name__ == "__main__":
    main()
