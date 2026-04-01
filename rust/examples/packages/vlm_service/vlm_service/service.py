#!/usr/bin/env python3
# pyright: reportArgumentType=false
"""VLM service: registers with robonix-server, serves chat completions over gRPC.

The agent discovers this service via the control plane, negotiates a channel,
then calls `VlmService.ChatStream` (server streaming) when available, or falls
back to unary `VlmService.Chat`. Both are defined in
`robonix-interfaces/robonix_proto/vlm.proto` (generated from ROS IDL under
`lib/vlm/`).

Required environment variables:
  VLM_API_KEY       API key for the VLM/LLM provider

Optional environment variables:
  VLM_BASE_URL      OpenAI-compatible API base URL (default: Qwen DashScope)
  VLM_MODEL         Model name (default: qwen3-vl-plus)
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
        pc = d / "proto_stubs"
        pg = d / "proto_gen"
        # Prefer proto_stubs/ (stubs built with the active venv's protobuf version)
        # over proto_gen/ which may have been generated with an incompatible newer version.
        if pc.is_dir() and (pc / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pc))
            if pg.is_dir():
                sys.path.append(str(pg))
            return
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
  export VLM_MODEL="model-name"                   # optional

Examples for common providers:

  # Qwen (Alibaba DashScope) — default
  export VLM_API_KEY="sk-xxx"
  export VLM_BASE_URL="https://dashscope.aliyuncs.com/compatible-mode/v1"
  export VLM_MODEL="qwen3-vl-plus"

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
                "streaming_rpc_method": "/robonix.vlm.VlmService/ChatStream",
                "stream_request_type": "ChatStream_Request",
                "stream_event_type": "ChatStreamEvent",
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
    model = os.environ.get("VLM_MODEL", "qwen3-vl-plus")

    def _openai_chat(messages, tools=None, tool_choice=None, max_tokens=0, stream=False):
        kwargs: dict = {"model": model, "messages": messages, "stream": stream}
        if tools:
            kwargs["tools"] = tools
        if tool_choice:
            kwargs["tool_choice"] = tool_choice
        if max_tokens:
            kwargs["max_tokens"] = max_tokens
        out = client.chat.completions.create(**kwargs)
        if stream:
            return out
        return out.choices[0]

    def handle_chat(request, context):
        """OpenAI-compatible chat completions proxy — forwards tools for function calling."""
        req_messages = _build_openai_messages(request)
        tools_list = _build_openai_tools(request)
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

    def _build_openai_messages(request):
        req_messages = []
        for m in request.messages:
            if m.image_base64:
                parts = []
                if m.content:
                    parts.append({"type": "text", "text": m.content})
                parts.append({
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{m.image_base64}"},
                })
                req_messages.append({"role": m.role, "content": parts})
            else:
                req_messages.append({"role": m.role, "content": m.content})
        return req_messages

    def _build_openai_tools(request):
        if not request.tools:
            return None
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
        return tools_list

    def handle_chat_stream(request, context):
        """Server-streaming chat: yields text deltas, then tool calls, then finish."""
        req_messages = _build_openai_messages(request)
        tools_list = _build_openai_tools(request)
        tc_mode = request.tool_choice if request.tool_choice else None

        stream_iter = _openai_chat(
            req_messages,
            tools=tools_list,
            tool_choice=tc_mode,
            max_tokens=request.max_tokens,
            stream=True,
        )

        # Accumulate tool call deltas: index → {id, name, arguments}
        tc_acc: dict[int, dict] = {}
        finish = "stop"

        for chunk in stream_iter:
            if not chunk.choices:
                continue
            delta = chunk.choices[0].delta
            fr = chunk.choices[0].finish_reason

            if delta and delta.content:
                yield vlm_pb2.ChatStreamEvent(text_delta=delta.content)

            if delta and delta.tool_calls:
                for tc_delta in delta.tool_calls:
                    idx = tc_delta.index
                    if idx not in tc_acc:
                        tc_acc[idx] = {"id": "", "name": "", "arguments": ""}
                    if tc_delta.id:
                        tc_acc[idx]["id"] = tc_delta.id
                    if tc_delta.function and tc_delta.function.name:
                        tc_acc[idx]["name"] += tc_delta.function.name
                    if tc_delta.function and tc_delta.function.arguments:
                        tc_acc[idx]["arguments"] += tc_delta.function.arguments

            if fr:
                finish = fr

        for idx in sorted(tc_acc.keys()):
            tc = tc_acc[idx]
            yield vlm_pb2.ChatStreamEvent(
                tool_call=robonix_msg_pb2.ToolCall(
                    id=tc["id"],
                    name=tc["name"],
                    arguments_json=tc["arguments"],
                )
            )

        yield vlm_pb2.ChatStreamEvent(finish_reason=finish)

    class VlmHandler(vlm_pb2_grpc.VlmServiceServicer):
        def Chat(self, request, context):
            return handle_chat(request, context)

        def ChatStream(self, request, context):
            return handle_chat_stream(request, context)

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
            abstract_interface_id="robonix/sys/model/vlm/chat",
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
