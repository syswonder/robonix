#!/usr/bin/env python3
# pyright: reportArgumentType=false
"""VLM service: registers with robonix-atlas, serves chat completions over gRPC.

The agent discovers this service via the control plane, negotiates a channel,
then calls contract `SysModelVlmChat.Stream` (server streaming). Wire types live in
`vlm.proto`; the gRPC service is only in `robonix_contracts.proto` (see
`rust/contracts` + codegen).

Required environment variables:
  VLM_API_KEY       API key for the VLM/LLM provider

Optional environment variables:
  VLM_BASE_URL      OpenAI-compatible API base URL (default: Qwen DashScope)
  VLM_MODEL         Model name (default: qwen3-vl-plus)
  VLM_MESSAGE_FORMAT  Request message format: "openai" or "gemini" (default: openai)
  ROBONIX_ATLAS    Control-plane address (default: localhost:50051)
  VLM_BIND_ADDR     If set, try this host first when binding the data-plane (default: try 127.0.0.1 then 0.0.0.0).
                    The service picks a free TCP port (DeclareInterface listen_port) so it does not collide with host services on 50100+.

Control plane (must match robonix-pilot discovery):
  Registers under `robonix/sys/model/vlm` with interface `chat` (contract id
  `robonix/sys/model/vlm/chat`). The agent uses `QueryNodes.contract_id`
  by default; override with `ROBONIX_VLM_CONTRACT_ID`, or empty contract
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
import robonix_contracts_pb2_grpc
import robonix_msg_pb2
import vlm_pb2

_HELP = """\
ERROR: VLM_API_KEY environment variable is not set.

vlm_service.py requires an OpenAI-compatible VLM/LLM API endpoint.
Set the following environment variables before running:

  export VLM_API_KEY="your-api-key"
  export VLM_BASE_URL="https://api-endpoint/v1"   # optional
  export VLM_MODEL="model-name"                   # optional
  export VLM_MESSAGE_FORMAT="openai"              # optional

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
                "proto_file": "robonix-interfaces/robonix_proto/robonix_contracts.proto",
                "service": "robonix.contracts.SysModelVlmChat",
                "streaming_rpc_method": "/robonix.contracts.SysModelVlmChat/Stream",
                "stream_request_type": "robonix.vlm.ChatStream_Request",
                "stream_event_type": "robonix.vlm.ChatStreamEvent",
            },
        }
    )


def main() -> None:
    if "VLM_API_KEY" not in os.environ:
        print(_HELP, file=sys.stderr)
        sys.exit(1)

    channel = grpc.insecure_channel(os.environ.get("ROBONIX_ATLAS", "localhost:50051"))
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
    message_format = os.environ.get("VLM_MESSAGE_FORMAT", "openai").strip().lower()

    def _openai_chat(messages, tools=None, tool_choice=None, max_tokens=0, stream=False):
        kwargs: dict = {"model": model, "messages": messages, "stream": stream}
        if tools:
            kwargs["tools"] = tools
        if tool_choice:
            kwargs["tool_choice"] = tool_choice
        if max_tokens:
            kwargs["max_tokens"] = max_tokens
        try:
            out = client.chat.completions.create(**kwargs)
        except Exception as e:
            print(f"[vlm-debug] OpenAI API error: {e}", flush=True)
            # Dump the messages that caused the error (without full base64)
            for i, m in enumerate(messages):
                c = m.get("content", "")
                if isinstance(c, list):
                    summary = []
                    for p in c:
                        if isinstance(p, dict) and p.get("type") == "image_url":
                            url = p.get("image_url", {}).get("url", "")
                            summary.append(f"image_url(len={len(url)}, first60={url[:60]!r})")
                        else:
                            summary.append(str(p)[:100])
                    print(f"[vlm-debug] err msg[{i}]: role={m.get('role')}, parts={summary}", flush=True)
                else:
                    print(f"[vlm-debug] err msg[{i}]: role={m.get('role')}, content_len={len(str(c))}", flush=True)
            raise
        if stream:
            return out
        return out.choices[0]

    def _part_to_openai(part):
        kind = getattr(part, "kind", "")
        if kind == "text":
            text = getattr(part, "text", "")
            return {"type": "text", "text": text} if text else None
        if kind in {"image_url", "inline_data"}:
            uri = getattr(part, "uri", "")
            data_b64 = getattr(part, "data_base64", "")
            mime = getattr(part, "mime_type", "") or "image/jpeg"
            if uri:
                return {"type": "image_url", "image_url": {"url": uri}}
            if data_b64:
                return {
                    "type": "image_url",
                    "image_url": {"url": f"data:{mime};base64,{data_b64}"},
                }
        return None

    def _build_openai_messages(request):
        req_messages = []

        for m in request.messages:
            openai_parts = []
            for part in getattr(m, "parts", []) or []:
                mapped = _part_to_openai(part)
                if mapped:
                    openai_parts.append(mapped)

            if not openai_parts and m.image_base64:
                print(
                    f"[vlm-debug] FALLBACK to image_base64! role={m.role}, "
                    f"parts_count={len(getattr(m, 'parts', []) or [])}, "
                    f"image_base64_len={len(m.image_base64)}, "
                    f"image_base64_first40={m.image_base64[:40]!r}",
                    flush=True,
                )
                # Dump each part for diagnosis
                for j, part in enumerate(getattr(m, "parts", []) or []):
                    print(
                        f"[vlm-debug]   part[{j}]: kind={getattr(part, 'kind', '?')}, "
                        f"text_len={len(getattr(part, 'text', '') or '')}, "
                        f"data_base64_len={len(getattr(part, 'data_base64', '') or '')}, "
                        f"uri={getattr(part, 'uri', '')!r}",
                        flush=True,
                    )
                if m.content:
                    openai_parts.append({"type": "text", "text": m.content})
                openai_parts.append({
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{m.image_base64}"},
                })

            if not openai_parts and m.content:
                content = m.content
            else:
                content = openai_parts

            msg = {"role": m.role, "content": content}

            tool_calls = []
            for tc in getattr(m, "tool_calls", []) or []:
                tool_calls.append({
                    "id": tc.id,
                    "type": "function",
                    "function": {
                        "name": tc.name,
                        "arguments": tc.arguments_json,
                    },
                })
            if tool_calls:
                msg["tool_calls"] = tool_calls

            tool_call_id = getattr(m, "tool_call_id", "")
            if tool_call_id:
                msg["tool_call_id"] = tool_call_id

            name = getattr(m, "name", "")
            if name:
                msg["name"] = name

            # Legacy fallback: old wire format cannot express valid OpenAI tool linkage.
            if m.role == "tool" and not tool_call_id:
                text = m.content or "[tool returned output]"
                if openai_parts:
                    text_parts = [{"type": "text", "text": f"Tool result:\n{text}"}]
                    text_parts.extend(openai_parts)
                    req_messages.append({"role": "user", "content": text_parts})
                else:
                    req_messages.append({"role": "user", "content": f"Tool result:\n{text}"})
                continue

            req_messages.append(msg)
        return req_messages

    def _build_gemini_contents(request):
        contents = []
        for m in request.messages:
            role = "model" if m.role == "assistant" else "user"
            parts = []

            for part in getattr(m, "parts", []) or []:
                kind = getattr(part, "kind", "")
                if kind == "text" and getattr(part, "text", ""):
                    parts.append({"text": part.text})
                elif kind in {"image_url", "inline_data"} and getattr(part, "data_base64", ""):
                    parts.append({
                        "inline_data": {
                            "mime_type": getattr(part, "mime_type", "") or "image/jpeg",
                            "data": part.data_base64,
                        }
                    })
                elif kind == "function_call":
                    parts.append({
                        "function_call": {
                            "name": getattr(part, "tool_name", ""),
                            "args": json.loads(getattr(part, "tool_arguments_json", "") or "{}"),
                        }
                    })
                elif kind == "function_response":
                    parts.append({
                        "function_response": {
                            "name": getattr(part, "tool_call_id", ""),
                            "response": json.loads(getattr(part, "tool_result_json", "") or "{}"),
                        }
                    })

            if not parts and getattr(m, "content", ""):
                parts.append({"text": m.content})
            if getattr(m, "image_base64", "") and not any("inline_data" in p for p in parts):
                parts.append({
                    "inline_data": {
                        "mime_type": "image/jpeg",
                        "data": m.image_base64,
                    }
                })
            if parts:
                contents.append({"role": role, "parts": parts})
        return contents

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
        if message_format == "gemini":
            raise NotImplementedError(
                "VLM_MESSAGE_FORMAT=gemini is defined in the wire model, but vlm_service "
                "stream transport is still wired to OpenAI-compatible chat.completions. "
                "Use openai for now or implement a Gemini transport adapter."
            )

        req_messages = _build_openai_messages(request)
        tools_list = _build_openai_tools(request)
        tc_mode = request.tool_choice if request.tool_choice else None
        print(f"[vlm-debug] req_messages: {req_messages}")

        # Debug: log messages being sent to OpenAI API
        for i, msg in enumerate(req_messages):
            role = msg.get("role", "?")
            content = msg.get("content", "")
            has_tool_calls = bool(msg.get("tool_calls"))
            tool_call_id = msg.get("tool_call_id", "")
            if isinstance(content, str):
                preview = content[:120] + ("..." if len(content) > 120 else "")
                has_image = False
            elif isinstance(content, list):
                has_image = any(
                    p.get("type") == "image_url" for p in content if isinstance(p, dict)
                )
                text_parts = [
                    p.get("text", "")[:80] for p in content
                    if isinstance(p, dict) and p.get("type") == "text"
                ]
                preview = "; ".join(text_parts)
                if has_image:
                    for p in content:
                        if isinstance(p, dict) and p.get("type") == "image_url":
                            url = p.get("image_url", {}).get("url", "")
                            print(
                                f"[vlm-debug] msg[{i}] image_url len={len(url)}, "
                                f"first_60={url[:60]!r}",
                                flush=True,
                            )
            else:
                preview = str(content)[:120]
                has_image = False
            print(
                f"[vlm-debug] msg[{i}]: role={role}, has_image={has_image}, "
                f"tool_calls={has_tool_calls}, tool_call_id={tool_call_id!r}, "
                f"content_preview={preview!r}",
                flush=True,
            )

        print(
            f"[vlm-debug] total {len(req_messages)} messages, "
            f"{len(tools_list) if tools_list else 0} tools, tool_choice={tc_mode}",
            flush=True,
        )

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

    class VlmHandler(robonix_contracts_pb2_grpc.SysModelVlmChatServicer):
        def Stream(self, request, context):
            return handle_chat_stream(request, context)

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    robonix_contracts_pb2_grpc.add_SysModelVlmChatServicer_to_server(
        VlmHandler(), server
    )

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
            contract_id="robonix/sys/model/vlm/chat",
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
