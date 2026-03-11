# system/model — Model API (OpenAI-aligned)

Interfaces use `request_json` / `response_json` (std_msgs/String). Wire format is JSON; schemas follow OpenAI API where applicable.

## chat_completions

- **request_json**: `{ "model": str, "messages": [{"role": "user"|"assistant"|"system", "content": str}], "temperature": float?, "max_tokens": int?, "stream": bool? }`
- **response_json**: `{ "id": str, "choices": [{"message": {"role", "content"}, "finish_reason": str}], "usage": {"prompt_tokens", "completion_tokens", "total_tokens"} }`

## embeddings

- **request_json**: `{ "input": str | [str], "model": str, "dimensions": int? }`
- **response_json**: `{ "data": [{"embedding": [float], "index": int}], "usage": {"prompt_tokens", "total_tokens"} }`

## vision_completions (VLM)

- **request_json**: `{ "model": str, "messages": [{"role": str, "content": str | [{"type": "text", "text": str} | {"type": "image_url", "image_url": {"url": str}}]}], "max_tokens": int?, "temperature": float? }`
- **response_json**: same as chat_completions (`id`, `choices[]`, `usage`)
