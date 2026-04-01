# Streaming event payload for `vlm/srv/ChatStream.srv`.
# Exactly one of these fields should be populated by the producer.
string text_delta
robonix_msg/ToolCall tool_call
string finish_reason
