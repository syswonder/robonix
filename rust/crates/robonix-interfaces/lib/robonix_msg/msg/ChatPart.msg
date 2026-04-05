# Typed chat content part compatible with OpenAI/Gemini style multimodal messages.
#
# kind:
#   - "text"
#   - "image_url"
#   - "inline_data"
#   - "function_call"
#   - "function_response"
string kind

# text
string text

# image_url / inline_data
string mime_type
string data_base64
string uri

# function_call
string tool_name
string tool_arguments_json

# function_response
string tool_call_id
string tool_result_json
