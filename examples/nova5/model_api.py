# SPDX-License-Identifier: MulanPSL-2.0
"""Request one structured RGB decision from the same API used by Pilot."""
import base64
import json
import os
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def decide(images, schema_path, prompt, log_path):
    base = os.environ.get("VLM_BASE_URL", "").rstrip("/")
    key = os.environ.get("VLM_API_KEY", "")
    model = "gpt-6-astra"
    if not base or not key:
        raise RuntimeError("Set VLM_BASE_URL and VLM_API_KEY before running")
    content = [{"type": "text", "text": prompt}]
    for image in images:
        data = base64.b64encode(Path(image).read_bytes()).decode("ascii")
        content.append({"type": "image_url", "image_url": {
            "url": f"data:image/png;base64,{data}", "detail": "high"}})
    schema = json.loads(Path(schema_path).read_text(encoding="utf-8"))
    body = {
        "model": model,
        "reasoning_effort": os.environ.get("VLM_REASONING_EFFORT", "high"),
        "messages": [{"role": "user", "content": content}],
        "response_format": {"type": "json_schema", "json_schema": {
            "name": "nova5_decision", "strict": False, "schema": schema}},
    }
    for attempt in range(2):
        request = Request(base + "/chat/completions", data=json.dumps(body).encode(),
                          headers={"Authorization": "Bearer " + key,
                                   "Content-Type": "application/json",
                                   "User-Agent": "Robonix-Nova5/1.0",
                                   "Accept": "application/json"})
        try:
            with urlopen(request, timeout=180) as response:
                result = json.load(response)
            break
        except HTTPError as error:
            if attempt == 0 and error.code in (400, 422):
                body["response_format"] = {"type": "json_object"}
                continue
            raise RuntimeError(f"Model API returned HTTP {error.code}") from error
        except URLError as error:
            raise RuntimeError(f"Model API connection failed: {error.reason}") from error
    answer = result["choices"][0]["message"]["content"]
    if isinstance(answer, list):
        answer = "".join(part.get("text", "") for part in answer)
    decision = json.loads(answer)
    if not isinstance(decision, dict) or decision.get("decision") not in ("proceed", "abort"):
        raise ValueError("Model API returned an invalid decision")
    log_path.write_text(json.dumps({"model": model, "decision": decision,
                                    "usage": result.get("usage")}, ensure_ascii=False, indent=2))
    return decision
