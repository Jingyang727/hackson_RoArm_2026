"""Company-only OpenAI-compatible chat gateway (never log credentials)."""
from __future__ import annotations

import json
import os
import socket
import ssl
from urllib import request, error
from urllib.parse import urlparse

from .core import TrialPolicy


MAX_RESPONSE_BYTES = 2 * 1024 * 1024


def _extract_changes(content):
    """Read the FINAL assistant content, never reasoning_content.

    Accept strict JSON, markdown-fenced JSON, or text surrounding a JSON
    object. Do not use reasoning text as a fallback. Reject ambiguity.
    """
    if isinstance(content, list):
        content = "".join(part.get("text", "") for part in content
                          if isinstance(part, dict) and part.get("type") == "text")
    if not isinstance(content, str):
        raise ValueError("company model final content is not text")
    decoder = json.JSONDecoder()
    candidates = []
    for i, char in enumerate(content):
        if char != "{":
            continue
        try:
            obj, _ = decoder.raw_decode(content[i:])
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict) and set(obj) == {"changes"}:
            candidates.append(obj)
    if len(candidates) != 1:
        raise ValueError("company model final content must contain exactly one changes JSON object")
    return candidates[0]["changes"]


class CompanyModelGateway:
    """Use only an explicitly configured company HTTPS endpoint.

    No public fallback. Never persist/print tokens, raw model reasoning or
    response bodies. Test with injected urlopen mocks before hardware use.
    """
    def __init__(self, base_url=None, model=None, timeout_s=None):
        self.base_url = (base_url if base_url is not None else
                         os.getenv("EVOARM_COMPANY_LLM_BASE_URL", "")).rstrip("/")
        self.model = model if model is not None else os.getenv("EVOARM_COMPANY_LLM_MODEL", "")
        if timeout_s is None:
            timeout_s = float(os.getenv("EVOARM_COMPANY_LLM_TIMEOUT_S", "90"))
        if not isinstance(timeout_s, (int, float)) or not 5 <= timeout_s <= 300:
            raise ValueError("company gateway timeout must be between 5 and 300 seconds")
        self.timeout_s = float(timeout_s)

    def configured(self):
        return bool(self.base_url and self.model and os.getenv("EVOARM_COMPANY_LLM_TOKEN"))

    def complete_json(self, system, content):
        """Company Grok perception/review only; returns final JSON, never reasoning.

        This method has no robot tools and never opens a public fallback.
        Callers independently validate the returned schema before using it.
        """
        if not self.configured():
            raise RuntimeError("company Grok gateway not configured")
        if not self.model.lower().startswith("grok"):
            raise ValueError("pick/place agent requires the configured company Grok model")
        endpoint = urlparse(self.base_url)
        if (endpoint.scheme != "https" or not endpoint.hostname or endpoint.username
                or endpoint.password or endpoint.query or endpoint.fragment):
            raise ValueError("company gateway must be an HTTPS base URL")
        req = request.Request(
            self.base_url + "/chat/completions",
            data=json.dumps({"model": self.model, "temperature": 0.1,
                             "messages": [{"role": "system", "content": system},
                                          {"role": "user", "content": content}]}).encode(),
            headers={"Content-Type": "application/json", "Authorization":
                     "Bearer " + os.environ["EVOARM_COMPANY_LLM_TOKEN"]}, method="POST")
        try:
            with request.urlopen(req, timeout=self.timeout_s) as response:
                raw = response.read(MAX_RESPONSE_BYTES + 1)
        except error.HTTPError as exc:
            raise RuntimeError(f"company Grok HTTP {exc.code}") from exc
        except (error.URLError, TimeoutError, socket.timeout, ssl.SSLError) as exc:
            raise RuntimeError("company Grok transport failed; request not retried") from exc
        if len(raw) > MAX_RESPONSE_BYTES:
            raise ValueError("company Grok response too large")
        answer = json.loads(raw)["choices"][0]["message"]["content"]
        if not isinstance(answer, str):
            raise ValueError("company Grok final content must be JSON text")
        answer = answer.strip()
        if answer.startswith("```json\n") and answer.endswith("```"):
            answer = answer[8:-3].strip()
        result = json.loads(answer)
        if not isinstance(result, dict):
            raise ValueError("company Grok final JSON must be an object")
        return result

    def propose_next(self, policy: dict, evaluation: dict):
        if not self.configured():
            raise RuntimeError("company LLM gateway not configured; no external model fallback")
        endpoint = urlparse(self.base_url)
        if (endpoint.scheme != "https" or not endpoint.hostname or
                endpoint.username or endpoint.password or endpoint.fragment or endpoint.query):
            raise ValueError("gateway URL must be an approved HTTPS base URL")
        token = os.environ["EVOARM_COMPANY_LLM_TOKEN"]
        system = (
            "Analyze one SIMULATED robotic grasp. Return ONE final JSON object "
            '{"changes":{"offset_x_mm":-1.0}}. Allowed change keys: '
            "offset_x_mm, offset_y_mm, grasp_z_offset_mm, gripper_close_deg, approach_speed. "
            "Change only one evidence-supported parameter by a small amount. "
            "Do not include chain-of-thought in final content, motor commands or secrets."
        )
        payload = {
            "model": self.model,
            "temperature": 0.2,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": json.dumps({
                    "policy": policy,
                    "evaluation": evaluation,
                    "task": "Propose a single small parameter change for the next simulated experiment.",
                }, ensure_ascii=False)},
            ],
        }
        req = request.Request(
            self.base_url + "/chat/completions",
            data=json.dumps(payload).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json",
                     "Authorization": "Bearer " + token},
        )
        try:
            with request.urlopen(req, timeout=self.timeout_s) as resp:
                if resp.status != 200:
                    raise RuntimeError("company gateway returned HTTP " + str(resp.status))
                raw = resp.read(MAX_RESPONSE_BYTES + 1)
        except error.HTTPError as exc:
            raise RuntimeError("company gateway returned HTTP " + str(exc.code)) from exc
        except (error.URLError, TimeoutError, socket.timeout, ssl.SSLError) as exc:
            # Preserve cause for debugging (TLS cert, DNS, proxy, timeout).
            # Do not print request headers or the bearer token.
            why = getattr(exc, "reason", exc)
            raise RuntimeError(
                f"company gateway connection error ({type(why).__name__}): {why}"
            ) from exc
        if len(raw) > MAX_RESPONSE_BYTES:
            raise RuntimeError("company gateway response exceeds 2 MiB limit")
        try:
            result = json.loads(raw.decode("utf-8"))
            message = result["choices"][0]["message"]
            # reasoning_content is deliberately ignored; it may be long or
            # sensitive and must never be used as JSON policy output.
            content = message["content"]
        except (KeyError, IndexError, TypeError, UnicodeError, ValueError) as exc:
            raise ValueError("company gateway returned invalid chat completion JSON") from exc
        changes = _extract_changes(content)
        proposed = TrialPolicy(**policy).updated(changes)
        return {key: getattr(proposed, key) for key in changes}
