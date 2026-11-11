"""Company-only LLM gateway adapter. No third-party fallback or secret logging."""
from __future__ import annotations

import json
import os
from urllib import request, error
from urllib.parse import urlparse

from .core import TrialPolicy


class CompanyModelGateway:
    """OpenAI-compatible internal gateway, opt-in and closed by default.

    Use only your organization's approved internal endpoint. Token is read
    from the process environment on each request and NEVER persisted.
    """
    def __init__(self, base_url=None, model=None, timeout_s=30):
        self.base_url = (base_url if base_url is not None else
                         os.getenv("EVOARM_COMPANY_LLM_BASE_URL", "")).rstrip("/")
        self.model = model if model is not None else os.getenv("EVOARM_COMPANY_LLM_MODEL", "")
        self.timeout_s = timeout_s

    def configured(self):
        return bool(self.base_url and self.model and os.getenv("EVOARM_COMPANY_LLM_TOKEN"))

    def propose_next(self, policy: dict, evaluation: dict):
        if not self.configured():
            raise RuntimeError("company LLM gateway not configured; no external model fallback")
        endpoint = urlparse(self.base_url)
        if (endpoint.scheme != "https" or not endpoint.hostname or
                endpoint.username or endpoint.password or endpoint.fragment or
                endpoint.query):
            raise ValueError("gateway URL must be an approved HTTPS base URL")
        token = os.environ["EVOARM_COMPANY_LLM_TOKEN"]
        system = (
            "You analyze a SIMULATED robot grasp experiment. Respond ONLY JSON with a "
            "changes object using allowed keys offset_x_mm, offset_y_mm, "
            "grasp_z_offset_mm, gripper_close_deg, approach_speed. "
            "Make small evidence-based changes; do not return motor commands, "
            "URLs, secrets or other fields. No markdown."
        )
        payload = {
            "model": self.model,
            "temperature": 0.2,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": json.dumps({
                    "policy": policy,
                    "evaluation": evaluation,
                    "task": "Propose one small parameter adjustment for the next experiment.",
                }, ensure_ascii=False)},
            ],
        }
        encoded = json.dumps(payload).encode("utf-8")
        req = request.Request(
            self.base_url + "/chat/completions",
            data=encoded, method="POST",
            headers={"Content-Type": "application/json",
                     "Authorization": "Bearer " + token},
        )
        try:
            with request.urlopen(req, timeout=self.timeout_s) as resp:
                if resp.status != 200:
                    raise RuntimeError("company gateway returned HTTP " + str(resp.status))
                if int(resp.headers.get("Content-Length", "0")) > 128000:
                    raise RuntimeError("company gateway response too large")
                raw = resp.read(128001)
        except error.HTTPError as exc:
            # No response body: upstream may include sensitive request metadata.
            raise RuntimeError("company gateway returned HTTP " + str(exc.code)) from None
        except error.URLError:
            raise RuntimeError("company gateway unreachable") from None
        if len(raw) > 128000:
            raise RuntimeError("company gateway response too large")
        result = json.loads(raw.decode("utf-8"))
        message = result["choices"][0]["message"]["content"]
        obj = json.loads(message)
        if not isinstance(obj, dict) or set(obj) != {"changes"}:
            raise ValueError("company model must return only a changes object")
        proposed = TrialPolicy(**policy).updated(obj["changes"])
        return {key: getattr(proposed, key) for key in obj["changes"]}
