"""Stable error envelopes shared across transport and local validation."""
from typing import Literal
from pydantic import BaseModel, ConfigDict


class VLAError(ValueError):
    def __init__(self, code: str, message: str, *, request_id: str | None = None):
        super().__init__(message)
        self.code, self.request_id = code, request_id

    def report(self) -> dict:
        return {"schema_version": 1, "code": self.code,
                "message": str(self), "request_id": self.request_id}


class ErrorResponse(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    schema_version: Literal[1] = 1
    code: str
    message: str
    request_id: str | None = None
