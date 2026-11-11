"""Fail-closed replay diagnostics, distinct from ordinary saved task failures."""


class ReplayError(ValueError):
    def __init__(self, code: str, detail: str):
        self.code = code
        super().__init__(detail)

    def report(self) -> dict:
        return {'status': 'ERROR', 'code': self.code, 'detail': str(self)}


class ReplayBlocked(ReplayError):
    def report(self) -> dict:
        return {**super().report(), 'status': 'BLOCKED'}
