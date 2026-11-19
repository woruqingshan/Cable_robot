import json
from types import SimpleNamespace
from vla_server.diagnostics import InferenceDiagnostics


def test_action_diagnostic_preserves_values_and_has_no_execution(tmp_path):
    from deployment.local.test_support import observation
    from vla_server.policies.mock import HoldPositionPolicy
    obs=observation()
    policy=HoldPositionPolicy();policy.start()
    result=policy.predict(obs)
    recorder=InferenceDiagnostics(tmp_path)
    recorder.success(obs,result,started_utc="fixture",duration_s=.1,policy=policy)
    recorder.error(request_id=obs.request_id,code="FIXTURE_ERROR")
    rows=[json.loads(line) for line in recorder.path.read_text().splitlines()]
    assert rows[0]["action"]["values"]==result.values
    assert rows[0]["finite"] is True
    assert rows[0]["execution_enabled"] is False
    assert "data_base64" not in recorder.path.read_text()
    assert rows[1]["code"]=="FIXTURE_ERROR"
    assert not hasattr(recorder,"execute")


def test_existing_http_app_records_response_and_validation_error(tmp_path):
    import pytest
    pytest.importorskip('fastapi')
    import asyncio
    from vla_server.app import create_app
    from vla_server.config import ServerConfig
    from vla_server.policies.mock import HoldPositionPolicy
    from deployment.local.test_support import observation
    app=create_app(policy=HoldPositionPolicy(),config=ServerConfig(diagnostic_directory=str(tmp_path)))
    async def request(body):
        sent=False;messages=[]
        async def receive():
            nonlocal sent
            if not sent:
                sent=True
                return {'type':'http.request','body':body,'more_body':False}
            await asyncio.sleep(10)
            return {'type':'http.disconnect'}
        async def send(message):messages.append(message)
        scope={'type':'http','asgi':{'version':'3.0'},'http_version':'1.1','method':'POST',
            'scheme':'http','path':'/v1/infer','raw_path':b'/v1/infer','query_string':b'',
            'headers':[(b'content-type',b'application/json')],'client':('127.0.0.1',123),
            'server':('127.0.0.1',8000)}
        await asyncio.wait_for(app(scope,receive,send),3)
        return next(m['status'] for m in messages if m['type']=='http.response.start')
    async def check():
        async with app.router.lifespan_context(app):
            assert await request(json.dumps(observation().model_dump()).encode())==200
            assert await request(b'{')==422
    asyncio.run(check())
    rows=[json.loads(line) for file in tmp_path.glob('*.jsonl') for line in file.read_text().splitlines()]
    assert [r['kind'] for r in rows]==['inference_result','inference_error']
    assert rows[0]['finite'] and rows[0]['execution_enabled'] is False
    assert rows[1]['code']=='INVALID_JSON'
