import json
import pytest
from deployment.local.probe_finite_readback import main, validate_previous, validate_response


def valid():
    return dict(complete=True,stage='D2',source_type='unknown',canonical_observation_eligible=False,
        batches=[dict(complete=True,runtime_source_sha='a'*40,
            channels=[dict(servo_id=i,error=None,outer_frame_validated=True) for i in [2,3,4,5,10]])])


def test_d3_requires_successful_same_source_d2(tmp_path):
    path=tmp_path/'d2.json'
    with pytest.raises(ValueError):validate_previous('D3','a'*40,None)
    path.write_text(json.dumps(dict(expected_source_sha='a'*40,client_status='RESPONSE_RECEIVED',
        response_success=True,diagnostic=valid())))
    validate_previous('D3','a'*40,path)
    with pytest.raises(ValueError):validate_previous('D3','b'*40,path)
    data=json.loads(path.read_text());data['diagnostic']['complete']=False
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):validate_previous('D3','a'*40,path)


def test_response_checks_runtime_identity_and_completeness():
    assert validate_response(valid(),'a'*40)
    with pytest.raises(ValueError):validate_response(valid(),'b'*40)
    data=valid();data['batches'][0]['channels']=[]
    with pytest.raises(ValueError):validate_response(data,'a'*40)
    data=valid();data['canonical_observation_eligible']=True
    with pytest.raises(ValueError):validate_response(data,'a'*40)
    assert not validate_response({'complete':False},'a'*40)


def test_existing_output_refused_before_ros_or_rpc(tmp_path):
    path=tmp_path/'existing';path.write_text('keep')
    with pytest.raises(FileExistsError):
        main(['--stage','D2','--expected-source-sha','a'*40,'--output',str(path)])
    assert path.read_text()=='keep'
