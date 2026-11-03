"""One owned, persistent JSONL backend process with bounded I/O and cleanup."""
import json
import os
from pathlib import Path
import selectors
import subprocess
import tempfile
import threading
import time
import uuid

from .protocol import (CollisionError, VERSION, strict_json, make_init,
                       check_response, check_ready, check_query)


class CollisionClient:
    """Sequential requests; owns only its direct child, never name-matched processes."""
    def __init__(self, executable, *, timeout=15):
        if isinstance(timeout,bool) or not isinstance(timeout,(int,float)) or not 0<timeout<3600:
            raise CollisionError('timeout must be finite positive seconds below 3600')
        self.executable=Path(executable).resolve(strict=True)
        self.timeout=timeout;self.identity=None;self.ready=None;self.process=None;self.cleanup=None
        self._stderr=tempfile.TemporaryFile();self._buffer=b'';self._lock=threading.Lock();self._broken=False
        try:
            self.process=subprocess.Popen([str(self.executable)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                                          stderr=self._stderr,bufsize=0)
            os.set_blocking(self.process.stdin.fileno(),False)
            os.set_blocking(self.process.stdout.fileno(),False)
        except BaseException:
            self._stderr.close();raise

    def _exchange(self, payload):
        if self._broken or self.cleanup is not None:raise CollisionError('backend connection unavailable')
        with self._lock:
            request_id=uuid.uuid4().hex
            request={'protocol_version':VERSION,'id':request_id,**payload}
            data=(json.dumps(request,allow_nan=False,separators=(',',':'))+'\n').encode()
            if len(data)>16*1024*1024:raise CollisionError('request too large')
            end=time.monotonic()+self.timeout
            try:
                with selectors.DefaultSelector() as selector:
                    selector.register(self.process.stdin,selectors.EVENT_WRITE)
                    while data:
                        if not selector.select(max(0,end-time.monotonic())):raise CollisionError('backend write timeout')
                        n=os.write(self.process.stdin.fileno(),data);data=data[n:]
                        if time.monotonic()>=end:raise CollisionError('backend write timeout')
                    selector.unregister(self.process.stdin);selector.register(self.process.stdout,selectors.EVENT_READ)
                    while b'\n' not in self._buffer:
                        if not selector.select(max(0,end-time.monotonic())):raise CollisionError('backend response timeout')
                        chunk=os.read(self.process.stdout.fileno(),65536)
                        if not chunk:raise CollisionError('backend exited without complete response')
                        self._buffer+=chunk
                        if len(self._buffer)>32*1024*1024:raise CollisionError('response too large')
                    line,self._buffer=self._buffer.split(b'\n',1)
                reply=strict_json(line.decode('utf-8'))
                return check_response(reply,request_id,self.identity)
            except BaseException as exc:
                self._broken=True
                if isinstance(exc,(KeyboardInterrupt,SystemExit)):raise
                raise CollisionError(str(exc)) from exc

    def initialize(self, model, scene, *, geometry_profile=None, semantic=None):
        """INIT once; READY is accepted only after independent identity checks."""
        try:
            init=make_init(model,scene)
            if semantic is not None:
                if geometry_profile is not None:raise CollisionError('semantic mode requires official geometry without profile override')
                init.update(semantic.wire())
            if geometry_profile is not None:init.update(geometry_profile.wire())
            reply=self._exchange({'op':'INIT',**init})
            self.identity=check_ready(reply,init,self.executable)
            if reply.get('backend_status')!='READY':raise CollisionError('not READY')
            self.ready=reply
            if geometry_profile is not None:reply['effective_model_xml']=geometry_profile.effective_xml
            return reply
        except BaseException:
            self._broken=True;self.close();raise

    def candidates(self, *, trials=0):
        """Run standard MoveIt candidate generation on an isolated native scene.

        Zero selects a bounded thread-aware count. Recommendations never install ACM.
        Use a separate client session for this diagnostic operation.
        """
        if self.identity is None:raise CollisionError('initialize first')
        if type(trials) is not int or not 0<=trials<=100000:raise CollisionError('invalid candidate trial count')
        reply=self._exchange({'op':'CANDIDATES','trials':trials})
        if reply.get('backend_status')!='CANDIDATES' or reply.get('semantic_acm_modified') is not False or reply.get('finite_sampling_is_safety_proof') is not False:raise CollisionError('invalid candidate acknowledgement')
        names={v['name'] for v in self.ready['audit']['links']};seen=set()
        for p in reply['candidate_pairs']:
            pair=p.get('links',[])
            if len(pair)!=2 or pair[0]>=pair[1] or not set(pair)<=names or tuple(pair) in seen or p.get('applied') is not False or p.get('review_status')!='UNREVIEWED':raise CollisionError('invalid candidate pair')
            seen.add(tuple(pair))
        return reply

    def set_policy(self, canonical, digest):
        """Explicit protocol-v1 policy replacement; semantic sessions reject overlays."""
        previous=self.identity
        if previous is None:raise CollisionError('initialize first')
        self.identity=None
        try:
            reply=self._exchange({'op':'POLICY','policy_canonical_json':canonical,'policy_sha256':digest})
            expected={**previous,'policy_sha256':digest}
            if reply.get('identity')!=expected or reply.get('backend_status')!='POLICY_SET':
                raise CollisionError('policy acknowledgement mismatch')
            self.identity=expected
            return reply
        except BaseException:
            self.identity=previous;self._broken=True;raise

    def query_positions(self, positions, *, geometry_audit=None, task_scope=None):
        """Send full URDF state including fixed/mimic joints; never infer CLEAR."""
        if self.identity is None:raise CollisionError('initialize first')
        payload={'op':'QUERY','joint_positions':positions}
        if geometry_audit is not None:payload['geometry_audit']=geometry_audit
        if task_scope is not None:payload.update(task_scope_canonical_json=task_scope[0],task_scope_sha256=task_scope[1],task_scope_context=self.identity)
        try:
            reply=check_query(self._exchange(payload))
            if self.ready and 'semantic' in self.ready.get('audit',{}):
                from .semantic import check_semantic_query
                check_semantic_query(reply,self.ready['audit']['semantic']['actual_disabled_pairs'])
            if task_scope is not None:
                from .scope import check_scope_result
                check_scope_result(reply,task_scope,self.identity)
            return reply
        except BaseException:
            self._broken=True;raise

    def query(self, model, q, *, auxiliary_positions=None, geometry_audit=None, task_scope=None):
        """Reuse G1 joint-state/mimic semantics before submitting a full state."""
        return self.query_positions(model.joint_positions(q,auxiliary_positions),geometry_audit=geometry_audit,task_scope=task_scope)

    def close(self):
        """Return resource release and exit-code evidence separately."""
        if self.cleanup is not None:return self.cleanup
        graceful=False;errors=[]
        try:
            if self.process and self.process.poll() is None and not self._broken:
                reply=self._exchange({'op':'SHUTDOWN'});graceful=reply.get('backend_status')=='STOPPED'
        except Exception as exc:errors.append(str(exc))
        if self.process:
            if self.process.stdin:self.process.stdin.close()
            try:self.process.wait(timeout=min(self.timeout,2))
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=2)
            if self.process.stdout:self.process.stdout.close()
        self._stderr.seek(0,2);size=self._stderr.tell();self._stderr.seek(max(0,size-16000))
        log=self._stderr.read().decode(errors='replace');self._stderr.close()
        code=self.process.returncode if self.process else None
        self.cleanup={'resources_released':self.process is None or self.process.poll() is not None,
                      'exit_code':code,'exit_normal':code==0 and graceful,'graceful_shutdown':graceful,
                      'errors':errors,'stderr_tail':log}
        return self.cleanup

    def __enter__(self):return self

    def __exit__(self,*args):self.close()
