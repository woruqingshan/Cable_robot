"""Bounded asynchronous feedback archive. Stopping/failed recording leaves the bus alive."""
import csv
import json
from pathlib import Path
import queue
import threading
import time

FIELDS=('sequence','servo_id','raw_pulse','nominal_rad','source_type','physical_calibrated',
 'read_started_at_ns','response_received_at_ns','read_latency_ns','host_transaction_span_ns',
 'scheduled_ns','schedule_started_ns','schedule_lateness_ns','skipped_periods',
 'outer_frame_validated','error','tx_frame_hex','rx_frame_hex')

class FeedbackRecorder:
    def __init__(self, directory, *, capacity=256):
        self.path=Path(directory);self.path.mkdir(parents=True,exist_ok=False)
        self.queue=queue.Queue(maxsize=capacity)
        self.lock=threading.Lock();self.active=True;self.error=None
        self.accepted=self.written=self.rejected=0
        self.started_ns=time.monotonic_ns();self.stopped_ns=None
        self.ready=threading.Event()
        self.thread=threading.Thread(target=self._run,name='feedback_recording',daemon=True)
        self.thread.start()
        if not self.ready.wait(2) or self.error:
            self.active=False
            raise RuntimeError('RECORDING_START_FAILED:'+str(self.error))

    def submit(self,report):
        with self.lock:
            if not self.active or self.error:return
            try:self.queue.put_nowait(report);self.accepted+=1
            except queue.Full:
                self.rejected+=1;self.error='RECORDING_QUEUE_FULL';self.active=False

    def _run(self):
        try:
            with (self.path/'feedback.csv').open('x',newline='') as csv_file, (self.path/'raw_feedback.jsonl').open('x') as raw:
                writer=csv.DictWriter(csv_file,fieldnames=FIELDS);writer.writeheader()
                self.ready.set()
                while True:
                    try:report=self.queue.get(timeout=.05)
                    except queue.Empty:
                        if not self.active:break
                        continue
                    try:
                        raw.write(json.dumps(report,separators=(',',':'))+'\n')
                        converted=report.get('conversion_candidate',{})
                        angles=dict(zip(converted.get('joint_names',[]),converted.get('positions',[])))
                        for channel in report['channels']:
                            row={k:channel.get(k) for k in FIELDS}
                            name='r_joint' if channel['servo_id']==10 else 'joint'+str(channel['servo_id'])
                            row.update(sequence=report['sequence'],nominal_rad=angles.get(name),
                                physical_calibrated=False,host_transaction_span_ns=report.get('host_transaction_span_ns'))
                            for k in ('scheduled_ns','schedule_started_ns','schedule_lateness_ns','skipped_periods'):
                                row[k]=report.get(k)
                            writer.writerow(row)
                        self.written+=1
                    finally:self.queue.task_done()
        except Exception as e:
            self.error='RECORDING_IO_FAILED:'+type(e).__name__;self.active=False
        finally:self.ready.set()

    def status(self):
        return dict(directory=str(self.path),active=self.active,error=self.error,
            accepted_batches=self.accepted,written_batches=self.written,rejected_batches=self.rejected,
            started_monotonic_ns=self.started_ns,stopped_monotonic_ns=self.stopped_ns,
            complete=not self.thread.is_alive() and self.error is None and self.accepted==self.written)

    def stop(self):
        with self.lock:self.active=False;self.stopped_ns=time.monotonic_ns()
        self.thread.join(timeout=2)
        if self.thread.is_alive():self.error='RECORDING_DRAIN_NOT_COMPLETE'
        result=self.status()
        (self.path/'recording_manifest.json').write_text(json.dumps(result,indent=2))
        return result
