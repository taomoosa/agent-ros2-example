"""Execute one preflighted sequence under the original deadline and stop epoch."""

import time
import uuid
from .protocol import BridgeError, Reply


async def execute(bridge, steps, arm_ids, coupled, deadline, trace, generation):
    sequence_id = uuid.uuid4().hex
    started = False
    try:
        async def send(operation, payload):
            if bridge._generation != generation:
                raise BridgeError(409,'Sequence interrupted by stop',outcome='unknown')
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise BridgeError(504,'Sequence deadline expired',outcome='unknown')
            result = await bridge._command(operation,'',payload,remaining,request_id=trace)
            if bridge._generation != generation:
                raise BridgeError(409,'Sequence interrupted by stop',outcome='unknown')
            if result.status != 200 or result.payload.get('success') is not True:
                error = BridgeError(result.status if result.status >= 400 else 409,
                                    result.payload.get('error','Primitive failed'),outcome=result.payload.get('outcome'))
                error.payload.update(result.payload,success=False)
                if result.status >= 500:
                    error.payload.setdefault("outcome", "unknown")
                raise error
            if result.payload.get('coordinated') is not True or sorted(result.payload.get('completed_arm_ids',[])) != sorted(arm_ids):
                raise BridgeError(502,'Primitive did not confirm every requested arm',outcome='unknown')
            return result
        await send('prepare',dict(sequence_id=sequence_id,steps=steps,arm_ids=arm_ids,coupled=coupled))
        for index,step in enumerate(steps):
            started = True
            await send(step['operation'],dict(step,sequence_id=sequence_id,step_index=index))
            await bridge._wait_state(arm_ids,deadline,trace,generation)
            if bridge._generation != generation:
                raise BridgeError(409,'Sequence interrupted during state verification',outcome='unknown')
            bridge._check_health(arm_ids, expect_grasp=step.get('phase') in {'close','lift'},
                                 expect_released=step.get('phase') == 'open')
        return Reply(payload=dict(success=True,coordinated=True,completed_arm_ids=arm_ids))
    except BridgeError as exc:
        # Even failed preflight can leave a reservation; stop clears it. Never replay.
        if bridge._generation == generation:
            try:
                stopped = await bridge._command('stop','',dict(arm_ids=arm_ids),
                    bridge.config.server.stop_timeout,request_id=trace)
                confirmed = (stopped.status == 200 and stopped.payload.get('success') is True
                             and stopped.payload.get('coordinated') is True
                             and sorted(stopped.payload.get('completed_arm_ids',[])) == sorted(arm_ids))
                bridge._all_stopped = confirmed and set(arm_ids)=={a.id for a in bridge.config.arms}
                exc.payload.update(stop_result=stopped.payload,operator_required=not confirmed)
                if not confirmed:
                    exc.payload['outcome'] = 'unknown'
            except Exception as stop_error:
                exc.payload.update(outcome='unknown',operator_required=True,stop_error=str(stop_error))
        if started:
            exc.payload.setdefault('failed_phase',steps[index].get('phase',steps[index]['operation']))
        raise
