"""ROS adapter template: measured hardware hooks, sequence admission and stopping."""

import copy
import json
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from .protocol import BridgeError, Reply


@dataclass
class CommandContext:
    deadline: float
    cancelled: threading.Event = field(default_factory=threading.Event)

    def check(self):
        if self.cancelled.is_set() or time.monotonic() >= self.deadline:
            raise BridgeError(504, 'Hardware execution interrupted or deadline expired', outcome='unknown')


class HardwareBackend:
    """Implement hooks with nonblocking controller futures and measured completion.

    prepare must validate the ENTIRE group/sequence without moving. A true
    capability declares actual synchronized execution, not parallel service calls.
    Hooks must honor context.check()/deadline while waiting and stop actual hardware.
    """
    coordinated_motion = False
    coordinated_gripper = False
    coupled_transfer = False
    joint_targets = False
    recovery = False

    # HARDWARE INTEGRATION: implement these four hooks in your ROS package.
    async def prepare(self, steps, arm_ids, coupled, context):
        # Resolve named goals from backend-owned coordinates and freeze the
        # resulting plans for move. Check TCP support/calibration here as needed.
        raise BridgeError(501, 'Hardware preflight is not implemented')

    async def move(self, targets, arm_ids, duration, coupled, context):
        # Pose reference is explicit: tcp or flange. Pixel/plan goals use tcp.
        # Named goals carry only a name; use the plan frozen during prepare.
        raise BridgeError(501, 'Hardware motion is not implemented')

    async def gripper(self, openings, arm_ids, context):
        raise BridgeError(501, 'Hardware gripper operation is not implemented')

    async def stop(self, arm_ids, context):
        raise BridgeError(501, 'Hardware stop is not implemented', outcome='unknown')

    # HARDWARE INTEGRATION: optional; never substitute an unconditional home/open.
    async def recover(self, arm_ids, context):
        raise BridgeError(501, 'Safe hardware recovery is not supported')


class PrimitiveAdapter:
    """Shared service handler; reusable with a Node or an isolated test backend."""
    def __init__(self, config, backend, ros_now):
        self.config, self.backend, self.ros_now = config, backend, ros_now
        self.sequence = None
        self.seen = deque(maxlen=256)
        self.epoch = 0
        self.busy = False
        self.stopping = 0
        self.active_ids = []
        self.contexts = []
        self.lock = threading.RLock()

    def ids(self, ids):
        if (not isinstance(ids,list) or not ids or any(not isinstance(a,str) for a in ids)
                or len(set(ids)) != len(ids) or not set(ids) <= {a.id for a in self.config.arms}):
            raise BridgeError(422, 'arm_ids must be distinct configured arms')
        return ids

    async def call_backend(self, function, *args):
        try:
            result = await function(*args)
        except BridgeError:
            raise
        except Exception as exc:
            raise BridgeError(502, f'Hardware backend failed: {exc}', outcome='unknown') from exc
        if not isinstance(result,dict) or type(result.get('success')) is not bool:
            raise BridgeError(502,'Backend must report boolean measured completion',outcome='unknown')
        return result

    async def handle(self, request, response):
        context = None
        owns_busy = False
        try:
            import math
            if not math.isfinite(request.timeout_sec) or not 0 < request.timeout_sec <= 15000:
                raise BridgeError(422, 'timeout_sec must be finite and in (0,15000]')
            remaining = request.timeout_sec
            if request.deadline_ns:
                remaining = min(remaining,(request.deadline_ns-self.ros_now())/1e9)
            if remaining <= 0:
                raise BridgeError(504,'Driver request expired before admission',outcome='not_started')
            context = CommandContext(time.monotonic()+remaining)
            payload = json.loads(request.payload_json)
            if not isinstance(payload,dict) or request.resource_id:
                raise BridgeError(422,'Primitive requests require an object and empty resource_id')
            operation = request.operation
            with self.lock:
                if operation == 'stop':
                    ids = self.ids(payload.get('arm_ids'))
                    if set(payload) != {'arm_ids'}:
                        raise BridgeError(422,'stop accepts only arm_ids')
                    if len(self.active_ids)>1 and set(ids).intersection(self.active_ids):
                        ids = [a.id for a in self.config.arms if a.id in set(ids)|set(self.active_ids)]
                    self.stopping += 1
                    self.epoch += 1
                    epoch = self.epoch
                    for pending in self.contexts:
                        pending.cancelled.set()
                    self.sequence = None
                else:
                    if self.busy or self.stopping:
                        raise BridgeError(409,'Another primitive is active; stop remains available')
                    self.busy = owns_busy = True
                    epoch = self.epoch
                self.contexts.append(context)
            if operation == 'stop':
                # The bridge expands coupled scope; direct driver callers do so too.
                if set(payload) != {'arm_ids'}:
                    raise BridgeError(422,'stop accepts only arm_ids')
                result = await self.call_backend(self.backend.stop,ids,context)
            elif operation == 'prepare':
                if set(payload) != {'sequence_id','steps','arm_ids','coupled'}:
                    raise BridgeError(422,'prepare requires sequence_id, steps, arm_ids, coupled')
                ids = self.ids(payload['arm_ids'])
                sequence_id = payload['sequence_id']
                if not isinstance(sequence_id,str) or not sequence_id or sequence_id in self.seen or self.sequence is not None:
                    raise BridgeError(409,'Sequence is repeated or another sequence is reserved')
                steps = payload['steps']
                coupled = payload['coupled']
                if type(coupled) is not bool or not isinstance(steps,list) or not 1 <= len(steps) <= 16:
                    raise BridgeError(422,'Invalid coupled flag or step count')
                from .motion import PoseTarget, JointTarget, NamedTarget
                for step in steps:
                    if not isinstance(step,dict) or self.ids(step.get('arm_ids')) != ids:
                        raise BridgeError(422,'Each step must cover the prepared arm_ids in order')
                    op = step.get('operation')
                    allowed = {'operation','arm_ids','phase'}
                    if op == 'move':
                        allowed |= {'targets','duration'}
                        if len(ids)>1 and not self.backend.coordinated_motion:
                            raise BridgeError(501,'Coordinated motion is unsupported')
                        if not isinstance(step.get('targets'),list) or len(step['targets']) != len(ids):
                            raise BridgeError(422,'One resolved target is required per arm')
                        for target in step['targets']:
                            if target.get('kind') == 'joints':
                                JointTarget.model_validate(target)
                                if not self.backend.joint_targets:
                                    raise BridgeError(501,'Joint targets are unsupported')
                            elif target.get('kind') == 'named':
                                NamedTarget.model_validate(target)
                            else:
                                PoseTarget.model_validate(target)
                        duration = step.get('duration')
                        if type(duration) not in (float,int) or not math.isfinite(duration) or not .1 <= duration <= 60:
                            raise BridgeError(422,'duration must be finite in [0.1,60]')
                    elif op == 'gripper':
                        allowed |= {'opening','openings'}
                        if ('opening' in step) == ('openings' in step):
                            raise BridgeError(422,'Provide opening or openings, exclusively')
                        openings = step.get('openings',[step.get('opening')]*len(ids))
                        if not isinstance(openings,list) or len(openings)!=len(ids) or any(type(v) not in (float,int) or not math.isfinite(v) or not 0<=v<=1 for v in openings):
                            raise BridgeError(422,'One finite opening in [0,1] is required per arm')
                        if len(ids)>1 and not self.backend.coordinated_gripper:
                            raise BridgeError(501,'Coordinated grippers are unsupported')
                    elif op == 'recover':
                        if not self.backend.recovery:
                            raise BridgeError(501,'Safe recovery is unsupported')
                    else:
                        raise BridgeError(422,f'Unknown primitive: {op}')
                    if set(step)-allowed:
                        raise BridgeError(422,f'Unexpected primitive fields: {sorted(set(step)-allowed)}')
                if coupled and len(ids)>1 and not self.backend.coupled_transfer:
                    raise BridgeError(501,'Coupled object transfer is unsupported')
                self.active_ids = list(ids)
                result = await self.call_backend(self.backend.prepare,copy.deepcopy(steps),ids,coupled,context)
                context.check()
                with self.lock:
                    if epoch != self.epoch:
                        raise BridgeError(409,'Preflight superseded by stop',outcome='unknown')
                    if result.get('success') is True:
                        self.seen.append(sequence_id)
                        self.sequence = dict(id=sequence_id,steps=copy.deepcopy(steps),index=0,
                                             coupled=coupled,deadline=context.deadline,arm_ids=ids)
            else:
                with self.lock:
                    sequence = self.sequence
                    if sequence is None or payload.get('sequence_id') != sequence['id'] or payload.get('step_index') != sequence['index']:
                        raise BridgeError(409,'Unknown, repeated or out-of-order primitive')
                    expected = dict(sequence['steps'][sequence['index']],sequence_id=sequence['id'],step_index=sequence['index'])
                    if payload != expected or operation != expected['operation']:
                        raise BridgeError(422,'Primitive differs from the preflighted step')
                    context.deadline = min(context.deadline,sequence['deadline'])
                    ids = sequence['arm_ids']
                context.check()
                if operation == 'move':
                    result = await self.call_backend(self.backend.move,payload['targets'],ids,payload['duration'],sequence['coupled'],context)
                elif operation == 'gripper':
                    result = await self.call_backend(self.backend.gripper,payload.get('openings',[payload.get('opening')]*len(ids)),ids,context)
                else:
                    result = await self.call_backend(self.backend.recover,ids,context)
                if not isinstance(result,dict) or type(result.get('success')) is not bool:
                    raise BridgeError(502,'Backend must report boolean measured completion',outcome='unknown')
                with self.lock:
                    if self.sequence is sequence:
                        sequence['index'] += 1
                        if result.get('success') is not True or sequence['index'] == len(sequence['steps']):
                            self.sequence = None
            context.check()
            if epoch != self.epoch:
                raise BridgeError(409,'Primitive superseded by stop',outcome='unknown')
            if not isinstance(result,dict) or type(result.get('success')) is not bool:
                raise BridgeError(502,'Backend must report boolean measured completion',outcome='unknown')
            return Reply(payload=dict(result,coordinated=True,completed_arm_ids=ids) if result['success'] else result).to_ros(response)
        except BridgeError as exc:
            return Reply.from_error(exc).to_ros(response)
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            return Reply.from_error(BridgeError(422,str(exc))).to_ros(response)
        except Exception as exc:
            return Reply.from_error(BridgeError(502,f'Hardware backend failed: {exc}',outcome='unknown')).to_ros(response)
        finally:
            with self.lock:
                if context in self.contexts:
                    self.contexts.remove(context)
                if owns_busy:
                    self.busy = False
                if context is not None and 'epoch' in locals() and operation == 'stop':
                    self.stopping -= 1
                if self.sequence is None and not self.busy and not self.stopping:
                    self.active_ids = []


# Import rclpy only for the ROS wrapper; backend/handler tests can use plain asyncio.
def driver_node(config, backend, **kwargs):
    """Create the ROS node; supply measured telemetry from your own publishers."""
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from ros2_agent_interfaces.srv import RobotRequest
    node = Node('primitive_driver', **config.server.node_options(kwargs))
    node.adapter = PrimitiveAdapter(config,backend,lambda:node.get_clock().now().nanoseconds)
    node.command_service = node.create_service(RobotRequest,config.server.driver_service,node.adapter.handle,
                                               callback_group=ReentrantCallbackGroup())
    return node
