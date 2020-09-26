from typing import Optional

from aioros_action import ActionClient
from aioros_action import create_client
from aioros import NodeHandle

from aioros_tf2.abc import BufferInterface
from aioros_tf2.exceptions import ConnectivityException
from aioros_tf2.exceptions import ExtrapolationException
from aioros_tf2.exceptions import InvalidArgumentException
from aioros_tf2.exceptions import LookupException
from aioros_tf2.exceptions import TimeoutException
from aioros_tf2.exceptions import TransformException

from genpy import Duration
from genpy import Time
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import LookupTransformAction
from tf2_msgs.msg import LookupTransformGoal
from tf2_msgs.msg import TF2Error


class BufferActionClient(BufferInterface):

    def __init__(self, ns: str) -> None:
        self._ns = ns
        self._action_client: Optional[ActionClient] = None

    async def init(
        self,
        node_handle: NodeHandle
    ) -> None:
        self._action_client = await create_client(
            node_handle,
            self._ns,
            LookupTransformAction)

    async def close(self) -> None:
        if self._action_client:
            await self._action_client.close()
            self._action_client = None

    async def wait_for_server(self) -> None:
        await self._action_client.wait_for_server()

    async def lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: Time,
        timeout: Optional[Duration] = None
    ) -> TransformStamped:
        return await self._call_action(
            LookupTransformGoal(
                target_frame=target_frame,
                source_frame=source_frame,
                source_time=time,
                timeout=timeout or Duration(),
                advanced=False))

    async def lookup_transform_full(
        self,
        target_frame: str,
        target_time: Time,
        source_frame: str,
        source_time: Time,
        fixed_frame: str,
        timeout: Optional[Duration] = None
    ) -> TransformStamped:
        return await self._call_action(
            LookupTransformGoal(
                target_frame=target_frame,
                source_frame=source_frame,
                source_time=source_time,
                timeout=timeout or Duration(),
                target_time=target_time,
                fixed_frame=fixed_frame,
                advanced=True))

    async def can_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: Time,
        timeout: Optional[Duration] = None
    ) -> bool:
        try:
            self.lookup_transform(
                target_frame,
                source_frame,
                time,
                timeout)
            return True
        except TransformException:
            return False

    async def can_transform_full(
        self,
        target_frame: str,
        target_time: Time,
        source_frame: str,
        source_time: Time,
        fixed_frame: str,
        timeout: Optional[Duration] = None
    ) -> bool:
        try:
            self.lookup_transform_full(
                target_frame,
                target_time,
                source_frame,
                source_time,
                fixed_frame,
                timeout)
            return True
        except TransformException:
            return False

    async def _call_action(
        self,
        goal: LookupTransformGoal
    ) -> TransformStamped:
        goal_handle = self._action_client.send_goal(goal)
        result = await goal_handle.wait_for_result()

        if result.error.error != TF2Error.NO_ERROR:
            if result.error.error == TF2Error.LOOKUP_ERROR:
                raise LookupException(result.error.error_string)
            elif result.error.error == TF2Error.CONNECTIVITY_ERROR:
                raise ConnectivityException(result.error.error_string)
            elif result.error.error == TF2Error.EXTRAPOLATION_ERROR:
                raise ExtrapolationException(result.error.error_string)
            elif result.error.error == TF2Error.INVALID_ARGUMENT_ERROR:
                raise InvalidArgumentException(result.error.error_string)
            elif result.error.error == TF2Error.TIMEOUT_ERROR:
                raise TimeoutException(result.error.error_string)
            else:
                raise TransformException(result.error.error_string)

        return result.transform


async def create_buffer_action_client(
    node_handle: NodeHandle,
    ns: str,
) -> BufferActionClient:
    buffer_action_client = BufferActionClient(ns)
    await buffer_action_client.init(node_handle)
    return buffer_action_client
