from abc import ABC
from abc import abstractmethod
from typing import Optional
from typing import TypeVar

from genpy import Duration
from genpy import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer_interface import TransformRegistration

T = TypeVar('T')


class BufferInterface(ABC):

    def __init__(self):
        self._registration = TransformRegistration()

    async def transform(
        self,
        object_stamped: T,
        target_frame: str,
        timeout: Optional[Duration] = None
    ) -> T:
        do_transform = self._registration.get(type(object_stamped))
        return do_transform(
            object_stamped,
            await self.lookup_transform(
                target_frame,
                object_stamped.header.frame_id,
                object_stamped.header.stamp,
                timeout))

    async def transform_full(
        self,
        object_stamped: T,
        target_frame: str,
        target_time: Time,
        fixed_frame: str,
        timeout: Optional[Duration] = None
    ) -> T:
        do_transform = self._registration.get(type(object_stamped))
        return do_transform(
            object_stamped,
            await self.lookup_transform_full(
                target_frame,
                target_time,
                object_stamped.header.frame_id,
                object_stamped.header.stamp,
                fixed_frame,
                timeout))

    @abstractmethod
    async def lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: Time,
        timeout: Optional[Duration] = None
    ) -> TransformStamped:
        pass

    @abstractmethod
    async def lookup_transform_full(
        self,
        target_frame: str,
        target_time: Time,
        source_frame: str,
        source_time: Time,
        fixed_frame: str,
        timeout: Optional[Duration] = None
    ) -> TransformStamped:
        pass

    @abstractmethod
    async def can_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: Time,
        timeout: Optional[Duration] = None
    ) -> bool:
        pass

    @abstractmethod
    async def can_transform_full(
        self,
        target_frame: str,
        target_time: Time,
        source_frame: str,
        source_time: Time,
        fixed_frame: str,
        timeout: Optional[Duration] = None
    ) -> bool:
        pass
