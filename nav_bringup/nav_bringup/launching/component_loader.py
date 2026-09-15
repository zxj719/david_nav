"""Bounded fail-closed component loading owned by the same ROS launch context.

Humble's stock loader logs unsuccessful LoadNode responses without failing the
launch. This adapter uses the stock request conversion, but a failed response
or deadline raises through launch's completion future and shuts down its owned
processes. It creates no detached process or ROS executor.
"""

import asyncio
import math
from typing import Sequence

from composition_interfaces.srv import LoadNode
from launch import LaunchContext
from launch.action import Action
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
from launch_ros.descriptions import ComposableNode
from launch_ros.ros_adapters import get_ros_node


class CheckedComponentLoader(Action):
    """Load components sequentially with one bounded wall-clock startup deadline."""

    def __init__(self, target: str, descriptions: Sequence[ComposableNode],
                 timeout_sec: float = 90.0) -> None:
        """Store unexecuted descriptions and validate the load deadline.

        Args:
            target: Absolute container node name.
            descriptions: Component descriptions using frozen parameters and remappings.
            timeout_sec: Total deadline after the owned container starts.

        Raises:
            ValueError: The deadline is not finite and positive.
        """
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('component load timeout must be finite and positive')
        super().__init__()
        self._target = target
        self._descriptions = tuple(descriptions)
        self._timeout = timeout_sec

    def execute(self, context: LaunchContext) -> None:
        """Create requests and register one launch-owned asynchronous completion task.

        Args:
            context: Launch context owning the ROS adapter and shutdown lifecycle.
        """
        requests = [get_composable_node_load_request(description, context)
                    for description in self._descriptions]
        client = get_ros_node(context).create_client(
            LoadNode, self._target + '/_container/load_node')
        context.add_completion_future(context.asyncio_loop.create_task(
            self._load(context, client, requests)))

    async def _load(self, context, client, requests):
        deadline = context.asyncio_loop.time() + self._timeout
        pending = None
        try:
            for request in requests:
                if context.is_shutdown:
                    return
                while not client.service_is_ready():
                    if not await self._poll(context, deadline, request.node_name):
                        return
                if context.is_shutdown:
                    return
                pending = client.call_async(request)
                while not pending.done():
                    if not await self._poll(context, deadline, request.node_name):
                        return
                if context.is_shutdown:
                    return
                response = pending.result()
                if response is None or not response.success:
                    detail = response.error_message if response else 'empty response'
                    raise RuntimeError(f'component {request.node_name} failed to load: {detail}')
                pending = None
        finally:
            if pending is not None and not pending.done():
                pending.cancel()
            # The launch ROS adapter owns and destroys this client on shutdown.

    async def _poll(self, context, deadline, name):
        if context.is_shutdown:
            return False
        if context.asyncio_loop.time() >= deadline:
            raise RuntimeError(f'component {name} load timed out after {self._timeout}s')
        await asyncio.sleep(0.05)
        return not context.is_shutdown
