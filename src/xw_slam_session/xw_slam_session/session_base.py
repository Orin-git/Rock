#!/usr/bin/env python3
"""Shared tiny session base for P0 stubs."""

from xw_interfaces.msg import TaskProgress, TaskResult
from xw_interfaces.srv import SessionControl


class SessionMixin:
    capability: str = 'session'
    service_name: str = '/xw/session/unknown/control'

    def _setup_session(self) -> None:
        self._active = False
        self._command_id = ''
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)
        self.create_service(SessionControl, self.service_name, self._on_control)

    def _on_control(self, req: SessionControl.Request, res: SessionControl.Response):
        if req.start:
            self._active = True
            self._command_id = req.command_id or f'{self.capability}-start'
            self._on_start(req.payload_json or '{}')
            res.success = True
            res.message = f'{self.capability} started'
            res.state = 'active'
            self._publish_progress('active')
        else:
            self._on_stop()
            self._active = False
            res.success = True
            res.message = f'{self.capability} stopped'
            res.state = 'idle'
            self._publish_result(0, 'stopped')
        return res

    def _on_start(self, payload_json: str) -> None:
        pass

    def _on_stop(self) -> None:
        pass

    def _publish_progress(self, phase: str) -> None:
        p = TaskProgress()
        p.stamp = self.get_clock().now().to_msg()
        p.command_id = self._command_id
        p.capability = self.capability
        p.phase = phase
        self._progress_pub.publish(p)

    def _publish_result(self, code: int, message: str) -> None:
        r = TaskResult()
        r.stamp = self.get_clock().now().to_msg()
        r.command_id = self._command_id
        r.capability = self.capability
        r.code = code
        r.message = message
        self._result_pub.publish(r)
