#!/usr/bin/env python3
"""不产生运动的假上位机：只走拒绝、畸形、取消与重连路径，绝不下发有效目标。

用于 2026-09-20 的整机验收。所有 navigate_to 的 target_id 都是不存在的点位，
因此任务在准入阶段就被拒绝，执行器不会向 Nav2 下发任何目标，机器人不可能移动。

用法: python3 nomotion_agent.py <transcript.jsonl> [PORT]
"""
import asyncio
import json
import sys
import time

import websockets

TRANSCRIPT = sys.argv[1]
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8101

BOGUS = 'POI_NO_SUCH_POINT_20260920'
connections = [0]
done = asyncio.Event()


def record(direction, payload, note=''):
    """把一帧写进逐行 JSON 记录，同时打到标准输出便于现场观察。"""
    row = {'t': round(time.time(), 3), 'dir': direction, 'note': note, 'payload': payload}
    with open(TRANSCRIPT, 'a', encoding='utf-8') as handle:
        handle.write(json.dumps(row, ensure_ascii=False) + '\n')
    print('[agent]', direction, note, json.dumps(payload, ensure_ascii=False)[:200], flush=True)


async def send(ws, payload, note):
    """发送一帧；payload 为字符串时按原样发出，用于畸形输入。"""
    raw = payload if isinstance(payload, str) else json.dumps(payload, ensure_ascii=False)
    await ws.send(raw)
    record('agent->executor', payload, note)


async def scenario(ws):
    """按顺序跑完全部无运动用例，每步之间留出事件回传的时间。"""
    await asyncio.sleep(1.0)
    await send(ws, {'type': 'ping'}, 'S1 心跳：type 优先于 action')
    await asyncio.sleep(1.0)

    await send(ws, 'this-is-not-json', 'S2 无法解析的帧：应记录并忽略，不断连')
    await asyncio.sleep(1.0)

    await send(ws, '[1, 2, 3]', 'S3 JSON 非对象：应记录并忽略')
    await asyncio.sleep(1.0)

    await send(ws, {'action': 'navigate_to', 'request_id': 'probe-unknown-poi',
                    'sub_id': 1, 'target_id': BOGUS},
               'S4 未知点位：准入阶段拒绝，不向 Nav2 下发目标')
    await asyncio.sleep(3.0)

    await send(ws, {'action': 'navigate_to', 'request_id': 'probe-bad-target',
                    'sub_id': 2, 'target_id': 12345},
               'S5 target_id 非字符串：畸形但 request_id 可恢复，应回 on_error')
    await asyncio.sleep(3.0)

    await send(ws, {'action': 'no_such_action', 'request_id': 'probe-unknown-action'},
               'S6 未知 action：记录并忽略')
    await asyncio.sleep(1.0)

    await send(ws, {'action': 'abort_navigation', 'request_id': 'probe-idle-abort'},
               'S7 空闲时取消不存在的任务：不新增 ACK，不崩溃')
    await asyncio.sleep(2.0)

    await send(ws, {'action': 'update_poi_list', 'version': 1,
                    'poi_list': [{'id': BOGUS, 'name': '不应被创建'}]},
               'S8 目录更新：只重命名已存在点位，不创建未知点，无 ACK')
    await asyncio.sleep(2.0)

    await send(ws, {'action': 'navigate_to', 'request_id': 'probe-unknown-poi',
                    'sub_id': 3, 'target_id': BOGUS},
               'S9 同一 request_id 再次拒绝：状态机不残留旧任务')
    await asyncio.sleep(3.0)

    record('agent', {}, 'S10 主动断开连接，验证执行器重连且不取消导航')
    await ws.close()


async def handler(ws, path=None):
    """每次连接：第一次跑用例，第二次只确认重连成功。"""
    connections[0] += 1
    index = connections[0]
    record('event', {'connection': index}, '执行器已连接')
    reader = asyncio.ensure_future(pump(ws))
    try:
        if index == 1:
            await scenario(ws)
        else:
            record('event', {'connection': index}, 'S11 重连成功，保持空闲 10 秒')
            await asyncio.sleep(10.0)
            done.set()
        await reader
    except websockets.ConnectionClosed:
        pass
    finally:
        reader.cancel()
        record('event', {'connection': index}, '连接结束')


async def pump(ws):
    """把执行器发来的每一帧都记下来；ping 由本端回 pong。"""
    try:
        async for raw in ws:
            try:
                message = json.loads(raw)
            except Exception:
                record('executor->agent', {'raw': raw[:400]}, '非 JSON')
                continue
            record('executor->agent', message, '')
            if message.get('type') == 'ping':
                await ws.send(json.dumps({'type': 'pong'}))
                record('agent->executor', {'type': 'pong'}, '回应心跳')
    except websockets.ConnectionClosed:
        pass


async def main():
    """监听回环端口，跑完全部用例后退出，总时长有上限。"""
    async with websockets.serve(handler, '127.0.0.1', PORT, ping_interval=None):
        record('event', {'port': PORT}, '假上位机已监听，绝不下发有效目标')
        try:
            await asyncio.wait_for(done.wait(), timeout=180.0)
        except asyncio.TimeoutError:
            record('event', {}, '总时长到上限，结束')
    record('event', {'connections': connections[0]}, '假上位机退出')


asyncio.get_event_loop().run_until_complete(main())
