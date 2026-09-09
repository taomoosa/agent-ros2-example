"""Bound synchronous transport work without blocking the event loop or shutdown."""

import asyncio
import threading


async def thread_call(function, timeout, *, abandoned=None):
  loop = asyncio.get_running_loop()
  result = loop.create_future()
  def deliver(value, error):
    if result.done():
      if error is None and abandoned is not None:
        threading.Thread(target=abandoned, args=(value,), daemon=True).start()
      return
    if error is not None:
      result.set_exception(error)
    else:
      result.set_result(value)
  def work():
    try:
      value, error = function(), None
    except BaseException as exc:
      value, error = None, exc
    try:
      loop.call_soon_threadsafe(deliver, value, error)
    except RuntimeError:
      if error is None and abandoned is not None:
        abandoned(value)
  threading.Thread(target=work, daemon=True).start()
  return await asyncio.wait_for(result, timeout)


async def bounded_cleanup(awaitable, timeout):
  task = asyncio.ensure_future(awaitable)
  done, _ = await asyncio.wait({task}, timeout=timeout)
  if not done:
    task.cancel()
    task.add_done_callback(lambda t: None if t.cancelled() else t.exception())
    return False
  await task
  return True
