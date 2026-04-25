import traceback
from contextlib import contextmanager

class LockTimeoutError(TimeoutError):
    ...

@contextmanager
def locked(lock, timeout=0.5):
    if not lock.acquire(timeout=timeout):
        raise LockTimeoutError("Lock timed out while acquiring!")
    try:
        yield
    finally:
        lock.release()


def format_error_message(error: Exception, simplify=True):
    """Simple exception formatting for ros logging"""

    if simplify:
        return f"{type(error).__name__}: {str(error)}"
    else:
        return "".join(
            traceback.format_exception(type(error), error, error.__traceback__)
        )
