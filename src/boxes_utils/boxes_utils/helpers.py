import traceback

def format_error_message(error: Exception, simplify=True):
    """Simple exception formatting for ros logging"""

    if simplify:
        return f"{type(error).__name__}: {str(error)}"
    else:
        return "".join(
            traceback.format_exception(type(error), error, error.__traceback__)
        )