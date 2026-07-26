"""One place that turns an environment string into a config value.

`type(current)(raw)` looks like a tidy way to coerce an override, and it is wrong
for bools in the worst possible direction: `bool("0")` is `True`, so a flag set to
`0` in `.env` to turn a feature *off* turns it *on*.

There are no bool fields in `Config` today only because the trap was dodged twice
while writing this batch of work — `use_gyro` was declared `int` specifically to
avoid it, and `SimConfig.from_env` handles bools by hand. Two near-misses is
enough; both should call this instead.
"""
FALSEY = ("", "0", "false", "no", "off")


def coerce(current, raw):
    """Parse `raw` into the type of `current`, failing with a message that names
    the offending value rather than a bare `ValueError` from `float()`."""
    if isinstance(current, bool):
        return raw.strip().lower() not in FALSEY
    try:
        return type(current)(raw)
    except (TypeError, ValueError):
        raise ValueError(
            f"cannot read {raw!r} as {type(current).__name__}") from None
