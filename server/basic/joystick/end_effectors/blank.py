import math

from . import EndEffectorBase, EndEffectorContext


class BlankEndEffector(EndEffectorBase):
    NAME = "blank"

    def __init__(self, context: EndEffectorContext, **options) -> None:
        super().__init__(context, **options)
        sender = getattr(self.carm, "set_blank", None)
        if not callable(sender):
            raise RuntimeError("current Carm SDK does not support blank control")
        self._sender = sender

    def set_eff(self, trigger: int) -> bool:
        if isinstance(trigger, bool) or not isinstance(trigger, (int, float)):
            raise ValueError("blank trigger must be numeric")
        value = float(trigger)
        if not math.isfinite(value):
            raise ValueError("blank trigger must be finite")
        ratio = max(0.0, min(value, 127.0)) / 127.0
        return self._sender(ratio) is True
