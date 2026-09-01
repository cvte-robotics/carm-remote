import math

from . import EndEffectorBase, EndEffectorContext


class BlankEndEffector(EndEffectorBase):
    NAME = "blank"

    def __init__(self, context: EndEffectorContext, **options) -> None:
        super().__init__(context, **options)

    def set_eff(self, trigger: int) -> bool:
        ## 校验trigger值是否为数字类型且有限
        if isinstance(trigger, bool) or not isinstance(trigger, (int, float)):
            raise ValueError("blank trigger must be numeric")
        value = float(trigger)
        if not math.isfinite(value):
            raise ValueError("blank trigger must be finite")

        ## 这里可以添加自定义的处理逻辑，例如打印触发值等等
        print("Blank end-effector received trigger:", value)
        return True
