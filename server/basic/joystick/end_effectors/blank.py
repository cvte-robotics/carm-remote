"""空白 end-effector：只校验 trigger，不驱动任何硬件，用于占位与联调。"""

import math

from . import EndEffectorBase, EndEffectorContext


class BlankEndEffector(EndEffectorBase):
    """不产生硬件副作用的示例 end-effector 实现。"""

    NAME = "blank"

    def __init__(self, context: EndEffectorContext, **options) -> None:
        """绑定 context（无额外状态）。"""
        super().__init__(context, **options)

    def set_eff(self, trigger: int) -> bool:
        """校验 trigger 后打印并直接返回。

        Args: trigger 为 provider 归一化后的触发值。
        Returns: 恒为 True。
        Raises: ValueError trigger 不是数值或不是有限值。
        """
        ## 校验trigger值是否为数字类型且有限
        if isinstance(trigger, bool) or not isinstance(trigger, (int, float)):
            raise ValueError("blank trigger must be numeric")
        value = float(trigger)
        if not math.isfinite(value):
            raise ValueError("blank trigger must be finite")

        ## 这里可以添加自定义的处理逻辑，例如打印触发值等等
        print("Blank end-effector received trigger:", value)
        return True
