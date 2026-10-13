"""Carm 原生法兰/夹爪/灵巧手（flange/gripper/hand）的行为等价适配器。"""

import math

from . import EndEffectorBase, EndEffectorContext


class CarmEndEffector(EndEffectorBase):
    """把 provider 归一化 trigger 映射为 Carm 原生 end-effector 位置指令。"""

    NAME = "carm"
    DEFAULT_TORQUE = 25.0

    def __init__(
        self,
        context: EndEffectorContext,
        **options,
    ) -> None:
        """读取 Carm end-effector 类型与限位配置，预计算速度/力矩参数。

        Raises: RuntimeError 配置缺失或字段非法，或 SDK 不支持所需接口。
        """
        super().__init__(context, **options)
        self._type = self._read_end_effector_type()
        self._dof = 0
        self._lower = ()
        self._upper = ()
        self._velocity = ()
        self._torque = ()
        if self._type == "flange":
            return

        get_config = getattr(self.carm, "get_eeff_config", None)
        if not callable(get_config):
            raise RuntimeError("current Carm SDK does not provide get_eeff_config")
        config = get_config()
        if not isinstance(config, dict):
            raise RuntimeError("Carm end-effector configuration must be a dictionary")

        configured_type = str(config.get("eeff_type") or self._type or "").strip().lower()
        if configured_type:
            self._type = configured_type
        dof = config.get("dof")
        if isinstance(dof, bool) or not isinstance(dof, int) or dof < 0:
            raise RuntimeError("Carm end-effector configuration has invalid dof")
        self._dof = dof
        if self._type == "flange" or self._dof == 0:
            return

        self._lower = self._read_vector(config, "eeff_lower", allow_negative=True)
        self._upper = self._read_vector(config, "eeff_upper", allow_negative=True)
        maximum_velocity = self._read_vector(config, "eeff_vel")
        maximum_torque = self._read_vector(config, "eeff_tau")
        if any(lower > upper for lower, upper in zip(self._lower, self._upper)):
            raise RuntimeError("Carm end-effector lower limit exceeds upper limit")

        self._velocity = (
            maximum_velocity if self._type == "hand" else (0.0,) * self._dof
        )
        self._torque = tuple(
            min(self.DEFAULT_TORQUE, maximum) for maximum in maximum_torque
        )
        if not callable(getattr(self.carm, "set_eeff", None)):
            raise RuntimeError("current Carm SDK does not provide set_eeff")

    def _read_end_effector_type(self) -> str:
        """读取并规范化 Carm SDK 暴露的 end-effector 类型。"""
        raw_type = getattr(self.carm, "end_effector_type", None)
        if callable(raw_type):
            raw_type = raw_type()
        return str(getattr(raw_type, "value", raw_type) or "").strip().lower()

    def _read_vector(self, config: dict, key: str, allow_negative: bool = False):
        """读取按 dof 截断的数值向量配置，可选允许负值。"""
        values = config.get(key)
        if not isinstance(values, (list, tuple)) or len(values) < self._dof:
            raise RuntimeError("Carm end-effector configuration has invalid %s" % key)
        normalized = []
        for value in values[:self._dof]:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise RuntimeError("Carm end-effector configuration has invalid %s" % key)
            number = float(value)
            if not math.isfinite(number) or (not allow_negative and number < 0.0):
                raise RuntimeError("Carm end-effector configuration has invalid %s" % key)
            normalized.append(number)
        return tuple(normalized)

    def set_eff(self, trigger: int) -> bool:
        """把 provider 归一化 trigger 映射为 Carm 原生位置并下发。

        Args: trigger 为 provider 归一化后的触发值（按 0-127 归一化）。
        Returns: SDK 确认下发成功时为 True；法兰或 0 自由度配置直接返回 True。
        Raises: ValueError trigger 非数值或非有限值。
        """
        if self._type == "flange" or self._dof == 0:
            return True
        if isinstance(trigger, bool) or not isinstance(trigger, (int, float)):
            raise ValueError("Carm end-effector trigger must be numeric")
        value = float(trigger)
        if not math.isfinite(value):
            raise ValueError("Carm end-effector trigger must be finite")
        ratio = max(0.0, min(value, 127.0)) / 127.0
        position = [
            upper - ratio * (upper - lower)
            for lower, upper in zip(self._lower, self._upper)
        ]
        result = self.carm.set_eeff(
            position,
            list(self._velocity),
            list(self._torque),
        )
        return result is True or result == 1
