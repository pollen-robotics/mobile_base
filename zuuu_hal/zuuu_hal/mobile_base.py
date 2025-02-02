from pyvesc.VESC import MultiVESC
from typing import Deque, Optional, Any
from collections import deque


class MobileBase:
    """Mobile base representation and the interface with low level controllers."""

    def __init__(
        self,
        serial_port: str = "/dev/vesc_wheels",
        left_wheel_id: Optional[int] = 24,
        right_wheel_id: Optional[int] = 72,
        back_wheel_id: Optional[int] = 116,
        fake_hardware: bool = False,
    ) -> None:
        params = [
            {"can_id": left_wheel_id, "has_sensor": True, "start_heartbeat": True},
            {"can_id": right_wheel_id, "has_sensor": True, "start_heartbeat": True},
            {"can_id": back_wheel_id, "has_sensor": True, "start_heartbeat": True},
        ]
        self.fake_hardware: bool = fake_hardware

        # Battery parameters. These values are conservative since the battery cells are nearly empty around 3.3V.
        # The current battery's BMS shuts down at 20V ±1V, so each cell would be ~2.86V ±0.14V.
        self.battery_cell_warn_voltage: float = 3.5
        self.battery_cell_min_voltage: float = 3.3
        self.battery_nb_cells: int = 7
        self.battery_check_period: int = 60

        # Wheel measurement counters and physical parameters.
        self.left_wheel_nones: int = 0
        self.right_wheel_nones: int = 0
        self.back_wheel_nones: int = 0
        self.wheel_radius: float = 0.21 / 2.0
        self.wheel_to_center: float = 0.19588
        self.half_poles: float = 15.0

        # Wheel RPM values.
        self.left_wheel_rpm: float = 0.0
        self.right_wheel_rpm: float = 0.0
        self.back_wheel_rpm: float = 0.0
        self.left_wheel_avg_rpm: float = 0.0
        self.right_wheel_avg_rpm: float = 0.0
        self.back_wheel_avg_rpm: float = 0.0

        # Deques to hold recent RPM measurements.
        self.left_wheel_rpm_deque: Deque[float] = deque(maxlen=10)
        self.right_wheel_rpm_deque: Deque[float] = deque(maxlen=10)
        self.back_wheel_rpm_deque: Deque[float] = deque(maxlen=10)

        # Initialize measurements depending on whether hardware is fake.
        init_measurements_value: Optional[str] = (
            "No measurements in fake_hardware mode" if fake_hardware else None
        )
        self.left_wheel_measurements: Optional[Any] = init_measurements_value
        self.right_wheel_measurements: Optional[Any] = init_measurements_value
        self.back_wheel_measurements: Optional[Any] = init_measurements_value

        if not self.fake_hardware:
            self._multi_vesc = MultiVESC(serial_port=serial_port, vescs_params=params)
            (
                self.left_wheel,
                self.right_wheel,
                self.back_wheel,
            ) = self._multi_vesc.controllers

    def read_all_measurements(self) -> None:
        """Reads all the measurements for the left, right, and back wheels."""
        if self.fake_hardware:
            return
        self.left_wheel_measurements = self.left_wheel.get_measurements()
        self.right_wheel_measurements = self.right_wheel.get_measurements()
        self.back_wheel_measurements = self.back_wheel.get_measurements()

    def deque_to_avg(self, dq: Deque[float]) -> float:
        """Returns the average of the values contained in the deque."""
        if not dq:
            raise ValueError("Deque is empty; cannot compute average.")
        total = 0.0
        for value in dq:
            total += value
        return total / len(dq)