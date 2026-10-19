#!/usr/bin/env python3
"""Republish PX4 v1.17 versioned topics onto the names Ohho already reads.

PX4 ``generate_topic_name`` appends ``_v<MESSAGE_VERSION>`` when that constant
is non-zero. On tag v1.17.0 and ``px4_msgs`` branch ``release/1.17``,
``VehicleStatus``, ``VehicleLocalPosition``, and ``BatteryStatus`` are version
1, so the firmware publishes ``/fmu/out/vehicle_status_v1`` (and the matching
local-position and battery topics). ``ohho-os`` @ ``ohh-290-px4-dds``
subscribes to the unversioned names.

Data-writer QoS in PX4 v1.17 is best-effort and transient-local. Depth is not
a match rule; this relay uses depth 1, which is what Ohho requests. The
process republishes telemetry only. It does not publish command topics, does
not arm, and does not set the hardware-arming environment variable.
"""

from __future__ import annotations

from typing import Any

# (px4_msgs class name, topic PX4 publishes, topic Ohho subscribes to)
RELAYS: tuple[tuple[str, str, str], ...] = (
    ("VehicleStatus", "/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_status"),
    (
        "VehicleLocalPosition",
        "/fmu/out/vehicle_local_position_v1",
        "/fmu/out/vehicle_local_position",
    ),
    ("BatteryStatus", "/fmu/out/battery_status_v1", "/fmu/out/battery_status"),
)


def _qos() -> Any:
    from rclpy.qos import (  # type: ignore[import-not-found]
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def main() -> None:
    import rclpy  # type: ignore[import-not-found]
    from px4_msgs.msg import (  # type: ignore[import-not-found]
        BatteryStatus,
        VehicleLocalPosition,
        VehicleStatus,
    )

    classes = {
        "VehicleStatus": VehicleStatus,
        "VehicleLocalPosition": VehicleLocalPosition,
        "BatteryStatus": BatteryStatus,
    }
    rclpy.init()
    node = rclpy.create_node("ohho_px4_topic_relay")
    qos = _qos()
    try:
        for name, source, dest in RELAYS:
            publisher = node.create_publisher(classes[name], dest, qos)

            def _forward(message: Any, pub: Any = publisher) -> None:
                pub.publish(message)

            node.create_subscription(classes[name], source, _forward, qos)
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
