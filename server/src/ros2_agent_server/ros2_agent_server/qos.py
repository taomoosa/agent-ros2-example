"""Service history for concurrent HTTP requests and their burst responses."""

import copy

from rclpy.qos import qos_profile_services_default


def http_service_qos():
    profile = copy.copy(qos_profile_services_default)
    # The default depth of ten can lose the first requests/responses in a
    # twelve-camera burst even with reliable delivery. Keep more history on
    # both the HTTP gateway client and bridge service without changing DDS defaults.
    profile.depth = 64
    return profile
