# ALFA Messages

This package defines the message and service interfaces used across the ALFA project for communication between nodes. It is built on top of the ROS 2 interface definition system and is compatible with the `ament` build system.

## Features

- Custom message types for configuration, status monitoring, and metrics.
- Service interface for dynamic runtime configuration of ALFA nodes.
- Well-structured definitions to facilitate ROS 2 communication and introspection.

## Message Definitions

### `AlfaAlivePing.msg`

Used by ALFA nodes to broadcast their presence and basic runtime status.

Fields:
- `string node_name`
- `string node_type`
- `string config_service_name`
- `int8 current_status`
- `string config_tag`
- `ConfigMessage[] default_configurations`

### `AlfaMetrics.msg`

Broadcasted periodically or on-demand to report node-specific performance metrics.

Fields:
- `string message_tag`
- `MetricMessage[] metrics`

### `ConfigMessage.msg`

Represents a single configuration parameter and its value.

Fields:
- `string config_name`
- `float64 config`

### `MetricMessage.msg`

Encapsulates a single metric with its name, value, and units.

Fields:
- `string metric_name`
- `float64 metric`
- `string units`

## Service Definition

### `AlfaConfigure.srv`

Allows reconfiguration of a node by passing a list of new configuration values.

Request:
- `ConfigMessage[] request`

Response:
- `bool success`
- `string status_message`
