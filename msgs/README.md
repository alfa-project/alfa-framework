# ALFA Messages

This ROS 2 package defines custom message and service types used internally by the ALFA framework. These definitions are used to support node discovery, dynamic configuration, and runtime metric reporting between ALFA components.

## Message Definitions

### `MetricMessage.msg`

Represents a single performance or system metric.

- `string metric_name` — Name of the metric
- `float64 metric` — Measured value
- `string units` — Units associated with the metric (e.g., ms, MB/s)

### `ConfigMessage.msg`

Encapsulates a configuration parameter.

- `string config_name` — Name of the parameter
- `float64 config` — Value to be applied

### `AlfaMetrics.msg`

A container for multiple metrics reported by a node.

- `string message_tag` — Identifier for the metric report (e.g., timestamp, frame ID)
- `MetricMessage[] metrics` — List of metrics reported

### `AlfaAlivePing.msg`

Heartbeat message sent by each ALFA node to advertise its presence and capabilities.

- `string node_name` — Name of the node
- `string node_type` — Role or type of the node (e.g., Extension, Sensor, Manager)
- `string config_service_name` — Name of the configuration service the node exposes
- `int8 current_status` — Node status code (custom-defined per system)
- `string config_tag` — Identifier for the current configuration set
- `ConfigMessage[] default_configurations` — List of default configuration parameters

## Service Definition

### `AlfaConfigure.srv`

Used to configure or reconfigure an ALFA node at runtime.

**Request:**

- `string node_name`  
- `string node_type`  
- `string config_service_name`  
- `int8 current_status`  
- `string config_tag`  
- `ConfigMessage[] default_configurations`

**Response:** *(no fields defined — service acts as a trigger)*