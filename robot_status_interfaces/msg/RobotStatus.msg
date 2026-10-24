# Snapshot of the (simulated) robot's health, published periodically.

# Overall health level
uint8 LEVEL_OK=0
uint8 LEVEL_WARN=1
uint8 LEVEL_ERROR=2

std_msgs/Header header
float32 battery_percentage    # 0.0 - 100.0
float32 battery_voltage       # volts (simulated 3S LiPo: 10.5 V empty .. 12.6 V full)
float32 temperature_celsius
uint8 level                   # one of the LEVEL_* constants above
string message                # human-readable summary
