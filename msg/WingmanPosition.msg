# WingmanPosition.msg
uint64 timestamp			  # time since system start (microseconds)
uint8 target_system                       # System ID

float32 wingman_latitude                  # Wingman Latitude (WGS84), degE7
float32 wingman_longitude                 # Wingman Longitude (WGS84), degE7
float32 wingman_altitude                  # Wingman Altitude (MSL, mm). Positive for up.
float32 wingman_heading                   # Wingman Heading (Yaw angle), 0.
