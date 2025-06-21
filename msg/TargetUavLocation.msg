# TargetUavLocation.msg
# Sets the TARGET coordinates of the vehicle local origin (0,0,0) position.
uint64 timestamp            # Time since system start (microseconds)
uint8 target_system           # System ID
int32 latitude                # Latitude (WGS84), in degrees * 1E7
int32 longitude               # Longitude (WGS84), in degrees * 1E7
int32 altitude                # Altitude (MSL), in millimeters. Positive for up.
uint64 time_usec              # Timestamp (UNIX Epoch time or time since system boot) in microseconds.
