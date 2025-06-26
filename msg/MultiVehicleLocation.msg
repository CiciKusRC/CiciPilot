# MultiVehicleLocation.msg
# Sets the MULTIVEHICLE coordinates of the vehicle local origin (0,0,0) position.
uint64 timestamp			  # time since system start (microseconds)
uint8 target_system                       # System ID

int32 vehicle_1_latitude                  # Vehicle 1 Latitude (WGS84), degE7
int32 vehicle_1_longitude                 # Vehicle 1 Longitude (WGS84), degE7
int32 vehicle_1_altitude                  # Vehicle 1 Altitude (MSL, mm). Positive for up.
int32 vehicle_1_heading                   # Vehicle 1 Heading (Yaw angle), 0.0..359.99 degrees
int32 vehicle_2_latitude                  # Vehicle 2 Latitude (WGS84), degE7
int32 vehicle_2_longitude                 # Vehicle 2 Longitude (WGS84), degE7
int32 vehicle_2_altitude                  # Vehicle 2 Altitude (MSL, mm). Positive for up.
int32 vehicle_2_heading                   # Vehicle 2 Heading (Yaw angle), 0.0..359.99 degrees
int32 vehicle_3_latitude                  # Vehicle 3 Latitude (WGS84), degE7
int32 vehicle_3_longitude                 # Vehicle 3 Longitude (WGS84), degE7
int32 vehicle_3_altitude                  # Vehicle 3 Altitude (MSL, mm). Positive for up.
int32 vehicle_3_heading                   # Vehicle 3 Heading (Yaw angle), 0.0..359.99 degrees

uint64 time_usec                          # Timestamp (UNIX Epoch time or time since system boot, us)
