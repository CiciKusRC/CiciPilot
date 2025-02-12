# This message is used to dump the raw gps communication to the log.


uint64 timestamp            # time since system start (microseconds)

float32 accelcmd_x          # resultant lateral acceleration reference [m/s^2]
float32 accelcmd_y          # lateral acceleration demand only for maintaining curvature [m/s^2]
float32 los_angle	    # line of sight angle [rad]
uint8 	kamikaze_status	    # current phase of kamikaze navigation
