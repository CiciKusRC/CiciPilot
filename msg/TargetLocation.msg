# This message is used to dump the raw gps communication to the log.


uint64 timestamp            # time since system start (microseconds)
float32 target_x	    # exit latitude [deg]
float32 target_y	    # exit longitude [deg]
float32 target_z	    # exit altitude [m]
float32 tracker_status         # tracker status, true if the target is being tracked, false otherwise
