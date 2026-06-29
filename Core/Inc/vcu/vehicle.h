#ifndef VEHICLE_H
#define VEHICLE_H

enum VehicleReturnCode {
    VEHICLE_RC_OK,
    VEHICLE_RC_NULL_POINTER,
};

enum VehicleRequestedState {
    VEHICLE_REQUESTED_STATE_IDLE = 0,
    VEHICLE_REQUESTED_STATE_READY = 1,
    VEHICLE_REQUESTED_STATE_DRIVE = 2,
};

struct VehicleHandler {
    enum VehicleRequestedState requested_state;

    bool higher_than_60v;

    bool shutdown_end_closed;
};

#endif // VEHICLE_H
