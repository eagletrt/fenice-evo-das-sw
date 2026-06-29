#ifndef TSAC_H
#define TSAC_H

#include <stdint.h>

#define TSAC_MAX_START_PRECHARGE_TIME_MS (25000)
#define TSAC_MAX_PRECHARGE_TIME_MS (30000)

enum TsacReturnCode {
    TSAC_RC_OK,
    TSAC_RC_ERROR,
    TSAC_RC_INVALID_STATUS,
};

enum TsacStatus : uint8_t {
    TSAC_STATUS_OFF,
    TSAC_STATUS_PRECHARGING,
    TSAC_STATUS_ON,
    TSAC_STATUS_FATAL,
    TSAC_STATUS_UNKNOWN,
    TSAC_STATUS_COUNT,
};

struct TsacHandler {
    enum TsacStatus status;
};

#endif // TSAC_H
