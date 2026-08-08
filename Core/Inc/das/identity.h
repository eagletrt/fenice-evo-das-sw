#ifndef IDENTITY_H
#define IDENTITY_H

#define IDENTITY_VERSION_MAJOR (0U)
#define IDENTITY_VERSION_MINOR (1U)
#define IDENTITY_VERSION_PATCH (0U)

#define IDENTITY_VERSION_DIRTY (0U)

#define IDENTITY_VERSION_INFO_COMMIT_HASH (0x9ABCDEF)

#include <stdint.h>

enum IdentityReturnCode {
    IDENTITY_RC_OK,
    IDENTITY_RC_ERROR,
};

struct IdentityHandler {
    uint32_t build_time;
    uint32_t last_tick_ms_status;
    uint32_t last_tick_ms_version;
    uint32_t last_tick_ms_libcan_version;
};

#endif // IDENTITY_H
