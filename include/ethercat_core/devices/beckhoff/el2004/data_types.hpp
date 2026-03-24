#pragma once

#include <cstdint>

namespace ethercat_core::beckhoff::el2004 {

struct Command {
    bool output_1 = false;
    bool output_2 = false;
    bool output_3 = false;
    bool output_4 = false;
};

struct Status {
    uint8_t output_byte = 0;
};

} // namespace ethercat_core::beckhoff::el2004
