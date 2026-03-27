#pragma once

#include <cstdint>
#include <vector>

namespace ethercat_core::beckhoff::el5032 {

// PDO object: 0x6000:0x11 encoder value, mapped via 0x1A00.
// TX PDO is 10 bytes.  Encoder count is 25 bits starting at bit 16.
static constexpr int      TX_PDO_SIZE            = 10;
static constexpr uint32_t ENCODER_MASK_25BIT      = (1u << 25) - 1u;
static constexpr uint16_t PDO_INDEX               = 0x1A00;
static constexpr uint16_t REGISTER_INDEX          = 0x6000;
static constexpr uint8_t  REGISTER_SUBINDEX       = 0x11;

struct Command {};  // encoder terminal — no output

struct Data {
    int32_t              encoder_value_raw    = 0;
    uint32_t             encoder_count_25bit  = 0;
    std::vector<uint8_t> raw_pdo;
};

} // namespace ethercat_core::beckhoff::el5032
