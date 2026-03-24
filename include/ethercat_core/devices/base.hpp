#pragma once

#include <any>
#include <cstdint>
#include <string>
#include <vector>

namespace ethercat_core {

struct SlaveIdentity {
    std::string name;
    int         position     = -1;   // 0-based position in the EtherCAT chain
    uint32_t    vendor_id    = 0;
    uint32_t    product_code = 0;
};

// Specifies one SDO object to read during master startup (pre-OP phase).
struct SdoReadSpec {
    std::string name;
    uint16_t    index;
    uint8_t     subindex;
    // Supported data_type strings: "u8","s8","u16","s16","u32","s32","f32","bytes"
    std::string data_type;
};

// Abstract adapter interface — one concrete implementation per device type.
// Each adapter owns the conversion between application-level command/status
// types and the raw PDO byte buffers that SOEM exchanges on every cycle.
class ISlaveAdapter {
public:
    virtual ~ISlaveAdapter() = default;

    // Byte size of the output (master→slave) PDO buffer.
    virtual int rxPdoSize() const = 0;

    // Byte size of the input (slave→master) PDO buffer.
    virtual int txPdoSize() const = 0;

    // Encode a typed command (wrapped in std::any) to raw PDO bytes.
    // Returns a zero-filled vector of rxPdoSize() bytes if command is empty.
    virtual std::vector<uint8_t> packRxPdo(const std::any& command) = 0;

    // Decode raw PDO bytes from the slave into a typed status (returned as std::any).
    virtual std::any unpackTxPdo(
        const uint8_t* data,
        int            size,
        uint64_t       seq           = 0,
        int64_t        stamp_ns      = 0,
        int64_t        cycle_time_ns = 0,
        int64_t        dc_error_ns   = 0
    ) = 0;

    const SlaveIdentity& identity() const { return identity_; }

protected:
    explicit ISlaveAdapter(SlaveIdentity id) : identity_(std::move(id)) {}
    SlaveIdentity identity_;
};

} // namespace ethercat_core
