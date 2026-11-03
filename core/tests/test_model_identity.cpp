// Model unit: the directory, class blocks, identity (FC11 / CTCL), the identification block, soft limits, strokes, the
// class-7 limits, stored rows and the ranges the driver enforces. A MapReader over register words copied from the units
// tested (rod EC-R6S, gripper EC-GRBP8M, PCON-CB + RCP6: VERIFIED reads in the Python edition) stands in for the bus;
// this edition itself is UNVERIFIED on hardware.
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/directory.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/reader.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "harness.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;

namespace {

/// A unit's registers as a map of words; an unmapped address reads 0000 (the unit does not reject it).
class MapReader : public UnitReader {
public:
    explicit MapReader(std::map<int, std::uint16_t> words, int slave = 1) : words_(std::move(words)), slave_(slave) {}

    int slave() const override { return slave_; }

    std::vector<std::uint16_t> read_registers(int address, int count) override {
        ++reads;
        for (int i = 0; i < count; ++i) {
            const int a = address + i;
            if (timeout.count(a)) throw ModbusTimeout("no answer at " + util::hex(static_cast<std::uint64_t>(a)));
            if (refuse.count(a)) throw ModbusSlaveError(2, 3);
            if (transport_error.count(a)) throw TransportError("port gone");
        }
        std::vector<std::uint16_t> out;
        for (int i = 0; i < count; ++i) {
            const auto it = words_.find(address + i);
            out.push_back(it == words_.end() ? 0 : it->second);
        }
        return out;
    }

    std::vector<std::uint8_t> report_slave_id() override {
        if (fc11_error == 1) throw ModbusTimeout("no answer");
        if (fc11_error == 2) throw ModbusSlaveError(1, 0x11);
        if (fc11_error == 3) throw TransportError("port gone");
        return slave_id;
    }

    void set(int address, std::uint16_t value) { words_[address] = value; }
    void set32(int address, std::uint32_t value) {
        words_[address] = static_cast<std::uint16_t>(value >> 16);
        words_[address + 1] = static_cast<std::uint16_t>(value & 0xFFFF);
    }
    void refuse_range(int first, int last) {
        for (int a = first; a <= last; ++a) refuse.insert(a);
    }

    std::vector<std::uint8_t> slave_id;
    int fc11_error = 0;  // 0 answers, 1 silent, 2 refused, 3 transport failure
    std::set<int> refuse;
    std::set<int> timeout;
    std::set<int> transport_error;
    int reads = 0;

private:
    std::map<int, std::uint16_t> words_;
    int slave_;
};

// ---- generated fixtures: register words copied from FC03 reads of the units tested (rod EC-R6S, gripper EC-GRBP8M,
// PCON-CB + RCP6), taken from the Python edition's sim_fixtures.py (serial numbers replaced by placeholders there).
struct HexBlock {
    int address;
    const char* hex;
};

const char* const kRodIndex =
    "000100080016010001000000000000000001010000060001010000000000000000020002000A00010200028001000001"
        "00030004003A000103000380000000020004000200150001040004800000000200050002000300200500580000000002"
        "0006000100240001060000000000000000070102001A0001070051000000000200080002002400010800520000000002"
        "00090002003400010900530000000002000C0000000200020C00560000000002000D0002000A00010D000D8004000001"
        "000F0100000D00010F000000000000000010010100080300100050000000000200120002000700017F00000000000000"
        "010000020010000180000000000000000013000000100001810000000000000000140000001000018200000000000000"
        "02000000001A000184008480000000020101000300160001900000000000000001020001000300019800000000000000"
        "0103000100070001990000000000000001040000000300019A00000000000000";
const HexBlock kRodBlocks[] = {
    {0x0100, "60000BD700000317008000004300009A"},
    {0x0280, "00010000000200000003000000040000000500000006000000070000000800000009000000100000"},
    {0x0300,
        "EC000002000000020000000300000000000000250000384000000000000265450008001F000000000000076600000684"
        "0000054A000005AA000007AB0000000200030003000000000000000000000000000000000000000000000000000000FF"
        "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
        "000000000000000000000000EC800000000003E8000000000000000000000000000000000000000000000000000000F3"
        "0000010400000018000000F30000034F000009C40000000800000000000000D5FFFFFFFF00000000"},
    {0x0380,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000B0000000C0000"
        "000D0000000E0000000F00000010000000110000001B0000001C0000001D0000001E0000001F00000020000000210000"
        "0022000000230000002400000025000000260000002700000028000000290000002A0000002B0000002C0000002D0000"
        "002E0000002F0000003000000031000000340000003700000038000000390000003A0000003B0000003C0000003D0000"
        "003E0000003F000000400000004100000042000000430000004500000046000000490000004A0000"},
    {0x0400,
        "0000000000000000000000000000021A000004B0000055F000005DC00000000100000000000000000000000000000000"
        "00000BB80000000000000000000000000000000000000000000000000000000B00000000"},
    {0x0480,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000C000000190000"
        "001A0000001B0000001C0000001D0000001E0000001F0000002000000028000000290000"},
    {0x0600,
        "53494D2D30303030303033202020202053494D2D303030303030332020202020303030300000000020170406B2002380"
        "0000000000000000000000000000000045432D5236532D3135302D312D504E2020202020202020202020202020202020"
        "202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020"},
    {0x0700,
        "00003AB6FFFFFFE2000007D000013880000000640000006405350000000007D00000004600000000014A007400000096"
        "0000177000001770FFFFFFFFFFFFFFFF000001F40000003200000064000000640000001E000001F4000003E800000003"
        "0000003800000000"},
    {0x0800,
        "20400000000000000000000600000005000000000000000000000000000100000000000A000138800000001E00000059"
        "000000FF00000000000007D0000027100000001E000000FF00000000000000000000271000002710000000640000000A"
        "0000001C000000140000005A000000000000000000000000000100000000000000000000000000000000000000000000"},
    {0x0900,
        "000A001E00000016000000FF000006A4000006A400000007000002FB000024F200000000000000330000000000000320"
        "00000014000000140901008C00000007000007D00000001E000000B30002BF200000012C0000019000001770000007D0"
        "000000FF000000FF000000000000000000000001000000C80000000F00000000000000B2000000000000003200000000"
        "000000000000007D0000003C00002710000003E8000006400000000000000C2500004A5A0000006400000023000002FB"
        "000024F20000001F000006A400010000"},
    {0x0C00, "00003AB6FFFFFFE200003AB6FFFFFFE2"},
    {0x0D80, "000100000002000000030000000400000005000000060000000700000008000000090000000C0000"},
    {0x5000, "000100000003000000040000000C0000000D0000000E0000000F000000100000"},
    {0x5100,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000F000000100000"
        "001100000012000000130000001400000015000000160000001700000018000000190000001A0000001D000000210000"
        "0022000000230000"},
    {0x5200,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000B0000000C0000"
        "000D0000000E0000000F0000001000000011000000120000001300000014000000150000001600000017000000180000"
        "0021000000220000002A0000002C0000002D00000032000000360000003B0000003C000000420000004E000000550000"},
    {0x5300,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000B0000000C0000"
        "000D0000000E0000000F0000001000000011000000120000001300000014000000150000001600000017000000180000"
        "00190000001A0000001B0000001C0000001D0000001E0000001F000000210000002400000027000000280000002A0000"
        "00320000003300000034000000380000004B000000620000006300000064000000650000006600000068000000690000"
        "006A0000006B000000770000007D0000"},
    {0x5600, "00010000000200000001000000020000"},
    {0x5800, "000100000002000000030000"},
    {0x8400,
        "0000388F00000690000000F6000000F6000001B3000294410000003200200029FFFFFFFF00640064FFFFFFFFFFFFFFFF"
        "00000019FFFFFFFF0000388F00000690FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000000000000FFFFFFFF00000000"
        "0000000000000000"},
    {0x8480,
        "0001000000020000000300000004000000050000000700000008000000090000000A0000000B0000000C0000000D0000"
        "000E0000000F0000001000000011000000120000001300000014000000150000001700000018000000190000001B0000"
        "001C0000001D0000"},
};

const char* const kGripperIndex =
    "0001000800190100010000000000000000010100000B0001010000000000000000020002000A00010200028001000001"
        "00030004003A000103000380000000020004000200130001040004800000000200050002000300200500580000000002"
        "000600010024000106000000000000000007010200160001070051000000000200080002002200010800520000000002"
        "00090002002A00010900530000000002000C0000000200020C00560000000002000D0002000A00010D000D8004000001"
        "000F0100000D00010F000000000000000010010100080300100050000000000200120002000500017F00000000000000"
        "010000020010000180000000000000000013000000100001810000000000000000140000001000018200000000000000"
        "020000000017000184008480000000020101000300160001900000000000000001090001000E00019100000000000000"
        "010200010003000198000000000000000103000100070001990000000000000001040000000300019A00000000000000"
        "010A0000000600019B0000000000000001110000000500019D00000000000000";
const HexBlock kGripperBlocks[] = {
    {0x0100, "60002BD700001317008000004384009A0000000000000000000000000000000000000000"},
    {0x0280, "00010000000200000003000000040000000500000006000000070000000800000009000000100000"},
    {0x0300,
        "EC0A00020000000200000007000000000000001F0055B6DB00000000000BBF0B0008801F0000000000000553000008AF"
        "0000071E0000000300040004000000000000000000000000000000000000000000000000000005FF0000000000000000"
        "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
        "00000000EC830001000003E8000000000000000000000000000000000000000000000000000000F3000000EC00000022"
        "000000EC000000770000000600000001FFFFFFFFFFFFFFFD0000000800000464000000DB00000001"},
    {0x0380,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000D0000000E0000"
        "000F00000010000000110000001B0000001C0000001D0000001E0000001F000000200000002100000022000000230000"
        "002400000025000000260000002700000028000000290000002A0000002B0000002C0000002D0000002E0000002F0000"
        "003000000031000000340000003700000038000000390000003A0000003B0000003C0000003D0000003E0000003F0000"
        "0040000000410000004200000043000000490000004A000000650000006D00000072000000750000"},
    {0x0400,
        "0000012C0000000000000000000000000000028A00000D4800005DC000000001FFFF4925000000000000B6DB00000BB8"
        "0000012C000000000000000000000000000000000000003800000038"},
    {0x0480,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A000000190000001A0000"
        "001B0000001D0000001E000000280000002900000037000000400000"},
    {0x0600,
        "53494D2D30303030303032202020202053494D2D30303030303032202020202030413031000000012024112600000709"
        "0000000045432D47524250384D2D323045432D47524250384D2D32302D312D312D504E2D57412D574C32202020202020"
        "202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020"},
    {0x0700,
        "000003D4FFFFFFE200000069000014B40000001E0000001E00080000000001F40000006400000000FFFFFFFF00000000"
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000001E000003E800000004000000240000000000000032"},
    {0x0800,
        "20400000000000000000000600000005000000000000000000000000000100000000000A000014B40000001E0000007F"
        "0000016700000400000001F4000014B40000001E000000FF0000000000000000000014B4000014B4000000640000000A"
        "18191A10292A2B100000005F00000000000000000000800800010000000000000000000000000000"},
    {0x0900,
        "000000460000073A000000070000001D000001630000000000000000000000080000400000000014000000640D01208C"
        "00000007000001F40000001E000000FF0002BF20000000320000986100020000000007D0000000FF0000016900000000"
        "0000000000000000000000B20023000000000000000000320000B861000000640001000A000000000000000000000000"
        "000000000000000000000000000000CC0000000100007530"},
    {0x0C00, "00000406FFFFFFE200000406FFFFFFE2"},
    {0x0D80, "000100000002000000030000000400000005000000060000000700000008000000090000000C0000"},
    {0x5000, "000100000003000000040000000C0000000D0000000E0000000F000000100000"},
    {0x5100,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000F000000100000"
        "0011000000120000001300000014000000190000001D0000002100000022000000230000003A0000"},
    {0x5200,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000B0000000C0000"
        "000D0000000E0000000F0000001000000011000000120000001300000014000000150000001600000017000000180000"
        "00190000001D0000002A0000002C0000002D00000032000000360000003B0000003C0000004E0000"},
    {0x5300,
        "000300000005000000060000000700000008000000090000000A0000000B0000000C0000000D0000000E0000000F0000"
        "00100000001100000012000000130000001400000015000000160000001700000018000000190000001A0000001B0000"
        "001C000000210000002400000026000000270000002800000062000000660000007D0000009100000092000000930000"
        "00940000009500000096000000990000009A0000009B0000"},
    {0x5600, "0001000000020000"},
    {0x5800, "000100000002000000030000"},
    {0x8400,
        "000026180000005800000103000000FC0001DA45000000030024002DFFFFFFFF000F001EFFFFFFFFFFFFFFFF00000057"
        "FFFFFFFF0000261800000058FFFFFFFFFFFFFFFFFFFFFFFF0000000000000000FFFFFFFF0000000000000000"},
    {0x8480,
        "00010000000200000004000000050000000700000008000000090000000A0000000B0000000C0000000D0000000E0000"
        "000F00000010000000110000001200000014000000150000001700000018000000190000001C0000001D0000"},
};

const char* const kPconIndex =
    "000100080016010001000000000000000001010000060001010000000000000000020002000A00010200028001000001"
        "00030004003E000103000380000000020004000200150001040004800000000200050002000300200500580000000002"
        "000600010020000106000000000000000007010200180001070051000000000200080002003500010800520000000002"
        "00090002003400010900530000000002000C0000000200020C00560000000002000D0002000A00010D000D8004000001"
        "000F0100000D00010F000000000000000010010100080300100050000000000200120002000700017F00000000000000"
        "010000020010000180000000000000000013000000100001810000000000000000140000001000018200000000000000"
        "02000000001B000184008480000000020101000300160001900000000000000001020001000300019800000000000000"
        "0103000100070001990000000000000001040000000300019A00000000000000";
const HexBlock kPconBlocks[] = {
    {0x0100, "6010CB7F0000011701880000030E00BE0000000000000000"},
    {0x0280, "00010000000200000003000000040000000500000006000000070000000800000009000000100000"},
    {0x0300,
        "AE440000000000400000007700000000000000220002D00040100300000161CA20888019000000690000081E000007FE"
        "000005AB00000591000007DB000000050010001000000000000000000000000000000000000000000000000000000000"
        "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
        "000000000000000000000000AE8200000000002200020C01000007D00000005910000804000002000000000000000000"
        "000000000000000000000104000000FF000000150000010100000000000007D000000001000000000000001100000000"
        "FFFFFFFF00000003"},
    {0x0380,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000B0000000C0000"
        "000D0000000E0000000F00000010000000110000001B0000001C0000001D0000001E0000001F00000020000000210000"
        "0022000000230000002400000025000000260000002700000028000000290000002A0000002B0000002C0000002D0000"
        "002E0000002F0000003000000031000000320000003300000034000000350000003700000038000000390000003A0000"
        "003B0000003C0000003D0000003E0000003F000000400000004100000042000000430000004500000046000000470000"
        "00490000004A0000"},
    {0x0400,
        "FFFFFEEC000000000000000000000000000004B0000036B000005DC000000001FFFFFA7B0000000000000000FFFFFA7B"
        "00000BB8FFFFFEECFFFFFA7B000000000000000000000000000000000000000200000000"},
    {0x0480,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000C000000190000"
        "001A0000001B0000001C0000001D0000001E0000001F0000002000000028000000290000"},
    {0x0600,
        "53494D2D30303030303035202020202053494D2D30303030303035202020202030443034000000002016031860110577"
        "00000000000000000000000000000000000000B8000000DC00000087000000000000000A000000DD0000000000000000"
        "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"},
    {0x0700,
        "00003AB6FFFFFFE200000640000148200000006400000064053D0000000007D0000000460000000000BF004100000096"
        "0000C3500000C3500000C3500000C350000001F40000003200000064000000640000001E000001F4000003E800000000"},
    {0x0800,
        "0C400000000000000000000600000005000000000000000000000000000100000000000A000148200000001E00000059"
        "000000FF00000000000007D0000027100000001E000000FF00000000000000000000271000002710000000640000000A"
        "00000000000000000000000000000000000000000000000000000000000000000000001C000000140000005A00000000"
        "0000000000000000000000000000000000000007000000000000000100010001C0A80001FFFFFF000000000000000000"
        "00000000FFFFFFFF000000000000000000000000"},
    {0x0900,
        "0002001E0000000E000000FF000006A4000006A400000007000001F600001442000000000000001A0000004800002000"
        "0000001400000014090024800000000800000FA00000001E000000590002BF20000001900000280000010000000007D0"
        "000000FF000000FF000000000000000000000001000000C80000000F00000000000000B2000000000000000000000000"
        "000000000000007D0000003C00002710000003E8000040000000000000000546000020490000006400000016000001F6"
        "000014420000001F000006A400010000"},
    {0x0C00, "00003AB6FFFFFFE200003AB6FFFFFFE2"},
    {0x0D80, "000100000002000000030000000400000005000000060000000700000008000000090000000C0000"},
    {0x5000, "000100000003000000040000000C0000000D0000000E0000000F000000100000"},
    {0x5100,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000F000000100000"
        "001100000012000000130000001400000015000000160000001700000018000000190000001A0000001D0000001E0000"},
    {0x5200,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000B0000000C0000"
        "000D0000000E0000000F0000001000000011000000120000001300000014000000150000001600000017000000180000"
        "00190000001A0000001B0000001C0000001D0000001E0000001F0000002000000021000000220000002A0000002C0000"
        "002D0000002E0000002F00000030000000310000003200000035000000360000003700000038000000390000003B0000"
        "003C0000003F000000420000004E000000550000"},
    {0x5300,
        "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000B0000000C0000"
        "000D0000000E0000000F0000001000000011000000120000001300000014000000150000001600000017000000180000"
        "00190000001A0000001B0000001C0000001D0000001E0000001F000000210000002400000027000000280000002A0000"
        "00320000003300000034000000380000004B000000620000006300000064000000650000006600000068000000690000"
        "006A0000006B000000770000007D0000"},
    {0x5600, "0001000000020000"},
    {0x5800, "000100000002000000030000"},
    {0x7F00, "0C0100000000000000000000000000FF000000000000000000000000"},
    {0x8000,
        "8000000064510000000000000801D00300000003000000041EA8F31C0000002250434F4E202020204342202028464229"
        "00000000000000008CC24C62000000A7"},
    {0x8100, "010102020303040405050606000000000000171716160F0F0D0D0B0B10100E0E"},
    {0x8200, "0101020203030404050506060D0D12121A1A1C1C0F0F0B0B0E0E1B1B10101F1E"},
    {0x8400,
        "00006F7600000BDA00000104000001050000019C000213E8000000000024002A1FE4AF240031001E000000D61EE38009"
        "0000000F000000E200006F7600000BDA0000006300000060000000001EF613450000000000000000FFFFFFFFFFFFFFFF"
        "000000000000000000000000"},
    {0x8480,
        "0001000000020000000300000004000000050000000700000008000000090000000A0000000B0000000C0000000D0000"
        "000E0000000F0000001000000011000000120000001300000014000000150000001700000018000000190000001B0000"
        "001C0000001D0000001E0000"},
};

std::map<int, std::uint16_t> fixture_map(const char* index_hex, const HexBlock* blocks, std::size_t count) {
    std::map<int, std::uint16_t> words;
    auto put = [&](int address, const char* hex) {
        const std::vector<std::uint16_t> w = iai_actuator_driver::util::hex_words(hex);
        for (std::size_t i = 0; i < w.size(); ++i) words[address + static_cast<int>(i)] = w[i];
    };
    put(0, index_hex);
    for (std::size_t i = 0; i < count; ++i) put(blocks[i].address, blocks[i].hex);
    return words;
}


MapReader make_unit(const char* profile) {
    std::map<int, std::uint16_t> words;
    std::vector<std::uint8_t> id;
    const std::string p = profile;
    if (p == "rod") {
        words = fixture_map(kRodIndex, kRodBlocks, sizeof(kRodBlocks) / sizeof(kRodBlocks[0]));
        id = {0xEC, 0xFF, 0x00, 0x00, 0x02};
    } else if (p == "gripper") {
        words = fixture_map(kGripperIndex, kGripperBlocks, sizeof(kGripperBlocks) / sizeof(kGripperBlocks[0]));
        id = {0xEC, 0xFF, 0x0A, 0x00, 0x02};
    } else {
        words = fixture_map(kPconIndex, kPconBlocks, sizeof(kPconBlocks) / sizeof(kPconBlocks[0]));
        id = {0xAE, 0xFF, 0x44, 0x00, 0x00};
    }
    MapReader reader(std::move(words));
    reader.slave_id = id;
    return reader;
}

bool contains(const std::vector<std::string>& list, const std::string& part) {
    for (const std::string& s : list)
        if (s.find(part) != std::string::npos) return true;
    return false;
}

}  // namespace

// The register words of a profile ("rod", "gripper", "pcon"), shared with the other model test files.
std::map<int, std::uint16_t> model_fixture_words(const std::string& profile) {
    if (profile == "rod") return fixture_map(kRodIndex, kRodBlocks, sizeof(kRodBlocks) / sizeof(kRodBlocks[0]));
    if (profile == "gripper")
        return fixture_map(kGripperIndex, kGripperBlocks, sizeof(kGripperBlocks) / sizeof(kGripperBlocks[0]));
    return fixture_map(kPconIndex, kPconBlocks, sizeof(kPconBlocks) / sizeof(kPconBlocks[0]));
}

// ------------------------------------------------------------------------------------------ directory

TEST(model_directory_index_header_and_records) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    REQUIRE(dir.header().has_value());
    CHECK_EQ(dir.header()->model_code, 1);
    CHECK_EQ(dir.header()->words_per_record, 8);
    CHECK_EQ(dir.header()->record_count, 22);
    CHECK_EQ(dir.header()->rx_buffer, 0x0100);
    CHECK(dir.header()->layout_ok());
    CHECK_EQ(dir.records().size(), 22u);
    CHECK(!dir.empty());
    const IndexCategory* limits = dir.find(7);
    REQUIRE(limits != nullptr);
    CHECK_EQ(limits->data_address, 0x0700);
    CHECK_EQ(limits->id_list_address, 0x5100);
    CHECK_EQ(limits->member_count, 0x1A);
    CHECK_EQ(limits->member_size, 2);
    CHECK_EQ(limits->name(), reg::category_name(7));
    CHECK_EQ(limits->extent().words, 52);
    CHECK(limits->extent().exact);
    CHECK(dir.find(0x7777) == nullptr);
    CHECK_THROWS(dir.get(0x7777), std::out_of_range);
    CHECK_EQ(dir.get(0x10).list_count, 768);
}

TEST(model_directory_from_words_matches_read) {
    const std::map<int, std::uint16_t> words = model_fixture_words("rod");
    std::vector<std::uint16_t> table;
    for (int a = 0; a < 184; ++a) table.push_back(words.at(a));
    const UnitDirectory dir = UnitDirectory::from_words(table);
    CHECK_EQ(dir.records().size(), 22u);
    CHECK_EQ(dir.get(3).data_address, 0x0300);
    // a table shorter than its header announces is refused; a header of 5 words is the minimum
    CHECK_THROWS(UnitDirectory::from_words(std::vector<std::uint16_t>(table.begin(), table.begin() + 40)),
                 std::invalid_argument);
    CHECK_THROWS(IndexHeader::from_words({1, 8, 22}), std::invalid_argument);
    CHECK_THROWS(IndexCategory::from_words({1, 2, 3}), std::invalid_argument);
    CHECK_EQ(UnitDirectory::parse_records(std::vector<std::uint16_t>(table.begin() + 8, table.begin() + 8 + 20)).size(), 2u);
}

TEST(model_directory_without_a_directory_is_empty) {
    MapReader reader(std::map<int, std::uint16_t>{});  // every word reads 0000: a zero header
    const UnitDirectory dir = UnitDirectory::read(reader);
    CHECK(dir.empty());
    REQUIRE(dir.header().has_value());
    CHECK(!dir.header()->layout_ok());
    CHECK(dir.find(7) == nullptr);
    // a silent unit is an exception the caller handles
    MapReader silent(std::map<int, std::uint16_t>{});
    silent.timeout.insert(0);
    CHECK_THROWS(UnitDirectory::read(silent), ModbusTimeout);
}

TEST(model_directory_block_extents_and_readable_words) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    // class 1 has a fixed member order: 6 members = 8 words; class 6 is a known mixed block (72 words)
    CHECK_EQ(dir.get(1).extent().words, 8);
    CHECK(dir.get(1).extent().exact);
    CHECK_EQ(dir.get(6).extent().words, 0x48);
    CHECK_EQ(dir.readable_words(dir.get(6)), 0x48);
    // an unknown mixed block (class 0x12: 7 members) is uncertain and never reaches the next block
    const IndexCategory& c12 = dir.get(0x12);
    CHECK_EQ(c12.extent().words, 7);
    CHECK(!c12.extent().exact);
    CHECK_EQ(c12.extent().words_max, 14);
    CHECK_EQ(dir.readable_words(c12), 14);
    // the next block after class 0xC (0x0C00, 8 words) is at 0x0D00 but the extent is exact
    CHECK_EQ(dir.readable_words(dir.get(0xC)), 8);
}

TEST(model_directory_read_block_bounds) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    const std::vector<std::uint16_t> all = dir.read_block(reader, 0xC);
    CHECK_EQ(all.size(), 8u);
    CHECK_EQ(all[0], 0x0000);
    CHECK_EQ(all[1], 0x3AB6);  // ZNM entry 0 = 150.30 mm
    const std::vector<std::uint16_t> part = dir.read_block(reader, 0xC, 2, 2);
    CHECK_EQ(part.size(), 2u);
    CHECK_EQ(part[1], 0xFFE2);
    // outside the block: nothing is read
    const int before = reader.reads;
    CHECK_THROWS(dir.read_block(reader, 0xC, 6, 4), std::out_of_range);
    CHECK_THROWS(dir.read_block(reader, 0xC, -1, 2), std::out_of_range);
    CHECK_THROWS(dir.read_block(reader, 0xC, 0, 0), std::out_of_range);
    CHECK_THROWS(dir.read_block(reader, 0x7777), std::out_of_range);
    CHECK_EQ(reader.reads, before);
    // a block of uncertain size may be read up to the largest extent its record allows
    CHECK_EQ(dir.read_block(reader, 0x12, 0, 14).size(), 14u);
}

TEST(model_directory_read_class_items_by_id) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    const ClassBlock block = dir.read_class(reader, 7);
    CHECK_EQ(block.class_id, 7);
    CHECK_EQ(block.address, 0x0700);
    CHECK_EQ(block.words, 52);
    CHECK_EQ(block.member_size, 2);
    REQUIRE(block.id_map_address.has_value());
    CHECK_EQ(*block.id_map_address, 0x5100);
    CHECK_EQ(block.items.size(), 26u);
    const ClassItem* limm = block.item(1);
    REQUIRE(limm != nullptr);
    CHECK_EQ(limm->signed_value(), 15030);
    CHECK_NEAR(limm->scaled(), 150.30, 1e-9);
    CHECK_EQ(limm->key(), "7.1");
    CHECK_EQ(limm->address, 0x0700);
    CHECK_EQ(limm->hex(), "00003AB6");
    const ClassItem* liml = block.item(2);
    REQUIRE(liml != nullptr);
    CHECK_EQ(liml->signed_value(), -30);
    CHECK_NEAR(liml->scaled(), -0.30, 1e-9);
    CHECK_EQ(liml->hex(), "FFFFFFE2");
    const ClassItem* maxv = block.item(reg::kAxisItemMaxv);
    REQUIRE(maxv != nullptr);
    CHECK_NEAR(maxv->scaled(), 800.0, 1e-9);
    CHECK(block.item(0x7777) == nullptr);
    CHECK(block.item(1, 3) == nullptr);
}

TEST(model_directory_read_class_multi_entry_and_keys) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    const ClassBlock zones = dir.read_class(reader, 0xC);
    CHECK_EQ(zones.entries, 2);
    CHECK_EQ(zones.items.size(), 4u);
    const ClassItem* second = zones.item(1, 1);
    REQUIRE(second != nullptr);
    CHECK_EQ(second->key(), "C.1[1]");
    CHECK_EQ(second->entry, 1);
    CHECK_EQ(second->address, 0x0C04);
    const ClassItem* first = zones.item(2);
    REQUIRE(first != nullptr);
    CHECK_EQ(first->key(), "C.2");
    // class 8 (hex above 9 only for the class number)
    CHECK_EQ(dir.read_class(reader, 8).item(10)->key(), "8.10");
}

TEST(model_directory_read_class_refuses_unknown_and_huge_classes) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    CHECK_THROWS(dir.read_class(reader, 0x7777), std::out_of_range);
    // the position table (12288 words) is read row by row, not as a class
    CHECK_THROWS(dir.read_class(reader, 0x10), std::invalid_argument);
    CHECK_EQ(dir.read_class(reader, 7, 52).words, 52);
    CHECK_THROWS(dir.read_class(reader, 7, 51), std::invalid_argument);
    // a class without an ID map comes back word by word with no item IDs (class 0x101 monitor: mixed sizes)
    const ClassBlock monitor = dir.read_class(reader, 0x101);
    CHECK(!monitor.id_map_address.has_value());
    CHECK_EQ(monitor.items.size(), 34u);
    CHECK(!monitor.items[0].item_id.has_value());
    CHECK_EQ(monitor.items[0].key(), "101.?");
}

TEST(model_directory_support_words_and_accl) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    const auto support = dir.read_support_words(reader);
    REQUIRE(support.has_value());
    CHECK_EQ(support->size(), 6u);
    CHECK_EQ(support->at("SUPT"), 0x60000BD7u);
    CHECK_EQ(support->at("MVPT"), 0x0317u);
    CHECK_EQ(support->at("SPTS"), 0x0080u);
    CHECK_EQ(support->at("SUP2"), 0x4300009Au);
    CHECK(support->find("SUP3") == support->end());  // the rod has 6 members, no SUP3
    // the gripper has 11 members
    MapReader gripper = make_unit("gripper");
    const UnitDirectory gdir = UnitDirectory::read(gripper);
    const auto gsupport = gdir.read_support_words(gripper);
    REQUIRE(gsupport.has_value());
    CHECK_EQ(gsupport->size(), 11u);
    CHECK_EQ(gsupport->at("SUP2"), 0x4384009Au);
    CHECK_EQ(gsupport->at("SUP3"), 0u);
    CHECK_EQ(*gdir.read_accl(gripper), 1u);  // bits 8-11 = extensibility level 0
    CHECK_EQ(*dir.read_accl(reader), 0u);
    // no class 1 / no class 6 in an empty directory
    const UnitDirectory none;
    CHECK(!none.read_support_words(reader).has_value());
    CHECK(!none.read_accl(reader).has_value());
}

// ------------------------------------------------------------------------------------------ identity

TEST(model_identity_slave_id_and_ctcl) {
    const auto slider = SlaveIdentity::from_slave_id({0xEC, 0xFF, 0x02, 0x00, 0x13});
    REQUIRE(slider.has_value());
    CHECK_EQ(slider->system_code, 0xEC);
    REQUIRE(slider->run_status.has_value());
    CHECK_EQ(*slider->run_status, 0xFF);
    CHECK_EQ(slider->unit_code, 2);
    CHECK_EQ(slider->firmware, "0013");
    CHECK_EQ(slider->source, "FC11");
    const auto pcon = SlaveIdentity::from_slave_id({0xAE, 0xFF, 0x44, 0x00, 0x00});
    REQUIRE(pcon.has_value());
    CHECK_EQ(pcon->unit_code, 0x44);
    CHECK(!SlaveIdentity::from_slave_id({0xEC, 0xFF, 0x02, 0x00}).has_value());
    CHECK(!SlaveIdentity::from_slave_id({}).has_value());
    const auto ctcl = SlaveIdentity::from_ctcl(0xEC0A0002u);
    REQUIRE(ctcl.has_value());
    CHECK_EQ(ctcl->system_code, 0xEC);
    CHECK(!ctcl->run_status.has_value());
    CHECK_EQ(ctcl->unit_code, 0x0A);
    CHECK_EQ(ctcl->firmware, "0002");
    CHECK_EQ(ctcl->source, "CTCL 0x0300");
    CHECK(!SlaveIdentity::from_ctcl(0x00020013u).has_value());
}

TEST(model_identity_identification_block) {
    const std::map<int, std::uint16_t> words = model_fixture_words("gripper");
    std::vector<std::uint16_t> block;
    for (int a = 0x600; a < 0x600 + 0x48; ++a) block.push_back(words.at(a));
    const IdentificationBlock ident = IdentificationBlock::from_words(block);
    REQUIRE(ident.model_name.has_value());
    CHECK_EQ(*ident.model_name, "EC-GRBP8M-20-1-1-PN-WA-WL2");
    REQUIRE(ident.serial_number.has_value());
    CHECK_EQ(*ident.serial_number, "SIM-0000002");
    REQUIRE(ident.user_name.has_value());
    CHECK_EQ(*ident.user_name, "EC-GRBP8M-20");
    REQUIRE(ident.accl.has_value());
    CHECK_EQ(*ident.accl, 1u);
    REQUIRE(ident.extensibility_level().has_value());
    CHECK_EQ(*ident.extensibility_level(), 0);
    IdentificationBlock leveled = ident;
    leveled.accl = 0x0300u;
    CHECK_EQ(*leveled.extensibility_level(), 3);
    // a short block: the fields past its end are not there
    const IdentificationBlock shortened = IdentificationBlock::from_words(std::vector<std::uint16_t>(block.begin(), block.begin() + 0x10));
    CHECK(shortened.serial_number.has_value());
    CHECK(!shortened.model_name.has_value());
    CHECK(!shortened.accl.has_value());
    CHECK(!shortened.extensibility_level().has_value());
    // binary data where an EC keeps its text is not a text field (a CON controller)
    std::vector<std::uint16_t> binary(0x48, 0x0001);
    const IdentificationBlock bin = IdentificationBlock::from_words(binary);
    CHECK(!bin.serial_number.has_value());
    CHECK(!bin.model_name.has_value());
}

TEST(model_identity_rod_read_device_info) {
    MapReader reader = make_unit("rod");
    const DeviceInfo info = read_device_info(reader);
    CHECK_EQ(info.slave, 1);
    CHECK(info.errors.empty());
    CHECK_EQ(*info.system_code, 0xEC);
    CHECK_EQ(*info.system_name, "EC 24 V");
    CHECK_EQ(*info.run_status, 0xFF);
    CHECK_EQ(*info.unit_code, 0x00);
    CHECK_EQ(*info.unit_name, "EC standard");
    CHECK_EQ(*info.firmware, "0002");
    CHECK_EQ(*info.identity_source, "FC11");
    CHECK(*info.unit_known);
    CHECK_EQ(*info.product_family, "EC");
    CHECK_EQ(*info.model_name, "EC-R6S-150-1-PN");
    CHECK_EQ(info.model(), "EC-R6S-150-1-PN");
    CHECK_EQ(info.controller(), "EC 24 V, unit 0x00 (EC standard)");
    CHECK_EQ(*info.serial_number, "SIM-0000003");
    CHECK_EQ(*info.axis_number, 0);
    CHECK(!info.controller_name.has_value());
    CHECK_EQ(info.slave_id_raw.size(), 5u);
    CHECK_EQ(info.categories.size(), 22u);
    REQUIRE(info.index_header.has_value());
    CHECK_EQ(info.index_header->record_count, 22);
    REQUIRE(info.category(7) != nullptr);
    CHECK_EQ(info.category(7)->data_address, 0x0700);
    CHECK(info.category(0x7777) == nullptr);
    // the limit fields are the Axis's to fill
    CHECK(!info.soft_limits_mm.has_value());
    CHECK(!info.max_speed_mm_s.has_value());
    CHECK_EQ(info.units, "mm");
    CHECK_EQ(info.kind(), "unknown");  // AKID is read with the limits
    CHECK(!info.actuator().has_value());
}

TEST(model_identity_given_directory_is_not_read_again) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    reader.refuse_range(0x0000, 0x00FF);  // the index table can no longer be read
    const DeviceInfo info = read_device_info(reader, &dir);
    CHECK(info.errors.empty());
    CHECK_EQ(info.categories.size(), 22u);
    CHECK_EQ(*info.model_name, "EC-R6S-150-1-PN");
}

TEST(model_identity_gripper_and_extensibility) {
    MapReader reader = make_unit("gripper");
    const DeviceInfo info = read_device_info(reader);
    CHECK(info.errors.empty());
    CHECK_EQ(*info.unit_name, "EC(MS) \"2-size EC\"");
    CHECK_EQ(info.controller(), "EC 24 V, unit 0x0A (EC(MS) \"2-size EC\")");
    CHECK_EQ(*info.model_name, "EC-GRBP8M-20-1-1-PN-WA-WL2");
    CHECK_EQ(*info.user_name, "EC-GRBP8M-20");
    CHECK_EQ(*info.extensibility_level(), 0);
    CHECK_EQ(*info.accl, 1u);
    CHECK_EQ(info.categories.size(), 25u);
}

TEST(model_identity_pcon_controller_name_and_family) {
    MapReader reader = make_unit("pcon");
    const DeviceInfo info = read_device_info(reader);
    CHECK(info.errors.empty());
    CHECK_EQ(*info.product_family, "PCON");
    CHECK_EQ(*info.system_name, "PCON");
    CHECK_EQ(*info.unit_code, 0x44);
    CHECK(!*info.unit_known);  // no family name for this code (display only); the unit is driven the same way
    REQUIRE(info.controller_name.has_value());
    CHECK_EQ(*info.controller_name, "PCON-CB (FB)");
    CHECK(!info.model_name.has_value());  // a CON controller keeps binary data where an EC keeps its model string
    CHECK_EQ(info.model(), "PCON-CB (FB)");
    CHECK_EQ(*info.axis_number, 0);
}

TEST(model_identity_unknown_unit_is_described_not_refused) {
    MapReader reader = make_unit("rod");
    reader.slave_id = {0x99, 0xFF, 0x7E, 0x01, 0x00};
    const DeviceInfo info = read_device_info(reader);
    CHECK_EQ(*info.system_code, 0x99);
    CHECK_EQ(*info.product_family, "IAI controller");
    CHECK(!info.unit_name.has_value());
    CHECK(!*info.unit_known);
    CHECK_EQ(info.controller(), "system code 0x99, unit 0x7E");
    CHECK(info.errors.empty());  // reported as what it says it is: no note about the code
    CHECK(!contains(info.errors, "UNVERIFIED"));
    // an EC unit code without a family name is reported the same way
    reader.slave_id = {0xEC, 0xFF, 0x7E, 0x01, 0x00};
    const DeviceInfo ec_unknown = read_device_info(reader);
    CHECK_EQ(*ec_unknown.product_family, "EC");
    CHECK_EQ(*ec_unknown.unit_code, 0x7E);
    CHECK(!*ec_unknown.unit_known);
    CHECK(ec_unknown.errors.empty());
    // the conveyor control motor code is named, not an EC actuator
    reader.slave_id = {0xEC, 0xFF, 0x10, 0x00, 0x01};
    CHECK_EQ(*read_device_info(reader).unit_name, "CCM conveyor control motor (not an EC actuator)");
}

TEST(model_identity_short_fc11_answer_is_collected) {
    MapReader reader = make_unit("rod");
    reader.slave_id = {0xEC, 0xFF, 0x00};
    const DeviceInfo info = read_device_info(reader);
    CHECK(contains(info.errors, "report slave id: short answer ec ff 00"));
    CHECK(info.slave_id_raw.size() == 3u);
    // CTCL gave the identity instead
    CHECK_EQ(*info.identity_source, "CTCL 0x0300");
}

TEST(model_identity_ctcl_fallback_when_fc11_is_silent) {
    MapReader reader = make_unit("rod");
    reader.fc11_error = 1;
    const DeviceInfo info = read_device_info(reader);  // collected, never thrown
    CHECK_EQ(*info.identity_source, "CTCL 0x0300");
    CHECK_EQ(*info.system_code, 0xEC);
    CHECK_EQ(*info.unit_code, 0x00);
    CHECK_EQ(*info.firmware, "0002");
    CHECK(!info.run_status.has_value());
    CHECK(info.slave_id_raw.empty());
    CHECK(contains(info.errors, "report slave id: not answered"));
    CHECK(contains(info.errors, "identity read from CTCL (class 3 item 1) instead of FC11"));
    CHECK_EQ(*info.unit_name, "EC standard");
    CHECK_EQ(*info.model_name, "EC-R6S-150-1-PN");
    // refused (an exception response) is collected the same way
    reader.fc11_error = 2;
    const DeviceInfo refused = read_device_info(reader);
    CHECK(contains(refused.errors, "report slave id:"));
    CHECK_EQ(*refused.identity_source, "CTCL 0x0300");
}

TEST(model_identity_failing_parts_are_collected) {
    MapReader reader = make_unit("rod");
    reader.refuse_range(0x0600, 0x0647);  // the identification block
    const DeviceInfo no_ident = read_device_info(reader);
    CHECK(contains(no_ident.errors, "identification block:"));
    CHECK(!no_ident.model_name.has_value());
    CHECK_EQ(*no_ident.unit_code, 0x00);  // everything else is there

    MapReader dead = make_unit("rod");
    dead.refuse_range(0x0000, 0x0007);  // no index table
    dead.fc11_error = 1;
    const DeviceInfo nothing = read_device_info(dead);
    CHECK(contains(nothing.errors, "index table:"));
    CHECK(contains(nothing.errors, "report slave id: not answered"));
    CHECK(!nothing.system_code.has_value());
    CHECK_EQ(nothing.controller(), "unknown");
    CHECK(nothing.categories.empty());

    MapReader odd = make_unit("rod");
    odd.set(1, 4);  // words per record 4: a layout the driver does not understand
    const DeviceInfo layout = read_device_info(odd);
    CHECK(contains(layout.errors, "index table: unexpected layout"));
    CHECK(layout.categories.empty());
    REQUIRE(layout.index_header.has_value());
    CHECK_EQ(layout.index_header->words_per_record, 4);
}

TEST(model_identity_axno_mismatch_is_reported) {
    MapReader reader = make_unit("rod");
    reader.set32(0x8004, 5);
    const DeviceInfo info = read_device_info(reader);
    CHECK_EQ(*info.axis_number, 5);
    CHECK(contains(info.errors, "AXNO (0x8004) = 5, but slave address 1 means axis 0"));
}

TEST(model_identity_a_transport_failure_propagates) {
    MapReader reader = make_unit("rod");
    reader.fc11_error = 3;
    CHECK_THROWS(read_device_info(reader), TransportError);
    MapReader line = make_unit("rod");
    line.transport_error.insert(0x0600);
    CHECK_THROWS(read_device_info(line), TransportError);
    MapReader gone = make_unit("rod");
    gone.transport_error.insert(0);
    CHECK_THROWS(read_device_info(gone), TransportError);
}

// ------------------------------------------------------------------------------------------ soft limits and strokes

TEST(model_limits_soft_limits_by_item_id) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    const SoftLimits soft = read_soft_limits(reader, dir);
    REQUIRE(soft.limits.has_value());
    CHECK_NEAR(soft.limits->min, -0.30, 1e-9);
    CHECK_NEAR(soft.limits->max, 150.30, 1e-9);
    CHECK_EQ(soft.note, "read from 0x0700 (items 7.1 @0x0700 / 7.2 @0x0702 by ID)");
    REQUIRE(soft.p1_mm.has_value());
    CHECK_NEAR(*soft.p1_mm, 150.0, 1e-9);  // item 7.1 minus the 0.30 margin
}

TEST(model_limits_soft_limits_from_the_first_four_words) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    reader.refuse_range(0x5100, 0x5133);  // the class 7 ID map cannot be read
    const SoftLimits soft = read_soft_limits(reader, dir);
    REQUIRE(soft.limits.has_value());
    CHECK_NEAR(soft.limits->max, 150.30, 1e-9);
    CHECK_EQ(soft.note, "read from 0x0700 (first 4 words of class 7)");
    // without any directory the default address is read
    MapReader plain = make_unit("rod");
    const SoftLimits unknown = read_soft_limits(plain, UnitDirectory());
    REQUIRE(unknown.limits.has_value());
    CHECK_EQ(unknown.note, "read from 0x0700 (first 4 words of class 7)");
}

TEST(model_limits_soft_limits_refused_or_implausible) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    reader.refuse_range(0x0700, 0x0707);
    reader.refuse_range(0x5100, 0x5133);
    const SoftLimits refused = read_soft_limits(reader, dir);
    CHECK(!refused.limits.has_value());
    CHECK(refused.note.find("0x0700 not readable") == 0);
    CHECK(!refused.p1_mm.has_value());
    // a span below 0.01 mm is not a stroke
    MapReader flat = make_unit("rod");
    flat.set32(0x0700, 100);
    flat.set32(0x0702, 100);
    const SoftLimits implausible = read_soft_limits(flat, dir);
    CHECK(!implausible.limits.has_value());
    CHECK(implausible.note.find("implausible values at 0x0700") == 0);
    // no class 7 in the directory
    std::vector<IndexCategory> without7;
    for (const IndexCategory& c : dir.records())
        if (c.category_id != 7) without7.push_back(c);
    const SoftLimits missing = read_soft_limits(reader, UnitDirectory(dir.header(), without7));
    CHECK(!missing.limits.has_value());
    CHECK_EQ(missing.note, "the index table has no soft-limit category (7)");
    // a timeout is not a note
    MapReader silent = make_unit("rod");
    silent.timeout.insert(0x0700);
    silent.refuse_range(0x5100, 0x5133);
    CHECK_THROWS(read_soft_limits(silent, dir), ModbusTimeout);
}

TEST(model_limits_unit_stroke_and_effective_stroke) {
    const auto stroke = unit_stroke(Span{-0.30, 150.30});
    REQUIRE(stroke.has_value());
    CHECK_NEAR(stroke->min, 0.0, 1e-9);
    CHECK_NEAR(stroke->max, 150.0, 1e-9);
    // a span too small for the margin is left as it is
    const auto tiny = unit_stroke(Span{0.0, 0.5});
    REQUIRE(tiny.has_value());
    CHECK(*tiny == (Span{0.0, 0.5}));
    CHECK(!unit_stroke(std::nullopt).has_value());

    const std::optional<Span> unit = Span{0.0, 150.0};
    CHECK(!effective_stroke(std::nullopt, std::nullopt).has_value());
    CHECK(*effective_stroke(std::nullopt, unit) == (Span{0.0, 150.0}));
    CHECK(*effective_stroke(Span{10.0, 20.0}, std::nullopt) == (Span{10.0, 20.0}));
    CHECK(*effective_stroke(Span{10.0, 100.0}, unit) == (Span{10.0, 100.0}));
    CHECK(*effective_stroke(Span{-50.0, 500.0}, unit) == (Span{0.0, 150.0}));  // a configured stroke only narrows
    CHECK(*effective_stroke(Span{100.0, 400.0}, unit) == (Span{100.0, 150.0}));
    CHECK(!effective_stroke(Span{200.0, 300.0}, unit).has_value());            // no overlap
    CHECK(!effective_stroke(Span{150.0, 300.0}, unit).has_value());            // touching is not a range
}

// ------------------------------------------------------------------------------------------ class-7 limits

TEST(model_limits_rod_axis_limits_match_the_python_edition) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    const AxisLimits l = read_axis_limits(reader, dir);
    REQUIRE(l.max_speed_mm_s.has_value());
    CHECK_NEAR(*l.max_speed_mm_s, 800.0, 1e-9);
    CHECK_NEAR(*l.max_accel_g, 1.0, 1e-9);
    CHECK_NEAR(*l.max_decel_g, 1.0, 1e-9);
    CHECK_NEAR(*l.max_push_speed_mm_s, 20.0, 1e-9);
    CHECK_NEAR(*l.max_push_pct, 70.0, 1e-9);
    CHECK_NEAR(l.min_push_pct, 20.0, 1e-9);
    CHECK(!l.min_push_from_unit);
    CHECK_EQ(*l.push_force_factor, 56);
    CHECK_EQ(*l.push_force_offset, 0);
    CHECK_EQ(*l.actuator_kind, 3);
    CHECK_EQ(l.kind(), "rod");
    CHECK_EQ(l.units(), "mm");
    CHECK(!*l.rotary_atyp);
    CHECK(!*l.rotary());
    CHECK(*l.supports_push);
    CHECK(*l.supports_relative);
    CHECK_NEAR(*l.lead_mm, 20.0, 1e-9);
    CHECK_NEAR(*l.min_band, 0.08, 1e-9);  // MINI 3.67, enforced (SUP2 CINP + ATYP RINP)
    CHECK(!l.default_speed_mm_s.has_value());
    CHECK_EQ(l.source, "class 7 by item ID");
    CHECK(l.notes.empty());
    CHECK(*l.push_available());
    const auto estimate = l.push_force_estimate(35.0);
    REQUIRE(estimate.has_value());
    CHECK_NEAR(*estimate, 28.0, 1e-9);  // TCPC x % / MXPP + TCOA
    const auto cap = l.unit_speed_cap();
    CHECK_NEAR(*cap.first, 800.0, 1e-9);
    CHECK_EQ(cap.second, "MAXV item 7.4 @0x0706");  // the Python edition's ec_max_speed_note names the address
}

TEST(model_limits_gripper_and_pcon_axis_limits) {
    MapReader gripper = make_unit("gripper");
    const AxisLimits g = read_axis_limits(gripper, UnitDirectory::read(gripper));
    CHECK_NEAR(*g.max_speed_mm_s, 53.0, 1e-9);
    CHECK_NEAR(*g.max_accel_g, 0.30, 1e-9);
    CHECK_NEAR(*g.max_push_pct, 100.0, 1e-9);
    CHECK_NEAR(g.min_push_pct, 50.0, 1e-9);
    CHECK(g.min_push_from_unit);  // MNPP 7.58 was read
    CHECK_EQ(*g.actuator_kind, 4);
    CHECK_EQ(g.kind(), "gripper");
    CHECK(!g.min_band.has_value());
    CHECK_NEAR(*g.lead_mm, 1.05, 1e-9);
    CHECK_EQ(*g.push_force_factor, 36);

    MapReader pcon = make_unit("pcon");
    const AxisLimits p = read_axis_limits(pcon, UnitDirectory::read(pcon));
    CHECK_NEAR(*p.max_speed_mm_s, 840.0, 1e-9);
    CHECK(!p.actuator_kind.has_value());  // a CON controller has no AKID
    CHECK_EQ(p.kind(), "unknown");
    CHECK(!*p.rotary_atyp);
    CHECK_NEAR(*p.min_band, 0.01, 1e-9);
    CHECK_NEAR(*p.lead_mm, 16.0, 1e-9);
    CHECK(!p.push_force_factor.has_value());
}

TEST(model_limits_rotary_axis_is_in_degrees) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    // AKID is item 7.33 (found through the ID map): make it a rotary kind
    const ClassBlock c7 = dir.read_class(reader, 7);  // named: item() points into it
    const ClassItem* akid = c7.item(reg::kAxisItemAkid);
    REQUIRE(akid != nullptr);
    reader.set32(akid->address, 1);
    const AxisLimits l = read_axis_limits(reader, dir);
    CHECK_EQ(*l.actuator_kind, 1);
    CHECK_EQ(l.units(), "deg");
    CHECK_EQ(l.kind(), "rotary");
    CHECK(*l.rotary());
    // without AKID the ATYP TYPE bit decides
    AxisLimits atyp;
    atyp.rotary_atyp = true;
    CHECK_EQ(atyp.units(), "deg");
    CHECK_EQ(atyp.kind(), "rotary");
    atyp.actuator_kind = 3;  // AKID wins when present
    CHECK_EQ(atyp.units(), "mm");
    CHECK(!*atyp.rotary());
}

TEST(model_limits_class_7_refused_leaves_fields_empty_with_a_note) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    reader.refuse_range(0x0700, 0x0733);
    const AxisLimits l = read_axis_limits(reader, dir);
    CHECK(!l.max_speed_mm_s.has_value());
    CHECK(!l.max_accel_g.has_value());
    CHECK(!l.actuator_kind.has_value());
    CHECK(!l.lead_mm.has_value());
    REQUIRE(l.notes.size() >= 1u);
    CHECK(l.notes[0].find("class 7 not readable") == 0);
    CHECK(l.source.find("class 7 not readable") == 0);
    // the default command speed (item 8.10) is the fallback cap
    REQUIRE(l.default_speed_mm_s.has_value());
    CHECK_NEAR(*l.default_speed_mm_s, 800.0, 1e-9);
    const auto cap = l.unit_speed_cap();
    CHECK_NEAR(*cap.first, 800.0, 1e-9);
    CHECK(cap.second.find("0x0812 (item 8.10 VCMD") == 0);
    // the push default stays the recommendation; SUPT (class 1) still says the unit can push
    CHECK_NEAR(l.min_push_pct, 20.0, 1e-9);
    CHECK(l.supports_push.has_value());
    CHECK(*l.push_available());
}

TEST(model_limits_without_a_directory_or_class) {
    MapReader reader = make_unit("rod");
    const AxisLimits none = read_axis_limits(reader, UnitDirectory());
    REQUIRE(none.notes.size() >= 1u);
    CHECK_EQ(none.notes[0], "class 7: the index table is not readable");
    CHECK(!none.max_speed_mm_s.has_value());
    const auto cap = none.unit_speed_cap();
    CHECK(!cap.first.has_value());
    CHECK(cap.second.find("MAXV 7.4 not readable") == 0);
    // a directory without class 7
    const UnitDirectory dir = UnitDirectory::read(reader);
    std::vector<IndexCategory> without7;
    for (const IndexCategory& c : dir.records())
        if (c.category_id != 7) without7.push_back(c);
    const AxisLimits missing = read_axis_limits(reader, UnitDirectory(dir.header(), without7));
    CHECK(missing.notes[0].find("class 7:") == 0);
    CHECK(missing.notes[0].find("is not in this unit's directory") != std::string::npos);
}

TEST(model_limits_a_timeout_propagates) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    reader.timeout.insert(0x0700);
    CHECK_THROWS(read_axis_limits(reader, dir), ModbusTimeout);
}

TEST(model_limits_item_without_id_map_is_a_note) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    std::vector<IndexCategory> records = dir.records();
    for (IndexCategory& c : records)
        if (c.category_id == 7) c.id_list_address = 0;
    const AxisLimits l = read_axis_limits(reader, UnitDirectory(dir.header(), records));
    REQUIRE(!l.notes.empty());
    CHECK_EQ(l.notes[0], "class 7 has no ID map");
    CHECK(!l.max_speed_mm_s.has_value());
}

TEST(model_limits_implausible_values_are_dropped) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    const ClassBlock block = dir.read_class(reader, 7);
    reader.set32(block.item(reg::kAxisItemMaxv)->address, 0);          // 0.00 mm/s is not a maximum
    reader.set32(block.item(reg::kAxisItemMaxa)->address, 5000);       // 50 G
    reader.set32(block.item(reg::kAxisItemMxpp)->address, 5000);       // more than 1000 %
    const AxisLimits l = read_axis_limits(reader, dir);
    CHECK(!l.max_speed_mm_s.has_value());
    CHECK(!l.max_accel_g.has_value());
    CHECK(!l.max_push_pct.has_value());
    CHECK(l.max_decel_g.has_value());
}

TEST(model_limits_push_availability_rules) {
    AxisLimits l;
    CHECK(!l.push_available().has_value());  // nothing read
    l.max_push_pct = 70.0;
    CHECK(*l.push_available());
    l.max_push_pct = 0.0;
    CHECK(!*l.push_available());             // MXPP 0 = no push
    l.max_push_pct = 70.0;
    l.push_force_factor = 0;
    CHECK(!*l.push_available());             // TCPC 0 = no push
    l.push_force_factor = 56;
    l.supports_push = false;
    CHECK(!*l.push_available());             // SUPT without PUSH
    AxisLimits none;
    CHECK(!none.push_force_estimate(10.0).has_value());
}

// ------------------------------------------------------------------------------------------ stored rows

TEST(model_limits_max_positions_rules) {
    MapReader rod = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(rod);
    CHECK_EQ(read_max_positions(rod, dir, 0xEC), 3);        // an EC keeps rows 0-2
    CHECK_EQ(read_max_positions(rod, dir, std::nullopt), 3);

    MapReader pcon = make_unit("pcon");
    CHECK_EQ(read_max_positions(pcon, UnitDirectory::read(pcon), 0xAE), 64);  // MAXP 3.2 = 64 on the PCON

    // MAXP 2 (the 2-position mode of an EC): the EC floor of 3 applies, a CON controller reports its own count
    MapReader two = make_unit("rod");
    const ClassBlock c3 = dir.read_class(two, 3);
    two.set32(c3.item(reg::kControllerItemMaxp)->address, 2);
    CHECK_EQ(read_max_positions(two, dir, 0xEC), 3);
    CHECK_EQ(read_max_positions(two, dir, 0xAE), 2);
    CHECK_EQ(read_max_positions(two, dir, 0x99), 2);
    CHECK_EQ(read_max_positions(two, dir, 0xED), 3);

    // MAXP larger than the table: the table's row count caps it
    MapReader many = make_unit("pcon");
    const UnitDirectory pdir = UnitDirectory::read(many);
    const ClassBlock p3 = pdir.read_class(many, 3);
    many.set32(p3.item(reg::kControllerItemMaxp)->address, 5000);
    const int rows = pdir.get(0x10).list_count;
    CHECK_EQ(read_max_positions(many, pdir, 0xAE), rows < 5000 ? rows : 5000);

    // MAXP unreadable (refused class 3): the directory's row count; no directory at all: 3
    MapReader refused = make_unit("pcon");
    refused.refuse_range(0x0300, 0x03FF);
    CHECK_EQ(read_max_positions(refused, pdir, 0xAE), rows);
    CHECK_EQ(read_max_positions(refused, UnitDirectory(), 0xAE), 3);
    CHECK_EQ(read_max_positions(refused, UnitDirectory(), 0xEC), 3);
    // MAXP 0 means not set
    MapReader zero = make_unit("pcon");
    zero.set32(p3.item(reg::kControllerItemMaxp)->address, 0);
    CHECK_EQ(read_max_positions(zero, pdir, 0xAE), rows);
    // a timeout propagates
    MapReader silent = make_unit("pcon");
    silent.timeout.insert(p3.item(reg::kControllerItemMaxp)->address);
    CHECK_THROWS(read_max_positions(silent, pdir, 0xAE), ModbusTimeout);
}

// ------------------------------------------------------------------------------------------ ranges

namespace {

RangeInputs rod_inputs(MapReader& reader) {
    const UnitDirectory dir = UnitDirectory::read(reader);
    RangeInputs in;
    in.limits = read_axis_limits(reader, dir);
    const SoftLimits soft = read_soft_limits(reader, dir);
    in.soft_limits = soft.limits;
    in.soft_limits_note = soft.note;
    in.family = "EC";
    in.rows = read_max_positions(reader, dir, 0xEC);
    return in;
}

}  // namespace

TEST(model_ranges_come_from_the_unit) {
    MapReader reader = make_unit("rod");
    const RangeInputs in = rod_inputs(reader);
    const AxisRanges r = make_ranges(in);
    CHECK_EQ(r.units, "mm");
    CHECK_EQ(r.kind, "rod");
    CHECK_EQ(r.family, "EC");
    CHECK_NEAR(*r.position.min, 0.0, 1e-9);
    CHECK_NEAR(*r.position.max, 150.0, 1e-9);
    CHECK_EQ(r.position.unit, "mm");
    CHECK_EQ(r.position.source,
             "soft limits 7.1 / 7.2 (read from 0x0700 (items 7.1 @0x0700 / 7.2 @0x0702 by ID)) minus the 0.30 margin");
    CHECK_NEAR(*r.speed.min, 0.01, 1e-9);
    CHECK_NEAR(*r.speed.max, 800.0, 1e-9);
    CHECK_EQ(r.speed.unit, "mm/s");
    CHECK_NEAR(*r.speed.unit_max, 800.0, 1e-9);
    CHECK(!r.speed.configured.has_value());
    CHECK_EQ(r.speed.source, "MAXV item 7.4 @0x0706");
    CHECK_NEAR(*r.accel.max, 1.0, 1e-9);
    CHECK_EQ(r.accel.source, "MAXA 7.5");
    CHECK_NEAR(*r.decel.max, 1.0, 1e-9);
    CHECK_NEAR(*r.accel_decel.max, 1.0, 1e-9);
    CHECK_EQ(r.accel_decel.source, "min(MAXA 7.5, MAXD 7.6): one value for both on a 0x9900 move");
    CHECK(*r.push.available);
    CHECK_NEAR(*r.push.min, 1.0, 1e-9);
    CHECK_NEAR(*r.push.max, 70.0, 1e-9);
    CHECK_NEAR(*r.push.recommended_min, 20.0, 1e-9);
    CHECK_NEAR(*r.push.unit_max, 70.0, 1e-9);
    CHECK_NEAR(*r.band.min, 0.08, 1e-9);
    CHECK_NEAR(*r.band.max, 10.0, 1e-9);
    CHECK_EQ(r.band.source, "driver limit 10 ; MINI 3.67");
    CHECK_NEAR(*r.push_band.max, 150.0, 1e-9);
    CHECK_EQ(r.push_band.source, "the stroke");
    CHECK_NEAR(*r.stored_rows.max, 2.0, 1e-9);
    CHECK_EQ(*r.stored_rows.count, 3);
    CHECK_EQ(r.stored_rows.unit, "row");
    CHECK_NEAR(*r.jog_step.max, 150.0, 1e-9);
    CHECK_NEAR(*r.speed_override.min, 1.0, 1e-9);
    CHECK_NEAR(*r.speed_override.max, 100.0, 1e-9);
}

TEST(model_ranges_a_configured_cap_only_narrows) {
    MapReader reader = make_unit("rod");
    RangeInputs in = rod_inputs(reader);
    in.configured_max_speed = 300.0;
    in.configured_max_accel = 0.5;
    in.configured_max_push = 40.0;
    in.configured_stroke = Span{10.0, 100.0};
    CHECK_NEAR(speed_cap(in), 300.0, 1e-9);
    CHECK_NEAR(accel_cap(in), 0.5, 1e-9);
    CHECK_NEAR(push_cap_pct(in), 40.0, 1e-9);
    AxisRanges r = make_ranges(in);
    CHECK_NEAR(*r.speed.max, 300.0, 1e-9);
    CHECK_NEAR(*r.speed.unit_max, 800.0, 1e-9);
    CHECK_NEAR(*r.speed.configured, 300.0, 1e-9);
    CHECK_NEAR(*r.accel.max, 0.5, 1e-9);
    CHECK_NEAR(*r.accel.configured, 0.5, 1e-9);
    CHECK_NEAR(*r.push.max, 40.0, 1e-9);
    CHECK_NEAR(*r.position.min, 10.0, 1e-9);
    CHECK_NEAR(*r.position.max, 100.0, 1e-9);
    CHECK(r.position.source.find("intersected with the configured stroke") != std::string::npos);
    CHECK_NEAR(*r.push_band.max, 90.0, 1e-9);
    // a configured cap above the unit's maximum cannot widen it
    in.configured_max_speed = 5000.0;
    in.configured_max_accel = 9.0;
    in.configured_max_push = 100.0;
    in.configured_stroke = Span{-500.0, 500.0};
    CHECK_NEAR(speed_cap(in), 800.0, 1e-9);
    CHECK_NEAR(accel_cap(in), 1.0, 1e-9);
    CHECK_NEAR(push_cap_pct(in), 70.0, 1e-9);
    r = make_ranges(in);
    CHECK_NEAR(*r.position.min, 0.0, 1e-9);
    CHECK_NEAR(*r.position.max, 150.0, 1e-9);
    // a stroke that does not overlap the unit's leaves no position range
    in.configured_stroke = Span{300.0, 400.0};
    r = make_ranges(in);
    CHECK(!r.position.min.has_value());
    CHECK(!r.position.max.has_value());
    CHECK(!r.push_band.max.has_value());
    CHECK_NEAR(*r.band.max, 10.0, 1e-9);
}

TEST(model_ranges_fallbacks_only_while_the_unit_is_unknown) {
    RangeInputs none;
    CHECK_NEAR(speed_cap(none), kFallbackSpeedCap, 1e-9);
    CHECK_NEAR(accel_cap(none), kFallbackAccelCapG, 1e-9);
    CHECK_NEAR(push_cap_pct(none), kFallbackPushCapPct, 1e-9);
    const AxisRanges r = make_ranges(none);
    CHECK_EQ(r.units, "mm");
    CHECK_EQ(r.kind, "unknown");
    CHECK_EQ(r.family, reg::kGenericFamily);
    CHECK(!r.position.min.has_value());
    CHECK(!r.speed.unit_max.has_value());
    CHECK_NEAR(*r.speed.max, 100.0, 1e-9);
    CHECK(!r.push.available.has_value());
    CHECK_NEAR(*r.push.recommended_min, reg::kMnppDefaultPct, 1e-9);
    CHECK_NEAR(*r.band.min, 0.01, 1e-9);
    CHECK_EQ(r.band.source, "driver limit 10");
    CHECK_EQ(*r.stored_rows.count, reg::kPositionActiveRows);
    // a configured cap is used while the unit's own maximum is unknown
    none.configured_max_speed = 30.0;
    CHECK_NEAR(speed_cap(none), 30.0, 1e-9);
    // the fallback never lowers a unit that answered: the gripper's MAXV (53) is above no fallback issue, the rod's 800 is
    MapReader reader = make_unit("rod");
    CHECK(speed_cap(rod_inputs(reader)) > kFallbackSpeedCap);
}

TEST(model_ranges_no_push_unit_and_unit_in_degrees) {
    MapReader reader = make_unit("rod");
    RangeInputs in = rod_inputs(reader);
    in.limits->max_push_pct = 0.0;  // MXPP 0 = no push
    AxisRanges r = make_ranges(in);
    CHECK(!*r.push.available);
    CHECK_NEAR(*r.push.max, 0.0, 1e-9);
    in = rod_inputs(reader);
    in.limits->actuator_kind = 1;   // rotary
    r = make_ranges(in);
    CHECK_EQ(r.units, "deg");
    CHECK_EQ(r.kind, "rotary");
    CHECK_EQ(r.speed.unit, "deg/s");
    CHECK_EQ(r.position.unit, "deg");
    CHECK_EQ(r.jog_step.unit, "deg");
    // a small stroke limits the band
    in = rod_inputs(reader);
    in.configured_stroke = Span{0.0, 4.0};
    CHECK_NEAR(*make_ranges(in).band.max, 4.0, 1e-9);
}
