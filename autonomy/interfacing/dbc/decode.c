#ifndef DBCPPP_BYTE_ORDER_LITTLE_ENDIAN
#error "Please pass -DDBCPPP_BYTE_ORDER_LITTLE_ENDIAN=<1|0>"
#endif
#include <stdint.h>
#define bswap_16(value) ((((value) & 0xff) << 8) | ((value) >> 8))
#define bswap_32(value) \
    (((uint32_t)bswap_16((uint16_t)((value) & 0xffff)) << 16) | \
    (uint32_t)bswap_16((uint16_t)((value) >> 16)))
#define bswap_64(value) \
    (((uint64_t)bswap_32((uint32_t)((value) & 0xffffffff)) \
    << 32) | \
    (uint64_t)bswap_32((uint32_t)((value) >> 32)))
uint64_t dbcppp_native_to_big(uint64_t& v)
{
#if DBCPPP_BYTE_ORDER_LITTLE_ENDIAN == 1
    return bswap_64(v);
#else
    return v;
#endif
}
uint64_t dbcppp_native_to_little(uint64_t& v)
{
#if DBCPPP_BYTE_ORDER_LITTLE_ENDIAN == 1
    return v;
#else
    return bswap_64(v);
#endif
}
uint64_t dbcppp_decode_DutyCycleCmd_1_DutyCycle(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_DutyCycleCmd_1_DutyCycle(uint64_t value)
{
    return value * 1e-05 + 0;
}
uint64_t dbcppp_decode_CurrentLoopCmd_257_IqCurrent(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_CurrentLoopCmd_257_IqCurrent(uint64_t value)
{
    return value * 0.001 + 0;
}
uint64_t dbcppp_decode_CurrentBrakeCmd_513_BrakeCurrent(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_CurrentBrakeCmd_513_BrakeCurrent(uint64_t value)
{
    return value * 0.001 + 0;
}
uint64_t dbcppp_decode_VelocityLoopCmd_769_VelocityERPM(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_VelocityLoopCmd_769_VelocityERPM(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_PositionLoopCmd_1025_PositionDeg(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_PositionLoopCmd_1025_PositionDeg(uint64_t value)
{
    return value * 0.0001 + 0;
}
uint64_t dbcppp_decode_SetOriginCmd_1281_OriginMode(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 56ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_SetOriginCmd_1281_OriginMode(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_PositionVelocityCmd_1537_PosVelPosition(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_PositionVelocityCmd_1537_PosVelPosition(uint64_t value)
{
    return value * 0.0001 + 0;
}
uint64_t dbcppp_decode_PositionVelocityCmd_1537_PosVelSpeed(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 16ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull)
    {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_PositionVelocityCmd_1537_PosVelSpeed(uint64_t value)
{
    return value * 10 + 0;
}
uint64_t dbcppp_decode_PositionVelocityCmd_1537_PosVelAccel(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 0ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull)
    {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_PositionVelocityCmd_1537_PosVelAccel(uint64_t value)
{
    return value * 10 + 0;
}
uint64_t dbcppp_decode_MITControlCmd_2049_MIT_KP(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 52ull;
    data &= 4095ull;
    return data;
}
double dbcppp_rawToPhys_MITControlCmd_2049_MIT_KP(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_MITControlCmd_2049_MIT_KD(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 40ull;
    data &= 4095ull;
    return data;
}
double dbcppp_rawToPhys_MITControlCmd_2049_MIT_KD(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_MITControlCmd_2049_MIT_Position(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 24ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_MITControlCmd_2049_MIT_Position(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_MITControlCmd_2049_MIT_Velocity(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 12ull;
    data &= 4095ull;
    return data;
}
double dbcppp_rawToPhys_MITControlCmd_2049_MIT_Velocity(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_MITControlCmd_2049_MIT_Torque(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 0ull;
    data &= 4095ull;
    return data;
}
double dbcppp_rawToPhys_MITControlCmd_2049_MIT_Torque(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_FeedbackConfigCmd_4097_FbkCfgReserved(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 16ull;
    data &= 281474976710655ull;
    return data;
}
double dbcppp_rawToPhys_FeedbackConfigCmd_4097_FbkCfgReserved(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_FeedbackConfigCmd_4097_FbkCfgParam(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 0ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_FeedbackConfigCmd_4097_FbkCfgParam(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ServoStatusFeedback_10497_FbkPosition(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 48ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull)
    {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_ServoStatusFeedback_10497_FbkPosition(uint64_t value)
{
    return value * 0.1 + 0;
}
uint64_t dbcppp_decode_ServoStatusFeedback_10497_FbkSpeed(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull)
    {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_ServoStatusFeedback_10497_FbkSpeed(uint64_t value)
{
    return value * 10 + 0;
}
uint64_t dbcppp_decode_ServoStatusFeedback_10497_FbkCurrent(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 16ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull)
    {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_ServoStatusFeedback_10497_FbkCurrent(uint64_t value)
{
    return value * 0.01 + 0;
}
uint64_t dbcppp_decode_ServoStatusFeedback_10497_FbkTemperature(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 8ull;
    data &= 255ull;
    if (data & 18446744073709551488ull)
    {
        data |= 18446744073709551488ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_ServoStatusFeedback_10497_FbkTemperature(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ServoStatusFeedback_10497_FbkErrorCode(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 0ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_ServoStatusFeedback_10497_FbkErrorCode(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ExtPositionFeedback_10753_ExtPosition(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_ExtPositionFeedback_10753_ExtPosition(uint64_t value)
{
    return value * 0.01 + 0;
}
uint64_t dbcppp_decode_ServoStartFeedback_11265_StartByte0(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 56ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_ServoStartFeedback_11265_StartByte0(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ServoStartFeedback_11265_StartByte1(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 48ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_ServoStartFeedback_11265_StartByte1(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ServoStartFeedback_11265_StartByte2(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 40ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_ServoStartFeedback_11265_StartByte2(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ServoStartFeedback_11265_StartByte3(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_ServoStartFeedback_11265_StartByte3(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_FingerPosLoopCmd_4096_MCP_x(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 48ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_FingerPosLoopCmd_4096_MCP_x(uint64_t value)
{
    return value * 0.00549325 + 0;
}
uint64_t dbcppp_decode_FingerPosLoopCmd_4096_MCP_y(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_FingerPosLoopCmd_4096_MCP_y(uint64_t value)
{
    return value * 0.00549325 + 0;
}
uint64_t dbcppp_decode_FingerPosLoopCmd_4096_PIP_x(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 16ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_FingerPosLoopCmd_4096_PIP_x(uint64_t value)
{
    return value * 0.00549325 + 0;
}
uint64_t dbcppp_decode_FingerPosLoopCmd_4096_DIP_x(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 0ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_FingerPosLoopCmd_4096_DIP_x(uint64_t value)
{
    return value * 0.00549325 + 0;
}
#ifndef DBCPPP_BYTE_ORDER_LITTLE_ENDIAN
#error "Please pass -DDBCPPP_BYTE_ORDER_LITTLE_ENDIAN=<1|0>"
#endif
#include <stdint.h>
#define bswap_16(value) ((((value) & 0xff) << 8) | ((value) >> 8))
#define bswap_32(value) \
    (((uint32_t)bswap_16((uint16_t)((value) & 0xffff)) << 16) | \
    (uint32_t)bswap_16((uint16_t)((value) >> 16)))
#define bswap_64(value) \
    (((uint64_t)bswap_32((uint32_t)((value) & 0xffffffff)) \
    << 32) | \
    (uint64_t)bswap_32((uint32_t)((value) >> 32)))
uint64_t dbcppp_native_to_big(uint64_t& v)
{
#if DBCPPP_BYTE_ORDER_LITTLE_ENDIAN == 1
    return bswap_64(v);
#else
    return v;
#endif
}
uint64_t dbcppp_native_to_little(uint64_t& v)
{
#if DBCPPP_BYTE_ORDER_LITTLE_ENDIAN == 1
    return v;
#else
    return bswap_64(v);
#endif
}
uint64_t dbcppp_decode_DutyCycleCmd_1_DutyCycle(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_DutyCycleCmd_1_DutyCycle(uint64_t value)
{
    return value * 1e-05 + 0;
}
uint64_t dbcppp_decode_CurrentLoopCmd_257_IqCurrent(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_CurrentLoopCmd_257_IqCurrent(uint64_t value)
{
    return value * 0.001 + 0;
}
uint64_t dbcppp_decode_CurrentBrakeCmd_513_BrakeCurrent(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_CurrentBrakeCmd_513_BrakeCurrent(uint64_t value)
{
    return value * 0.001 + 0;
}
uint64_t dbcppp_decode_VelocityLoopCmd_769_VelocityERPM(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_VelocityLoopCmd_769_VelocityERPM(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_PositionLoopCmd_1025_PositionDeg(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_PositionLoopCmd_1025_PositionDeg(uint64_t value)
{
    return value * 0.0001 + 0;
}
uint64_t dbcppp_decode_SetOriginCmd_1281_OriginMode(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 56ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_SetOriginCmd_1281_OriginMode(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_PositionVelocityCmd_1537_PosVelPosition(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_PositionVelocityCmd_1537_PosVelPosition(uint64_t value)
{
    return value * 0.0001 + 0;
}
uint64_t dbcppp_decode_PositionVelocityCmd_1537_PosVelSpeed(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 16ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull)
    {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_PositionVelocityCmd_1537_PosVelSpeed(uint64_t value)
{
    return value * 10 + 0;
}
uint64_t dbcppp_decode_PositionVelocityCmd_1537_PosVelAccel(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 0ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull)
    {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_PositionVelocityCmd_1537_PosVelAccel(uint64_t value)
{
    return value * 10 + 0;
}
uint64_t dbcppp_decode_MITControlCmd_2049_MIT_KP(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 52ull;
    data &= 4095ull;
    return data;
}
double dbcppp_rawToPhys_MITControlCmd_2049_MIT_KP(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_MITControlCmd_2049_MIT_KD(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 40ull;
    data &= 4095ull;
    return data;
}
double dbcppp_rawToPhys_MITControlCmd_2049_MIT_KD(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_MITControlCmd_2049_MIT_Position(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 24ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_MITControlCmd_2049_MIT_Position(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_MITControlCmd_2049_MIT_Velocity(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 12ull;
    data &= 4095ull;
    return data;
}
double dbcppp_rawToPhys_MITControlCmd_2049_MIT_Velocity(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_MITControlCmd_2049_MIT_Torque(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 0ull;
    data &= 4095ull;
    return data;
}
double dbcppp_rawToPhys_MITControlCmd_2049_MIT_Torque(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_FeedbackConfigCmd_4097_FbkCfgReserved(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 16ull;
    data &= 281474976710655ull;
    return data;
}
double dbcppp_rawToPhys_FeedbackConfigCmd_4097_FbkCfgReserved(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_FeedbackConfigCmd_4097_FbkCfgParam(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 0ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_FeedbackConfigCmd_4097_FbkCfgParam(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ServoStatusFeedback_10497_FbkPosition(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 48ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull)
    {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_ServoStatusFeedback_10497_FbkPosition(uint64_t value)
{
    return value * 0.1 + 0;
}
uint64_t dbcppp_decode_ServoStatusFeedback_10497_FbkSpeed(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull)
    {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_ServoStatusFeedback_10497_FbkSpeed(uint64_t value)
{
    return value * 10 + 0;
}
uint64_t dbcppp_decode_ServoStatusFeedback_10497_FbkCurrent(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 16ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull)
    {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_ServoStatusFeedback_10497_FbkCurrent(uint64_t value)
{
    return value * 0.01 + 0;
}
uint64_t dbcppp_decode_ServoStatusFeedback_10497_FbkTemperature(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 8ull;
    data &= 255ull;
    if (data & 18446744073709551488ull)
    {
        data |= 18446744073709551488ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_ServoStatusFeedback_10497_FbkTemperature(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ServoStatusFeedback_10497_FbkErrorCode(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 0ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_ServoStatusFeedback_10497_FbkErrorCode(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ExtPositionFeedback_10753_ExtPosition(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 4294967295ull;
    if (data & 18446744071562067968ull)
    {
        data |= 18446744071562067968ull;
    }
    return data;
    return data;
}
double dbcppp_rawToPhys_ExtPositionFeedback_10753_ExtPosition(uint64_t value)
{
    return value * 0.01 + 0;
}
uint64_t dbcppp_decode_ServoStartFeedback_11265_StartByte0(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 56ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_ServoStartFeedback_11265_StartByte0(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ServoStartFeedback_11265_StartByte1(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 48ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_ServoStartFeedback_11265_StartByte1(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ServoStartFeedback_11265_StartByte2(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 40ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_ServoStartFeedback_11265_StartByte2(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_ServoStartFeedback_11265_StartByte3(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 255ull;
    return data;
}
double dbcppp_rawToPhys_ServoStartFeedback_11265_StartByte3(uint64_t value)
{
    return value * 1 + 0;
}
uint64_t dbcppp_decode_FingerPosLoopCmd_4096_MCP_x(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 48ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_FingerPosLoopCmd_4096_MCP_x(uint64_t value)
{
    return value * 0.00549325 + 0;
}
uint64_t dbcppp_decode_FingerPosLoopCmd_4096_MCP_y(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 32ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_FingerPosLoopCmd_4096_MCP_y(uint64_t value)
{
    return value * 0.00549325 + 0;
}
uint64_t dbcppp_decode_FingerPosLoopCmd_4096_PIP_x(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 16ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_FingerPosLoopCmd_4096_PIP_x(uint64_t value)
{
    return value * 0.00549325 + 0;
}
uint64_t dbcppp_decode_FingerPosLoopCmd_4096_DIP_x(const void* nbytes)
{
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data = dbcppp_native_to_big(data);
    data >>= 0ull;
    data &= 65535ull;
    return data;
}
double dbcppp_rawToPhys_FingerPosLoopCmd_4096_DIP_x(uint64_t value)
{
    return value * 0.00549325 + 0;
}
