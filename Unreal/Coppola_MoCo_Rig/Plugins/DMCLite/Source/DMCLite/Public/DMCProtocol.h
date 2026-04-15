#pragma once

#include "CoreMinimal.h"

/**
 * DMC-Lite Protocol Constants and Utilities
 * Ported from Arduino dmc-lite source (dmc_msg.h/cpp)
 */

namespace DMCLite
{
    // Protocol Constants
    static const uint8 SYNC_CHAR_D = 'D';
    static const uint8 SYNC_CHAR_F = 'F';
    static const uint16 HEADER_SIZE = 10;
    static const uint16 CHECKSUM_SIZE = 2;

    // Command IDs
    enum class ECommand : uint16
    {
        HI                  = 0x0001,
        GIO_OUT             = 0x0021,
        GIO_IN              = 0x0022,
        GIO_CAM             = 0x0023,
        MOTOR_STATUS        = 0x0030,
        MOTOR_MOVE          = 0x0031,
        MOTOR_STOP          = 0x0032,
        MOTOR_STOP_ALL      = 0x0033,
        MOTOR_GET_POSITION  = 0x0034,
        MOTOR_RESET_POSITION= 0x0035,
        MOTOR_JOG           = 0x0036,
        MOTOR_CONFIGURE     = 0x0037,
        MOTOR_SET_SPEED     = 0x0038,
        MOTOR_SET_LIMITS    = 0x0039,
        MOTOR_HARD_STOP     = 0x003A,

        RT_UPLOAD_MOVE_BEGIN    = 0x0100,
        RT_UPLOAD_MOVE_AXIS     = 0x0101,
        RT_UPLOAD_MOVE_DMX      = 0x0102,
        RT_UPLOAD_MOVE_TRIGGERS = 0x0104,
        RT_UPLOAD_MOVE_END      = 0x0103,

        RT_POSITION_FRAME       = 0x0110,
        RT_RUN_MOVE             = 0x0111,
        RT_SHOOT_FRAME          = 0x0112,
        RT_SHOOT_FRAME2         = 0x0115,
        RT_GO                   = 0x0113,
        RT_END                  = 0x0114,
        RT_STOP_LOOP            = 0x0116,
        RT_JOG_ALL              = 0x0120
    };

    /**
     * Fletcher-16 Checksum implementation matching Arduino DMC-Lite logic.
     * Calculated mod 255.
     */
    static uint16 ComputeChecksum(const uint8* Data, uint16 Bytes)
    {
        uint16 Sum1 = 0, Sum2 = 0;
        uint16 TLen;

        while (Bytes)
        {
            TLen = ((Bytes >= 20) ? 20 : Bytes);
            Bytes -= TLen;
            do
            {
                Sum2 += (Sum1 += *Data++);
                --TLen;
            } while (TLen);
            Sum1 %= 0xFF;
            Sum2 %= 0xFF;
        }
        return (Sum2 << 8) | Sum1;
    }

    /**
     * Finalizes a packet by calculating the zero-sum checksum bytes (c0, c1).
     * Appends these two bytes to the provided TArray.
     */
    static void FinalizePacket(TArray<uint8>& PacketData)
    {
        uint16 Checksum = ComputeChecksum(PacketData.GetData(), PacketData.Num());
        
        uint8 f0 = Checksum & 0xFF;
        uint8 f1 = (Checksum >> 8) & 0xFF;
        
        uint8 c0 = 0xFF - ((f0 + f1) % 0xFF);
        uint8 c1 = 0xFF - ((f0 + c0) % 0xFF);
        
        PacketData.Add(c0);
        PacketData.Add(c1);
    }

    /**
     * Verifies if a packet's checksum is valid (Zero-Sum check).
     */
    static bool IsPacketValid(const uint8* Data, uint16 TotalBytes)
    {
        return ComputeChecksum(Data, TotalBytes) == 0;
    }

    /**
     * Helper to create a basic packet header.
     * Note: Unreal is Little-Endian, which matches the Arduino protocol.
     */
    static TArray<uint8> CreateHeader(uint32 MessageID, ECommand Command, uint16 PayloadLength)
    {
        TArray<uint8> Header;
        Header.Reserve(HEADER_SIZE + PayloadLength + CHECKSUM_SIZE);
        
        Header.Add(SYNC_CHAR_D);
        Header.Add(SYNC_CHAR_F);
        
        // Append 4-byte ID
        Header.Append((uint8*)&MessageID, 4);
        
        // Append 2-byte Command
        Header.Append((uint8*)&Command, 2);
        
        // Append 2-byte Payload Length
        Header.Append((uint8*)&PayloadLength, 2);
        
        return Header;
    }
}