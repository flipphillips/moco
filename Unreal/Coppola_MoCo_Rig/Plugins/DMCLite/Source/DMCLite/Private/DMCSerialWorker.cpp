#include "DMCSerialWorker.h"
#include "HAL/RunnableThread.h"
#include "HAL/PlatformProcess.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
#else
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

FDMCSerialWorker::FDMCSerialWorker(const FString& InPortName, int32 InBaudRate)
    : PortName(InPortName)
    , BaudRate(InBaudRate)
    , bIsConnected(false)
{
#if PLATFORM_WINDOWS
    hSerial = INVALID_HANDLE_VALUE;
#else
    fd = -1;
#endif
}

FDMCSerialWorker::~FDMCSerialWorker()
{
    Stop();
}

bool FDMCSerialWorker::Init()
{
    return OpenPort();
}

uint32 FDMCSerialWorker::Run()
{
    while (StopTaskCounter.GetValue() == 0)
    {
        if (bIsConnected)
        {
            ProcessRead();
            ProcessWrite();
            FPlatformProcess::Sleep(0.001f); // 1ms poll rate
        }
        else
        {
            // Attempt reconnect every 2 seconds
            FPlatformProcess::Sleep(2.0f);
            OpenPort();
        }
    }
    return 0;
}

void FDMCSerialWorker::Stop()
{
    StopTaskCounter.Increment();
}

void FDMCSerialWorker::Exit()
{
    ClosePort();
}

void FDMCSerialWorker::EnqueuePacket(const TArray<uint8>& Packet)
{
    SendQueue.Enqueue(Packet);
}

bool FDMCSerialWorker::DequeuePacket(TArray<uint8>& OutPacket)
{
    return ReceiveQueue.Dequeue(OutPacket);
}

bool FDMCSerialWorker::OpenPort()
{
#if PLATFORM_WINDOWS
    FString FullPortName = PortName.StartsWith(TEXT("\\\\.\\")) ? PortName : FString::Printf(TEXT("\\\\.\\%s"), *PortName);
    hSerial = CreateFile(*FullPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    
    if (hSerial == INVALID_HANDLE_VALUE) return false;

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) return false;

    dcbSerialParams.BaudRate = BaudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    dcbSerialParams.fBinary = TRUE;
    dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE; // Critical for some Arduinos

    if (!SetCommState(hSerial, &dcbSerialParams)) return false;

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    if (!SetCommTimeouts(hSerial, &timeouts)) return false;

    bIsConnected = true;
    return true;
#else
    fd = open(TCHAR_TO_UTF8(*PortName), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return false;

    struct termios options;
    tcgetattr(fd, &options);
    
    speed_t speed = B115200; // Default to 115200
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    
    bIsConnected = true;
    return true;
#endif
}

void FDMCSerialWorker::ClosePort()
{
    bIsConnected = false;
#if PLATFORM_WINDOWS
    if (hSerial != INVALID_HANDLE_VALUE)
    {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
    }
#else
    if (fd != -1)
    {
        close(fd);
        fd = -1;
    }
#endif
}

void FDMCSerialWorker::ProcessRead()
{
    uint8 RawBuffer[512];
    int32 BytesRead = 0;

#if PLATFORM_WINDOWS
    DWORD dwBytesRead;
    if (ReadFile(hSerial, RawBuffer, sizeof(RawBuffer), &dwBytesRead, NULL))
    {
        BytesRead = (int32)dwBytesRead;
    }
#else
    BytesRead = read(fd, RawBuffer, sizeof(RawBuffer));
#endif

    if (BytesRead > 0)
    {
        for (int32 i = 0; i < BytesRead; ++i)
        {
            ReadBuffer.Add(RawBuffer[i]);
            
            // State Machine for Packet Framing
            if (ReadBuffer.Num() >= 2)
            {
                // Check for Sync 'DF'
                if (ReadBuffer[0] != 'D' || ReadBuffer[1] != 'F')
                {
                    // Shift until we find 'D'
                    ReadBuffer.RemoveAt(0);
                    continue;
                }

                // If we have at least the header
                if (ReadBuffer.Num() >= DMCLite::HEADER_SIZE)
                {
                    uint16 PayloadLen = *(uint16*)&ReadBuffer[8];
                    uint16 FullPacketSize = DMCLite::HEADER_SIZE + PayloadLen + DMCLite::CHECKSUM_SIZE;

                    if (ReadBuffer.Num() >= FullPacketSize)
                    {
                        // Extract Packet
                        TArray<uint8> CompletePacket;
                        CompletePacket.Append(ReadBuffer.GetData(), FullPacketSize);
                        
                        // Validate Checksum
                        if (DMCLite::IsPacketValid(CompletePacket.GetData(), CompletePacket.Num()))
                        {
                            ReceiveQueue.Enqueue(CompletePacket);
                        }

                        // Remove from buffer
                        ReadBuffer.RemoveAt(0, FullPacketSize);
                    }
                }
            }
        }
    }
}

void FDMCSerialWorker::ProcessWrite()
{
    TArray<uint8> Packet;
    while (SendQueue.Dequeue(Packet))
    {
#if PLATFORM_WINDOWS
        DWORD BytesWritten;
        WriteFile(hSerial, Packet.GetData(), Packet.Num(), &BytesWritten, NULL);
#else
        write(fd, Packet.GetData(), Packet.Num());
#endif
    }
}