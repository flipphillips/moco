#include "DMCSerialHandler.h"
#include "HAL/RunnableThread.h"
#include "HAL/PlatformFileManager.h"
#include "GenericPlatform/GenericPlatformFile.h"
#include "Misc/Paths.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

UDMCSerialHandler::UDMCSerialHandler()
    : Worker(nullptr)
    , Thread(nullptr)
{
}

void UDMCSerialHandler::BeginDestroy()
{
    Disconnect();
    Super::BeginDestroy();
}

bool UDMCSerialHandler::Connect(const FString& PortName, int32 BaudRate)
{
    Disconnect();

    Worker = new FDMCSerialWorker(PortName, BaudRate);
    Thread = FRunnableThread::Create(Worker, *FString::Printf(TEXT("DMCSerialWorker_%s"), *PortName));

    if (!Thread)
    {
        delete Worker;
        Worker = nullptr;
        return false;
    }

    return true;
}

TArray<FString> UDMCSerialHandler::GetAvailableSerialPorts()
{
    TArray<FString> Ports;

#if PLATFORM_WINDOWS
    HKEY hKey;
    if (RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("HARDWARE\\DEVICEMAP\\SERIALCOMM"), 0, KEY_READ, &hKey) == ERROR_SUCCESS)
    {
        TCHAR ValueName[256];
        TCHAR ValueData[256];
        DWORD Index = 0;

        while (true)
        {
            DWORD NameLen = sizeof(ValueName) / sizeof(TCHAR);
            DWORD DataLen = sizeof(ValueData);
            DWORD Type;
            if (RegEnumValue(hKey, Index++, ValueName, &NameLen, NULL, &Type, (LPBYTE)ValueData, &DataLen) != ERROR_SUCCESS)
                break;

            if (Type == REG_SZ)
            {
                Ports.Add(FString(ValueData));
            }
        }
        RegCloseKey(hKey);
    }
#else
    IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
    FString DevPath = TEXT("/dev/");

    class FSerialFileVisitor : public IPlatformFile::FDirectoryVisitor
    {
    public:
        TArray<FString>& Ports;
        FSerialFileVisitor(TArray<FString>& InPorts) : Ports(InPorts) {}
        virtual bool Visit(const TCHAR* FilenameOrDirectory, bool bIsDirectory) override
        {
            if (!bIsDirectory)
            {
                FString Name = FPaths::GetCleanFilename(FilenameOrDirectory);
#if PLATFORM_MAC
                if (Name.StartsWith(TEXT("tty.usb")) || Name.StartsWith(TEXT("tty.cu.usb")) || Name.StartsWith(TEXT("tty.SLAB")) || Name.StartsWith(TEXT("tty.usbmodem")))
#else // Linux
                if (Name.StartsWith(TEXT("ttyUSB")) || Name.StartsWith(TEXT("ttyACM")) || Name.StartsWith(TEXT("ttyAMA")) || Name.StartsWith(TEXT("ttyS")))
#endif
                {
                    Ports.Add(FString(FilenameOrDirectory));
                }
            }
            return true;
        }
    };

    FSerialFileVisitor Visitor(Ports);
    PlatformFile.IterateDirectory(*DevPath, Visitor);
#endif

    return Ports;
}

void UDMCSerialHandler::Disconnect()
{
    if (Thread)
    {
        Thread->Kill(true);
        delete Thread;
        Thread = nullptr;
    }

    if (Worker)
    {
        delete Worker;
        Worker = nullptr;
    }
}

bool UDMCSerialHandler::IsConnected() const
{
    return Worker && Worker->IsConnected();
}

void UDMCSerialHandler::SendCommand(int32 MessageID, int32 CommandID, const TArray<uint8>& Payload)
{
    if (!Worker || !Worker->IsConnected()) return;

    TArray<uint8> Packet = DMCLite::CreateHeader((uint32)MessageID, (DMCLite::ECommand)CommandID, Payload.Num());
    Packet.Append(Payload);
    DMCLite::FinalizePacket(Packet);

    Worker->EnqueuePacket(Packet);
}

void UDMCSerialHandler::Tick()
{
    if (!Worker) return;

    TArray<uint8> Packet;
    while (Worker->DequeuePacket(Packet))
    {
        OnPacketReceived.Broadcast(Packet);
    }
}