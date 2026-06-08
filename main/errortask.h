#pragma once

/// @brief  use User LED to send error code (like S-O-S)
class ErrorTask
{
public:
    enum class ErrorCode 
    {
        ecOK,
        ecGpsUartFail,
        ecTinyGpsFail,
        ecUM980Fail,
        ecSensorsFail,
        ecSdCardFilesystemFail
    };

    ErrorTask(ErrorCode code);
    void execute();

private:
    static const unsigned long SEND_PERIOD_MS = 5 * 1000;
    ErrorCode m_code;

    void sendOK();
    void sendGpsUartFail();
    void sendTinyGpsFail();
    void sendUM980Fail();
    void sendSensorsFail();
    void sendSdCardFilesystemFail(); 
};