#pragma once

/// Signals a persistent failure via the user LED. The success path (ecOK) is a one-shot
/// no-op -- sensorstask.cpp's own per-publish LED blink is the real "alive" signal, so there's
/// nothing additional to do here once the device is working.
class ErrorTask
{
public:
    enum class ErrorCode
    {
        ecOK,
        ecSensorsFail,
    };

    ErrorTask(ErrorCode code);
    void execute();

private:
    static const unsigned long SEND_PERIOD_MS = 5 * 1000;
    ErrorCode m_code;

    void sendSensorsFail();
};
