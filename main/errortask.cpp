#include "errortask.h"

#include "freertos/FreeRTOS.h"


ErrorTask::ErrorTask(ErrorCode code) :
    m_code{code}
{

}

void ErrorTask::execute()
{
    if (m_code != ErrorCode::ecSensorsFail)
        return; // ecOK: nothing to signal, the caller's task deletes itself right after

    // Real fallback: keep signaling indefinitely -- there's no other recovery path once
    // sensor init has failed (the device can't do its actual job).
    while (true) {
        sendSensorsFail();
        vTaskDelay(pdMS_TO_TICKS(SEND_PERIOD_MS));
    }
}

void ErrorTask::sendSensorsFail()
{
    //! \todo
}
