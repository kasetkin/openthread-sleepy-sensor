#include "errortask.h"

#include "common_utils.h"
#include "freertos/FreeRTOS.h"


ErrorTask::ErrorTask(ErrorCode code) :
    m_code{code}
{

}

void ErrorTask::execute()
{
    while (true) {
        switch (m_code) {
            case ErrorCode::ecOK:
                sendOK();
                break;
            case ErrorCode::ecGpsUartFail:
                sendGpsUartFail();
                break;
            case ErrorCode::ecTinyGpsFail:
                sendTinyGpsFail();
                break;
            case ErrorCode::ecUM980Fail:
                sendUM980Fail();
                break;
            case ErrorCode::ecSensorsFail:
                sendSensorsFail();
                break;
            case ErrorCode::ecSdCardFilesystemFail:
                sendSdCardFilesystemFail();
                break;
            default:
                sendOK();
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(SEND_PERIOD_MS));
    }
}


void ErrorTask::sendOK()
{
/// #warning directive is too strong to replace this \todo
    //! \todo 
}

void ErrorTask::sendGpsUartFail()
{
    //! \todo 
}

void ErrorTask::sendTinyGpsFail()
{
    //! \todo 
}

void ErrorTask::sendUM980Fail()
{
    //! \todo 
}

void ErrorTask::sendSensorsFail()
{
    //! \todo 
}

void ErrorTask::sendSdCardFilesystemFail()
{
    //! \todo 
}