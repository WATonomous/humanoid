#include <stm32g4xx_hal_fdcan.h>

#ifdef __cplusplus
extern "C"
{
#endif
    void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                       uint32_t ErrorStatusITs);
    void MX_FDCAN2_Init(void);
    void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle);
    void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle);

    typedef struct
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];
    } CAN_RxMessage_t;
    extern FDCAN_HandleTypeDef hfdcan2;
    extern QueueHandle_t can_rx_queue;

#ifdef __cplusplus
}
#endif

