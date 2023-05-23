#include "can_user_functions.h"
#include "can_messages.h"
#include "logger.h"


float CAN_error_rate = 0.0f;
uint32_t _CAN_tot_tx_count = 0, _CAN_err_count = 0;

void _CAN_error_handler(char *);
HAL_StatusTypeDef _CAN_activate_filter(CAN_HandleTypeDef *);
void _CAN_create_primary_can_filter(CAN_FilterTypeDef *);
void _CAN_create_secondary_can_filter(CAN_FilterTypeDef *);
HAL_StatusTypeDef _CAN_activate_interrupts(CAN_HandleTypeDef *);
void _CAN_process_incoming_rx(CAN_HandleTypeDef *, uint32_t);

/**
 * @brief Initialize CAN filters and notifications
 * */
bool CAN_user_init(CAN_HandleTypeDef * hcan) {
    /* Configure CAN filter */
    _CAN_activate_filter(hcan);

    /* Activate interrupts */
    _CAN_activate_interrupts(hcan);

    /* Start the CAN bus */
    if (HAL_CAN_Start(hcan) != HAL_OK)
        _CAN_error_handler("HAL_CAN_Start() failed");

    return true;
}

void _CAN_wait(CAN_HandleTypeDef *nwk) {
  uint32_t start_timestamp = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(nwk) == 0)
    if(HAL_GetTick() > start_timestamp + 5)
      return;
}

HAL_StatusTypeDef CAN_send(CAN_MessageTypeDef *msg, CAN_HandleTypeDef *nwk) {
    CAN_TxHeaderTypeDef header;
    header.StdId = msg->id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = msg->size;
    header.TransmitGlobalTime = DISABLE;

    _CAN_wait(nwk);

    _CAN_tot_tx_count++;
    CAN_error_rate = (float)_CAN_err_count / (float)_CAN_tot_tx_count;

    #if CAN_DEBUG
      LOG_write(LOGLEVEL_DEBUG, "[CAN] Sending [ID 0x%x]", msg->id);
    #endif

    return HAL_CAN_AddTxMessage(nwk, &header, msg->data, NULL);
}

/**
 * @brief Print the error message in the serial console
 * @param msg The message to send over UART
 * */
void _CAN_error_handler(char * msg) {
    _CAN_err_count++;
    CAN_error_rate = (float)_CAN_err_count / (float)_CAN_tot_tx_count;
    
    #if CAN_DEBUG
        LOG_write(LOGLEVEL_ERR, "[CAN/Error Handler] %s", msg);
    #endif
}

/**
 * @brief Activate the proper filter for the given CAN bus
 * @param hcan The CAN bus on which to activate the filter
 * */
HAL_StatusTypeDef _CAN_activate_filter(CAN_HandleTypeDef * hcan) {
    CAN_FilterTypeDef f;
    HAL_StatusTypeDef s;

    if (hcan == &CAN_PRIMARY_NETWORK)
        _CAN_create_primary_can_filter(&f);
    else
        _CAN_create_secondary_can_filter(&f);
    
    if ((s = HAL_CAN_ConfigFilter(hcan, &f)) != HAL_OK)
        _CAN_error_handler("Failed to initialize a CAN filter");
    
    return s;
}

/**
 * @brief Create the CAN filter for the primary CAN network
 * @param f A CAN_FilterTypeDef in which to store the filter data
 * */
void _CAN_create_primary_can_filter(CAN_FilterTypeDef * f) {
    f->FilterMode = CAN_FILTERMODE_IDMASK;
    f->FilterIdLow = 0;
    f->FilterIdHigh = 0xFFFF;
    f->FilterMaskIdHigh = 0;
    f->FilterMaskIdLow = 0;
    f->FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f->FilterBank = 0;
    f->FilterScale = CAN_FILTERSCALE_16BIT;
    f->FilterActivation = ENABLE;
}

/**
 * @brief Create the CAN filter for the secondary CAN network
 * @param f A CAN_FilterTypeDef in which to store the filter data
 * */
void _CAN_create_secondary_can_filter(CAN_FilterTypeDef * f) {
    f->FilterMode = CAN_FILTERMODE_IDMASK;
    f->FilterIdLow = 0;
    f->FilterIdHigh = 0xFFFF;
    f->FilterMaskIdHigh = 0;
    f->FilterMaskIdLow = 0;
    f->FilterFIFOAssignment = CAN_FILTER_FIFO1;
    f->FilterBank = 1;
    f->FilterScale = CAN_FILTERSCALE_16BIT;
    f->FilterActivation = ENABLE;
    f->SlaveStartFilterBank = 1;
}

/**
 * @brief Activate CAN notifications 
 * */
HAL_StatusTypeDef _CAN_activate_interrupts(CAN_HandleTypeDef * hcan) {
    HAL_StatusTypeDef s_rx;

    uint32_t irq_rx = (hcan == &CAN_PRIMARY_NETWORK) ?
        CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING;

    if ((s_rx = HAL_CAN_ActivateNotification(hcan, irq_rx)) != HAL_OK)
        _CAN_error_handler("Failed to activate a CAN RX interrupt");
    
    HAL_CAN_ActivateNotification(hcan,
        CAN_IT_TX_MAILBOX_EMPTY |
        CAN_IT_ERROR_WARNING |
        CAN_IT_ERROR_PASSIVE |
        CAN_IT_BUSOFF |
        CAN_IT_LAST_ERROR_CODE |
        CAN_IT_ERROR
    );
    
    return s_rx;
}

/**
 * @brief HAL callback for CAN errors
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    uint32_t e = hcan->ErrorCode;

    if (e & HAL_CAN_ERROR_EWG)
        _CAN_error_handler("Protocol Error Warning");
    if (e & HAL_CAN_ERROR_EPV)
        _CAN_error_handler("Error Passive");
    if (e & HAL_CAN_ERROR_BOF)
        _CAN_error_handler("Bus-off Error");
    if (e & HAL_CAN_ERROR_STF)
        _CAN_error_handler("Stuff Error");
    if (e & HAL_CAN_ERROR_FOR)
        _CAN_error_handler("Form Error");
    if (e & HAL_CAN_ERROR_ACK)
        _CAN_error_handler("ACK Error");
    if (e & HAL_CAN_ERROR_BR)
        _CAN_error_handler("Bit Recessive Error");
    if (e & HAL_CAN_ERROR_BD)
        _CAN_error_handler("Bit Dominant Error");
    if (e & HAL_CAN_ERROR_CRC)
        _CAN_error_handler("CRC Error");
    if (e & HAL_CAN_ERROR_RX_FOV0)
        _CAN_error_handler("FIFO0 Overrun");
    if (e & HAL_CAN_ERROR_RX_FOV1)
        _CAN_error_handler("FIFO1 Overrun");
    if (e & HAL_CAN_ERROR_TX_ALST0)
        _CAN_error_handler("Mailbox 0 TX failure (arbitration lost)");
    if (e & HAL_CAN_ERROR_TX_TERR0)
        _CAN_error_handler("Mailbox 0 TX failure (tx error)");
    if (e & HAL_CAN_ERROR_TX_ALST1)
        _CAN_error_handler("Mailbox 1 TX failure (arbitration lost)");
    if (e & HAL_CAN_ERROR_TX_TERR1)
        _CAN_error_handler("Mailbox 1 TX failure (tx error)");
    if (e & HAL_CAN_ERROR_TX_ALST2)
        _CAN_error_handler("Mailbox 2 TX failure (arbitration lost)");
    if (e & HAL_CAN_ERROR_TX_TERR2)
        _CAN_error_handler("Mailbox 2 TX failure (tx error)");
    if (e & HAL_CAN_ERROR_TIMEOUT)
        _CAN_error_handler("Timeout Error");
    if (e & HAL_CAN_ERROR_NOT_INITIALIZED)
        _CAN_error_handler("Peripheral not initialized");
    if (e & HAL_CAN_ERROR_NOT_READY)
        _CAN_error_handler("Peripheral not ready");
    if (e & HAL_CAN_ERROR_NOT_STARTED)
        _CAN_error_handler("Peripheral not strated");
    if (e & HAL_CAN_ERROR_PARAM)
        _CAN_error_handler("Parameter Error");
}

/**
 * @brief     HAL callback for RX on FIFO0
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
     _CAN_process_incoming_rx(hcan, CAN_RX_FIFO0);
}

/**
 * @brief     HAL callback for RX on FIFO1
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	_CAN_process_incoming_rx(hcan, CAN_RX_FIFO1);
}

// void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { }
// void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { }
// void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { }

/**
 * @brief   Callback invoked when a CAN message has arrived in the HW FIFO
 * */
#define CAN_DEBUG 1
void _CAN_process_incoming_rx(CAN_HandleTypeDef * hcan, uint32_t rx_fifo) {
    CAN_RxHeaderTypeDef header = {};
	CAN_MessageTypeDef msg = {};
    
    /* Get the last message */
	HAL_CAN_GetRxMessage(hcan, rx_fifo, &header, msg.data);
	msg.id = header.StdId;
	msg.size = header.DLC;

    #if CAN_DEBUG
        char txt[70] = "[CAN] RX: ";
        int offset = strlen(txt);
        offset += sprintf(txt+offset, "[ID: 0x%x - DLC: %d] ", msg.id, msg.size);
        for (int i = 0; i < msg.size; i++)
            offset += sprintf(txt+offset, "%x ", msg.data[i]);
        LOG_write(LOGLEVEL_DEBUG, txt);
    #endif

    /* Process the message */
    CANMSG_process_RX(msg);
}