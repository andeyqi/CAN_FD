#ifndef __CAN_H__
#define __CAN_H__

#include  <stdint.h>

#define CAN_STATUS_NORMAL           (0)
#define CAN_STATUS_SPI_ERROR        (1)
#define CAN_STATUS_CAN_BUS_ERROR    (2)

/* can extern api */
extern uint8_t xcp_get_can_state(void);
extern uint8_t xcp_get_cc(void);
extern uint8_t xcp_get_charge_state(void);
extern uint8_t xcp_get_diag_cmd(uint8_t *cmd);
extern uint8_t xcp_send_diag_data(uint8_t *data);
extern uint8_t xcp_send_can_msg(uint16_t id,uint8_t * data);

/* can irq handle function */
extern void gpio_can_irq_ind(void *args);
extern void gpio_can_wakeup_irq_ind(void *args);

/* can task entry function */
void tcan4550_thread_entry(void* parameter);

#endif  /* end of __CAN_H__ */
