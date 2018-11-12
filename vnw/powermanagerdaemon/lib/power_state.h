/*
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#ifndef power_state_H_
#define power_state_H_

#define ADDRESS                     "/tmp/pwrstate_socket"
/* The power manager daemon sends power state notification (suspend,
 * resume, shutdown) to the registered clients using one of the
 * below mentioned values.
 */
typedef enum {
  SYS_SUSPEND,
  SYS_RESUME,
  SYS_SHUTDOWN,
} system_state_t;

/* Clients can send acknowledgement to the power manager daemon after
 * they have finished processing the notification event using one of
 * the below mentioned values.
 */
typedef enum {
  SUSPEND_ACK,
  RESUME_ACK,
  SHUTDOWN_ACK,
  ERR,
} client_acknowledgement_t;

typedef struct power_state {
  system_state_t sys_state;
} power_state_t;

typedef struct client_ack {
  client_acknowledgement_t ack;
} client_ack_t;

/*
 * The clients will be notified of the power state change using this callback
 * function, which will be invoked from the power manager daemon whenever a
 * SUSPEND/RESUME/SHUTDOWN state change happens.
 */
typedef int (*callback)(power_state_t pwr_state);

/*
 * Each client should register with the power manager daemon with a callback
 * function which will be invoked from the power manager daemon whenever a
 * SUSPEND/RESUME/SHUTDOWN state change happends.
 * Return value:
 * 0 - success
 * -1 - error
 */
int pwr_state_notification_register(callback cbfunc);

/* Send a notification to the power manager daemon once the client is
 * done processing the notification event.
 * Sends the appropriate client_ack_t value based on the notification event i.e
 * either SUSPEND_ACK/RESUME_ACK/SHUTDOWN_ACK during success and
 * ERR during error or any timeout at the client side
 */
void send_acknowledgement(client_ack_t ack);
#endif
