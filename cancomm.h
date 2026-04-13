#ifndef CANCOMM_H
#define CANCOMM_H

#include "Arduino.h"
#include <pico/util/queue.h>
#include <SPI.h>
#include "hardware/sync.h"
#include "mcp2515.h"

extern int MY_NODE_ID;
extern const int CS_PIN;

extern MCP2515 can0;

extern queue_t xQueue1to0;
extern queue_t xQueue0to1;

void SendMySerialNumber();
void handleIncomingMessages();
void UpdateID(struct can_frame msg);
void selfList();
void broadcastMyIdentity();
void printList();
void setDuty(float d);
void serviceNetworkWakeup();
void noteSeenNode(uint8_t node_id);
void resetNodeElectionState();

enum Command : uint8_t {
  // Basic control and monitoring
  CMD_SET_DUTY            = 0,   // 'u'
  CMD_GET_DUTY            = 1,   // 'g u'
  CMD_SET_LUX_REF         = 2,   // 'r'
  CMD_GET_LUX_REF         = 3,   // 'g r'
  CMD_GET_LUX_MEASURE     = 4,   // 'g y'
  CMD_SET_FEEDBACK        = 10,  // 'f'
  CMD_GET_FEEDBACK        = 11,  // 'g f'

  // Optimization / bounds
  CMD_GET_HIGH_BOUND      = 21,  // 'g O'
  CMD_SET_HIGH_BOUND      = 22,  // 'O'
  CMD_GET_LOW_BOUND       = 23,  // 'g U'
  CMD_SET_LOW_BOUND       = 24,  // 'U'
  CMD_GET_CURRENT_BOUND   = 25,  // 'g L'
  CMD_GET_ENERGY_COST     = 26,  // 'g C'
  CMD_SET_ENERGY_COST     = 27,  // 'C'
  CMD_SYSTEM_RESTART      = 28,  // 'R'
  CMD_LIVE                = 29,  // 'l'
  CMD_CAL                 = 30,  // 'x' / 'cal'
  CMD_ERR                 = 31,
  CMD_PRINT_MATRIX        = 32,  // 'p'

  // Internal distributed synchronization
  CMD_DIST_STATE_DUTY     = 33,
  CMD_DIST_STATE_AUX      = 34,
  CMD_DIST_MODEL          = 35,

  // Distributed controller config
  CMD_SET_DIST_ALGO       = 36,  // 'A'
  CMD_GET_DIST_ALGO       = 37,  // 'g A'
  CMD_SET_CONTROL_MODE    = 38,  // 'm'
  CMD_GET_CONTROL_MODE    = 39,  // 'g m'
  CMD_GET_NODE_INFO       = 40,  // 'g i'
  CMD_HUB_SYNC            = 41   // internal broadcast
};

struct CoreMail {
  uint8_t command;
  uint8_t targetId;
  uint8_t sourceId;
  float   data;
  int8_t  ackerr = -1;
  bool    response = false;
  bool    isNew = true;
};

CoreMail ReceberMsg(struct can_frame msg);
void executeCommand(CoreMail mail);

Command translateCommand(String input);
String  commandToString(Command cmd);

void sendCanCoreFormat(CoreMail CoreMsg);
void WriteCore(CoreMail localCopy);

extern volatile CoreMail deskMailbox[4];
extern spin_lock_t *tableLock;

#endif
