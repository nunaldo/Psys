#include "cancomm.h"

#include <algorithm>
#include <cstring>
#include <vector>
#include "pico/unique_id.h"

int MY_NODE_ID = 0;

const int CS_PIN = 17;
MCP2515 can0(spi0, CS_PIN);
std::vector<uint64_t> allPICOID;

static unsigned long lastIdentityBroadcastMs = 0;
static uint16_t knownNodeMask = 0;
static bool hubSyncSent = false;

namespace {

void setupCanFiltering(uint8_t my_id) {
  if (can0.setConfigMode() != MCP2515::ERROR_OK) {
    Serial.println("Error: Could not enter Config Mode");
    return;
  }

  can0.setFilterMask(MCP2515::MASK0, false, 0x0F);
  can0.setFilterMask(MCP2515::MASK1, false, 0x0F);
  can0.setFilter(MCP2515::RXF0, false, (uint32_t)my_id);
  can0.setFilter(MCP2515::RXF1, false, (uint32_t)0);

  if (can0.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("Error: Could not return to Normal Mode");
  }
}

uint64_t localBoardUid() {
  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);

  uint64_t uid = 0;
  memcpy(&uid, board_id.id, sizeof(uid));
  return uid;
}

void registerIdentityValue(uint64_t value) {
  if (std::find(allPICOID.begin(), allPICOID.end(), value) == allPICOID.end()) {
    allPICOID.push_back(value);
    std::sort(allPICOID.begin(), allPICOID.end());
  }
}

}  // namespace

void noteSeenNode(uint8_t node_id) {
  if (node_id < 16) {
    knownNodeMask |= (1u << node_id);
  }
}

void SendMySerialNumber() {
  if (Serial) {
    MY_NODE_ID = 1;
    noteSeenNode((uint8_t)MY_NODE_ID);
    setupCanFiltering((uint8_t)MY_NODE_ID);
    hubSyncSent = false;
    lastIdentityBroadcastMs = millis();
    Serial.println("USB serial detected, promoting this node to hub (ID 1).");
    return;
  }

  selfList();
  MY_NODE_ID = 2;
  noteSeenNode((uint8_t)MY_NODE_ID);
  setupCanFiltering((uint8_t)MY_NODE_ID);
  broadcastMyIdentity();
  lastIdentityBroadcastMs = millis();
}

void broadcastMyIdentity() {
  struct can_frame msg;
  msg.can_id = 0x000;
  msg.can_dlc = 8;

  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);
  for (int i = 0; i < 8; ++i) {
    msg.data[i] = board_id.id[i];
  }

  if (can0.sendMessage(&msg) != MCP2515::ERROR_OK) {
    Serial.println("Identity broadcast failed");
  }
}

void selfList() {
  registerIdentityValue(localBoardUid());
}

void UpdateID(struct can_frame msg) {
  uint64_t value = 0;
  memcpy(&value, msg.data, sizeof(value));
  registerIdentityValue(value);

  if (MY_NODE_ID == 1) {
    return;
  }

  const uint64_t my_uid = localBoardUid();
  const int old_id = MY_NODE_ID;

  for (uint32_t i = 0; i < allPICOID.size(); ++i) {
    if (allPICOID[i] == my_uid) {
      MY_NODE_ID = (int)i + 2;
      break;
    }
  }

  noteSeenNode((uint8_t)MY_NODE_ID);
  if (MY_NODE_ID != old_id) {
    setupCanFiltering((uint8_t)MY_NODE_ID);
  }
}

void serviceNetworkWakeup() {
  const unsigned long now = millis();

  if (Serial && MY_NODE_ID != 1) {
    MY_NODE_ID = 1;
    noteSeenNode((uint8_t)MY_NODE_ID);
    setupCanFiltering((uint8_t)MY_NODE_ID);
    hubSyncSent = false;
    Serial.println("USB serial detected, promoting this node to hub (ID 1).");
  }

  if (MY_NODE_ID == 1) {
    if (!hubSyncSent) {
      CoreMail sync_mail;
      sync_mail.command = CMD_HUB_SYNC;
      sync_mail.targetId = 0;
      sync_mail.sourceId = (uint8_t)MY_NODE_ID;
      sync_mail.data = 1.0f;
      sync_mail.response = false;
      sendCanCoreFormat(sync_mail);
      hubSyncSent = true;
      lastIdentityBroadcastMs = now;
    }
    return;
  }

  if ((now - lastIdentityBroadcastMs) >= 1000) {
    broadcastMyIdentity();
    lastIdentityBroadcastMs = now;
  }
}

void resetNodeElectionState() {
  allPICOID.clear();
  knownNodeMask = 0;
  selfList();

  if (Serial) {
    MY_NODE_ID = 1;
    hubSyncSent = false;
  } else {
    MY_NODE_ID = 2;
  }

  noteSeenNode((uint8_t)MY_NODE_ID);
  setupCanFiltering((uint8_t)MY_NODE_ID);

  if (!Serial) {
    broadcastMyIdentity();
  }
  lastIdentityBroadcastMs = millis();
}

void sendCanCoreFormat(CoreMail core_msg) {
  struct can_frame can_msg;

  const uint32_t cmd_part = ((uint32_t)core_msg.command & 0x3F) << 5;
  const uint32_t resp_part = ((uint32_t)core_msg.response & 0x01) << 4;
  const uint32_t target_part = (core_msg.targetId & 0x0F);
  can_msg.can_id = cmd_part | resp_part | target_part;

  can_msg.can_dlc = 5;
  can_msg.data[0] = core_msg.sourceId;
  memcpy(&can_msg.data[1], &core_msg.data, sizeof(float));

  if (can0.sendMessage(&can_msg) != MCP2515::ERROR_OK) {
    Serial.println("Send Error!");
  }
}

CoreMail ReceberMsg(struct can_frame msg) {
  CoreMail new_mail;
  new_mail.targetId = msg.can_id & 0x0F;
  new_mail.response = (msg.can_id >> 4) & 0x01;
  new_mail.command = (msg.can_id >> 5) & 0x3F;
  new_mail.isNew = true;
  new_mail.sourceId = msg.data[0];
  memcpy(&new_mail.data, &msg.data[1], sizeof(float));
  return new_mail;
}

struct CommandMapping {
  const char* str;
  Command cmd;
};

const CommandMapping cmdTable[] = {
  {"u",   CMD_SET_DUTY},          {"g u", CMD_GET_DUTY},
  {"r",   CMD_SET_LUX_REF},       {"g r", CMD_GET_LUX_REF},
  {"g y", CMD_GET_LUX_MEASURE},
  {"f",   CMD_SET_FEEDBACK},      {"g f", CMD_GET_FEEDBACK},
  {"A",   CMD_SET_DIST_ALGO},     {"g A", CMD_GET_DIST_ALGO},
  {"g O", CMD_GET_HIGH_BOUND},    {"O",   CMD_SET_HIGH_BOUND},
  {"g U", CMD_GET_LOW_BOUND},     {"U",   CMD_SET_LOW_BOUND},
  {"g L", CMD_GET_CURRENT_BOUND}, {"g C", CMD_GET_ENERGY_COST},
  {"C",   CMD_SET_ENERGY_COST},
  {"m",   CMD_SET_CONTROL_MODE},  {"g m", CMD_GET_CONTROL_MODE},
  {"g i", CMD_GET_NODE_INFO},
  {"l",   CMD_LIVE},
  {"p",   CMD_PRINT_MATRIX},
  {"x",   CMD_CAL},
  {"cal", CMD_CAL}
};

const int tableSize = sizeof(cmdTable) / sizeof(cmdTable[0]);

Command translateCommand(String input) {
  for (int i = 0; i < tableSize; ++i) {
    if (input == cmdTable[i].str) {
      return cmdTable[i].cmd;
    }
  }
  Serial.println("Error: Unknown Command String");
  return CMD_ERR;
}

String commandToString(Command cmd) {
  for (int i = 0; i < tableSize; ++i) {
    if (cmd == cmdTable[i].cmd) {
      return String(cmdTable[i].str);
    }
  }
  return "unknown";
}
