#include <Arduino.h>
#include <hardware/sync.h>
#include <math.h>
#include <pico/util/queue.h>
#include "cancomm.h"
#include "pico/multicore.h"

int mailboxsz = 4;
queue_t xQueue1to0;
queue_t xQueue0to1;

void HubPrintData(CoreMail thisMail) {
  String frase = commandToString((Command)thisMail.command);
  Serial.println("Serial print resposta final: ");
  if (frase.length() == 1) {
    if (thisMail.data == 1) Serial.println("ack");
    if (thisMail.data == 0) Serial.println("err");
    return;
  }
  Serial.print(frase[2]);
  Serial.print(" ");
  Serial.print(thisMail.sourceId);
  Serial.print(" ");
  Serial.println(thisMail.data);
}

volatile CoreMail deskMailbox[4];
spin_lock_t *tableLock;

void WriteCore(CoreMail localCopy) {
  localCopy.isNew = true;
  if (!queue_try_add(&xQueue1to0, &localCopy)) {
    Serial.println("Queue Full!");
  }
}

CoreMail StringtoMail(String line) {
  line.trim();
  if (line.length() == 0) {
    Serial.println("erro em StringtoMail");
  }

  String cmdStr = "";
  int target_id = 0;
  float val_f = 0.0f;

  const int firstSpace = line.indexOf(' ');
  const int secondSpace = line.indexOf(' ', firstSpace + 1);

  if (line.startsWith("g ")) {
    if (secondSpace != -1) {
      cmdStr = line.substring(0, secondSpace);
      target_id = line.substring(secondSpace + 1).toInt();
    }
  } else {
    if (firstSpace != -1) {
      cmdStr = line.substring(0, firstSpace);
      if (secondSpace != -1) {
        target_id = line.substring(firstSpace + 1, secondSpace).toInt();
        val_f = line.substring(secondSpace + 1).toFloat();
      }
    } else {
      cmdStr = line;
    }
  }

  const Command cmd = translateCommand(cmdStr);
  if ((cmd == CMD_CAL || cmd == CMD_PRINT_MATRIX) && target_id == 0) {
    target_id = MY_NODE_ID;
  }

  CoreMail newTask;
  newTask.command = (uint8_t)cmd;
  newTask.data = val_f;
  newTask.sourceId = (uint8_t)MY_NODE_ID;
  newTask.targetId = (uint8_t)target_id;
  newTask.response = false;
  newTask.isNew = true;
  return newTask;
}

void printHelp() {
  Serial.println("\n==== COMMAND LIST ====");
  Serial.println("  cal        : Start calibration on this node.");
  Serial.println("  g i 0      : Print active node ids seen on the bus.");
  Serial.println("  u <i> <v>  : Set LED duty cycle (0.0 to 1.0).");
  Serial.println("  r <i> <v>  : Set target illuminance in lux.");
  Serial.println("  U <i> <v>  : Set LOW bound lux.");
  Serial.println("  O <i> <v>  : Set HIGH bound lux.");
  Serial.println("  m <i> <v>  : Set mode (0=FF, 1=PID, 2=DIST).");
  Serial.println("  A <i> <v>  : Set DIST algo (0=PRIMAL, 1=ADMM, 2=CONSENSUS).");
  Serial.println("  g m <i>    : Get current mode.");
  Serial.println("  g A <i>    : Get DIST algorithm.");
  Serial.println("===========================\n");
}

void setup1() {
  tableLock = spin_lock_init(16);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  const unsigned long start = millis();
  while (!Serial && millis() - start < 8000) { }

  queue_init(&xQueue1to0, sizeof(CoreMail), 32);
  queue_init(&xQueue0to1, sizeof(CoreMail), 32);

  SPI.begin();
  can0.reset();
  can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  can0.setNormalMode();

  delay(3000);
  SendMySerialNumber();

  Serial.print("Ready. Node ID: ");
  Serial.println(MY_NODE_ID);
}

char serialBuf[64];
int bufIdx = 0;
struct can_frame canMsgRx;
CoreMail newmail;

void loop1() {
  serviceNetworkWakeup();

  if (can0.readMessage(&canMsgRx) == MCP2515::ERROR_OK) {
    const bool isRawIdentity = (canMsgRx.can_id == 0x000 && canMsgRx.can_dlc == 8);
    if (isRawIdentity) {
      UpdateID(canMsgRx);
    } else {
      CoreMail mail = ReceberMsg(canMsgRx);
      bool consumed = false;

      if (!mail.response && mail.command == CMD_HUB_SYNC) {
        if (!Serial) {
          resetNodeElectionState();
        }
        consumed = true;
      }

      if (!consumed) {
        if (mail.response && mail.targetId == MY_NODE_ID) {
          WriteCore(mail);
          if (MY_NODE_ID == 1) {
            HubPrintData(mail);
          }
        }

        if (!mail.response && (mail.targetId == MY_NODE_ID || mail.targetId == 0)) {
          WriteCore(mail);
        }
      }
    }
  }

  if (queue_try_remove(&xQueue0to1, &newmail)) {
    if (newmail.targetId == 1 && MY_NODE_ID == 1 && newmail.response) {
      HubPrintData(newmail);
    } else {
      sendCanCoreFormat(newmail);
    }
  }

  while (Serial.available() > 0) {
    const char c = Serial.read();

    if (c == '\n' || c == '\r') {
      serialBuf[bufIdx] = '\0';
      if (bufIdx > 0) {
        String line = String(serialBuf);
        line.trim();

        if (line == "help") {
          printHelp();
        } else {
          CoreMail newMail = StringtoMail(line);
          if (newMail.targetId == 0) {
            WriteCore(newMail);
            sendCanCoreFormat(newMail);
          } else if (newMail.targetId == MY_NODE_ID) {
            WriteCore(newMail);
          } else {
            sendCanCoreFormat(newMail);
          }
        }
      }
      bufIdx = 0;
    } else if (bufIdx < (int)sizeof(serialBuf) - 1) {
      serialBuf[bufIdx++] = c;
    }
  }
}

