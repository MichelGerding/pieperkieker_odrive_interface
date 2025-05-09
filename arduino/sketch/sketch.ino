#include <Arduino.h>
#include "ODriveCAN.h"
#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"

#define POSITION_CONTROL_MODE 3
#define VELOCITY_CONTROL_MODE 2


#define CAN_BAUDRATE 250000
#define MCP2515_CS 10
#define MCP2515_INT 2
#define MCP2515_CLK_HZ 8000000

#define NUM_ODRIVES 6 // Change to match your number of ODrives

MCP2515Class& can_intf = CAN;
ODriveCAN* odrives[NUM_ODRIVES];

#define STEERING_MIN -0.75
#define STEERING_MAX 0.75

int STEERING_NODES[] = {0, 1};
int STEERING_NODES_COUNT = 2;

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

ODriveUserData odrive_user_data[NUM_ODRIVES];

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ((ODriveUserData*)user_data)->last_heartbeat = msg;
  ((ODriveUserData*)user_data)->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ((ODriveUserData*)user_data)->last_feedback = msg;
  ((ODriveUserData*)user_data)->received_feedback = true;
}

void onCanMessage(const CanMsg& msg) {
  for (auto odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

bool setupCan() {
  CAN.setPins(MCP2515_CS, MCP2515_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!CAN.begin(CAN_BAUDRATE)) {
    return false;
  }
  CAN.onReceive([](int packet_size) {
    if (packet_size > 8) return;
    CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
    CAN.readBytes(msg.buffer, packet_size);
    onCanMessage(msg);
  });
  return true;
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; i++) delay(100);
  Serial.println("Starting multi-ODrive controller");

  if (!setupCan()) {
    Serial.println("CAN failed to initialize.");
    while (true);
  }

  for (int i = 0; i < NUM_ODRIVES; i++) {
    odrives[i] = new ODriveCAN(wrap_can_intf(can_intf), i); // node_id = i
    odrives[i]->onFeedback(onFeedback, &odrive_user_data[i]);
    odrives[i]->onStatus(onHeartbeat, &odrive_user_data[i]);
  }

  Serial.println("Waiting for all ODrives...");
  while (true) {
    pumpEvents(can_intf);
    bool all_ready = true;
    for (int i = 0; i < NUM_ODRIVES; i++) {
      //Serial.print("Waiting for heartbeat from Odrive ");
      //Serial.println(i);
      if (!odrive_user_data[i].received_heartbeat) {
        all_ready = false;
        Serial.print("Missing heartbeat from ID ");
        Serial.println(i);
        break;
      }
    }
    if (all_ready) break;
    delay(100);
  }

  Serial.println("All ODrives found. Enabling closed-loop control...");

  for (int i = 0; i < NUM_ODRIVES; i++) {
    odrives[i]->clearErrors();
    odrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(200);
  }

  Serial.println("Ready for Serial Commands: P|O|V <index> <value>");
}

boolean array_contains(int arr[], int element, int len) {
  for (int i = 0; i < len; i++) {
    if (arr[i] == element) {
      return true;
    }
  }
  return false;
}

void processSerialCommand(String line) {
  char cmd;
  int id;
  char valStr[16];
  float val;

  line.trim();

  if (sscanf(line.c_str(), "%c %d %s", &cmd, &id, valStr) == 3) {
    val = atof(valStr);  // Convert the string to a float
  } else {
    Serial.print("Invalid command format: ");
    Serial.print(line);
    Serial.print(" - cmd: ");
    Serial.print(cmd);
    Serial.print(" id: ");
    Serial.print(id);
    Serial.print(" val: ");
    Serial.print(val);
    return;
  }
  if (id < 0 || id >= NUM_ODRIVES) {
    Serial.println("Invalid ODrive index.");
    return;
  }

  switch (cmd) {
    case 'P':
      if (array_contains(STEERING_NODES , id, STEERING_NODES_COUNT)) {
        if (val < STEERING_MIN) val = STEERING_MIN;
        if (val > STEERING_MAX) val = STEERING_MAX;
      }

      
      odrives[id]->setControllerMode(POSITION_CONTROL_MODE, 3);
      odrives[id]->setPosition(val);
      break;
    case 'V':
      if (array_contains(STEERING_NODES , id, STEERING_NODES_COUNT)) break;
      
      odrives[id]->setControllerMode(VELOCITY_CONTROL_MODE, 2);
      odrives[id]->setVelocity(val);
      break;
    default:
      Serial.println("Unknown command.");
  }
}

void loop() {
  pumpEvents(can_intf);

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      processSerialCommand(cmd);
    }
  }

  for (int i = 0; i < NUM_ODRIVES; i++) {
    if (odrive_user_data[i].received_feedback) {
      odrive_user_data[i].received_feedback = false;

      float pos = odrive_user_data[i].last_feedback.Pos_Estimate;
      float vel = odrive_user_data[i].last_feedback.Vel_Estimate;

      Serial.print("OD");
      Serial.print(i);
      Serial.print(": Pos=");
      Serial.print(pos, 3);
      Serial.print(" Vel=");
      Serial.println(vel, 3);
    }
  }
}
