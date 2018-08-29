#include "srslte/utils/cpp_wrappers.h"

// QUEUE uses to store and pass ue sync structures from one thread to another.
std::queue<short_ue_sync_t> ue_sync_queue;

// This MAP is used to transfer basic control messages from main therad to transmission thread.
// Initally, the key will be an increasing number but in the future it could be changed to be the timestamp sent by MAC.
std::map<uint64_t,basic_ctrl_t> phy_transmission_queue;

void push_ue_sync_to_queue(short_ue_sync_t *short_ue_sync) {
  // Push ue sync structure into queue.
  ue_sync_queue.push(*short_ue_sync);
}

void pop_ue_sync_from_queue(short_ue_sync_t *short_ue_sync) {
  // Retrieve element from queue.
  *short_ue_sync = ue_sync_queue.front();
  // Remove element from queue.
  ue_sync_queue.pop();
}

bool is_ue_queue_empty() {
  return ue_sync_queue.empty();
}

int ue_queue_size() {
  return ue_sync_queue.size();
}

// Functions to transfer basic control message from main thread to transmission thread.
void push_tx_basic_control_to_container(uint64_t timestamp, basic_ctrl_t *basic_ctrl) {
  // Push basic control into container.
  phy_transmission_queue.insert(std::pair<uint64_t,basic_ctrl_t>(timestamp,*basic_ctrl));
}

void pop_tx_basic_control_from_container(uint64_t timestamp, basic_ctrl_t *basic_ctrl) {
  // Retrieve mapped element from container.
  *basic_ctrl = phy_transmission_queue[timestamp];
  // Remove element from container.
  phy_transmission_queue.erase(timestamp);
}

bool is_tx_basic_control_container_empty() {
  return phy_transmission_queue.empty();
}

int tx_basic_control_container_size() {
  return phy_transmission_queue.size();
}
