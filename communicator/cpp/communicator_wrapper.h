#ifndef _COMMUNICATOR_WRAPPER_H
#define _COMMUNICATOR_WRAPPER_H

#ifdef __cplusplus

#include "LayerCommunicator.h"
#include "interf.pb.h"
#include <iostream>
#include <stdexcept>
#include <mutex>
#include <map>
#include <thread>
#include <condition_variable>
#include "../../srslte/include/srslte/intf/intf.h"

#define ENABLE_COMMUNICATOR_PRINT 0

#define COMMUNICATOR_DEBUG(x) do { if(ENABLE_COMMUNICATOR_PRINT) {std::cout << x;} } while (0)

struct layer_communicator_t {
  communicator::DefaultLayerCommunicator *layer_communicator_cpp;
};

struct message_t {
  communicator::Message message_cpp;
};

void parse_received_message(communicator::Message msg, void *msg_struct, unsigned char * const tx_data_buffer);

extern "C" {
#else
struct layer_communicator_t;
struct message_t;
#endif

typedef struct layer_communicator_t* LayerCommunicator_handle;

typedef struct message_t* message_handle;

void communicator_make(char* module_name, char* target_name, LayerCommunicator_handle* handle);

void communicator_send_phy_stat_message(LayerCommunicator_handle handle, stat_e type, phy_stat_t *phy_stats, message_handle *msg_handle);

void communicator_send_phy_stat_msg_dest(LayerCommunicator_handle handle, int destination, stat_e type, phy_stat_t *phy_stats, message_handle *msg_handle);

bool communicator_get_high_queue(LayerCommunicator_handle handle, void *msg_struct, message_handle *msg_handle, unsigned char * const tx_data_buffer);

void communicator_get_high_queue_wait(LayerCommunicator_handle handle, void *msg_struct, message_handle *msg_handle, unsigned char * const tx_data_buffer);

bool communicator_get_low_queue(LayerCommunicator_handle handle, void *msg_struct, message_handle *msg_handle, unsigned char * const tx_data_buffer);

void communicator_get_low_queue_wait(LayerCommunicator_handle handle, void *msg_struct, message_handle *msg_handle, unsigned char * const tx_data_buffer);

bool communicator_get_low_queue_wait_for(LayerCommunicator_handle handle, uint32_t timeout, void* const msg_struct, message_handle *msg_handle, unsigned char* const tx_data_buffer);

bool communicator_is_high_queue_empty(LayerCommunicator_handle handle);

bool communicator_is_low_queue_empty(LayerCommunicator_handle handle);

void communicator_print_message(message_handle msg_handle);

void communicator_free(LayerCommunicator_handle* handle);

void communicator_free_msg(message_handle *msg_handle);

void communicator_send_basic_control(LayerCommunicator_handle handle, basic_ctrl_t *basic_ctrl);

#ifdef __cplusplus
}
#endif

#endif /* _COMMUNICATOR_WRAPPER_H */
