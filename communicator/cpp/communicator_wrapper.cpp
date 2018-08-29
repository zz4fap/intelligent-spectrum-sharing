#include "communicator_wrapper.h"

using namespace SafeQueue;

void communicator_make(char* module_name, char* target_name, LayerCommunicator_handle* handle) {
  communicator::MODULE m = communicator::MODULE_UNKNOWN;
  communicator::MODULE t = communicator::MODULE_UNKNOWN;
  std::string m_str(module_name);
  bool success = true;
  success = success && MODULE_Parse(m_str, &m);
  std::string t_str(target_name);
  success = success && MODULE_Parse(t_str, &t);
  if(success && m != communicator::MODULE_UNKNOWN && t != communicator::MODULE_UNKNOWN) {
    *handle = new layer_communicator_t;
    (*handle)->layer_communicator_cpp = new communicator::DefaultLayerCommunicator(m, {t});
  } else {
    std::cerr << "[COMM ERROR] This should not have happened, invalid modules for communicator_make" << std::endl;
  }
}

void communicator_send_phy_stat_message(LayerCommunicator_handle handle, stat_e type, phy_stat_t *phy_stats, message_handle *msg_handle) {
  communicator_send_phy_stat_msg_dest(handle, handle->layer_communicator_cpp->getDestinationModule(), type, phy_stats, msg_handle);
}

// PHY sends RX statistics to other layers along with received data if available.
// ENUM RX_STAT, TX_STAT
void communicator_send_phy_stat_msg_dest(LayerCommunicator_handle handle, int destination, stat_e type, phy_stat_t *phy_stats, message_handle *msg_handle) {

  // Check if handle is not NULL.
  if(handle==NULL || handle->layer_communicator_cpp==NULL) {
    std::cout << "[COMM ERROR] Communicator Handle is NULL." << std::endl;
    exit(-1);
  }
  // Check if PHY stats is not NULL.
  if(phy_stats==NULL) {
    std::cout << "[COMM ERROR] PHY stats is NULL." << std::endl;
    exit(-1);
  }

  // Create a Internal Message.
  std::shared_ptr<communicator::Internal> internal = std::make_shared<communicator::Internal>();

  // Sequence number used by upper layer to track the response of PHY, i.e., correlates one basic_control message with a phy_stat message.
  internal->set_transaction_index(phy_stats->seq_number); // Transaction index is the same as sequence number.

  // Create PHY stat message.
  communicator::Phy_stat* stat = new communicator::Phy_stat();

  // Set statistics common for both RX and TX.
  stat->set_host_timestamp(phy_stats->host_timestamp);
  stat->set_fpga_timestamp(phy_stats->fpga_timestamp);
	stat->set_frame(phy_stats->frame);
	stat->set_ch(phy_stats->ch);
	stat->set_slot(phy_stats->slot);
	stat->set_mcs(phy_stats->mcs);
	stat->set_num_cb_total(phy_stats->num_cb_total);
	stat->set_num_cb_err(phy_stats->num_cb_err);
  stat->set_wrong_decoding_counter(phy_stats->wrong_decoding_counter);
  // Set the corresponding statistics for RX or TX.
  if(type == RX_STAT) {
    // Create a Receive_r message.
    communicator::Receive_r *receive_r = new communicator::Receive_r();
    // Set the status comming from PHY so that the upper layer knows what happened.
    receive_r->set_result((communicator::TRANSACTION_RESULT)phy_stats->status);
    // Create a PHY stat RX message
    communicator::Phy_rx_stat *stat_rx = new communicator::Phy_rx_stat();
    stat_rx->set_gain(phy_stats->stat.rx_stat.gain);
    stat_rx->set_cqi(phy_stats->stat.rx_stat.cqi);
    stat_rx->set_rssi(phy_stats->stat.rx_stat.rssi);
    stat_rx->set_rsrp(phy_stats->stat.rx_stat.rsrp);
    stat_rx->set_rsrq(phy_stats->stat.rx_stat.rsrp);
    stat_rx->set_sinr(phy_stats->stat.rx_stat.rsrq);
    stat_rx->set_detection_errors(phy_stats->stat.rx_stat.detection_errors);
    stat_rx->set_decoding_errors(phy_stats->stat.rx_stat.decoding_errors);
    stat_rx->set_peak_value(phy_stats->stat.rx_stat.peak_value);
    stat_rx->set_noise(phy_stats->stat.rx_stat.noise);
    stat_rx->set_decoded_cfi(phy_stats->stat.rx_stat.decoded_cfi);
    stat_rx->set_found_dci(phy_stats->stat.rx_stat.found_dci);
    stat_rx->set_last_noi(phy_stats->stat.rx_stat.last_noi);
    stat_rx->set_total_packets_synchronized(phy_stats->stat.rx_stat.total_packets_synchronized);
    stat_rx->set_length(phy_stats->stat.rx_stat.length);
    if(phy_stats->stat.rx_stat.data != NULL) {
      try {
        // Check if length is greater than 0.
        if(phy_stats->stat.rx_stat.length <= 0) {
          std::cout << "[COMM ERROR] Data length is incorrect: " << phy_stats->stat.rx_stat.length << " !" << std::endl;
          exit(-1);
        }
        // Instantiate string with data.
        std::string data_str((char*)phy_stats->stat.rx_stat.data, phy_stats->stat.rx_stat.length);
        receive_r->set_data(data_str);
      } catch(const std::length_error &e) {
        std::cout << "[COMM ERROR] Exception Caught " << e.what() << std::endl;
        std::cout << "[COMM ERROR] Exception Type " << typeid(e).name() << std::endl;
        exit(-1);
      }
    } else {
      if(phy_stats->status == PHY_SUCCESS) {
        std::cout << "[COMM WARNING] PHY status is success but received data is NULL..." << std::endl;
		  }
    }
    // Add PHY RX Stats to PHY Stat.
    stat->set_allocated_rx_stat(stat_rx);
    // Add PHY stat to Send_r message.
    receive_r->set_allocated_stat(stat);	// receive_r has ownership over the stat pointer
    // Add Send_r to the message
    internal->set_allocated_receiver(receive_r); // internal has ownership over the receive_r pointer
  } else if(type == TX_STAT) {
    // Create a Send_r message.
    communicator::Send_r *send_r = new communicator::Send_r();
    // Set the status comming from PHY so that the upper layer knows what happened.
    send_r->set_result((communicator::TRANSACTION_RESULT)phy_stats->status);
    // Create a PHY stat TX message
    communicator::Phy_tx_stat* stat_tx = new communicator::Phy_tx_stat();
    stat_tx->set_power(phy_stats->stat.tx_stat.power);
    stat_tx->set_channel_free_cnt(phy_stats->stat.tx_stat.channel_free_cnt);
    stat_tx->set_channel_busy_cnt(phy_stats->stat.tx_stat.channel_busy_cnt);
    stat_tx->set_free_energy(phy_stats->stat.tx_stat.free_energy);
    stat_tx->set_busy_energy(phy_stats->stat.tx_stat.busy_energy);
    stat_tx->set_total_dropped_slots(phy_stats->stat.tx_stat.total_dropped_slots);
    stat_tx->set_coding_time(phy_stats->stat.tx_stat.coding_time);
    stat_tx->set_rf_boost(phy_stats->stat.tx_stat.rf_boost);
    stat->set_allocated_tx_stat(stat_tx);
    // Add PHY stat to Send_r message.
    send_r->set_allocated_phy_stat(stat);	// send_r has ownership over the stat pointer
    // Add Send_r to the message
    internal->set_allocated_sendr(send_r); // internal has ownership over the send_r pointer
  } else if(type == SENSING_STAT) {
    // Create a Receive_r message.
    communicator::Receive_r *receive_r = new communicator::Receive_r();
    // Set the status comming from PHY so that the upper layer knows what happened.
    receive_r->set_result((communicator::TRANSACTION_RESULT)phy_stats->status);
    // Create a PHY stat RX message
    communicator::Phy_sensing_stat *stat_sensing = new communicator::Phy_sensing_stat();
    stat_sensing->set_frequency(phy_stats->stat.sensing_stat.frequency);
    stat_sensing->set_sample_rate(phy_stats->stat.sensing_stat.sample_rate);
    stat_sensing->set_gain(phy_stats->stat.sensing_stat.gain);
    stat_sensing->set_rssi(phy_stats->stat.sensing_stat.rssi);
    stat_sensing->set_length(phy_stats->stat.sensing_stat.length);
    if(phy_stats->stat.sensing_stat.data != NULL) {
      try {
        // Check if length is greater than 0.
        if(phy_stats->stat.sensing_stat.length <= 0) {
          std::cout << "[COMM ERROR] Data length is incorrect: " << phy_stats->stat.sensing_stat.length << " !" << std::endl;
          exit(-1);
        }
        // Instantiate string with data.
        std::string data_str((char*)phy_stats->stat.sensing_stat.data, phy_stats->stat.sensing_stat.length);
        receive_r->set_data(data_str);
      } catch(const std::length_error &e) {
        std::cout << "[COMM ERROR] Exception Caught " << e.what() << std::endl;
        std::cout << "[COMM ERROR] Exception Type " << typeid(e).name() << std::endl;
        exit(-1);
      }
    } else {
      if(phy_stats->status == PHY_SUCCESS) {
        std::cout << "[COMM ERROR] PHY status is success but sensing data is NULL..." << std::endl;
		  }
    }
    // Add PHY Sensing Stats to PHY Stat.
    stat->set_allocated_sensing_stat(stat_sensing);
    // Add PHY stat to Send_r message.
    receive_r->set_allocated_stat(stat);	// receive_r has ownership over the stat pointer
    // Add Send_r to the message
    internal->set_allocated_receiver(receive_r); // internal has ownership over the receive_r pointer
  } else {
    std::cout << "[COMM ERROR] Undefined type of statistics." << std::endl;
    exit(-1);
  }

  // Create a Message object what will be sent upwards.
  communicator::Message msg(communicator::MODULE_PHY, (communicator::MODULE)destination, internal);

  //Put message in the sensing queue.
  handle->layer_communicator_cpp->send(msg);

  // Add a Message object only if different from NULL.
  if(msg_handle != NULL) {
    *msg_handle = new message_t;
    (*msg_handle)->message_cpp = msg;
  }
}

// This function will retrieve messages (basic_control) addressed to the PHY from the QUEUE.
// OBS.: It is a blocking call.
// MUST cast to basic_ctrl_t *
void communicator_get_high_queue_wait(LayerCommunicator_handle handle, void *msg_struct, message_handle *msg_handle, unsigned char * const tx_data_buffer) {

  //WAIT UNTIL THERE IS A MESSAGE IN THE HIGH QUEUE
  communicator::Message msg = handle->layer_communicator_cpp->get_high_queue().pop_wait();
  // Parse message into the corresponding structure.
  parse_received_message(msg, msg_struct, tx_data_buffer);

  // Add a Message object only if diffrent from NULL.
  if(msg_handle != NULL) {
    *msg_handle = new message_t;
    (*msg_handle)->message_cpp = msg;
  }
}

// Non-blocking call.
bool communicator_get_high_queue(LayerCommunicator_handle handle, void *msg_struct, message_handle *msg_handle, unsigned char * const tx_data_buffer) {
  bool ret;
  communicator::Message msg;
  if(handle->layer_communicator_cpp->get_high_queue().pop(msg)) {
    ret = true;
    // Parse message into the corresponding structure.
    parse_received_message(msg, msg_struct, tx_data_buffer);

    // Add a Message object only if diffrent from NULL.
    if(msg_handle != NULL) {
      *msg_handle = new message_t;
      (*msg_handle)->message_cpp = msg;
    }
  } else {
    msg_struct = NULL;  // Make sure it returns NULL for the caller to check before using it.
    msg_handle = NULL;  // Make sure it returns NULL for the caller to check before using it.
    ret = false;
    std::cout << "[COMM ERROR] No message in High Priority QUEUE." << std::endl;
  }

  return ret;
}

// Blocking call.
void communicator_get_low_queue_wait(LayerCommunicator_handle handle, void *msg_struct, message_handle *msg_handle, unsigned char * const tx_data_buffer) {

  //WAIT UNTIL THERE IS A MESSAGE IN THE LOW QUEUE
  communicator::Message msg = handle->layer_communicator_cpp->get_low_queue().pop_wait();
  // Parse message into the corresponding structure.
  parse_received_message(msg, msg_struct, tx_data_buffer);

  // Add a Message object only if diffrent from NULL.
  if(msg_handle != NULL) {
    *msg_handle = new message_t;
    (*msg_handle)->message_cpp = msg;
  }
}

// Blocking call, it waits until someone pushes a message into the QUEUE or the waiting times out.
bool communicator_get_low_queue_wait_for(LayerCommunicator_handle handle, uint32_t timeout, void* const msg_struct, message_handle *msg_handle, unsigned char* const tx_data_buffer) {
  //WAIT UNTIL THERE IS A MESSAGE IN THE LOW QUEUE
  communicator::Message msg;
  bool ret = handle->layer_communicator_cpp->get_low_queue().pop_wait_for(std::chrono::milliseconds(timeout), msg);
  if(ret) {
    // Parse message into the corresponding structure.
    parse_received_message(msg, msg_struct, tx_data_buffer);

    // Add a Message object only if diffrent from NULL.
    if(msg_handle != NULL) {
      *msg_handle = new message_t;
      (*msg_handle)->message_cpp = msg;
    }
  }
  return ret;
}

// Non-blocking call.
bool communicator_get_low_queue(LayerCommunicator_handle handle, void *msg_struct, message_handle *msg_handle, unsigned char * const tx_data_buffer) {
  bool ret;
  //CHECK IF THERE IS A MESSAGE IN THE LOW QUEUE
  communicator::Message msg;

  if(handle->layer_communicator_cpp->get_low_queue().pop(msg)) {
    ret = true;
    // Parse message into the corresponding structure.
    parse_received_message(msg, msg_struct, tx_data_buffer);

    // Add a Message object only if diffrent from NULL.
    if(msg_handle != NULL) {
      *msg_handle = new message_t;
      (*msg_handle)->message_cpp = msg;
    }
  } else {
    msg_struct = NULL; // Make sure it returns NULL for the caller to check before using it.
    msg_handle = NULL; // Make sure it returns NULL for the caller to check before using it.
    ret = false;
  }

  return ret;
}

void communicator_print_message(message_handle msg_handle) {
  // Print the Message.
  std::cout << "[COMM PRINT] Get message " << msg_handle->message_cpp << std::endl;
}

void communicator_free(LayerCommunicator_handle* handle) {
  communicator::DefaultLayerCommunicator *ptr = ((*handle)->layer_communicator_cpp);
  delete ptr;
  delete *handle;
  *handle = NULL;
}

void communicator_free_msg(message_handle *msg_handle) {
  delete *msg_handle;
  *msg_handle = NULL;
}

void parse_received_message(communicator::Message msg, void *msg_struct, unsigned char * const tx_data_buffer) {

  if(msg_struct==NULL) {
    std::cout << "[COMM ERROR] Message struct is NULL." << std::endl;
    exit(-1);
  }

  // Check if message was addressed to PHY.
  if(msg.destination == communicator::MODULE::MODULE_PHY) {

    // Cast msg_struct to be a basic control struct.
    basic_ctrl_t *basic_ctrl = (basic_ctrl_t *)msg_struct;

    // Cast protobuf message into a Internal message one.
    std::shared_ptr<communicator::Internal> internal = std::static_pointer_cast<communicator::Internal>(msg.message);
    basic_ctrl->seq_number = internal->transaction_index();

    // Switch case used to select among different messages.
    switch (internal->payload_case()) {
      case communicator::Internal::kReceive:
      {
        // Copy from Internal message values into basic control structure.
        basic_ctrl->trx_flag = (trx_flag_e)internal->receive().basic_ctrl().trx_flag();
        // Check if the flag really is RX.
        if(basic_ctrl->trx_flag != PHY_RX_ST) {
          std::cout << "[COMM ERROR] TRX Flag different from RX!!" << std::endl;
          exit(-1);
        }
        basic_ctrl->send_to = internal->receive().basic_ctrl().send_to();
        basic_ctrl->bw_idx = internal->receive().basic_ctrl().bw_index();
        basic_ctrl->ch = internal->receive().basic_ctrl().ch();
        basic_ctrl->frame = internal->receive().basic_ctrl().frame();
        basic_ctrl->slot = internal->receive().basic_ctrl().slot();
        basic_ctrl->timestamp = internal->receive().basic_ctrl().timestamp();
        basic_ctrl->mcs = internal->receive().basic_ctrl().mcs();
        basic_ctrl->gain = internal->receive().basic_ctrl().gain();
        basic_ctrl->rf_boost = internal->receive().basic_ctrl().rf_boost();
        basic_ctrl->length = internal->receive().basic_ctrl().length();
        break;
      }
      case communicator::Internal::kSend:
      {
        // Copy from Internal message values into basic control structure.
        basic_ctrl->trx_flag = (trx_flag_e)internal->send().basic_ctrl().trx_flag();
        // Check if the flag really is TX.
        if(basic_ctrl->trx_flag != PHY_TX_ST) {
          std::cout << "[COMM ERROR] TRX Flag different from TX!!" << std::endl;
          exit(-1);
        }
        if(tx_data_buffer == NULL) {
          std::cout << "[COMM ERROR] TX data buffer is NULL!!" << std::endl;
          exit(-1);
        }
        basic_ctrl->send_to = internal->send().basic_ctrl().send_to();
        basic_ctrl->bw_idx = internal->send().basic_ctrl().bw_index();
        basic_ctrl->ch = internal->send().basic_ctrl().ch();
        basic_ctrl->frame = internal->send().basic_ctrl().frame();
        basic_ctrl->slot = internal->send().basic_ctrl().slot();
        basic_ctrl->timestamp = internal->send().basic_ctrl().timestamp();
        basic_ctrl->mcs = internal->send().basic_ctrl().mcs();
        basic_ctrl->gain = internal->send().basic_ctrl().gain();
        basic_ctrl->rf_boost = internal->send().basic_ctrl().rf_boost();
        basic_ctrl->length = internal->send().basic_ctrl().length();
        // Only do basic checking. Other checking is done by PHY.
        if(basic_ctrl->length <= 0) {
          std::cout << "[COMM ERROR] Invalid Basic control length field: " << basic_ctrl->length << std::endl;
          exit(-1);
        }
        // Only allocate memory for data if it is a basic_control for TX processing.
        size_t data_length = internal->send().app_data().data().length();
        // The data length field should have the same value as the basic control length.
        if(data_length != basic_ctrl->length) {
          std::cout << "[COMM ERROR] Basic control length: " << basic_ctrl->length << " does not match data length: " << data_length << std::endl;
          exit(-1);
        }
        memcpy(tx_data_buffer, (unsigned char*)internal->send().app_data().data().c_str(), data_length);
        basic_ctrl->data = tx_data_buffer;
        break;
      }
      case communicator::Internal::kSet:
      case communicator::Internal::kSetr:
      case communicator::Internal::kReceiver:
      case communicator::Internal::kStats:
      case communicator::Internal::kGet:
      case communicator::Internal::kGetr:
      case communicator::Internal::kSendr:
      default:
        std::cout << "[COMM ERROR] This message type is not handled by PHY!!" << std::endl;
        break;
    }
  } else {
    std::cout << "[COMM ERROR] Message not addressed to PHY." << std::endl;
    exit(-1);
  }
}

bool communicator_is_high_queue_empty(LayerCommunicator_handle handle) {
  return handle->layer_communicator_cpp->get_high_queue().empty();
}

bool communicator_is_low_queue_empty(LayerCommunicator_handle handle) {
  return handle->layer_communicator_cpp->get_low_queue().empty();
}

void communicator_send_basic_control(LayerCommunicator_handle handle, basic_ctrl_t *basic_ctrl) {

  std::shared_ptr<communicator::Internal> message = std::make_shared<communicator::Internal>();
  message->set_transaction_index(basic_ctrl->seq_number);
  message->set_owner_module(communicator::MODULE_MAC);
  communicator::Message mes = communicator::Message(communicator::MODULE_MAC, communicator::MODULE_PHY, message);

  communicator::Send* send = new communicator::Send();
  mes.message->set_allocated_send(send);

  communicator::Basic_ctrl *ctrl = new communicator::Basic_ctrl();
  send->set_allocated_basic_ctrl(ctrl);

  communicator::Application_data *app_data = new communicator::Application_data();
  send->set_allocated_app_data(app_data);

  ctrl->set_trx_flag((communicator::Basic_ctrl_TRX)basic_ctrl->trx_flag);
  ctrl->set_bw_index((communicator::BW_INDEX)basic_ctrl->bw_idx);
  ctrl->set_ch(basic_ctrl->ch);
  ctrl->set_timestamp(basic_ctrl->timestamp);
  ctrl->set_mcs(basic_ctrl->mcs);
  ctrl->set_gain(basic_ctrl->gain);
  ctrl->set_length(basic_ctrl->length);

  if(basic_ctrl->length > 0) {
    std::string *data_str = new std::string((char*)basic_ctrl->data, basic_ctrl->length);
    app_data->set_allocated_data(data_str);
  }

  handle->layer_communicator_cpp->send(mes);
}
