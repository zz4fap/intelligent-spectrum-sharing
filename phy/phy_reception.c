#include "phy_reception.h"

// RX local offset.
#define PHY_RX_LO_OFFSET +42.0e6

#define TIMED_COMMAND_ENABLED 0

#define NUMBER_OF_DECODED_DATA_BUFFERS 500

#define ENBALE_RX_INFO_PLOT 0

#define WRITE_RX_SUBFRAME_INTO_FILE 0

// *********** Global variables ***********
// This handle is used to pass some important objects to the PHY RX thread work function.
static phy_reception_t *phy_reception_handle = NULL;

// *********** Definition of functions ***********
// This function is used to set everything needed for the phy reception thread to run accordinly.
int phy_reception_initialize(LayerCommunicator_handle handle, srslte_rf_t *rf, transceiver_args_t *args) {
  // Allocate memory for a new Tx object.
  phy_reception_handle = (phy_reception_t*)srslte_vec_malloc(sizeof(phy_reception_t));
  // Check if memory allocation was correctly done.
  if(phy_reception_handle == NULL) {
    PHY_RX_ERROR("Error allocating memory for Rx context.\n",0);
    return -1;
  }
  // Initialize handle context object.
  phy_reception_init_handle();
  // Set RX sample rate accoring to the number of PRBs.
  phy_reception_set_rx_sample_rate(rf, args->nof_prb, args->use_std_carrier_sep);
  // Get bandwidth from number of physical resource blocks.
  double default_rx_bandwidth = helpers_get_bw_from_nprb(args->nof_prb);
  // Configure RX frequency and gains.
  phy_reception_set_initial_rx_freq_and_gain(rf, args->initial_rx_gain, args->initial_agc_gain, args->competition_center_frequency, args->competition_bw, default_rx_bandwidth, args->default_rx_channel);
  // Initialize struture with Cell parameters.
  phy_reception_init_cell_parameters(args->nof_prb, args->radio_id, args->nof_ports);
  // Convert from number of Resource Blocks to BW Index.
  uint32_t bw_idx = helpers_get_bw_index_from_prb(args->nof_prb);
  // Initialize structure with last configured RX basic control.
  phy_reception_init_last_basic_control(args->competition_center_frequency, args->initial_rx_gain, bw_idx, args->radio_id, args->default_rx_channel);
  // Do all UE related intilization: SYNC, MIB and Downlink.
  phy_reception_ue_init(rf, args->rnti, args->initial_rx_gain, args->initial_agc_gain, args->initial_subframe_index);
  // Try to stop RX Stream if that is open, flush reception buffer and open RX Stream.
  phy_reception_initialize_rx_stream(rf);
  // Set the maxium number of turbo decoder iterations.
  srslte_ue_dl_set_max_noi(&phy_reception_handle->ue_dl, args->max_turbo_decoder_noi);
  // Set phy reception handler.
  phy_reception_set_handler(handle, rf, args);
  // Initialize mutex for basic control fields access.
  if(pthread_mutex_init(&phy_reception_handle->phy_rx_basic_control_mutex, NULL) != 0) {
    PHY_RX_ERROR("Mutex for basic control access init failed.\n",0);
    return -1;
  }
  // Initialize mutex for ue sync structure access.
  if(pthread_mutex_init(&phy_reception_handle->phy_reception_ue_sync_mutex, NULL) != 0) {
    PHY_RX_ERROR("Mutex for ue sync structure access init failed.\n",0);
    return -1;
  }
  // Initialize conditional variable.
  if(pthread_cond_init(&phy_reception_handle->phy_reception_ue_sync_cv, NULL)) {
    PHY_RX_ERROR("Conditional variable init failed.\n",0);
    return -1;
  }
  // Start synchronization thread.
  if(phy_reception_start_sync_thread() < 0) {
    PHY_RX_ERROR("Error when starting PHY synchronization thread.\n",0);
    return -1;
  }
  // Start decoding thread.
  if(phy_reception_start_decoding_thread() < 0) {
    PHY_RX_ERROR("Error when starting PHY decoding thread.\n",0);
    return -1;
  }

#if(ENBALE_RX_INFO_PLOT==1)
  // Initialize plot.
  if(args->plot_rx_info == true) {
    init_plot();
  }
#endif
  return 0;
}

// Free all the resources used by the phy reception module.
int phy_reception_uninitialize(transceiver_args_t *args) {
  // Stop synchronization thread.
  if(phy_reception_stop_sync_thread() < 0) {
    PHY_RX_ERROR("Error when stopping PHY synchronization thread.\n",0);
    return -1;
  }
  // Stop decoding thread.
  if(phy_reception_stop_decoding_thread() < 0) {
    PHY_RX_ERROR("Error when stopping PHY decoding thread.\n",0);
    return -1;
  }
  // Destroy mutex for basic control fields access.
  pthread_mutex_destroy(&phy_reception_handle->phy_rx_basic_control_mutex);
  // Destroy mutex for ue sync structure access.
  pthread_mutex_destroy(&phy_reception_handle->phy_reception_ue_sync_mutex);
  // Free all related UE Downlink structures.
  phy_reception_ue_free(phy_reception_handle->rf, phy_reception_handle->initial_rx_gain);
  // Stop RX Stream and Flush reception buffer.
  phy_reception_stop_rx_stream_and_flush_buffer(phy_reception_handle->rf);
  // Free plot context object if plot is enabled.
#if(ENBALE_RX_INFO_PLOT==1)
  // free plot object.
  if(args->plot_rx_info == true) {
    free_plot();
  }
#endif
  // Free memory used to store Rx object.
  if(phy_reception_handle) {
    free(phy_reception_handle);
    phy_reception_handle = NULL;
  }
  return 0;
}

int phy_reception_start_decoding_thread() {
  // Enable receiving thread.
  phy_reception_handle->run_phy_decoding_thread = true;
  // Create threads to perform phy reception.
  pthread_attr_init(&phy_reception_handle->phy_reception_decoding_thread_attr);
  pthread_attr_setdetachstate(&phy_reception_handle->phy_reception_decoding_thread_attr, PTHREAD_CREATE_JOINABLE);
  // Create thread to sense channel.
  int rc = pthread_create(&phy_reception_handle->phy_reception_decoding_thread_id, &phy_reception_handle->phy_reception_decoding_thread_attr, phy_reception_decoding_work, (void *)&phy_reception_handle);
  if(rc) {
    PHY_RX_ERROR("Return code from PHY reception pthread_create() is %d\n", rc);
    return -1;
  }
  return 0;
}

int phy_reception_stop_decoding_thread() {
  phy_reception_handle->run_phy_decoding_thread = false; // Stop decoding thread.
  pthread_attr_destroy(&phy_reception_handle->phy_reception_decoding_thread_attr);
  int rc = pthread_join(phy_reception_handle->phy_reception_decoding_thread_id, NULL);
  if(rc) {
    PHY_RX_ERROR("Return code from phy reception pthread_join() is %d\n", rc);
    return -1;
  }
  return 0;
}

int phy_reception_start_sync_thread() {
  // Enable synchronization thread.
  phy_reception_handle->run_phy_synchronization_thread = true;
  // Create threads to perform phy synchronization.
  pthread_attr_init(&phy_reception_handle->phy_reception_sync_thread_attr);
  pthread_attr_setdetachstate(&phy_reception_handle->phy_reception_sync_thread_attr, PTHREAD_CREATE_JOINABLE);
  // Create thread to sense channel.
  int rc = pthread_create(&phy_reception_handle->phy_reception_sync_thread_id, &phy_reception_handle->phy_reception_sync_thread_attr, phy_reception_sync_work, (void *)&phy_reception_handle);
  if(rc) {
    PHY_RX_ERROR("Return code from PHY synchronization pthread_create() is %d\n", rc);
    return -1;
  }
  return 0;
}

int phy_reception_stop_sync_thread() {
  phy_reception_handle->run_phy_synchronization_thread = false; // Stop synchronization thread.
  // Notify condition variable.
  pthread_cond_signal(&phy_reception_handle->phy_reception_ue_sync_cv);
  pthread_attr_destroy(&phy_reception_handle->phy_reception_sync_thread_attr);
  int rc = pthread_join(phy_reception_handle->phy_reception_sync_thread_id, NULL);
  if(rc) {
    PHY_RX_ERROR("Return code from phy synchronization pthread_join() is %d\n", rc);
    return -1;
  }
  // Destory conditional variable.
  if(pthread_cond_destroy(&phy_reception_handle->phy_reception_ue_sync_cv) != 0) {
    PHY_RX_ERROR("Conditional variable destruction failed.\n",0);
    return -1;
  }
  return 0;
}

// Initialize struture with Cell parameters.
void phy_reception_init_cell_parameters(uint32_t num_of_prb, uint32_t radio_id, uint32_t nof_ports) {
  phy_reception_handle->cell_ue.nof_prb         = num_of_prb;         // nof_prb
  phy_reception_handle->cell_ue.nof_ports       = nof_ports;          // nof_ports
  phy_reception_handle->cell_ue.bw_idx          = 0;                  // bw idx
  phy_reception_handle->cell_ue.id              = radio_id;           // cell_id
  phy_reception_handle->cell_ue.cp              = SRSLTE_CP_NORM;     // cyclic prefix
  phy_reception_handle->cell_ue.phich_length    = SRSLTE_PHICH_NORM;  // PHICH length
  phy_reception_handle->cell_ue.phich_resources = SRSLTE_PHICH_R_1;   // PHICH resources
}

// Initialize structure with last configured RX basic control.
void phy_reception_init_last_basic_control(uint32_t center_freq, int32_t rx_gain, uint32_t bw_idx, uint32_t radio_id, uint32_t default_channel) {
  phy_reception_handle->last_rx_basic_control.trx_flag     = PHY_RX_ST;
  phy_reception_handle->last_rx_basic_control.seq_number   = 0;
  phy_reception_handle->last_rx_basic_control.send_to      = radio_id;
  phy_reception_handle->last_rx_basic_control.bw_idx       = bw_idx;
  phy_reception_handle->last_rx_basic_control.ch           = default_channel;
  phy_reception_handle->last_rx_basic_control.frame        = 0;
  phy_reception_handle->last_rx_basic_control.slot         = 0;
  phy_reception_handle->last_rx_basic_control.timestamp    = 0;
  phy_reception_handle->last_rx_basic_control.mcs          = 0;
  phy_reception_handle->last_rx_basic_control.gain         = rx_gain;
  phy_reception_handle->last_rx_basic_control.length       = 1;
}

void phy_reception_set_handler(LayerCommunicator_handle handle, srslte_rf_t *rf, transceiver_args_t *args) {
  phy_reception_handle->phy_comm_handle          = handle;
  phy_reception_handle->rf                       = rf;
  phy_reception_handle->initial_rx_gain          = args->initial_rx_gain;
  phy_reception_handle->competition_bw           = args->competition_bw;
  phy_reception_handle->competition_center_freq  = args->competition_center_frequency;
  phy_reception_handle->rnti                     = args->rnti;
  phy_reception_handle->initial_agc_gain         = args->initial_agc_gain;
  phy_reception_handle->use_std_carrier_sep      = args->use_std_carrier_sep;
  phy_reception_handle->initial_subframe_index   = args->initial_subframe_index;
  phy_reception_handle->add_tx_timestamp         = args->add_tx_timestamp;
  phy_reception_handle->plot_rx_info             = args->plot_rx_info;
}

void phy_reception_change_parameters(srslte_rf_t *rf, basic_ctrl_t *bc) {

  // Change Center frequency only if one of the parameters have changed.
  if(phy_reception_handle->last_rx_basic_control.ch != bc->ch || phy_reception_handle->last_rx_basic_control.bw_idx != bc->bw_idx) {
    // Channel Number.
    uint32_t rx_channel = bc->ch;
    // Bandwidth.
    float rx_bandwidth = helpers_get_bandwidth_float(bc->bw_idx);
    if(rx_bandwidth < 0.0) {
      PHY_RX_ERROR("Undefined BW Index: %d....\n",bc->bw_idx);
      exit(-1);
    }
    // Calculate cntral frequency for the channel.
    double rx_channel_center_freq = helpers_calculate_channel_center_frequency(phy_reception_handle->competition_center_freq, phy_reception_handle->competition_bw, rx_bandwidth, rx_channel);
    // Set central frequency for reception.
    double lo_offset = rf->num_of_channels == 1 ? 0.0:(double)PHY_RX_LO_OFFSET;
#if(TIMED_COMMAND_ENABLED==1)
    double actual_rx_freq = srslte_rf_set_rx_freq_cmd(rf, rx_channel_center_freq, lo_offset, bc->timestamp, 350, PHY_CHANNEL);
#else
    double actual_rx_freq = srslte_rf_set_rx_freq2(rf, rx_channel_center_freq, lo_offset, PHY_CHANNEL);
#endif
    // Check if actual frequency is inside a range of +/- 10 Hz.
    if(actual_rx_freq < (rx_channel_center_freq - 10.0) || actual_rx_freq > (rx_channel_center_freq + 10.0)) {
       PHY_RX_ERROR("Requested freq.: %1.2f [MHz] - Actual freq.: %1.2f [MHz]................\n", rx_channel_center_freq, actual_rx_freq);
    }
    // If bandwidth has changed then free and initialize UE DL.
    if(phy_reception_handle->last_rx_basic_control.bw_idx != bc->bw_idx) {
      // Stop synchronization thread before configuring a new bandwidth.
      if(phy_reception_stop_sync_thread() < 0) {
        PHY_RX_ERROR("Error stopping synchronization thread.\n",0);
        exit(-1);
      }
      // Stop decoding thread before configuring a new bandwidth.
      if(phy_reception_stop_decoding_thread() < 0) {
        PHY_RX_ERROR("Error stopping decoding thread.\n",0);
        exit(-1);
      }
      // Set the new number of PRB based on PRB retrieved from BW index..
      phy_reception_handle->cell_ue.nof_prb = helpers_get_prb_from_bw_index(bc->bw_idx);
      // Free all RX related structures.
      phy_reception_ue_free(rf, phy_reception_handle->initial_rx_gain);
      // Set the new RX sample rate based on the new PRB sent by the upper layer.
      phy_reception_set_rx_sample_rate(rf, phy_reception_handle->cell_ue.nof_prb, phy_reception_handle->use_std_carrier_sep);
      // Initialize all RX related structures.
      phy_reception_ue_init(rf, phy_reception_handle->rnti, phy_reception_handle->initial_rx_gain, phy_reception_handle->initial_agc_gain, phy_reception_handle->initial_subframe_index);
      // After configuring a new bandwidth the synchronization thread needs to be restarted.
      if(phy_reception_start_sync_thread() < 0) {
        PHY_RX_ERROR("Error starting synchronization thread.\n",0);
        exit(-1);
      }
      // After configuring a new bandwidth the decoding thread needs to be restarted.
      if(phy_reception_start_decoding_thread() < 0) {
        PHY_RX_ERROR("Error starting decoding thread.\n",0);
        exit(-1);
      }
    }
    // Update last tx basic control structure.
    set_channel_number(bc->ch);
    set_bw_index(bc->bw_idx);
    PHY_RX_INFO_TIME("RX <--- BW[%d]: %1.1f [MHz] - Channel: %d - Set freq to: %.2f [MHz] - Offset: %.2f [MHz]\n", bc->bw_idx, (rx_bandwidth/1000000.0), bc->ch, (actual_rx_freq/1000000.0),(lo_offset/1000000.0));
  }

  // If AGC is disabled then we set the new gain sent by MAC in a basic control command.
  // If AGC is enabled in command line with "-g -1" parameter, the gain sent by MAC layer is ignored as there is no point in setting a gain when AGC is ON.
  if(phy_reception_handle->initial_rx_gain >= 0.0 && bc->gain >= 0) {
    // Checking if last configured gain is different from the current one.
    if(phy_reception_handle->last_rx_basic_control.gain != bc->gain) {
      // Set new RX gain.
      float rx_gain = srslte_rf_set_rx_gain(rf, (float)bc->gain, PHY_CHANNEL);
      // Updating last configured RX basic control.
      set_rx_gain(bc->gain);
      // Print new RX gain.
      PHY_RX_INFO_TIME("RX gain set to: %.1f [dB]\n", rx_gain);
    }
  }

  // Check if number of slots is valid.
  if(bc->length <= 0) {
    PHY_RX_ERROR("Invalid number of slots. It MUST be greater than 0. Current value is: %d\n",bc->length);
    exit(-1);
  }
  // Set number of slots to be received.
  if(phy_reception_handle->last_rx_basic_control.length != bc->length) {
    set_number_of_expected_slots(bc->length);
    phy_reception_handle->last_rx_basic_control.length = bc->length;
    PHY_RX_DEBUG_TIME("Expected number of slots set to: %d\n", bc->length);
  }

  // Set sequence number.
  set_sequence_number(bc->seq_number);
}

void set_sequence_number(uint64_t seq_number) {
  // Lock a mutex prior to using the basic control object.
  pthread_mutex_lock(&phy_reception_handle->phy_rx_basic_control_mutex);
  phy_reception_handle->last_rx_basic_control.seq_number = seq_number;
  // Unlock mutex upon using the basic control object.
  pthread_mutex_unlock(&phy_reception_handle->phy_rx_basic_control_mutex);
}

uint64_t get_sequence_number() {
  uint64_t seq_number;
  // Lock a mutex prior to using the basic control object.
  pthread_mutex_lock(&phy_reception_handle->phy_rx_basic_control_mutex);
  seq_number = phy_reception_handle->last_rx_basic_control.seq_number;
  // Unlock mutex upon using the basic control object.
  pthread_mutex_unlock(&phy_reception_handle->phy_rx_basic_control_mutex);
  return seq_number;
}

void set_number_of_expected_slots(uint32_t length) {
  // Lock a mutex prior to using the basic control object.
  pthread_mutex_lock(&phy_reception_handle->phy_rx_basic_control_mutex);
  phy_reception_handle->last_rx_basic_control.length = length;
  // Unlock mutex upon using the basic control object.
  pthread_mutex_unlock(&phy_reception_handle->phy_rx_basic_control_mutex);
}

uint32_t get_number_of_expected_slots() {
  uint32_t length;
  // Lock a mutex prior to using the basic control object.
  pthread_mutex_lock(&phy_reception_handle->phy_rx_basic_control_mutex);
  length = phy_reception_handle->last_rx_basic_control.length;
  // Unlock mutex upon using the basic control object.
  pthread_mutex_unlock(&phy_reception_handle->phy_rx_basic_control_mutex);
  return length;
}

void set_channel_number(uint32_t channel) {
  // Lock a mutex prior to using the basic control object.
  pthread_mutex_lock(&phy_reception_handle->phy_rx_basic_control_mutex);
  phy_reception_handle->last_rx_basic_control.ch = channel;
  // Unlock mutex upon using the basic control object.
  pthread_mutex_unlock(&phy_reception_handle->phy_rx_basic_control_mutex);
}

uint32_t get_channel_number() {
  uint32_t channel;
  // Lock a mutex prior to using the basic control object.
  pthread_mutex_lock(&phy_reception_handle->phy_rx_basic_control_mutex);
  channel = phy_reception_handle->last_rx_basic_control.ch;
  // Unlock mutex upon using the basic control object.
  pthread_mutex_unlock(&phy_reception_handle->phy_rx_basic_control_mutex);
  return channel;
}

void set_bw_index(uint32_t bw_index) {
  // Lock a mutex prior to using the basic control object.
  pthread_mutex_lock(&phy_reception_handle->phy_rx_basic_control_mutex);
  phy_reception_handle->last_rx_basic_control.bw_idx = bw_index;
  // Unlock mutex upon using the basic control object.
  pthread_mutex_unlock(&phy_reception_handle->phy_rx_basic_control_mutex);
}

uint32_t get_bw_index() {
  uint32_t bw_index;
  // Lock a mutex prior to using the basic control object.
  pthread_mutex_lock(&phy_reception_handle->phy_rx_basic_control_mutex);
  bw_index = phy_reception_handle->last_rx_basic_control.bw_idx;
  // Unlock mutex upon using the basic control object.
  pthread_mutex_unlock(&phy_reception_handle->phy_rx_basic_control_mutex);
  return bw_index;
}

void set_rx_gain(uint32_t rx_gain) {
  // Lock a mutex prior to using the basic control object.
  pthread_mutex_lock(&phy_reception_handle->phy_rx_basic_control_mutex);
  phy_reception_handle->last_rx_basic_control.gain = rx_gain;
  // Unlock mutex upon using the basic control object.
  pthread_mutex_unlock(&phy_reception_handle->phy_rx_basic_control_mutex);
}

uint32_t get_rx_gain() {
  uint32_t rx_gain;
  // Lock a mutex prior to using the basic control object.
  pthread_mutex_lock(&phy_reception_handle->phy_rx_basic_control_mutex);
  rx_gain = phy_reception_handle->last_rx_basic_control.gain;
  // Unlock mutex upon using the basic control object.
  pthread_mutex_unlock(&phy_reception_handle->phy_rx_basic_control_mutex);
  return rx_gain;
}

void *phy_reception_decoding_work(void *h) {

  int decoded_slot_counter = 0, ue_mib_ret, sfn_offset, pdsch_num_rxd_bits;
  float rsrp = 0.0, rsrq = 0.0, noise = 0.0, rssi = 0.0, snr = 0.0;
  double decoding_time = 0.0;
  uint32_t decoded_data_bufffer_counter = 0, sfn = 0, rx_sfn = 0; // System frame number
  uint8_t data[NUMBER_OF_DECODED_DATA_BUFFERS][10000], bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
  phy_stat_t phy_rx_stat;
  srslte_cell_t rx_cell;
  short_ue_sync_t short_ue_sync;
  cf_t *subframe_buffer = NULL;

  // Set priority to RX thread.
  uhd_set_thread_priority(1.0, true);

  rx_state_t rx_state = DECODE_PDSCH;

  /****************************** PHY Reception loop - BEGIN ******************************/
  PHY_RX_DEBUG("Entering PHY Decoding thread loop.\n", 0);
  while(phy_reception_wait_queue_not_empty() && phy_reception_handle->run_phy_decoding_thread) {

    //struct timespec start_decoding;
    //clock_gettime(CLOCK_REALTIME, &start_decoding);

    // Reset number of decoded PDSCH bits every loop iteration.
    pdsch_num_rxd_bits = 0;

    // Get UE sync structure from queue. This struture also has a filed which points to a new synchronized and aligned subframe for decoding.
    phy_reception_pop_ue_sync_from_queue(&short_ue_sync);

    //phy_reception_print_ue_sync(&short_ue_sync,"********** decoding thread **********\n");

    // Create an alias to the input buffer containing the synchronized and aligned subframe.
    subframe_buffer = &phy_reception_handle->ue_sync.input_buffer[short_ue_sync.buffer_number][short_ue_sync.subframe_start_index];

    switch(rx_state) {
      case DECODE_MIB:
        if(short_ue_sync.sf_idx == 0) {
          ue_mib_ret = srslte_ue_mib_decode(&phy_reception_handle->ue_mib, subframe_buffer, bch_payload, NULL, &sfn_offset);
          if(ue_mib_ret < 0) {
            PHY_RX_ERROR("Error decoding UE MIB.\n",0);
            exit(-1);
          } else if(ue_mib_ret == SRSLTE_UE_MIB_FOUND) {
            srslte_pbch_mib_unpack(bch_payload, &rx_cell, &rx_sfn);
            srslte_cell_fprint(stdout, &rx_cell, rx_sfn);
            PHY_RX_PRINT("Decoded MIB. SFN: %d, offset: %d\n", rx_sfn, sfn_offset);
            rx_sfn = (rx_sfn + sfn_offset)%1024;
            rx_state = DECODE_PDSCH;
          }
        }
        break;
      case DECODE_PDSCH:
        PHY_RX_DEBUG("Attempting DL decode SFN = %d\n", sfn);

        // Function srslte_ue_dl_decode() returns the number of bits received.
        pdsch_num_rxd_bits = srslte_ue_dl_decode(&phy_reception_handle->ue_dl,
                                                subframe_buffer,
                                                data[decoded_data_bufffer_counter],
                                                sfn*10+short_ue_sync.sf_idx);

        //PHY_PROFILLING_AVG3("Average decoding time: %f - min: %f - max: %f - max counter %d - diff >= 0.5 ms: %d - total counter: %d - perc: %f\n",helpers_profiling_diff_time(start_decoding), 0.5, 1000);

        if(pdsch_num_rxd_bits < 0) {
          PHY_RX_ERROR("Error decoding UE DL.\n",0);
        } else if(pdsch_num_rxd_bits > 0) {

          uint32_t nof_prb = helpers_get_prb_from_bw_index(get_bw_index());
          if(decoded_slot_counter > 0) {
            rssi = SRSLTE_VEC_EMA(srslte_vec_avg_power_cf(subframe_buffer, SRSLTE_SF_LEN(srslte_symbol_sz(nof_prb))),rssi,0.05);
            rsrq = SRSLTE_VEC_EMA(srslte_chest_dl_get_rsrq(&phy_reception_handle->ue_dl.chest), rsrq, 0.1);
            rsrp = SRSLTE_VEC_EMA(srslte_chest_dl_get_rsrp(&phy_reception_handle->ue_dl.chest), rsrp, 0.05);
            noise = SRSLTE_VEC_EMA(srslte_chest_dl_get_noise_estimate(&phy_reception_handle->ue_dl.chest), noise, 0.05);
          } else {
            // Reset values.
            rssi = 0.0;
            rsrq = 0.0;
            rsrp = 0.0;
            noise = 0.0;
            // Calculate statistics.
            rssi = srslte_vec_avg_power_cf(subframe_buffer, SRSLTE_SF_LEN(srslte_symbol_sz(nof_prb)));
            rsrq = srslte_chest_dl_get_rsrq(&phy_reception_handle->ue_dl.chest);
            rsrp = srslte_chest_dl_get_rsrp(&phy_reception_handle->ue_dl.chest);
            noise = srslte_chest_dl_get_noise_estimate(&phy_reception_handle->ue_dl.chest);
          }

          if(isnan(rssi)) {
            rssi = 0.0;
          }
          if(isnan(rsrq)) {
            rsrq = 0.0;
          }
          if(isnan(noise)) {
            noise = 0.0;
          }
          if(isnan(rsrp)) {
            rsrp = 0.0;
          }
          // Calculate SNR out of RSRP and noise estimation.
          snr = 10*log10(rsrp/noise);

          // Set PHY RX Stats with valid values.
          // When data is correctly decoded return SUCCESS status.
          phy_rx_stat.status = PHY_SUCCESS;                           // Status tells upper layers that if successfully received data.
          phy_rx_stat.seq_number = get_sequence_number();             // Sequence number represents the counter of received slots.
          phy_rx_stat.host_timestamp = helpers_convert_host_timestamp(&short_ue_sync.peak_detection_timestamp); // Retrieve host's time. Host PC time value when (ch,slot) PHY data are demodulated
          phy_rx_stat.ch = get_channel_number();                      // Set the channel number where the data was received at.
          phy_rx_stat.mcs = phy_reception_handle->ue_dl.pdsch_cfg.grant.mcs.idx;	          // MCS index is decoded when the DCI is found and correctly decoded. Modulation Scheme. Range: [0, 28]. check TBS table num_byte_per_1ms_mcs[29] in intf.h to know MCS
          phy_rx_stat.num_cb_total = phy_reception_handle->ue_dl.nof_detected;	            // Number of Code Blocks (CB) received in the (ch, slot)
          phy_rx_stat.num_cb_err = phy_reception_handle->ue_dl.pkt_errors;		              // How many CBs get CRC error in the (ch, slot)
          phy_rx_stat.wrong_decoding_counter = phy_reception_handle->ue_dl.wrong_decoding_counter;
          // Assign the values to RX Stat structure.
          phy_rx_stat.stat.rx_stat.gain = get_rx_gain(); // Receiver gain (maybe not important now, but let's reserve it). dB*10. for example, value 789 means 78.9dB
          phy_rx_stat.stat.rx_stat.cqi = srslte_cqi_from_snr(snr);                // Channel Quality Indicator. Range: [1, 15]
          phy_rx_stat.stat.rx_stat.rssi = 10*log10(rssi);			                    // Received Signal Strength Indicator. Range: [–2^31, (2^31) - 1]. dBm*10. For example, value -567 means -56.7dBm.
          phy_rx_stat.stat.rx_stat.rsrp = 10*log10(rsrp);				                  // Reference Signal Received Power. Range: [-1400, -400]. dBm*10. For example, value -567 means -56.7dBm.
          phy_rx_stat.stat.rx_stat.rsrq = 10*log10(rsrq);				                  // Reference Signal Receive Quality. Range: [-340, -25]. dB*10. For example, value 301 means 30.1 dB.
          phy_rx_stat.stat.rx_stat.sinr = snr; 			                              // Signal to Interference plus Noise Ratio. Range: [–2^31, (2^31) - 1]. dB*10. For example, value 256 means 25.6 dB.
          phy_rx_stat.stat.rx_stat.peak_value = short_ue_sync.peak_value;
          phy_rx_stat.stat.rx_stat.noise = phy_reception_handle->ue_dl.noise_estimate;
          phy_rx_stat.stat.rx_stat.last_noi = srslte_ue_dl_last_noi(&phy_reception_handle->ue_dl);
          phy_rx_stat.stat.rx_stat.detection_errors = phy_reception_handle->ue_dl.wrong_decoding_counter;
          phy_rx_stat.stat.rx_stat.decoding_errors = phy_reception_handle->ue_dl.pkt_errors; // If there was a decoding error, then, check the counters below.
          phy_rx_stat.stat.rx_stat.filler_bits_error = phy_reception_handle->ue_dl.pdsch.dl_sch.filler_bits_error;
          phy_rx_stat.stat.rx_stat.nof_cbs_exceeds_softbuffer_size_error = phy_reception_handle->ue_dl.pdsch.dl_sch.nof_cbs_exceeds_softbuffer_size_error;
          phy_rx_stat.stat.rx_stat.rate_matching_error = phy_reception_handle->ue_dl.pdsch.dl_sch.rate_matching_error;
          phy_rx_stat.stat.rx_stat.cb_crc_error = phy_reception_handle->ue_dl.pdsch.dl_sch.cb_crc_error;
          phy_rx_stat.stat.rx_stat.tb_crc_error = phy_reception_handle->ue_dl.pdsch.dl_sch.tb_crc_error;
          phy_rx_stat.stat.rx_stat.total_packets_synchronized = phy_reception_handle->ue_dl.pkts_total; // Total number of slots synchronized. It contains correct and wrong slots.
          phy_rx_stat.stat.rx_stat.length = pdsch_num_rxd_bits/8;				          // How many bytes are after this header. It should be equal to current TB size.
          phy_rx_stat.stat.rx_stat.data = data[decoded_data_bufffer_counter];
          decoding_time = helpers_profiling_diff_time(short_ue_sync.peak_detection_timestamp);
          //PHY_PROFILLING_AVG3("Avg. sync + decoding time: %f - min: %f - max: %f - max counter %d - diff >= 0.5 ms: %d - total counter: %d - perc: %f\n", decoding_time, 0.5, 1000);

          //PHY_PROFILLING_AVG3("Avg. read samples + sync + decoding time: %f - min: %f - max: %f - max counter %d - diff >= 2ms: %d - total counter: %d - perc: %f\n", helpers_profiling_diff_time(short_ue_sync.start_of_rx_sample), 2.0, 1000);

          // Information on data decoding process.
          PHY_RX_INFO_TIME("[RX STATS]: RX slots: %d - Channel: %d - RX bytes: %d - CFO: %+2.2f [kHz] - Peak value: %1.2f - Noise: %1.4f - RSSI: %1.2f [dBm] - SINR: %4.1f [dB] - RSRQ: %1.2f [dB] - CQI: %d - MCS: %d - CID: %d - Total: %d - Error: %d - Last NOI: %d - Avg. NOI: %1.2f - Decoding time: %f [ms]\n", (decoded_slot_counter+1), phy_rx_stat.ch, phy_rx_stat.stat.rx_stat.length, short_ue_sync.cfo/1000.0, short_ue_sync.peak_value, phy_reception_handle->ue_dl.noise_estimate, phy_rx_stat.stat.rx_stat.rssi, phy_rx_stat.stat.rx_stat.sinr, phy_rx_stat.stat.rx_stat.rsrq, phy_rx_stat.stat.rx_stat.cqi, phy_rx_stat.mcs, 3*short_ue_sync.N_id_1 + short_ue_sync.N_id_2, phy_reception_handle->ue_dl.nof_detected, phy_reception_handle->ue_dl.pkt_errors, srslte_ue_dl_last_noi(&phy_reception_handle->ue_dl), srslte_ul_dl_average_noi(&phy_reception_handle->ue_dl), decoding_time);

          DEV_INFO("Out of sequence message: - SINR: %1.4f - RX byte: %d - host timestamp: %" PRIu64 "\n",phy_rx_stat.stat.rx_stat.sinr,data[decoded_data_bufffer_counter][0],phy_rx_stat.host_timestamp);

          // Uncomment this line to measure the number of packets received in one second.
          //helpers_measure_packets_per_second("RX");

#if(ENABLE_TX_TO_RX_TIME_DIFF==1)
          // Enable add_tx_timestamp to measure the time it takes for a transmitted packet to be received (i.e., detected/buffered/decoded).
          if(phy_reception_handle->add_tx_timestamp) {
            uint64_t rx_timestamp, tx_timestamp;
            struct timespec rx_timestamp_struct;
            clock_gettime(CLOCK_REALTIME, &rx_timestamp_struct);
            rx_timestamp = helpers_convert_host_timestamp(&rx_timestamp_struct);
            memcpy((void*)&tx_timestamp, (void*)data[decoded_data_bufffer_counter], sizeof(uint64_t));
            PHY_PROFILLING_AVG3("Diff between TX and RX time: %0.4f [s] - min: %f - max: %f - max counter %d - diff >= 1ms: %d - total counter: %d - perc: %f\n", (double)((rx_timestamp-tx_timestamp)/1000000000.0), 0.001, 1000);
          }
#endif

#if(ENBALE_RX_INFO_PLOT==1)
          if(phy_reception_handle->plot_rx_info == true) {
            plot_info(&phy_reception_handle->ue_dl, &phy_reception_handle->ue_sync);
          }
#endif

          // Send phy received (RX) statistics and TB (data) to upper layers.
          phy_reception_send_rx_statistics(phy_reception_handle->phy_comm_handle, &phy_rx_stat);

          // Only if packet is really received we update the received slot counter.
          decoded_slot_counter++;
          // Counter used to avoid data overlapping.
          decoded_data_bufffer_counter = (decoded_data_bufffer_counter + 1)%NUMBER_OF_DECODED_DATA_BUFFERS;
          PHY_RX_DEBUG("Decoded slot counter: %d\n",decoded_slot_counter);
        } else {
          // There was an error if the code reaches this point: (1) wrong CFI or DCI detected or (2) data was incorrectly decoded.
          rssi = 10.0*log10f(srslte_vec_avg_power_cf(subframe_buffer, short_ue_sync.frame_len));
          phy_rx_stat.status = PHY_ERROR;
          phy_rx_stat.seq_number = get_sequence_number();
          phy_rx_stat.ch = get_channel_number();
          phy_rx_stat.num_cb_total = phy_reception_handle->ue_dl.nof_detected;
          phy_rx_stat.stat.rx_stat.detection_errors = phy_reception_handle->ue_dl.wrong_decoding_counter;
          phy_rx_stat.stat.rx_stat.decoding_errors = phy_reception_handle->ue_dl.pkt_errors;
          phy_rx_stat.stat.rx_stat.filler_bits_error = phy_reception_handle->ue_dl.pdsch.dl_sch.filler_bits_error;
          phy_rx_stat.stat.rx_stat.nof_cbs_exceeds_softbuffer_size_error = phy_reception_handle->ue_dl.pdsch.dl_sch.nof_cbs_exceeds_softbuffer_size_error;
          phy_rx_stat.stat.rx_stat.rate_matching_error = phy_reception_handle->ue_dl.pdsch.dl_sch.rate_matching_error;
          phy_rx_stat.stat.rx_stat.cb_crc_error = phy_reception_handle->ue_dl.pdsch.dl_sch.cb_crc_error;
          phy_rx_stat.stat.rx_stat.tb_crc_error = phy_reception_handle->ue_dl.pdsch.dl_sch.tb_crc_error;
          phy_rx_stat.stat.rx_stat.rssi = rssi;
          phy_rx_stat.stat.rx_stat.peak_value = short_ue_sync.peak_value;
          phy_rx_stat.stat.rx_stat.noise = phy_reception_handle->ue_dl.noise_estimate;
          phy_rx_stat.stat.rx_stat.decoded_cfi = phy_reception_handle->ue_dl.decoded_cfi;
          phy_rx_stat.stat.rx_stat.found_dci = phy_reception_handle->ue_dl.found_dci;
          phy_rx_stat.stat.rx_stat.last_noi = srslte_ue_dl_last_noi(&phy_reception_handle->ue_dl);
          phy_rx_stat.stat.rx_stat.total_packets_synchronized = phy_reception_handle->ue_dl.pkts_total;
          // Send phy received (RX) statistics and TB (data) to upper layers.
          phy_reception_send_rx_statistics(phy_reception_handle->phy_comm_handle, &phy_rx_stat);
          PHY_RX_INFO_TIME("[RX STATS]: Detection errors: %d - Channel: %d - CFO: %+2.2f [kHz] - Peak value: %1.2f - RSSI: %3.2f [dBm] - Decoded CFI: %d - Found DCI: %d - Last NOI: %d - Avg. NOI: %1.2f - Noise: %1.4f - Decoding errors: %d\n", phy_reception_handle->ue_dl.wrong_decoding_counter,get_channel_number(),short_ue_sync.cfo/1000.0,short_ue_sync.peak_value,rssi,phy_reception_handle->ue_dl.decoded_cfi,phy_reception_handle->ue_dl.found_dci,srslte_ue_dl_last_noi(&phy_reception_handle->ue_dl), srslte_ul_dl_average_noi(&phy_reception_handle->ue_dl),phy_reception_handle->ue_dl.noise_estimate,phy_reception_handle->ue_dl.pkt_errors);
          //PHY_RX_PRINT_TIME("[RX STATS]: Detection errors: %d - Channel: %d - CFO: %+2.10f [kHz] - Peak value: %1.2f - RSSI: %3.2f [dBm] - Decoded CFI: %d - Found DCI: %d - Last NOI: %d - Avg. NOI: %1.2f - Noise: %1.4f - Decoding errors: %d\n",phy_reception_handle->ue_dl.wrong_decoding_counter,get_channel_number(),short_ue_sync.cfo/1000.0,short_ue_sync.peak_value,rssi,phy_reception_handle->ue_dl.decoded_cfi,phy_reception_handle->ue_dl.found_dci,srslte_ue_dl_last_noi(&phy_reception_handle->ue_dl), srslte_ul_dl_average_noi(&phy_reception_handle->ue_dl),phy_reception_handle->ue_dl.noise_estimate,phy_reception_handle->ue_dl.pkt_errors);
        }
        break;
    }
    if(short_ue_sync.sf_idx == 9) {
      sfn++;
      if(sfn == 1024) {
        sfn = 0;
        phy_reception_handle->ue_dl.pkt_errors = 0;
        phy_reception_handle->ue_dl.pkts_total = 0;
        phy_reception_handle->ue_dl.nof_detected = 0;
        PHY_RX_PRINT("------------------------\n",0);
      }
    }
    // Check if the number of decoded subframes is equal to the number of expected ones, then reset decoded_slot_counter.
    if(decoded_slot_counter >= get_number_of_expected_slots() || pdsch_num_rxd_bits <= 0) {
      decoded_slot_counter = 0;
    }
  }
  /****************************** PHY Decoding loop - END ******************************/
  PHY_RX_DEBUG("Leaving PHY Decoding thread.\n",0);
  // Exit thread with result code.
  pthread_exit(NULL);
}

void phy_reception_send_rx_statistics(LayerCommunicator_handle handle, phy_stat_t *phy_rx_stat) {
  // Set values to the RX Stats Structure.
  // Set frame number.
  phy_rx_stat->frame = 0;
  // Set time slot number.
  phy_rx_stat->slot = 0;
  // Set some default values.
  if(phy_rx_stat->status == PHY_SUCCESS) {
    phy_rx_stat->stat.rx_stat.decoded_cfi = 1;
    phy_rx_stat->stat.rx_stat.found_dci = 1;
  } else if(phy_rx_stat->status == PHY_TIMEOUT) {
    phy_rx_stat->host_timestamp = helpers_get_host_time_now(); // Host PC time value when reception timesout.
    phy_rx_stat->mcs = 100;
    phy_rx_stat->num_cb_total = 0;
    phy_rx_stat->num_cb_err = 0;
    phy_rx_stat->stat.rx_stat.cqi = 100;
    phy_rx_stat->stat.rx_stat.rssi = 0.0;
    phy_rx_stat->stat.rx_stat.rsrp = 0.0;
    phy_rx_stat->stat.rx_stat.rsrq = 0.0;
    phy_rx_stat->stat.rx_stat.sinr = 0.0;
    phy_rx_stat->stat.rx_stat.length = 0;
    phy_rx_stat->stat.rx_stat.data = NULL;
  } else if(phy_rx_stat->status == PHY_ERROR) {
    phy_rx_stat->host_timestamp = 0;
    phy_rx_stat->mcs = 100;
    phy_rx_stat->num_cb_err = 0;
    phy_rx_stat->stat.rx_stat.gain = 0;
    phy_rx_stat->stat.rx_stat.cqi = 0;
    phy_rx_stat->stat.rx_stat.rsrp = 0;
    phy_rx_stat->stat.rx_stat.rsrq = 0;
    phy_rx_stat->stat.rx_stat.sinr = 0;
    phy_rx_stat->stat.rx_stat.length = 0;
    phy_rx_stat->stat.rx_stat.data = NULL;
  }

  // Sending PHY RX statistics to upper layer.
  PHY_RX_DEBUG("Sending RX statistics information upwards...\n",0);
  // Send RX stats. There is a mutex on this function which prevents TX from sending statistics to PHY at the same time.
  communicator_send_phy_stat_message(handle, RX_STAT, phy_rx_stat, NULL);
  // Print RX stats information.
  HELPERS_PRINT_RX_STATS(phy_rx_stat);
}

void phy_reception_ue_init(srslte_rf_t *rf, uint16_t rnti, float initial_rx_gain, float initial_agc_gain, int initial_subframe_index) {
    // Initialize parameters for UE Cell.
    if(srslte_ue_sync_init(&phy_reception_handle->ue_sync, phy_reception_handle->cell_ue, srslte_rf_recv_with_time_wrapper, (void*)rf, initial_subframe_index)) {
      PHY_RX_ERROR("Error initiating ue_sync\n",0);
      exit(-1);
    }

    if(srslte_ue_mib_init(&phy_reception_handle->ue_mib, phy_reception_handle->cell_ue)) {
      PHY_RX_ERROR("Error initaiting UE MIB decoder\n",0);
      exit(-1);
    }

    if(srslte_ue_dl_init(&phy_reception_handle->ue_dl, phy_reception_handle->cell_ue)) {
      PHY_RX_ERROR("Error initiating UE downlink processing module\n",0);
      exit(-1);
    }

    // Configure downlink receiver for the SI-RNTI since will be the only one we'll use.
    // This is the User RNTI.
    srslte_ue_dl_set_rnti(&phy_reception_handle->ue_dl, rnti);

    // Set the expected CFI.
    srslte_ue_dl_set_expected_cfi(&phy_reception_handle->ue_dl, DEFAULT_CFI);

    if(initial_rx_gain < 0.0) {
      srslte_ue_sync_start_agc(&phy_reception_handle->ue_sync, srslte_rf_set_rx_gain_th_wrapper_, initial_agc_gain);
    }

    // Set initial CFO for ue_sync to 0.
    srslte_ue_sync_set_cfo(&phy_reception_handle->ue_sync, 0.0);

    // Reset PBCH decoding.
    srslte_pbch_decode_reset(&phy_reception_handle->ue_mib.pbch);
}

void phy_reception_stop_rx_stream_and_flush_buffer(srslte_rf_t *rf) {
  int error;
  if((error = srslte_rf_stop_rx_stream(rf, PHY_CHANNEL)) != 0) {
    PHY_RX_ERROR("Error stopping rx stream: %d....\n",error);
    exit(-1);
  }
  srslte_rf_flush_buffer(rf, PHY_CHANNEL);
}

void phy_reception_initialize_rx_stream(srslte_rf_t *rf) {
  int error;
  // Stop RX Stream and Flush Reception Buffer.
  phy_reception_stop_rx_stream_and_flush_buffer(rf);
  // Start RX Stream.
  if((error = srslte_rf_start_rx_stream(rf, PHY_CHANNEL)) != 0) {
    PHY_RX_ERROR("Error starting rx stream: %d....\n",error);
    exit(-1);
  }
}

// Free all related UE Downlink structures.
void phy_reception_ue_free(srslte_rf_t *rf, float initial_rx_gain) {
  // Terminate and join AGC thread only if AGC is enabled.
  if(initial_rx_gain < 0.0) {
    if(srslte_rf_finish_gain_thread(rf) < 0) {
      PHY_RX_ERROR("Error joining gain thread........\n",0);
    } else {
      PHY_RX_PRINT("Gain thread correctly joined.\n",0);
    }
  }
  // Free all UE related structures.
  srslte_ue_dl_free(&phy_reception_handle->ue_dl);
  PHY_RX_INFO("srslte_ue_dl_free done!\n",0);
  srslte_ue_sync_free_except_reentry(&phy_reception_handle->ue_sync);
  PHY_RX_INFO("srslte_ue_sync_free_except_reentry done!\n",0);
  // Free all MIB related resources.
  srslte_ue_mib_free(&phy_reception_handle->ue_mib);
  PHY_RX_INFO("srslte_ue_mib_free done!\n",0);
}

void phy_reception_set_rx_sample_rate(srslte_rf_t *rf, uint32_t nof_prb, bool use_std_carrier_sep) {
  int srate = -1;
  if(use_std_carrier_sep) {
    srate = srslte_sampling_freq_hz(nof_prb);
  } else {
    srate = helpers_non_std_sampling_freq_hz(nof_prb);
    PHY_RX_PRINT("Setting a non-standard sampling rate: %1.2f [MHz]\n",srate/1000000.0);
  }
  if(srate != -1) {
    float srate_rf = srslte_rf_set_rx_srate(rf, (double)srate, PHY_CHANNEL);
    if(srate_rf != srate) {
      PHY_RX_ERROR("Could not set RX sampling rate.\n",0);
      exit(-1);
    }
    PHY_RX_PRINT("Set RX sampling rate to: %.2f [MHz]\n", srate_rf/1000000.0);
  } else {
    PHY_RX_ERROR("Invalid number of PRB (RX): %d\n", nof_prb);
    exit(-1);
  }
}

void phy_reception_set_initial_rx_freq_and_gain(srslte_rf_t *rf, float rx_gain, float initial_agc_gain, double default_center_freq, double competition_bw, double rx_bandwidth, uint32_t default_rx_channel) {
  // Set receiver gain.
  if(rx_gain >= 0.0) {
    double gain = srslte_rf_set_rx_gain(rf, rx_gain, PHY_CHANNEL);
    PHY_RX_PRINT("Set RX gain to: %.1f [dB] (AGC NOT ENABLED).\n",gain);
  } else {
    PHY_RX_PRINT("Starting AGC thread...\n",0);
    if(srslte_rf_start_gain_thread(rf, false, PHY_CHANNEL)) {
      PHY_RX_ERROR("Error starting gain thread.\n",0);
      exit(-1);
    }
    srslte_rf_set_rx_gain(rf, initial_agc_gain, PHY_CHANNEL);
  }
  // Calculate central frequency for the channel.
  double rx_channel_center_freq = helpers_calculate_channel_center_frequency(default_center_freq, competition_bw, rx_bandwidth, default_rx_channel);
  // Set central frequency for reception.
  double lo_offset = rf->num_of_channels == 1 ? 0.0:(double)PHY_RX_LO_OFFSET;
  double current_rx_freq = srslte_rf_set_rx_freq2(rf, rx_channel_center_freq, lo_offset, PHY_CHANNEL);
  // Check if actual frequency is inside a range of +/- 10 Hz.
  if(current_rx_freq < (rx_channel_center_freq - 10.0) || current_rx_freq > (rx_channel_center_freq + 10.0)) {
     PHY_RX_ERROR("[Initialization] Requested freq.: %1.2f [MHz] - Actual freq.: %1.2f [MHz] - Center Frequency: %1.2f [MHz] - Competition BW: %1.2f [MHz] - PHY BW: %1.2f [MHz] - Channel: %d \n", rx_channel_center_freq/1000000.0, current_rx_freq/1000000.0, default_center_freq/1000000.0, competition_bw/1000000.0, rx_bandwidth/1000000.0, default_rx_channel);
  }
  srslte_rf_rx_wait_lo_locked(rf, PHY_CHANNEL);
  PHY_RX_PRINT("Set initial RX freq to: %.2f [MHz] with offset of: %.2f [MHz]\n", (current_rx_freq/1000000.0),(lo_offset/1000000.0));
}

// This is the function called when the synchronization thread is called/started.
void *phy_reception_sync_work(void *h) {

  int ret;
  short_ue_sync_t short_ue_sync;

  // Set priority to RX thread.
  uhd_set_thread_priority(1.0, true);

  // Set some constant parameters of short_ue_sync structure.
  short_ue_sync.frame_len = phy_reception_handle->ue_sync.frame_len;

  PHY_RX_DEBUG("Entering PHY synchronization thread loop.\n", 0);
  while(phy_reception_handle->run_phy_synchronization_thread) {

    // Timestamp the start of the reception procedure. We calculate the total time it takes to read, sync and decode the subframe on the decoding thread.
    //clock_gettime(CLOCK_REALTIME, &short_ue_sync.start_of_rx_sample);

    //struct timespec synchronization_start;
    //clock_gettime(CLOCK_REALTIME, &synchronization_start);

    // synchronize and align subframes.
    ret = srslte_ue_sync_get_subframe_buffer(&phy_reception_handle->ue_sync, PHY_CHANNEL);

#ifdef CORRECT_SAMPLE_OFFSET
    float sample_offset = (float) srslte_ue_sync_get_last_sample_offset(&phy_reception_handle->ue_sync)+srslte_ue_sync_get_sfo(&phy_reception_handle->ue_sync)/1000;
    srslte_ue_dl_set_sample_offset(phy_reception_handle->ue_dl, sample_offset);
#endif

    // srslte_ue_sync_get_buffer_new() returns 1 if it successfully synchronizes to a slot (also known as subframe).
    if(ret == 1) {
      //PHY_PROFILLING_AVG3("Average synchronization time: %f - min: %f - max: %f - max counter %d - diff >= 0.5 ms: %d - total counter: %d - perc: %f\n",helpers_profiling_diff_time(ue_sync->sfind.peak_detection_timestamp), 1.0, 1000);

      //struct timespec start_push_queue;
      //clock_gettime(CLOCK_REALTIME, &start_push_queue);

      // Update the short ue sync strucure with the current subframe counter number and other parameters.
      short_ue_sync.buffer_number = phy_reception_handle->ue_sync.previous_subframe_buffer_counter_value;
      short_ue_sync.subframe_start_index = phy_reception_handle->ue_sync.subframe_start_index;
      short_ue_sync.sf_idx = phy_reception_handle->ue_sync.sf_idx;
      short_ue_sync.peak_value = phy_reception_handle->ue_sync.sfind.peak_value;
      short_ue_sync.peak_detection_timestamp = phy_reception_handle->ue_sync.sfind.peak_detection_timestamp;
      short_ue_sync.cfo = srslte_ue_sync_get_carrier_freq_offset(&phy_reception_handle->ue_sync.sfind);
      short_ue_sync.N_id_1 = phy_reception_handle->ue_sync.sfind.N_id_1;
      short_ue_sync.N_id_2 = phy_reception_handle->ue_sync.sfind.N_id_2;

      //PHY_PROFILLING_AVG3("Average synchronization time: %f [ms] - min: %f [ms] - max: %f [ms] - max counter %d - diff >= 1.0 ms: %d - total counter: %d - perc: %f\n",helpers_profiling_diff_time(synchronization_start), 1.0, 1000);


#if(WRITE_RX_SUBFRAME_INTO_FILE==1)
      static unsigned int dump_cnt = 0;
      char output_file_name[200] = "f_ofdm_rx_side_assessment_5MHz.dat";
      srslte_filesink_t file_sink;
      if(dump_cnt==0) {
         filesink_init(&file_sink, output_file_name, SRSLTE_COMPLEX_FLOAT_BIN);
         // Write samples into file.
         filesink_write(&file_sink, &phy_reception_handle->ue_sync.input_buffer[short_ue_sync.buffer_number][short_ue_sync.subframe_start_index], SRSLTE_SF_LEN(srslte_symbol_sz(helpers_get_prb_from_bw_index(get_bw_index()))));
         // Close file.
         filesink_free(&file_sink);
      }
      dump_cnt++;
      PHY_RX_PRINT("File dumped: %d.\n",dump_cnt);
#endif

      // Push ue sync strucute to queue (FIFO).
      phy_reception_push_ue_sync_to_queue(&short_ue_sync);

      //double diff_queue = helpers_profiling_diff_time(start_push_queue);
      //if(diff_queue > 0.05)
      //  printf("UE Synch enQUEUE time: %f\n",diff_queue);

      //phy_reception_print_ue_sync(&short_ue_sync,"********** synchronization thread **********\n");
    } else if(ret == 0) {
      // No slot synchronized and aligned. We don't do nothing for now.
    } else if(ret < 0) {
      PHY_RX_ERROR("Error calling srslte_ue_sync_get_buffer_new()\n",0);
      exit(-1);
    }

    // We increment the subframe counter the first time if we have just found a peak and the subframe data was correctly decoded.
    if(phy_reception_handle->ue_sync.last_state == SF_FIND && phy_reception_handle->ue_sync.state == SF_TRACK) {
      phy_reception_handle->ue_sync.subframe_counter = 1;
      PHY_RX_DEBUG("First increment of subframe counter: %d.\n",phy_reception_handle->ue_sync.subframe_counter);
    } else if(phy_reception_handle->ue_sync.last_state == SF_TRACK && phy_reception_handle->ue_sync.state == SF_TRACK && phy_reception_handle->ue_sync.subframe_counter > 0) {
      // Increment the subframe counter in order to receive the correct number of subframes.
      phy_reception_handle->ue_sync.subframe_counter++;
    } else {
      phy_reception_handle->ue_sync.subframe_counter = 0;
    }

    if(phy_reception_handle->ue_sync.subframe_counter >= get_number_of_expected_slots()) {
      // Light weight way to reset ue_dl for new reception.
      if(srslte_ue_sync_init_reentry_loop(&phy_reception_handle->ue_sync)) {
        PHY_RX_ERROR("Error re-initiating ue_sync\n",0);
        exit(-1);
      }
    }
  }

  PHY_RX_DEBUG("Leaving PHY synchronization thread.\n",0);
  // Exit thread with result code.
  pthread_exit(NULL);
}

int srslte_rf_recv_wrapper(void *h, void *data, uint32_t nsamples, srslte_timestamp_t *t, size_t channel) {
  return srslte_rf_recv(h, data, nsamples, 1, channel);
}

int srslte_rf_recv_with_time_wrapper(void *h, void *data, uint32_t nsamples, srslte_timestamp_t *t, size_t channel) {
  return srslte_rf_recv_with_time(h, data, nsamples, 1, &(t->full_secs), &(t->frac_secs), channel);
}

int srslte_file_recv_wrapper(void *h, void *data, uint32_t nsamples, srslte_timestamp_t *t, size_t channel) {
  return srslte_filesource_read(h, data, nsamples);
}

double srslte_rf_set_rx_gain_th_wrapper_(void *h, double f) {
  return srslte_rf_set_rx_gain_th((srslte_rf_t*) h, f);
}

// Functions to transfer ue sync structure from sync thread to reception thread.
void phy_reception_push_ue_sync_to_queue(short_ue_sync_t *short_ue_sync) {
  // Lock mutex so that we can push ue sync to queue.
  pthread_mutex_lock(&phy_reception_handle->phy_reception_ue_sync_mutex);
  // Push ue sync into queue.
  push_ue_sync_to_queue(short_ue_sync);
  // Unlock mutex so that function can do other things.
  pthread_mutex_unlock(&phy_reception_handle->phy_reception_ue_sync_mutex);
  // Notify other thread that ue sync structure was pushed into queue.
  pthread_cond_signal(&phy_reception_handle->phy_reception_ue_sync_cv);
}

void phy_reception_pop_ue_sync_from_queue(short_ue_sync_t *short_ue_sync) {
  // Lock mutex so that we can pop ue sync structure from queue.
  pthread_mutex_lock(&phy_reception_handle->phy_reception_ue_sync_mutex);
  // Retrieve sync element from queue.
  pop_ue_sync_from_queue(short_ue_sync);
  // Unlock mutex.
  pthread_mutex_unlock(&phy_reception_handle->phy_reception_ue_sync_mutex);
}

bool phy_reception_wait_queue_not_empty() {
  bool ret = true;
  // Lock mutex so that we can wait for ue sync strucure.
  pthread_mutex_lock(&phy_reception_handle->phy_reception_ue_sync_mutex);
  // Wait for conditional variable only if container is empty.
  if(is_ue_queue_empty()) {
    // Wait for conditional variable to be true.
    pthread_cond_wait(&phy_reception_handle->phy_reception_ue_sync_cv, &phy_reception_handle->phy_reception_ue_sync_mutex);
    if(!phy_reception_handle->run_phy_decoding_thread || !phy_reception_handle->run_phy_synchronization_thread) {
      ret = false;
    }
  }
  // Unlock mutex.
  pthread_mutex_unlock(&phy_reception_handle->phy_reception_ue_sync_mutex);
  return ret;
}

void phy_reception_print_ue_sync(short_ue_sync_t *short_ue_sync, char* str) {
  printf("%s",str);
  printf("********** UE Sync Structure **********\n");
  printf("Buffer number: %d\n",short_ue_sync->buffer_number);
  printf("Subframe start index: %d\n",short_ue_sync->subframe_start_index);
  printf("Subframe index: %d\n",short_ue_sync->sf_idx);
  printf("Peak value: %0.2f\n",short_ue_sync->peak_value);
  printf("Timestamp: %" PRIu64 "\n" ,short_ue_sync->peak_detection_timestamp);
  printf("CID: %d\n",(3*short_ue_sync->N_id_1 + short_ue_sync->N_id_2));
  printf("CFO: %0.3f\n",short_ue_sync->cfo/1000.0);
  printf("Subframe length: %d\n",short_ue_sync->frame_len);
}

void phy_reception_init_handle() {
  // Variable used to stop phy decoding thread.
  phy_reception_handle->run_phy_decoding_thread = true;
  // Variable used to stop phy synchronization thread.
  phy_reception_handle->run_phy_synchronization_thread = true;
  // This pointer is used to point to the subframe.
  phy_reception_handle->sf_buffer = NULL;
}
