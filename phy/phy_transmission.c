#include "phy_transmission.h"

#define FIX_TX_OFFSET_SAMPLES 0 // Set the number of zeros to be padded before the slot.

// Number of padding zeros after the end of the last subframe. OBS.: 20 is the best value found so far.
#define NOF_PADDING_ZEROS 20

#define ADD_PSS_TO_ALL_SUBFRAMES 1 // Enable/Disable the insertion of PSS/SSS to all slots. By default we are always adding PSS to all the subframes.

#define ADD_PSS_ONLY_TO_VERY_FIRST_SUBFRAME 0 // Enabled/disable the insertion of PSS/SSS only to the very first subframe in a COT. When you enable this macro, thsi one ADD_PSS_TO_ALL_SUBFRAMES should be set to 0.

#define PHY_TX_LO_OFFSET -42.0e6 // TX local offset.

#define ENABLE_MIB_ENCODING 0 // Enable or disable MIB encoding. By default it is disabled.

#define WRITE_TX_SUBFRAME_INTO_FILE 0 // Enbale or disable dumping of Tx samples.

#define WAIT_FOR_BURST_ACK 0

#define TX_TIMED_COMMAND_ENABLED 0

#define TX_TIME_IN_ADVANCE 350

// Size of the current TX data.
#define TX_DATA_LENGTH 100*2292 // 100 TB * TB size for 5 MHz PHY BW and MCS 28, i.e., 2292 bytes.

// *********** Global variables ***********
// This handle is used to pass some important objects to the PHY TX thread work function.
static phy_transmission_t* phy_transmission_handle = NULL;

// ********************* Declaration of constants *********************
// MCS TB size mapping table
// how many bytes (TB size) in one slot under MCS 0~28 (this slot is our MF-TDMA slot, not LTE slot, here it refers to 1ms length!)
const unsigned int num_byte_per_1ms_mcs[6][29] = {{19,26,32,41,51,63,75,89,101,117,117,129,149,169,193,217,225,225,241,269,293,325,349,373,405,437,453,469,549}, // Values for 1.4 MHz BW
{49,65,81,109,133,165,193,225,261,293,293,333,373,421,485,533,573,573,621,669,749,807,871,935,999,1063,1143,1191,1383}, // Values for 3 MHz BW
{85,113,137,177,225,277,325,389,437,501,501,549,621,717,807,903,967,967,999,1143,1239,1335,1431,1572,1692,1764,1908,1980,2292}, // Values for 5 MHz BW
{173,225,277,357,453,549,645,775,871,999,999,1095,1239,1431,1620,1764,1908,1908,2052,2292,2481,2673,2865,3182,3422,3542,3822,3963,4587}, // Values for 10 MHz BW
{261,341,421,549,669,839,967,1143,1335,1479,1479,1620,1908,2124,2385,2673,2865,2865,3062,3422,3662,4107,4395,4736,5072,5477,5669,5861,6882}, // Values for 15 MHz BW
{349,453,573,717,903,1095,1287,1527,1764,1980,1980,2196,2481,2865,3182,3542,3822,3822,4107,4587,4904,5477,5861,6378,6882,7167,7708,7972,9422}}; // Values for 20 MHz BW

// *********** Definition of functions ***********
// This function is used to set everything needed for the phy reception thread to run accordinly.
int phy_transmission_initialize(LayerCommunicator_handle handle, srslte_rf_t *rf, transceiver_args_t *args, unsigned char ***circular_data_buffer_ptr) {
  // Allocate memory for a new Tx object.
  phy_transmission_handle = (phy_transmission_t*)srslte_vec_malloc(sizeof(phy_transmission_t));
  // Check if memory allocation was correctly done.
  if(phy_transmission_handle == NULL) {
    PHY_TX_ERROR("Error allocating memory for Tx context.\n",0);
    return -1;
  }
  // Initialize some necessary fields of the phy transmission handle.
  phy_transmission_init_handle();
  // Initialize all pointers to NULL.
  phy_transmission_init_tx_data_buffer();
  // Set TX sample rate for TX chain accoring to the number of PRBs.
  phy_transmission_set_tx_sample_rate(rf, args->nof_prb, args->use_std_carrier_sep);
  // Get bandwidth from number of physical resource blocks.
  double default_tx_bandwidth = helpers_get_bw_from_nprb(args->nof_prb);
  // Configure initial TX frequency and gain.
  phy_transmission_set_initial_tx_freq_and_gain(rf, args->initial_tx_gain, args->competition_center_frequency, args->competition_bw, default_tx_bandwidth, args->default_tx_channel);
  // Initilize cell struture with transmission values.
  phy_transmission_init_cell_parameters(args->nof_prb, args->radio_id, args->nof_ports);
  // Convert from number of Resource Blocks to BW Index.
  uint32_t bw_idx = helpers_get_bw_index_from_prb(args->nof_prb);
  // Initialize structure with last configured TX basic control.
  phy_transmission_init_last_basic_control(args->competition_center_frequency, args->initial_tx_gain, bw_idx, args->radio_id, args->default_tx_channel, args->rf_amp);
  // Initialize the filter length based on the index passed through command line.
#if(ENABLE_PHY_TX_FILTERING==1)
  if(args->trx_filter_idx > 0) {
    trx_filter_init_filter_length(args->trx_filter_idx);
  }
#endif
  // Allocate memory for transmission buffers.
  phy_transmission_init_buffers(rf, args->nof_prb, args->trx_filter_idx);
  // Initialize base station structures.
  phy_transmission_base_init(args->rnti, bw_idx, args->trx_filter_idx);
  PHY_TX_PRINT("phy_transmission_base_init done!\n",0);
  // Initial update of allocation with MCS 0 and initial number of resource blocks.
  phy_transmission_update_radl(0, args->nof_prb);
  // Set phy reception handler.
  phy_transmission_set_handler(handle, rf, args);
  PHY_TX_PRINT("phy_transmission_set_handler done!\n",0);
  // Make sure all buffer pointers are free before allocation.
  phy_transmission_free_tx_buffer();
  PHY_TX_PRINT("phy_transmission_free_tx_buffer done!\n",0);
  // Allocate memory for data sent by upper layers.
  phy_transmission_allocate_tx_buffer(TX_DATA_LENGTH);
  *circular_data_buffer_ptr = (unsigned char **)&(phy_transmission_handle->tx_data_buffer);
  PHY_TX_PRINT("phy_transmission_allocate_tx_buffer done!\n",0);
  // Initialize mutex.
  if(pthread_mutex_init(&(phy_transmission_handle->phy_transmission_mutex), NULL) != 0) {
    PHY_TX_ERROR("Mutex init failed.\n",0);
    return -1;
  }
  // Initialize conditional variable.
  if(pthread_cond_init(&(phy_transmission_handle->phy_transmission_cv), NULL)) {
    PHY_TX_ERROR("Conditional variable init failed.\n",0);
    return -1;
  }
  // Set all timeout counters to 0.
  bzero((phy_transmission_handle->channel_timeout_counter), sizeof(uint64_t) * MAX_NUM_OF_CHANNELS);
  // Set threda to run.
  phy_transmission_handle->run_phy_transmission_thread = true;
  // Create threads to perform phy reception.
  pthread_attr_init(&(phy_transmission_handle->phy_transmission_thread_attr));
  pthread_attr_setdetachstate(&(phy_transmission_handle->phy_transmission_thread_attr), PTHREAD_CREATE_JOINABLE);
  // Create thread to sense channel.
  int rc = pthread_create(&(phy_transmission_handle->phy_transmission_thread_id), &(phy_transmission_handle->phy_transmission_thread_attr), phy_transmission_work, (void*)phy_transmission_handle);
  if(rc) {
    PHY_TX_ERROR("Return code from phy reception pthread_create() is %d\n", rc);
    return -1;
  }
  PHY_TX_PRINT("PHY Tx intialization done!\n",0);
  return 0;
}

// Free all the resources used by the phy transmission module.
int phy_transmission_uninitialize() {
  // Stop transmission thread.
  phy_transmission_handle->run_phy_transmission_thread = false;
  // Notify condition variable.
  pthread_cond_signal(&(phy_transmission_handle->phy_transmission_cv));
  // Destroy thread.
  pthread_attr_destroy(&(phy_transmission_handle->phy_transmission_thread_attr));
  int rc = pthread_join(phy_transmission_handle->phy_transmission_thread_id, NULL);
  if(rc) {
    PHY_TX_ERROR("Return code from phy reception pthread_join() is %d\n", rc);
    return -1;
  }
  // Destroy mutex.
  pthread_mutex_destroy(&(phy_transmission_handle->phy_transmission_mutex));
  // Destory conditional variable.
  if(pthread_cond_destroy(&(phy_transmission_handle->phy_transmission_cv)) != 0) {
    PHY_TX_ERROR("Conditional variable destruction failed.\n",0);
    return -1;
  }
#if(ENABLE_PHY_TX_FILTERING==1)
  // Free FIR Filter kernel.
  trx_filter_free_simd_kernel_mm256();
#endif
  // Free tx data vector.
  phy_transmission_free_tx_buffer();
  // Free all base station structures.
  phy_transmission_base_free();
  PHY_TX_INFO("phy_transmission_base_free done!\n",0);
  // Free memory used for transmission buffers.
  phy_transmission_free_buffers();
  PHY_TX_INFO("phy_transmission_free_buffers done!\n",0);
  // Free memory used to store Tx object.
  if(phy_transmission_handle) {
    free(phy_transmission_handle);
    phy_transmission_handle = NULL;
  }
  return 0;
}

// Initialize struture with Cell parameters.
void phy_transmission_init_cell_parameters(uint32_t num_of_prb, uint32_t radio_id, uint32_t nof_ports) {
  phy_transmission_handle->cell_enb.nof_prb          = num_of_prb;         // nof_prb
  phy_transmission_handle->cell_enb.nof_ports        = nof_ports;          // nof_ports
  phy_transmission_handle->cell_enb.bw_idx           = 0;                  // bw idx
  phy_transmission_handle->cell_enb.id               = radio_id;           // cell_id
  phy_transmission_handle->cell_enb.cp               = SRSLTE_CP_NORM;     // cyclic prefix
  phy_transmission_handle->cell_enb.phich_length     = SRSLTE_PHICH_NORM;  // PHICH length
  phy_transmission_handle->cell_enb.phich_resources  = SRSLTE_PHICH_R_1;   // PHICH resources
}

// Initialize structure with last configured TX basic control.
void phy_transmission_init_last_basic_control(uint32_t center_freq, int32_t tx_gain, uint32_t bw_idx, uint32_t radio_id, uint32_t default_channel, float rf_boost) {
  phy_transmission_handle->last_tx_basic_control.trx_flag     = PHY_TX_ST;
  phy_transmission_handle->last_tx_basic_control.seq_number   = 0;
  phy_transmission_handle->last_tx_basic_control.send_to      = radio_id;
  phy_transmission_handle->last_tx_basic_control.bw_idx       = bw_idx;
  phy_transmission_handle->last_tx_basic_control.ch           = default_channel; // Initial channel is set to 0.
  phy_transmission_handle->last_tx_basic_control.frame        = 0;
  phy_transmission_handle->last_tx_basic_control.slot         = 0;
  phy_transmission_handle->last_tx_basic_control.timestamp    = 0;
  phy_transmission_handle->last_tx_basic_control.mcs          = 0;
  phy_transmission_handle->last_tx_basic_control.gain         = tx_gain;
  phy_transmission_handle->last_tx_basic_control.rf_boost     = rf_boost;
  phy_transmission_handle->last_tx_basic_control.length       = 1;
}

void phy_transmission_set_handler(LayerCommunicator_handle handle, srslte_rf_t *rf, transceiver_args_t *args) {
  phy_transmission_handle->phy_comm_handle         = handle;
  phy_transmission_handle->rf                      = rf;
  phy_transmission_handle->competition_bw          = args->competition_bw;
  phy_transmission_handle->competition_center_freq = args->competition_center_frequency;
  phy_transmission_handle->rnti                    = args->rnti;
  phy_transmission_handle->phy_filtering           = args->phy_filtering;
  phy_transmission_handle->use_std_carrier_sep     = args->use_std_carrier_sep;
  phy_transmission_handle->is_lbt_enabled          = args->lbt_threshold < 100.0?true:false;
  phy_transmission_handle->send_tx_stats_to_mac    = args->send_tx_stats_to_mac;
  phy_transmission_handle->add_tx_timestamp        = args->add_tx_timestamp;
  phy_transmission_handle->initial_subframe_index  = args->initial_subframe_index;
  phy_transmission_handle->rf_amp                  = args->rf_amp;
  phy_transmission_handle->trx_filter_idx          = args->trx_filter_idx;
}

// This fucntion is used to check and change configuration parameters related to transmission.
uint32_t phy_transmission_change_parameters(srslte_rf_t *rf, basic_ctrl_t *bc) {
  uint32_t bw_idx;

  // Bandwidth.
  float tx_bandwidth = helpers_get_bandwidth(bc->bw_idx, &bw_idx);
  if(tx_bandwidth < 0.0 || bw_idx >= 100) {
    PHY_TX_ERROR("Invalid TX BW Index: %d\n",bc->bw_idx);
    exit(-1);
  }

  // Check if channel number is outside the possible range.
  if(bc->ch > MAX_NUM_OF_CHANNELS) {
    PHY_TX_ERROR("Invalid Channel: %d\n",bc->ch);
    exit(-1);
  }

  // Change Center frequency only if one of the parameters have changed.
  if(phy_transmission_handle->last_tx_basic_control.ch != bc->ch || phy_transmission_handle->last_tx_basic_control.bw_idx != bc->bw_idx) {
    // Channel.
    uint32_t tx_channel = bc->ch;
    // Update monitoring channel only if there is more than 1 channel available.
    if(rf->num_of_channels > 1 && phy_transmission_handle->is_lbt_enabled) {
      // Update channel to be monitored by the RF Monitor.
      rf_monitor_set_current_channel_to_monitor(bc->ch);
    }
    // Calculate central TX frequency for the channels.
    double tx_channel_center_freq = helpers_calculate_channel_center_frequency(phy_transmission_handle->competition_center_freq, phy_transmission_handle->competition_bw, tx_bandwidth, tx_channel);
    // Set central frequency for transmission.
    double lo_offset = rf->num_of_channels == 1 ? 0.0:(double)PHY_TX_LO_OFFSET;
#if(WAIT_FOR_BURST_ACK==1 || TX_TIMED_COMMAND_ENABLED==0)
    double actual_tx_freq = srslte_rf_set_tx_freq2(rf, tx_channel_center_freq, lo_offset, PHY_CHANNEL);
#else
    if(phy_transmission_handle->last_tx_basic_control.gain != bc->gain) {
      srslte_rf_set_tx_freq_and_gain_cmd(rf, tx_channel_center_freq, lo_offset, (float)bc->gain, bc->timestamp, TX_TIME_IN_ADVANCE, PHY_CHANNEL);
      // Update last TX basic control structure.
      phy_transmission_handle->last_tx_basic_control.gain = bc->gain;
    } else {
      srslte_rf_set_tx_freq_cmd(rf, tx_channel_center_freq, lo_offset, bc->timestamp, TX_TIME_IN_ADVANCE, PHY_CHANNEL);
    }
    double actual_tx_freq = tx_channel_center_freq;
#endif
    // Check if actual frequency is inside a range of +/- 10 Hz.
    if(actual_tx_freq < (tx_channel_center_freq - 10.0) || actual_tx_freq > (tx_channel_center_freq + 10.0)) {
       PHY_TX_ERROR("Requested freq.: %1.2f [MHz] - Actual freq.: %1.2f [MHz]................\n", tx_channel_center_freq, actual_tx_freq);
    }
    // Set the global variable with the current channel center frequency.
    phy_transmission_set_channel_center_freq(actual_tx_freq);
    // Update last tx basic control structure.
    phy_transmission_handle->last_tx_basic_control.ch = bc->ch;
    PHY_TX_DEBUG_TIME("TX ---> BW[%d]: %1.1f [MHz] - Channel: %d - Set freq to: %.2f [MHz] - Offset: %.2f [MHz]\n", bc->bw_idx, (tx_bandwidth/1000000.0), bc->ch, (actual_tx_freq/1000000.0),(lo_offset/1000000.0));
  }

  // Check send_to field range.
  if(bc->send_to >= MAXIMUM_NUMBER_OF_RADIOS) {
    PHY_TX_ERROR("Invalid send_to field: %d, it must be less than or equal to %d.\n", bc->send_to, MAXIMUM_NUMBER_OF_RADIOS);
    exit(-1);
  }

  // Generate new PSS and SSS signals if either send_to or BW index is different from last one.
  if(phy_transmission_handle->last_tx_basic_control.send_to != bc->send_to || phy_transmission_handle->last_tx_basic_control.bw_idx != bc->bw_idx) {
    // Set eNodeB PRB based on BW index.
    phy_transmission_handle->cell_enb.nof_prb = helpers_get_prb_from_bw_index(bc->bw_idx);
    // If PHY filtering is enabled then it is needed to set the Cell ID.
    // Check send_to field and generate PSS/SSS signals only if PHY filtering is enabled.
    if(phy_transmission_handle->phy_filtering) {
      phy_transmission_handle->cell_enb.id = bc->send_to;
    }

#if SCATTER_DEBUG_MODE
    struct timespec start_enb_config;
    clock_gettime(CLOCK_REALTIME, &start_enb_config);
#endif

    // Free all related TX structures.
    phy_transmission_base_free();
    // We only do the following if the bandwidth has changed by the upper layers.
    if(phy_transmission_handle->last_tx_basic_control.bw_idx != bc->bw_idx) {
      // Free the transmission buffers.
      phy_transmission_free_buffers();
      // Set new TX sample rate.
      phy_transmission_set_tx_sample_rate(rf, phy_transmission_handle->cell_enb.nof_prb, phy_transmission_handle->use_std_carrier_sep);
      // Allocate memory for transmission buffers.
      phy_transmission_init_buffers(rf, phy_transmission_handle->cell_enb.nof_prb, phy_transmission_handle->trx_filter_idx);
    }
    // Initialize base station structures.
    phy_transmission_base_init(phy_transmission_handle->rnti, bc->bw_idx, phy_transmission_handle->trx_filter_idx);

#if SCATTER_DEBUG_MODE
    helpers_profiling_print_diff_time2(start_enb_config, "Reconfiguration Elapsed time = %f milliseconds.\n");
#endif
    // Update last basic control structure with new value.
    phy_transmission_handle->last_tx_basic_control.send_to = bc->send_to;
    PHY_TX_DEBUG_TIME("TX send_to set to: %d\n",bc->send_to);
  }

  // Check MCS index range.
  if(bc->mcs < 0 || bc->mcs > 28) {
    PHY_TX_ERROR("Invalid MCS: %d!!\n",bc->mcs);
    exit(-1);
  }

  // Change MCS only if MCS or BW have changed.
  if(phy_transmission_handle->last_tx_basic_control.mcs != bc->mcs || phy_transmission_handle->last_tx_basic_control.bw_idx != bc->bw_idx) {
    // Update allocation with number of resource blocks and MCS.
    phy_transmission_update_radl(bc->mcs, phy_transmission_handle->cell_enb.nof_prb);
    // Update last TX basic control structure.
    phy_transmission_handle->last_tx_basic_control.mcs = bc->mcs;
    phy_transmission_handle->last_tx_basic_control.bw_idx = bc->bw_idx;
    PHY_TX_DEBUG_TIME("MCS set to: %d - TX BW index set to: %d\n", bc->mcs, bc->bw_idx);
  }

  // Change TX gain only if the parameter has changed.
  if(phy_transmission_handle->last_tx_basic_control.gain != bc->gain) {
    // Set TX gain.
#if(TX_TIMED_COMMAND_ENABLED==1)
    float tx_gain = srslte_rf_set_tx_gain_cmd(rf, (float)bc->gain, bc->timestamp, TX_TIME_IN_ADVANCE, PHY_CHANNEL);
#else
    float tx_gain = srslte_rf_set_tx_gain(rf, (float)bc->gain, PHY_CHANNEL);
#endif
    // Update last TX basic control structure.
    phy_transmission_handle->last_tx_basic_control.gain = bc->gain;
    PHY_TX_DEBUG_TIME("TX gain set to: %.1f [dB]\n", tx_gain);
  }

  // Change RF boost only if the parameter has changed.
  if(bc->rf_boost > 0.0 && phy_transmission_handle->last_tx_basic_control.rf_boost != bc->rf_boost) {
    phy_transmission_handle->rf_amp = bc->rf_boost;
    phy_transmission_handle->last_tx_basic_control.rf_boost = bc->rf_boost;
    PHY_TX_DEBUG_TIME("RF boost set to: %1.2f\n", bc->rf_boost);
  }

  // Check if data size sent by MAC is not bigger than the expected.
  if(bc->length % num_byte_per_1ms_mcs[bw_idx][bc->mcs] != 0) {
    PHY_TX_ERROR("Data length set by MAC is invalid. Length field in Basic control command: %d -  expected size: %d\n",bc->length, num_byte_per_1ms_mcs[bw_idx][bc->mcs]);
    exit(-1);
  }

  return bw_idx;
}

void set_number_of_tx_offset_samples(int num_samples_to_offset) {
  phy_transmission_handle->number_of_tx_offset_samples = num_samples_to_offset;
}

int get_number_of_tx_offset_samples() {
  return phy_transmission_handle->number_of_tx_offset_samples;
}

void *phy_transmission_work(void *h) {

  phy_stat_t phy_tx_stat;
  srslte_rf_t *rf = phy_transmission_handle->rf;
  int num_tx_data_subframe, sf_idx, subframe_cnt, tx_data_offset, number_of_additional_samples, subframe_buffer_offset, ret = 0;
  uint32_t bw_idx, nof_zero_padding_samples;
  srslte_dci_msg_t dci_msg;
  bool start_of_burst, end_of_burst;
  time_t full_secs = 0;
  double frac_secs = 0.0, coding_time = 0.0;
  bool has_time_spec = false;
  struct timespec start_data_tx;
  basic_ctrl_t bc;
  lbt_stats_t lbt_stats;
  uint64_t number_of_dropped_packets = 0;
  uint32_t filter_zero_padding_length = 0;
  uint64_t fpga_time = 0;

#if(ENABLE_MIB_ENCODING==1)
  int nf;
  uint32_t sfn; // System frame number (SFN)
#endif

#if(ENABLE_PHY_TX_FILTERING==1)
  // Retrieve filter length only if it is enabled.
  uint32_t trx_filter_length = 0;
  if(phy_transmission_handle->trx_filter_idx > 0) {
    trx_filter_length = trx_filter_get_filter_length() - 1;
  }
#endif

  // Set priority to TX thread.
  uhd_set_thread_priority(1.0, true);

  //ProfilerStart("phy_transmission_work.prof");

  /****************************** PHY Transmission loop - BEGIN ******************************/
  PHY_TX_DEBUG("Entering PHY Transmission thread loop...\n", 0);
  while(phy_transmission_wait_container_not_empty() && phy_transmission_handle->run_phy_transmission_thread) {

    // Get Basic Control from FIFO. It must be a blocking function.
    phy_transmission_pop_tx_basic_control_from_container(&bc);

    // Timestamp start of transmission procedure.
    clock_gettime(CLOCK_REALTIME, &start_data_tx);

    // Change transmission parameters according to received basic control message.
    bw_idx = phy_transmission_change_parameters(rf, &bc);

    // Calculate number of slots to be transmitted.
    num_tx_data_subframe = (bc.length/num_byte_per_1ms_mcs[bw_idx][bc.mcs]);
    PHY_TX_DEBUG("Number of slots to be transmitted: %d\n", num_tx_data_subframe);

    // Check if there is time specificication.
    has_time_spec = false;
    if(bc.timestamp != 0) {
      has_time_spec = true;
      helpers_convert_host_timestamp_into_uhd_timestamp_us(bc.timestamp, &full_secs, &frac_secs);
      PHY_TX_DEBUG("Send at timestamp: %" PRIu64 " - FPGA time: %" PRIu64 "\n", bc.timestamp, (fpga_time = helpers_get_fpga_timestamp_us(rf)));
      PHY_TX_DEBUG("Time difference: %d\n",(int)(bc.timestamp-fpga_time));
    }

    // Reset all necessary counters before transmitting.
#if(ENABLE_MIB_ENCODING==1)
    nf = 0;
    sfn = 0;
#endif
    sf_idx = phy_transmission_handle->initial_subframe_index;
    tx_data_offset = 0;
    subframe_cnt = 0;
    start_of_burst = true;
    end_of_burst = false;
    number_of_dropped_packets = 0; // Number of dropped packets in one request to send from MAC.

    PHY_TX_DEBUG("Entering Transmission (TX) loop...\n", 0);
    while(subframe_cnt < num_tx_data_subframe && phy_transmission_handle->run_phy_transmission_thread) {

      bzero(phy_transmission_handle->sf_buffer_eb, sizeof(cf_t) * phy_transmission_handle->sf_n_re);

      // Increase subframe counter number.
      subframe_cnt++;

      // Add PSS/SSS to subframe.
      if(sf_idx == 0 || sf_idx == 5) {
        srslte_pss_put_slot(phy_transmission_handle->pss_signal, phy_transmission_handle->sf_buffer_eb, phy_transmission_handle->cell_enb.nof_prb, phy_transmission_handle->cell_enb.cp);
        srslte_sss_put_slot(sf_idx ? phy_transmission_handle->sss_signal5 : phy_transmission_handle->sss_signal0, phy_transmission_handle->sf_buffer_eb, phy_transmission_handle->cell_enb.nof_prb, phy_transmission_handle->cell_enb.cp);
      }

      // Add reference signal (RS).
      srslte_refsignal_cs_put_sf(phy_transmission_handle->cell_enb, 0, phy_transmission_handle->est.csr_signal.pilots[0][sf_idx], phy_transmission_handle->sf_buffer_eb);

      // Encode MIB and PBCH.
#if(ENABLE_MIB_ENCODING==1)
      srslte_pbch_mib_pack(&(phy_transmission_handle->cell_enb), sfn, phy_transmission_handle->bch_payload);
      if(sf_idx == 0) {
        srslte_pbch_encode(&(phy_transmission_handle->pbch), phy_transmission_handle->bch_payload, phy_transmission_handle->slot1_symbols, nf%4);
      }
#endif

      // Encode PCFICH.
      srslte_pcfich_encode(&(phy_transmission_handle->pcfich), DEFAULT_CFI, phy_transmission_handle->sf_symbols, sf_idx);

      // Encode PDCCH.
      PHY_TX_DEBUG("Putting DCI to location: n=%d, L=%d\n", phy_transmission_handle->locations[sf_idx][0].ncce, phy_transmission_handle->locations[sf_idx][0].L);
      srslte_dci_msg_pack_pdsch(&(phy_transmission_handle->ra_dl), SRSLTE_DCI_FORMAT1, &dci_msg, phy_transmission_handle->cell_enb.nof_prb, false);
      if(srslte_pdcch_encode(&(phy_transmission_handle->pdcch), &dci_msg, phy_transmission_handle->locations[sf_idx][0], phy_transmission_handle->rnti, phy_transmission_handle->sf_symbols, sf_idx, DEFAULT_CFI)) {
        PHY_TX_ERROR("Error encoding DCI message\n",0);
        exit(-1);
      }

      // Transmit PDCCH + PDSCH only when there is data to send.
      srslte_ra_dl_dci_to_grant(&(phy_transmission_handle->ra_dl), phy_transmission_handle->cell_enb.nof_prb, phy_transmission_handle->rnti, &(phy_transmission_handle->grant));
      if(srslte_pdsch_cfg(&(phy_transmission_handle->pdsch_cfg), phy_transmission_handle->cell_enb, &(phy_transmission_handle->grant), DEFAULT_CFI, sf_idx, 0)) {
        PHY_TX_ERROR("Error configuring PDSCH\n",0);
        exit(-1);
      }

      // Add TX timestamp to data.
#if(ENABLE_TX_TO_RX_TIME_DIFF==1)
      uint64_t tx_timestamp;
      struct timespec tx_timestamp_struct;
      if(phy_transmission_handle->add_tx_timestamp) {
        clock_gettime(CLOCK_REALTIME, &tx_timestamp_struct);
        tx_timestamp = helpers_convert_host_timestamp(&tx_timestamp_struct);
        memcpy((void*)(bc.data+tx_data_offset), (void*)&tx_timestamp, sizeof(uint64_t));
      }
#endif

      // Encode PDSCH.
      if(srslte_pdsch_encode(&(phy_transmission_handle->pdsch), &(phy_transmission_handle->pdsch_cfg), &(phy_transmission_handle->softbuffer), bc.data+tx_data_offset, phy_transmission_handle->sf_symbols)) {
        PHY_TX_ERROR("Error encoding PDSCH\n",0);
        exit(-1);
      }
      // Print transmitted data for debug purposes.
      DEV_INFO("Transmitted byte: %d\n",bc.data[tx_data_offset]);

      // Increment data offset when there is more than 1 slot to be transmitted.
      tx_data_offset = tx_data_offset + num_byte_per_1ms_mcs[bw_idx][bc.mcs];

      // Check if it is necessary to add zeros before the subframe.
      number_of_additional_samples = 0;
      subframe_buffer_offset = FIX_TX_OFFSET_SAMPLES;
      if(start_of_burst) {
        number_of_additional_samples = get_number_of_tx_offset_samples();
        subframe_buffer_offset = 0;
        PHY_TX_DEBUG("Adding %d zero samples before start of subframe.\n",0);
      }

#if(ENABLE_PHY_TX_FILTERING==1)
      // Apply filter with zero padding so that we have a kind of f-OFDM implementation.
      if(phy_transmission_handle->trx_filter_idx > 0) {
        // Transform to OFDM symbols.
        srslte_ofdm_tx_sf(&(phy_transmission_handle->ifft), phy_transmission_handle->sf_buffer_eb, phy_transmission_handle->subframe_ofdm_symbols);
        filter_zero_padding_length = 0;
        if(subframe_cnt == num_tx_data_subframe) {
          filter_zero_padding_length = trx_filter_length;
        }
        //struct timespec start_filter;
        //clock_gettime(CLOCK_REALTIME, &start_filter);
        trx_filter_run_fir_tx_filter_sse_mm256_complex3(phy_transmission_handle->subframe_ofdm_symbols, (phy_transmission_handle->sf_n_samples+filter_zero_padding_length), phy_transmission_handle->output_buffer+FIX_TX_OFFSET_SAMPLES, subframe_cnt, num_tx_data_subframe);
        //PHY_PROFILLING_AVG3("Avg. filtering time: %f [ms] - min: %f - max: %f - max counter %d - diff >= 1.5ms: %d - total counter: %d - perc: %f\n", helpers_profiling_diff_time(start_filter), 1.5, 1000);
      } else {
#endif
        // Case filtering is not enabled, then only OFDM generation is performed.
        filter_zero_padding_length = 0;
        // Transform to OFDM symbols.
        srslte_ofdm_tx_sf(&(phy_transmission_handle->ifft), phy_transmission_handle->sf_buffer_eb, phy_transmission_handle->output_buffer+FIX_TX_OFFSET_SAMPLES);
#if(ENABLE_PHY_TX_FILTERING==1)
      }
#endif

      // If last subframe to be transmitted then we flag that it is the end of burst.
      nof_zero_padding_samples = 0;
      if(subframe_cnt == num_tx_data_subframe) {
        // If this is the last subframe of a frame, then, set the end of burst flag to true.
        end_of_burst = true;
        // If this is the last subframe of a frame then we add some zeros at the end of this subframe.
        nof_zero_padding_samples = NOF_PADDING_ZEROS;
      }
      float norm_factor = (float) phy_transmission_handle->cell_enb.nof_prb/15/sqrtf(phy_transmission_handle->pdsch_cfg.grant.nof_prb);
      srslte_vec_sc_prod_cfc(phy_transmission_handle->output_buffer+FIX_TX_OFFSET_SAMPLES, (phy_transmission_handle->rf_amp*norm_factor), phy_transmission_handle->output_buffer+FIX_TX_OFFSET_SAMPLES, SRSLTE_SF_LEN_PRB(phy_transmission_handle->cell_enb.nof_prb)+filter_zero_padding_length);
      ret = srslte_rf_send_timed3(rf, (phy_transmission_handle->output_buffer+subframe_buffer_offset), (phy_transmission_handle->sf_n_samples+number_of_additional_samples+nof_zero_padding_samples+filter_zero_padding_length), full_secs, frac_secs, has_time_spec, true, start_of_burst, end_of_burst, phy_transmission_handle->is_lbt_enabled, (void*)&lbt_stats, PHY_CHANNEL);
      start_of_burst = false;

#if(WAIT_FOR_BURST_ACK==1)
      bool got_async_burst_ack = false;
      //loop through all messages for the ACK packet (may have underflow messages in queue)
      while(end_of_burst && !got_async_burst_ack){
         got_async_burst_ack = srslte_rf_is_burst_transmitted(rf, PHY_CHANNEL);
      }
      PHY_TX_DEBUG("Burst successfully transmitted.\n",0);
#endif

#if(WRITE_TX_SUBFRAME_INTO_FILE==1)
    static unsigned int dump_cnt = 0;
    char output_file_name[200] = "f_ofdm_tx_side_assessment_5MHz.dat";
    srslte_filesink_t file_sink;
    if(dump_cnt==0) {
      filesink_init(&file_sink, output_file_name, SRSLTE_COMPLEX_FLOAT_BIN);
      // Write samples into file.
      filesink_write(&file_sink, phy_transmission_handle->output_buffer+FIX_TX_OFFSET_SAMPLES, phy_transmission_handle->sf_n_samples+filter_zero_padding_length);
      // Close file.
      filesink_free(&file_sink);
      dump_cnt++;
      PHY_TX_PRINT("File dumped: %d.\n",dump_cnt);
    }
#endif

      // Do not transmit the slot(s) if LBT timed out and go to next TX Request (TX Basic Control + data) sent by upper layer.
      if(ret == RF_LBT_TIMEOUT_CODE) {
        phy_transmission_handle->channel_timeout_counter[bc.ch] = phy_transmission_handle->channel_timeout_counter[bc.ch] + num_tx_data_subframe;
        number_of_dropped_packets = num_tx_data_subframe;
        PHY_TX_PRINT("Total number of slots dropped as LBT timed-out for channel %d: %d\n",bc.ch,phy_transmission_handle->channel_timeout_counter[bc.ch]);
        break; // TODO: Check if it is better to drop only one slot or all of the subsequent ones when LBT times-out. When using the multiTB feature, i.e., TX of more than 1 TB at once, should we drop all of the slots, or just the one for which the LBT procedure has timed-out, the subsequent slots are transmitted if LBT allows TX.
      }

#if(ADD_PSS_TO_ALL_SUBFRAMES==0)
  // Add PSS/SSS only to the very first subframe in a COT.
  #if(ADD_PSS_ONLY_TO_VERY_FIRST_SUBFRAME==1)
      // here we use subframe 1 to send the remaning data, however, it could be any subframe different from 0 and 5.
      sf_idx = 1;
  #else
      // Increase the subframe number. If it is disabled, then subframe number is always equal to initial_subframe_index and then PSS/SSS is always added to the subframe.
      if(sf_idx == SRSLTE_NSUBFRAMES_X_FRAME-1) {
        sf_idx = 0;
  #if(ENABLE_MIB_ENCODING==1)
        nf++;
        sfn = (sfn + 1) % 1024;
  #endif
      } else {
        sf_idx = sf_idx + 1;
      }
  #endif // ADD_PSS_ONLY_TO_VERY_FIRST_SUBFRAME
#endif // ADD_PSS_TO_ALL_SUBFRAMES

    }

    // Check if transmission of TX stats to MAc is enabled.
    if(phy_transmission_handle->send_tx_stats_to_mac) {
      // Calculate coding time.
      coding_time = helpers_profiling_diff_time(start_data_tx);
      // Create a PHY TX Stat struture to inform upper layers transmission was successful.
      // Set common values to the TX Stats Structure.
      phy_tx_stat.seq_number = bc.seq_number;                     // Sequence number used by upper layer to track the response of PHY, i.e., correlates one basic_control message with a phy_stat message.
      phy_tx_stat.host_timestamp = helpers_get_host_time_now();   // Host PC time value when (ch,slot) PHY data are demodulated.
      phy_tx_stat.ch = bc.ch;				                              // Channel number which in turn is translated to a central frequency. Range: [0, 59]
      phy_tx_stat.mcs = bc.mcs;                                   // Set MCS to unspecified number. If this number is receiber by upper layer it means nothing was received and status MUST be checked.
      phy_tx_stat.num_cb_total = num_tx_data_subframe;            // Number of slots requested to be transmitted.
      phy_tx_stat.num_cb_err = number_of_dropped_packets;         // Number of slots dropped for the current request from MAC.
      phy_tx_stat.stat.tx_stat.power = bc.gain;                   // Gain used for transmission.
      phy_tx_stat.stat.tx_stat.channel_free_cnt = lbt_stats.channel_free_cnt;
      phy_tx_stat.stat.tx_stat.channel_busy_cnt = lbt_stats.channel_busy_cnt;
      phy_tx_stat.stat.tx_stat.free_energy = lbt_stats.free_energy;
      phy_tx_stat.stat.tx_stat.busy_energy = lbt_stats.busy_energy;
      phy_tx_stat.stat.tx_stat.total_dropped_slots = phy_transmission_handle->channel_timeout_counter[bc.ch]; // Total number of slots dropped so far for the current channel.
      phy_tx_stat.stat.tx_stat.coding_time = coding_time;
      phy_tx_stat.stat.tx_stat.rf_boost = phy_transmission_handle->rf_amp;     // RF boost is applied to slot before its actual transmission.
      // Send phy transmission (TX) statistics.
      phy_transmission_send_tx_statistics(phy_transmission_handle->phy_comm_handle, &phy_tx_stat, ret);
    }

    //helpers_measure_packets_per_second("TX");

    // Print TX statistics on screen.
    PHY_TX_INFO_TIME("[TX STATS]: TX slots: %d - PRB: %d - Channel: %d - Freq: %.2f [MHz] - MCS: %d - TX Gain: %d - CID: %d - LBT Timeout: %d - # LBT Timeouts: %d - Coding time: %f [ms]\n", num_tx_data_subframe, phy_transmission_handle->cell_enb.nof_prb, bc.ch, phy_transmission_get_channel_center_freq()/1000000.0, bc.mcs, bc.gain, phy_transmission_handle->cell_enb.id, (ret == RF_LBT_TIMEOUT_CODE ? 1:0), phy_transmission_handle->channel_timeout_counter[bc.ch], helpers_profiling_diff_time(start_data_tx));

    //PHY_PROFILLING_AVG3("Avg. coding time: %f [ms] - min: %f - max: %f - max counter %d - diff >= 0.5ms: %d - total counter: %d - perc: %f\n", helpers_profiling_diff_time(start_data_tx), 0.5, 1000);
  }
  /****************************** PHY Transmission loop - END ******************************/

  //ProfilerStop();

  PHY_TX_DEBUG("Leaving PHY Transmission thread.\n",0);
  // Exit thread with result code.
  pthread_exit(NULL);
}

void phy_transmission_send_tx_statistics(LayerCommunicator_handle handle, phy_stat_t *phy_tx_stat, int ret) {
  // Set common values to the TX Stats Structure.
  phy_tx_stat->status = (ret > 0) ? PHY_SUCCESS : PHY_LBT_TIMEOUT; // Layer receceiving this message MUST check the statistics it is carrying for real status of the current request.
  phy_tx_stat->frame  = 0;                                         // Frame number.
  phy_tx_stat->slot   = 0;			                                   // Time slot number. Range: [0, 2000]

  // Sending PHY TX statistics to upper layer.
  PHY_TX_DEBUG("Sending TX statistics information upwards...\n",0);
  // Send TX stats. There is a mutex on this function which prevents RX from sending statistics to PHY at the same time.
  communicator_send_phy_stat_message(handle, TX_STAT, phy_tx_stat, NULL);
  // Print TX stats information.
  HELPERS_PRINT_TX_STATS(phy_tx_stat);
}

unsigned int phy_transmission_reverse(register unsigned int x) {
  x = (((x & 0xaaaaaaaa) >> 1) | ((x & 0x55555555) << 1));
  x = (((x & 0xcccccccc) >> 2) | ((x & 0x33333333) << 2));
  x = (((x & 0xf0f0f0f0) >> 4) | ((x & 0x0f0f0f0f) << 4));
  x = (((x & 0xff00ff00) >> 8) | ((x & 0x00ff00ff) << 8));
  return((x >> 16) | (x << 16));
}

uint32_t phy_transmission_prbset_to_bitmask(uint32_t nof_prb) {
  uint32_t mask = 0;
  int nb = (int) ceilf((float) nof_prb / srslte_ra_type0_P(nof_prb));
  for(int i = 0; i < nb; i++) {
    if(i >= 0 && i < nb) {
      mask = mask | (0x1<<i);
    }
  }
  return phy_transmission_reverse(mask)>>(32-nb);
}

int phy_transmission_update_radl(uint32_t mcs, uint32_t nof_prb) {
  bzero(&(phy_transmission_handle->ra_dl), sizeof(srslte_ra_dl_dci_t));
  phy_transmission_handle->ra_dl.harq_process = 0;
  phy_transmission_handle->ra_dl.mcs_idx = mcs;
  phy_transmission_handle->ra_dl.ndi = 0;
  phy_transmission_handle->ra_dl.rv_idx = 0;
  phy_transmission_handle->ra_dl.alloc_type = SRSLTE_RA_ALLOC_TYPE0;
  phy_transmission_handle->ra_dl.type0_alloc.rbg_bitmask = phy_transmission_prbset_to_bitmask(nof_prb);
  return 0;
}

void phy_transmission_init_buffers(srslte_rf_t *rf, uint32_t nof_prb, uint32_t trx_filter_idx) {
  // calculate number of resource elements and IQ samples.
  phy_transmission_handle->sf_n_re = 2 * SRSLTE_CP_NORM_NSYMB * nof_prb * SRSLTE_NRE;
  phy_transmission_handle->sf_n_samples = 2 * SRSLTE_SLOT_LEN(srslte_symbol_sz(nof_prb));

  PHY_TX_PRINT("sf_n_re: %d\n",phy_transmission_handle->sf_n_re);
  PHY_TX_PRINT("sf_n_samples: %d\n",phy_transmission_handle->sf_n_samples);

  // Retrieve filter length.
  uint32_t trx_filter_length = 0;

#if(ENABLE_PHY_TX_FILTERING==1)
  if(trx_filter_idx > 0) {
    trx_filter_length = trx_filter_get_filter_length() - 1;
  }
#endif

  // Retrieve device name.
  const char *devname = srslte_rf_name(rf);
  PHY_TX_DEBUG("Device name: %s\n",devname);

  // Decide the number of samples in a subframe.
  int number_of_subframe_samples = phy_transmission_handle->sf_n_samples;
  set_number_of_tx_offset_samples(0);
  if(strcmp(devname,DEVNAME_X300) == 0 && FIX_TX_OFFSET_SAMPLES > 0) {
    number_of_subframe_samples = phy_transmission_handle->sf_n_samples + FIX_TX_OFFSET_SAMPLES;
    set_number_of_tx_offset_samples(FIX_TX_OFFSET_SAMPLES);
    PHY_TX_DEBUG("HW: %s and TX Offset: %d - zero padding: %d\n",devname,FIX_TX_OFFSET_SAMPLES,NOF_PADDING_ZEROS);
  }

  // Increase the number of samples so that there is room for padding zeros.
  if(NOF_PADDING_ZEROS > 0) {
    number_of_subframe_samples = number_of_subframe_samples + NOF_PADDING_ZEROS;
  }

  // init memory.
  phy_transmission_handle->subframe_ofdm_symbols = (cf_t*)srslte_vec_malloc(sizeof(cf_t)*(phy_transmission_handle->sf_n_samples+trx_filter_length));
  if(!phy_transmission_handle->subframe_ofdm_symbols) {
    PHY_TX_ERROR("Error allocating memory to subframe_ofdm_symbols",0);
    exit(-1);
  }
  // Set allocated memory to 0.
  bzero(phy_transmission_handle->subframe_ofdm_symbols, sizeof(cf_t)*(phy_transmission_handle->sf_n_samples+trx_filter_length));
  PHY_TX_PRINT("subframe_ofdm_symbols allocated and zeroed\n",0);

  // init memory.
  phy_transmission_handle->output_buffer = (cf_t*)srslte_vec_malloc(sizeof(cf_t)*(number_of_subframe_samples+trx_filter_length));
  if(!phy_transmission_handle->output_buffer) {
    PHY_TX_ERROR("Error allocating memory to output_buffer",0);
    exit(-1);
  }
  // Set allocated memory to 0.
  // TODO: when online bandwidth change is implemented this setting of memory to 0 will have an impact on the time to change the bandwidth. That should be taken into account.
  bzero(phy_transmission_handle->output_buffer, sizeof(cf_t)*(number_of_subframe_samples+trx_filter_length));
  PHY_TX_PRINT("output_buffer allocated and zeroed\n",0);

  // init memory.
  phy_transmission_handle->sf_buffer_eb = (cf_t*)srslte_vec_malloc(sizeof(cf_t)*phy_transmission_handle->sf_n_re);
  if(!phy_transmission_handle->sf_buffer_eb) {
    PHY_TX_ERROR("Error allocating memory to sf_buffer_eb",0);
    exit(-1);
  }
  PHY_TX_PRINT("sf_buffer_eb allocated and zeroed\n",0);
}

void phy_transmission_free_buffers() {
  if(phy_transmission_handle->sf_buffer_eb) {
    free(phy_transmission_handle->sf_buffer_eb);
    phy_transmission_handle->sf_buffer_eb = NULL;
  }
  if(phy_transmission_handle->output_buffer) {
    free(phy_transmission_handle->output_buffer);
    phy_transmission_handle->output_buffer = NULL;
  }
  if(phy_transmission_handle->subframe_ofdm_symbols) {
    free(phy_transmission_handle->subframe_ofdm_symbols);
    phy_transmission_handle->subframe_ofdm_symbols = NULL;
  }
  PHY_TX_PRINT("phy_transmission_free_buffers DONE!\n",0);
}

void phy_transmission_base_init(uint16_t rnti, uint32_t bw_idx, uint32_t trx_filter_idx) {
#if(ENABLE_PHY_TX_FILTERING==1)
  // Create filter kernel.
  if(trx_filter_idx > 0) {
   trx_filter_create_tx_simd_kernel_mm256(helpers_get_bw_index(bw_idx));
  }
#endif
  // create ifft object.
  if(srslte_ofdm_tx_init(&(phy_transmission_handle->ifft), phy_transmission_handle->cell_enb.cp, phy_transmission_handle->cell_enb.nof_prb)) {
    PHY_TX_ERROR("Error creating iFFT object\n",0);
    exit(-1);
  }
  srslte_ofdm_set_normalize(&(phy_transmission_handle->ifft), true);
  if(srslte_pbch_init(&(phy_transmission_handle->pbch), phy_transmission_handle->cell_enb)) {
    PHY_TX_ERROR("Error creating PBCH object\n",0);
    exit(-1);
  }

  if(srslte_regs_init(&(phy_transmission_handle->regs), phy_transmission_handle->cell_enb)) {
    PHY_TX_ERROR("Error initiating regs\n",0);
    exit(-1);
  }

  if(srslte_pcfich_init(&(phy_transmission_handle->pcfich), &(phy_transmission_handle->regs), phy_transmission_handle->cell_enb)) {
    PHY_TX_ERROR("Error creating PBCH object\n",0);
    exit(-1);
  }

  if(srslte_regs_set_cfi(&(phy_transmission_handle->regs), DEFAULT_CFI)) {
    PHY_TX_ERROR("Error setting CFI\n",0);
    exit(-1);
  }

  if(srslte_pdcch_init(&(phy_transmission_handle->pdcch), &(phy_transmission_handle->regs), phy_transmission_handle->cell_enb)) {
    PHY_TX_ERROR("Error creating PDCCH object\n",0);
    exit(-1);
  }

  if(srslte_pdsch_init(&(phy_transmission_handle->pdsch), phy_transmission_handle->cell_enb)) {
    PHY_TX_ERROR("Error creating PDSCH object\n",0);
    exit(-1);
  }

  srslte_pdsch_set_rnti(&(phy_transmission_handle->pdsch), rnti);

  if(srslte_softbuffer_tx_init(&(phy_transmission_handle->softbuffer), phy_transmission_handle->cell_enb.nof_prb)) {
    PHY_TX_ERROR("Error initiating soft buffer\n",0);
    exit(-1);
  }
  // Generate PSS/SSS signals.
  int N_id_2 = phy_transmission_handle->cell_enb.id % 3;
  srslte_pss_generate(phy_transmission_handle->pss_signal, N_id_2);
  srslte_sss_generate(phy_transmission_handle->sss_signal0, phy_transmission_handle->sss_signal5, phy_transmission_handle->cell_enb.id);
  // Generate CRS signals
  if(srslte_chest_dl_init(&(phy_transmission_handle->est), phy_transmission_handle->cell_enb)) {
    PHY_TX_ERROR("Error initializing equalizer\n",0);
    exit(-1);
  }
  // Initialize slot (subframe).
  for(int i = 0; i < SRSLTE_MAX_PORTS; i++) { // now there's only 1 port
    phy_transmission_handle->sf_symbols[i] = phy_transmission_handle->sf_buffer_eb;
#if(ENABLE_MIB_ENCODING==1)
    phy_transmission_handle->slot1_symbols[i] = &(phy_transmission_handle->sf_buffer_eb[SRSLTE_SLOT_LEN_RE(phy_transmission_handle->cell_enb.nof_prb, phy_transmission_handle->cell_enb.cp)]);
#endif
  }
  // Initiate valid DCI locations.
  for(int i = 0; i < SRSLTE_NSUBFRAMES_X_FRAME; i++) {
    srslte_pdcch_ue_locations(&(phy_transmission_handle->pdcch), phy_transmission_handle->locations[i], 30, i, DEFAULT_CFI, rnti);
  }
  // Reset softbuffer x reset.
  srslte_softbuffer_tx_reset(&(phy_transmission_handle->softbuffer));
}

void phy_transmission_base_free() {
  srslte_softbuffer_tx_free(&(phy_transmission_handle->softbuffer));
  srslte_pdsch_free(&(phy_transmission_handle->pdsch));
  srslte_pdcch_free(&(phy_transmission_handle->pdcch));
  srslte_regs_free(&(phy_transmission_handle->regs));
  srslte_pbch_free(&(phy_transmission_handle->pbch));
  srslte_ofdm_tx_free(&(phy_transmission_handle->ifft));
}

void phy_transmission_set_tx_sample_rate(srslte_rf_t *rf, uint32_t nof_prb, bool use_std_carrier_sep) {
  int srate = -1;
  if(use_std_carrier_sep) {
    srate = srslte_sampling_freq_hz(nof_prb);
  } else {
    srate = helpers_non_std_sampling_freq_hz(nof_prb);
    PHY_TX_PRINT("Setting a non-standard sampling rate: %1.2f [MHz]\n",srate/1000000.0);
  }
  if(srate != -1) {
    float srate_rf = srslte_rf_set_tx_srate(rf, (double)srate, PHY_CHANNEL);
    if(srate_rf != srate) {
      PHY_TX_ERROR("Could not set TX sampling rate\n",0);
      exit(-1);
    }
    PHY_TX_PRINT("Set TX sampling rate to: %.2f [MHz]\n",srate_rf/1000000.0);
  } else {
    PHY_TX_ERROR("Invalid number of PRB (TX): %d\n", nof_prb);
    exit(-1);
  }
}

void phy_transmission_set_initial_tx_freq_and_gain(srslte_rf_t *rf, float default_tx_gain, double default_center_freq, double competition_bw, double tx_bandwidth, uint32_t default_tx_channel) {
  // Set default TX gain.
  float current_tx_gain = srslte_rf_set_tx_gain(rf, default_tx_gain, PHY_CHANNEL);
  // Calculate default central TX frequency.
  double tx_channel_center_freq = helpers_calculate_channel_center_frequency(default_center_freq, competition_bw, tx_bandwidth, default_tx_channel);
  // Set default central frequency for transmission.
  double lo_offset = rf->num_of_channels == 1 ? 0.0:(double)PHY_TX_LO_OFFSET;
  double actual_tx_freq = srslte_rf_set_tx_freq2(rf, tx_channel_center_freq, lo_offset, PHY_CHANNEL);
  // Check if actual frequency is inside a range of +/- 10 Hz.
  if(actual_tx_freq < (tx_channel_center_freq - 10.0) || actual_tx_freq > (tx_channel_center_freq + 10.0)) {
     PHY_TX_ERROR("[Initialization] Requested freq.: %1.2f [MHz] - Actual freq.: %1.2f [MHz] - Center Frequency: %1.2f [MHz] - Competition BW: %1.2f [MHz] - PHY BW: %1.2f [MHz] - Channel: %d \n", tx_channel_center_freq/1000000.0, actual_tx_freq/1000000.0, default_center_freq/1000000.0, competition_bw/1000000.0, tx_bandwidth/1000000.0, default_tx_channel);
  }
  phy_transmission_set_channel_center_freq(actual_tx_freq);
  PHY_TX_PRINT("Set TX gain to: %.1f dB\n", current_tx_gain);
  PHY_TX_PRINT("Set initial TX freq to: %.2f [MHz] with offset of: %.2f [MHz]\n", (actual_tx_freq/1000000.0),(lo_offset/1000000.0));
}

// Functions to transfer basic control message from main thread to transmission thread.
void phy_transmission_push_tx_basic_control_to_container(basic_ctrl_t *basic_ctrl) {
  // Lock mutex so that we can push basic control to container.
  pthread_mutex_lock(&(phy_transmission_handle->phy_transmission_mutex));
  // Push basic control into container.
  push_tx_basic_control_to_container(phy_transmission_handle->phy_transmission_wr_cnt, basic_ctrl);
  // Increment counter so that itcan be used as next key.
  phy_transmission_handle->phy_transmission_wr_cnt++;
  // Unlock mutex so that function can do other things.
  pthread_mutex_unlock(&(phy_transmission_handle->phy_transmission_mutex));
  // Notify other thread that basic control was pushed into container.
  pthread_cond_signal(&(phy_transmission_handle->phy_transmission_cv));
}

void phy_transmission_pop_tx_basic_control_from_container(basic_ctrl_t *basic_ctrl) {
  // Lock mutex so that we can push basic control to container.
  pthread_mutex_lock(&(phy_transmission_handle->phy_transmission_mutex));
  // Debugging the pace of pushing and popping from the QUEUE.
#if(ENABLE_TX_IPC_DEBUG==1)
  PHY_TX_PRINT("Retrieving Tx control number %d from queue.\n", phy_transmission_handle->phy_transmission_rd_cnt);
#endif
  // Retrieve mapped element from container.
  pop_tx_basic_control_from_container(phy_transmission_handle->phy_transmission_rd_cnt, basic_ctrl);
  // Increment conter for reading.
  phy_transmission_handle->phy_transmission_rd_cnt++;
  // Unlock mutex.
  pthread_mutex_unlock(&(phy_transmission_handle->phy_transmission_mutex));
}

bool phy_transmission_wait_container_not_empty() {
  bool ret = true;
  // Lock mutex so that we can wait for basic control.
  pthread_mutex_lock(&(phy_transmission_handle->phy_transmission_mutex));
  // Wait for conditional variable only if container is empty.
  if(is_tx_basic_control_container_empty()) {
    // Wait for conditional variable to be true.
    pthread_cond_wait(&(phy_transmission_handle->phy_transmission_cv), &(phy_transmission_handle->phy_transmission_mutex));
    if(!phy_transmission_handle->run_phy_transmission_thread) {
      ret = false;
    }
  }
  // Unlock mutex.
  pthread_mutex_unlock(&(phy_transmission_handle->phy_transmission_mutex));
  return ret;
}

void phy_transmission_set_channel_center_freq(double freq) {
  phy_transmission_handle->tx_channel_center_frequency = freq;
}

double phy_transmission_get_channel_center_freq() {
  return phy_transmission_handle->tx_channel_center_frequency;
}

void phy_transmission_allocate_tx_buffer(size_t tx_data_buffer_length) {
  for(int i = 0; i < NUMBER_OF_TX_DATA_BUFFERS; i++) {
    phy_transmission_handle->tx_data_buffer[i] = (unsigned char*)srslte_vec_malloc(tx_data_buffer_length);
    if(phy_transmission_handle->tx_data_buffer[i] == NULL) {
      PHY_TX_ERROR("[COMM ERROR] Malloc failed.\n",0);
      exit(-1);
    }
  }
}

void phy_transmission_free_tx_buffer() {
  for(int i = 0; i < NUMBER_OF_TX_DATA_BUFFERS; i++) {
    if(phy_transmission_handle->tx_data_buffer[i] != NULL) {
      free(phy_transmission_handle->tx_data_buffer[i]);
      phy_transmission_handle->tx_data_buffer[i] = NULL;
    }
  }
}

void phy_transmission_init_tx_data_buffer() {
  for(int i = 0; i < NUMBER_OF_TX_DATA_BUFFERS; i++) {
    phy_transmission_handle->tx_data_buffer[i] = NULL;
  }
}

void phy_transmission_init_handle() {
  phy_transmission_handle->number_of_tx_offset_samples = 0;
  phy_transmission_handle->sf_n_re = 0;
  phy_transmission_handle->sf_n_samples = 0;

  phy_transmission_handle->sf_buffer_eb = NULL;
  phy_transmission_handle->output_buffer = NULL;
  phy_transmission_handle->subframe_ofdm_symbols = NULL;

  phy_transmission_handle->phy_transmission_wr_cnt = 0;
  phy_transmission_handle->phy_transmission_rd_cnt = 0;
}
