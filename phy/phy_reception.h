#ifndef _PHY_RECEPTION_H_
#define _PHY_RECEPTION_H_

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <string.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <uhd.h>

#include "srslte/srslte.h"
#include "srslte/intf/intf.h"

#include "../../../../communicator/cpp/communicator_wrapper.h"
#include "helpers.h"
#include "transceiver.h"
#include "plot.h"

// ****************** Definition of macros ***********************
#define ENABLE_PHY_RX_PRINTS 1

#define PHY_RX_PRINT(_fmt, ...) do { if(ENABLE_PHY_RX_PRINTS && scatter_verbose_level >= 0) { \
  fprintf(stdout, "[PHY RX PRINT]: " _fmt, __VA_ARGS__); } } while(0)

#define PHY_RX_DEBUG(_fmt, ...) do { if(ENABLE_PHY_RX_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_DEBUG) { \
  fprintf(stdout, "[PHY RX DEBUG]: " _fmt, __VA_ARGS__); } } while(0)

#define PHY_RX_INFO(_fmt, ...) do { if(ENABLE_PHY_RX_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_INFO) { \
  fprintf(stdout, "[PHY RX INFO]: " _fmt, __VA_ARGS__); } } while(0)

#define PHY_RX_ERROR(_fmt, ...) do { fprintf(stdout, "[PHY RX ERROR]: " _fmt, __VA_ARGS__); } while(0)

#define PHY_RX_PRINT_TIME(_fmt, ...) do { if(ENABLE_PHY_RX_PRINTS && scatter_verbose_level >= 0) { \
  char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[PHY RX PRINT]: %s - " _fmt, date_time_str, __VA_ARGS__); } } while(0)

#define PHY_RX_DEBUG_TIME(_fmt, ...) do { if(ENABLE_PHY_RX_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_DEBUG) { \
  char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[PHY RX DEBUG]: %s - " _fmt, date_time_str, __VA_ARGS__); } } while(0)

#define PHY_RX_INFO_TIME(_fmt, ...) do { if(ENABLE_PHY_RX_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_INFO) { \
  char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[PHY RX INFO]: %s - " _fmt, date_time_str, __VA_ARGS__); } } while(0)

#define PHY_RX_ERROR_TIME(_fmt, ...) do { char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[PHY RX ERROR]: %s - " _fmt, date_time_str, __VA_ARGS__); } while(0)

// ****************** Definition of types ******************
typedef struct {
  LayerCommunicator_handle phy_comm_handle;
  srslte_rf_t *rf;
  float initial_rx_gain;
  double competition_bw;
  double competition_center_freq;
  uint16_t rnti;
  float initial_agc_gain;
  bool use_std_carrier_sep;
  int initial_subframe_index; // Set the subframe index number to be used to start from.
  bool add_tx_timestamp;
  bool plot_rx_info;

  pthread_attr_t phy_reception_decoding_thread_attr;
  pthread_t phy_reception_decoding_thread_id;

  pthread_attr_t phy_reception_sync_thread_attr;
  pthread_t phy_reception_sync_thread_id;

  // Variable used to stop phy decoding thread.
  volatile sig_atomic_t run_phy_decoding_thread;
  // Variable used to stop phy synchronization thread.
  volatile sig_atomic_t run_phy_synchronization_thread;

  // This basic controls stores the last configured values.
  basic_ctrl_t last_rx_basic_control;

  // This mutex is used to synchronize the access to the last configured basic control.
  pthread_mutex_t phy_rx_basic_control_mutex;

  // This pointer is used to point to the subframe.
  cf_t *sf_buffer;

  // Structures used to decode subframes.
  srslte_ue_dl_t ue_dl;
  srslte_ue_sync_t ue_sync;
  srslte_cell_t cell_ue;
  srslte_ue_mib_t ue_mib;

  // Mutex used to synchronize between synchronization and decoding thread.
  pthread_mutex_t phy_reception_ue_sync_mutex;
  // Condition variable used to synchronize between synchronization and decoding thread.
  pthread_cond_t phy_reception_ue_sync_cv;

} phy_reception_t;

typedef enum {DECODE_MIB, DECODE_PDSCH} rx_state_t;

// *************** Declaration of functions ***************
void phy_reception_init_last_basic_control(uint32_t center_freq, int32_t rx_gain, uint32_t bw_idx, uint32_t radio_id, uint32_t default_channel);

void phy_reception_set_handler(LayerCommunicator_handle handle, srslte_rf_t *rf, transceiver_args_t *args);

int phy_reception_initialize(LayerCommunicator_handle handle, srslte_rf_t *rf, transceiver_args_t *args);

int phy_reception_uninitialize(transceiver_args_t *args);

void phy_reception_change_parameters(srslte_rf_t *rf, basic_ctrl_t *bc);

void phy_reception_init_cell_parameters(uint32_t num_of_prb, uint32_t radio_id, uint32_t nof_ports);

void *phy_reception_decoding_work(void *h);

void phy_reception_send_rx_statistics(LayerCommunicator_handle handle, phy_stat_t *phy_rx_stat);

void set_channel_number(uint32_t channel);

void set_bw_index(uint32_t bw_index);

void set_number_of_expected_slots(uint32_t length);

void set_rx_gain(uint32_t rx_gain);

uint32_t get_rx_gain();

void set_sequence_number(uint64_t seq_number);

uint64_t get_sequence_number();

void phy_reception_ue_init(srslte_rf_t *rf, uint16_t rnti, float initial_rx_gain, float initial_agc_gain, int initial_subframe_index);

void phy_reception_stop_rx_stream_and_flush_buffer(srslte_rf_t *rf);

void phy_reception_initialize_rx_stream(srslte_rf_t *rf);

void phy_reception_ue_free(srslte_rf_t *rf, float initial_rx_gain);

void phy_reception_set_rx_sample_rate(srslte_rf_t *rf, uint32_t nof_prb, bool use_std_carrier_sep);

void phy_reception_set_initial_rx_freq_and_gain(srslte_rf_t *rf, float rx_gain, float initial_agc_gain, double default_center_freq, double competition_bw, double rx_bandwidth, uint32_t default_rx_channel);

int srslte_rf_recv_wrapper(void *h, void *data, uint32_t nsamples, srslte_timestamp_t *t, size_t channel);

int srslte_rf_recv_with_time_wrapper(void *h, void *data, uint32_t nsamples, srslte_timestamp_t *t, size_t channel);

int srslte_file_recv_wrapper(void *h, void *data, uint32_t nsamples, srslte_timestamp_t *t, size_t channel);

double srslte_rf_set_rx_gain_th_wrapper_(void *h, double f);

int phy_reception_start_decoding_thread();

int phy_reception_stop_decoding_thread();

int phy_reception_start_sync_thread();

int phy_reception_stop_sync_thread();

void *phy_reception_sync_work(void *h);

void phy_reception_push_ue_sync_to_queue(short_ue_sync_t *short_ue_sync);

void phy_reception_pop_ue_sync_from_queue(short_ue_sync_t *short_ue_sync);

bool phy_reception_wait_queue_not_empty();

void phy_reception_print_ue_sync(short_ue_sync_t *ue_sync, char* str);

void phy_reception_init_handle();

#endif // _PHY_RECEPTION_H_
