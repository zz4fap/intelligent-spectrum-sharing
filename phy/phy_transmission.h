#ifndef _PHY_TRANSMISSION_H_
#define _PHY_TRANSMISSION_H_

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <string.h>
//#include <gperftools/profiler.h>

#include <uhd.h>

#include "srslte/srslte.h"
#include "srslte/intf/intf.h"

#include "../../../../communicator/cpp/communicator_wrapper.h"
#include "helpers.h"
#include "transceiver.h"

// By default Tx filtering is enabled in the CMakefile and can be disabled with the following command: "cmake -DENABLE_PHY_TX_FILTERING=OFF ../"
#if(ENABLE_PHY_TX_FILTERING==1)
#include "trx_filter.h"
#endif

// ****************** Definition of macros ***********************
// As we can have overlapping of TX data we create a vector with a given number of buffers.
#define NUMBER_OF_TX_DATA_BUFFERS 500

#define ENABLE_PHY_TX_PRINTS 1

#define ENABLE_TX_IPC_DEBUG 0

#define PHY_TX_PRINT(_fmt, ...) do { if(ENABLE_PHY_TX_PRINTS && scatter_verbose_level >= 0) { \
  fprintf(stdout, "[PHY TX PRINT]: " _fmt, __VA_ARGS__); } } while(0)

#define PHY_TX_DEBUG(_fmt, ...) do { if(ENABLE_PHY_TX_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_DEBUG) { \
  fprintf(stdout, "[PHY TX DEBUG]: " _fmt, __VA_ARGS__); } } while(0)

#define PHY_TX_INFO(_fmt, ...) do { if(ENABLE_PHY_TX_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_INFO) { \
  fprintf(stdout, "[PHY TX INFO]: " _fmt, __VA_ARGS__); } } while(0)

#define PHY_TX_ERROR(_fmt, ...) do { fprintf(stdout, "[PHY TX ERROR]: " _fmt, __VA_ARGS__); } while(0)

#define PHY_TX_PRINT_TIME(_fmt, ...) do { if(ENABLE_PHY_TX_PRINTS && scatter_verbose_level >= 0) { \
  char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[PHY TX PRINT]: %s - " _fmt, date_time_str, __VA_ARGS__); } } while(0)

#define PHY_TX_DEBUG_TIME(_fmt, ...) do { if(ENABLE_PHY_TX_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_DEBUG) { \
  char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[PHY TX DEBUG]: %s - " _fmt, date_time_str, __VA_ARGS__); } } while(0)

#define PHY_TX_INFO_TIME(_fmt, ...) do { if(ENABLE_PHY_TX_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_INFO) { \
  char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[PHY TX INFO]: %s - " _fmt, date_time_str, __VA_ARGS__); } } while(0)

#define PHY_TX_ERROR_TIME(_fmt, ...) do { char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[PHY TX ERROR]: %s - " _fmt, date_time_str, __VA_ARGS__); } while(0)

#define MAX_NUM_OF_CHANNELS 58

// ****************** Definition of types ******************
typedef struct {
  LayerCommunicator_handle phy_comm_handle;
  srslte_rf_t *rf;
  double competition_bw;
  double competition_center_freq;
  uint16_t rnti;
  bool phy_filtering;
  bool use_std_carrier_sep;
  bool is_lbt_enabled;
  bool send_tx_stats_to_mac;
  bool add_tx_timestamp;
  int initial_subframe_index; // Set the subframe index number to be used to start from.
  float rf_amp;
  uint32_t trx_filter_idx;

  pthread_attr_t phy_transmission_thread_attr;
  pthread_t phy_transmission_thread_id;

  // This variable is used to stop phy reception thread.
  volatile sig_atomic_t run_phy_transmission_thread;

  // This basic controls stores the last configured values.
  basic_ctrl_t last_tx_basic_control;

  int number_of_tx_offset_samples;
  int sf_n_re;
  int sf_n_samples;

  cf_t *sf_buffer_eb;
  cf_t *output_buffer;
  cf_t *subframe_ofdm_symbols;

  srslte_cell_t cell_enb;

  srslte_ofdm_t ifft;
  srslte_pbch_t pbch;
  srslte_pcfich_t pcfich;
  srslte_pdcch_t pdcch;
  srslte_pdsch_t pdsch;
  srslte_pdsch_cfg_t pdsch_cfg;
  srslte_softbuffer_tx_t softbuffer;
  srslte_regs_t regs;
  srslte_ra_dl_dci_t ra_dl;
  srslte_ra_dl_grant_t grant;

  cf_t pss_signal[SRSLTE_PSS_LEN];
  float sss_signal0[SRSLTE_SSS_LEN]; // for subframe 0
  float sss_signal5[SRSLTE_SSS_LEN]; // for subframe 5
  uint8_t bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
#if(ENABLE_MIB_ENCODING==1)
  cf_t *slot1_symbols[SRSLTE_MAX_PORTS];
#endif
  cf_t *sf_symbols[SRSLTE_MAX_PORTS];
  srslte_chest_dl_t est;
  srslte_dci_location_t locations[SRSLTE_NSUBFRAMES_X_FRAME][30];

  // Mutex used to synchronize between main and transmission threads.
  pthread_mutex_t phy_transmission_mutex;
  // Condition variable used to synchronize between main and transmission threads.
  pthread_cond_t phy_transmission_cv;
  // Counter used to index the insetion in the container.
  uint64_t phy_transmission_wr_cnt;
  // Counter used to index the reading/removal from the container.
  uint64_t phy_transmission_rd_cnt;

  // Timeout counter for all the channel we might have during the compatition.
  uint64_t channel_timeout_counter[MAX_NUM_OF_CHANNELS]; // At most there will be 57 channels, 80 MHz / 1.4 MHz.
  // Holds the center frequency used in the last basic control message received from upper layers.
  double tx_channel_center_frequency;

  // This is a point to the TX data.
  unsigned char *tx_data_buffer[NUMBER_OF_TX_DATA_BUFFERS];

} phy_transmission_t;

// *************** Declaration of functions ***************
void phy_transmission_init_last_basic_control(uint32_t center_freq, int32_t tx_gain, uint32_t bw_idx, uint32_t radio_id, uint32_t default_channel, float rf_boost);

void phy_transmission_init_cell_parameters(uint32_t num_of_prb, uint32_t radio_id, uint32_t nof_ports);

void phy_transmission_set_handler(LayerCommunicator_handle handle, srslte_rf_t *rf, transceiver_args_t *args);

int phy_transmission_initialize(LayerCommunicator_handle handle, srslte_rf_t *rf, transceiver_args_t *args, unsigned char ***ciruclar_data_buffer_ptr);

int phy_transmission_uninitialize();

uint32_t phy_transmission_change_parameters(srslte_rf_t *rf, basic_ctrl_t *bc);

void *phy_transmission_work(void *h);

void phy_transmission_send_rx_statistics(LayerCommunicator_handle handle, phy_stat_t *phy_tx_stat);

void set_number_of_tx_offset_samples(int num_samples_to_offset);

int get_number_of_tx_offset_samples();

unsigned int phy_transmission_reverse(register unsigned int x);

uint32_t phy_transmission_prbset_to_bitmask(uint32_t nof_prb);

int phy_transmission_update_radl(uint32_t mcs, uint32_t nof_prb);

void phy_transmission_init_buffers(srslte_rf_t *rf, uint32_t nof_prb, uint32_t trx_filter_idx);

void phy_transmission_free_buffers();

void phy_transmission_base_init(uint16_t rnti, uint32_t bw_idx, uint32_t trx_filter_idx);

void phy_transmission_base_free();

void phy_transmission_set_tx_sample_rate(srslte_rf_t *rf, uint32_t nof_prb, bool use_std_carrier_sep);

void phy_transmission_set_initial_tx_freq_and_gain(srslte_rf_t *rf, float default_tx_gain, double default_center_freq, double competition_bw, double tx_bandwidth, uint32_t default_tx_channel);

void phy_transmission_send_tx_statistics(LayerCommunicator_handle handle, phy_stat_t *phy_tx_stat, int ret);

void phy_transmission_push_tx_basic_control_to_container(basic_ctrl_t *basic_ctrl);

void phy_transmission_pop_tx_basic_control_from_container(basic_ctrl_t *basic_ctrl);

bool phy_transmission_wait_container_not_empty();

void phy_transmission_set_channel_center_freq(double freq);

double phy_transmission_get_channel_center_freq();

void phy_transmission_allocate_tx_buffer(size_t tx_data_buffer_length);

void phy_transmission_free_tx_buffer();

void phy_transmission_init_tx_data_buffer();

void phy_transmission_init_handle();

#endif // _PHY_TRANSMISSION_H_
