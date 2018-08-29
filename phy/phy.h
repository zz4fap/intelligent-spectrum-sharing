#ifndef _PHY_H_
#define _PHY_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <assert.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>
#include <float.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <errno.h>
#include <time.h>
#include <stddef.h>
#include <stdint.h>

#include "srslte/srslte.h"
#include "srslte/intf/intf.h"

#include "../../../../communicator/cpp/communicator_wrapper.h"
#include "helpers.h"
#include "transceiver.h"
#include "phy_reception.h"
#include "phy_transmission.h"

#ifndef DISABLE_RF
#include "srslte/rf/rf.h"
#include "srslte/rf/rf_utils.h"
#else
#error Compiling PHY with no RF support. Add RF support.
#endif

//#define CORRECT_SAMPLE_OFFSET

#define ENABLE_SENSING_THREAD 0

// Set to 1 the macro below to enable the adjustment of FPGA time from time to time.
#define ADJUST_FPGA_TIME 1

#define PHY_DEBUG_1 1
#define PHY_DEBUG_2 2

#define ENABLE_PHY_PRINTS 1

#define PHY_PRINT(_fmt, ...) do { if(ENABLE_PHY_PRINTS && scatter_verbose_level >= 0) { \
  fprintf(stdout, "[TRX PRINT]: " _fmt, __VA_ARGS__); } } while(0)

#define PHY_DEBUG(_fmt, ...) do { if(ENABLE_PHY_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_DEBUG) { \
  fprintf(stdout, "[TRX DEBUG]: " _fmt, __VA_ARGS__); } } while(0)

#define PHY_INFO(_fmt, ...) do { if(ENABLE_PHY_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_INFO) { \
  fprintf(stdout, "[TRX INFO]: " _fmt, __VA_ARGS__); } } while(0)

#define PHY_ERROR(_fmt, ...) do { fprintf(stdout, "[TRX ERROR]: " _fmt, __VA_ARGS__); } while(0)

#define PHY_PRINT_TIME(_fmt, ...) do { if(ENABLE_PHY_PRINTS && scatter_verbose_level >= 0) { \
  char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[TRX PRINT]: %s - " _fmt, date_time_str, __VA_ARGS__); } } while(0)

#define PHY_DEBUG_TIME(_fmt, ...) do { if(ENABLE_PHY_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_DEBUG) { \
  char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[TRX DEBUG]: %s - " _fmt, date_time_str, __VA_ARGS__); } } while(0)

#define PHY_INFO_TIME(_fmt, ...) do { if(ENABLE_PHY_PRINTS && scatter_verbose_level >= SRSLTE_VERBOSE_INFO) { \
  char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[TRX INFO]: %s - " _fmt, date_time_str, __VA_ARGS__); } } while(0)

#define PHY_ERROR_TIME(_fmt, ...) do { char date_time_str[30]; helpers_get_data_time_string(date_time_str); \
  fprintf(stdout, "[TRX ERROR]: %s - " _fmt, date_time_str, __VA_ARGS__); } while(0)

// ********************* Declaration of types *********************
typedef struct {
  srslte_rf_t rf;
  transceiver_args_t prog_args;
  bool go_exit;
  // This counter is used to change the current buffer so that we don't have overlapping even when TX is fast enough.
  uint32_t tx_data_buffer_cnt;
} phy_handle_t;

// ********************* Declaration of functions *********************
void phy_fsm(basic_ctrl_t *bc, LayerCommunicator_handle handle);

void open_rf_device();

void close_rf_device();

void set_master_clock_rate();

// Set FPGA time now to host time.
inline void phy_set_fpga_time(srslte_rf_t *rf) {
  struct timespec host_time_now;
  // Retrieve current time from host PC.
  clock_gettime(CLOCK_REALTIME, &host_time_now);
  srslte_rf_set_time_now(rf, host_time_now.tv_sec, (double)host_time_now.tv_nsec/1000000000LL);
  PHY_DEBUG("FPGA Time set to: %f\n",((double)(uintmax_t)host_time_now.tv_sec + (double)host_time_now.tv_nsec/1000000000LL));
}

inline static void phy_adjust_fpga_time(srslte_rf_t *rf) {
  static uint64_t time_adjust_cnt = 0;
  // Adjust FPGA time if it is different from host pc.
  time_adjust_cnt++;
  if(time_adjust_cnt==200) {
    uint64_t fpga_time = helpers_get_fpga_timestamp_us(rf);
    uint64_t host_time = helpers_get_host_timestamp();
    if((host_time-fpga_time) > 500) {
      phy_set_fpga_time(rf);
      PHY_DEBUG("FPGA: %" PRIu64 " - Host: %" PRIu64 " - diff: %d\n",fpga_time,host_time,(host_time-fpga_time));
    }
    time_adjust_cnt = 0;
  }
}

#endif // _PHY_H_
