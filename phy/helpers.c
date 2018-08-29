#include "helpers.h"

uint32_t helpers_get_prb_from_bw_index(uint32_t bw_idx) {
  uint32_t nof_prb;
  switch(bw_idx) {
    case BW_IDX_OneDotFour:
      nof_prb = 6;
      break;
    case BW_IDX_Three:
      nof_prb = 15;
      break;
    case BW_IDX_Five:
      nof_prb = 25;
      break;
    case BW_IDX_Ten:
      nof_prb = 50;
      break;
    case BW_IDX_Fifteen:
      nof_prb = 75;
      break;
    case BW_IDX_Twenty:
      nof_prb = 100;
      break;
    default:
      nof_prb = 25;
  }
  return nof_prb;
}

void helpers_get_data_time_string(char* date_time_str) {
  struct timeval tmnow;
  struct tm *tm;
  char usec_buf[20];
  gettimeofday(&tmnow, NULL);
  tm = localtime(&tmnow.tv_sec);
  strftime(date_time_str,30,"[%d/%m/%Y %H:%M:%S", tm);
  strcat(date_time_str,".");
  sprintf(usec_buf,"%06ld]",tmnow.tv_usec);
  strcat(date_time_str,usec_buf);
}


#if(USE_NOMINAL_BW==1)
float helpers_get_bandwidth(uint32_t index, uint32_t *bw_idx) {

  float bw;

  switch(index) {
    case BW_IDX_OneDotFour:
      bw = 1400000.0;
      break;
    case BW_IDX_Three:
      bw = 3000000.0;
      break;
    case BW_IDX_Five:
      bw = 5000000.0;
      break;
    case BW_IDX_Ten:
      bw = 10000000.0;
      break;
    case BW_IDX_Fifteen:
      bw = 15000000.0;
      break;
    case BW_IDX_Twenty:
      bw = 20000000.0;
      break;
    case BW_IDX_UNKNOWN:
    default:
      bw = -1.0;
  }

  *bw_idx = helpers_get_bw_index(index);

  return bw;
}
#else
// This is the function used when we don't want a guard band of 250 KHz between channels.
// The channels will be closer to each other. They will start just after the useful channel BW, e.g.,
// for 5 MHz PHY BW the useful BW is 25*12*15000 = 4.5 MHz.
float helpers_get_bandwidth(uint32_t index, uint32_t *bw_idx) {

  float bw;

  switch(index) {
    case BW_IDX_OneDotFour:
      bw = 1080000.0;
      break;
    case BW_IDX_Three:
      bw = 2700000.0;
      break;
    case BW_IDX_Five:
      bw = 4500000.0;
      break;
    case BW_IDX_Ten:
      bw = 9000000.0;
      break;
    case BW_IDX_Fifteen:
      bw = 13500000.0;
      break;
    case BW_IDX_Twenty:
      bw = 18000000.0;
      break;
    case BW_IDX_UNKNOWN:
    default:
      bw = -1.0;
  }

  *bw_idx = helpers_get_bw_index(index);

  return bw;
}
#endif

#if(USE_NOMINAL_BW==1)
float helpers_get_bandwidth_float(uint32_t index) {

  float bw;

  switch(index) {
    case BW_IDX_OneDotFour:
      bw = 1400000.0;
      break;
    case BW_IDX_Three:
      bw = 3000000.0;
      break;
    case BW_IDX_Five:
      bw = 5000000.0;
      break;
    case BW_IDX_Ten:
      bw = 10000000.0;
      break;
    case BW_IDX_Fifteen:
      bw = 15000000.0;
      break;
    case BW_IDX_Twenty:
      bw = 20000000.0;
      break;
    case BW_IDX_UNKNOWN:
    default:
      bw = -1.0;
  }

  return bw;
}
#else
float helpers_get_bandwidth_float(uint32_t index) {

  float bw;

  switch(index) {
    case BW_IDX_OneDotFour:
      bw = 1080000.0;
      break;
    case BW_IDX_Three:
      bw = 2700000.0;
      break;
    case BW_IDX_Five:
      bw = 4500000.0;
      break;
    case BW_IDX_Ten:
      bw = 9000000.0;
      break;
    case BW_IDX_Fifteen:
      bw = 13500000.0;
      break;
    case BW_IDX_Twenty:
      bw = 18000000.0;
      break;
    case BW_IDX_UNKNOWN:
    default:
      bw = -1.0;
  }

  return bw;
}
#endif

uint32_t helpers_get_bw_index_from_prb(uint32_t nof_prb) {
  uint32_t bw_idx;
  switch(nof_prb) {
    case 6:
      bw_idx = BW_IDX_OneDotFour;
      break;
    case 15:
      bw_idx = BW_IDX_Three;
      break;
    case 25:
      bw_idx = BW_IDX_Five;
      break;
    case 50:
      bw_idx = BW_IDX_Ten;
      break;
    case 75:
      bw_idx = BW_IDX_Fifteen;
      break;
    case 100:
      bw_idx = BW_IDX_Twenty;
      break;
    default:
      bw_idx = 0;
  }
  return bw_idx;
}

void helpers_print_basic_control(basic_ctrl_t* bc) {
  printf("*************************** Basic Control ***************************\n"\
         "TRX mode: %s\n"\
         "Seq. number: %d\n"\
         "BW: %d\n"\
         "Channel: %d\n"\
         "Frame: %d\n"\
         "Slot: %d\n"\
         "MCS: %d\n"\
         "Gain: %d\n",TRX_MODE(bc->trx_flag),bc->seq_number,BW_STRING(bc->bw_idx),bc->ch,bc->frame,bc->slot,bc->mcs,bc->gain);
  if(bc->trx_flag == PHY_RX_ST) { // RX
    printf("Num of slots: %d\n",bc->length);
  } else if(bc->trx_flag == PHY_TX_ST) { // TX
    printf("Length: %d\n",bc->length);
    printf("Data: ");
    for(int i = 0; i < bc->length; i++) {
      if(i < (bc->length-1)) {
        printf("%d, ", bc->data[i]);
      } else {
        printf("%d\n", bc->data[i]);
      }
    }
  }
  printf("*********************************************************************\n");
}

void helpers_print_rx_statistics(phy_stat_t *phy_stat) {
  // Print PHY RX Stats Structure.
  printf("******************* PHY RX Statistics *******************\n"\
    "Seq. number: %d\n"\
    "Status: %d\n"\
    "Host Timestamp: %" PRIu64 "\n"\
    "FPGA Timestamp: %d\n"\
    "Channel: %d\n"\
    "MCS: %d\n"\
    "Num. Packets: %d\n"\
    "Num. Errors: %d\n"\
    "Gain: %d\n"\
    "CQI: %d\n"\
    "RSSI: %1.2f\n"\
    "RSRP: %1.2f\n"\
    "RSRQ: %1.2f\n"\
    "SINR: %1.2f\n"\
    "Length: %d\n"\
    "*********************************************************************\n"\
    ,phy_stat->seq_number,phy_stat->status,phy_stat->host_timestamp,phy_stat->fpga_timestamp,phy_stat->ch,phy_stat->mcs,phy_stat->num_cb_total,phy_stat->num_cb_err,phy_stat->stat.rx_stat.gain,phy_stat->stat.rx_stat.cqi,phy_stat->stat.rx_stat.rssi,phy_stat->stat.rx_stat.rsrp,phy_stat->stat.rx_stat.rsrq,phy_stat->stat.rx_stat.sinr,phy_stat->stat.rx_stat.length);
}

void helpers_print_tx_statistics(phy_stat_t *phy_stat) {
  // Print PHY TX Stats Structure.
  printf("******************* PHY TX Statistics *******************\n"\
    "Seq. number: %d\n"\
    "Status: %d\n"\
    "Host Timestamp: %" PRIu64 "\n"\
    "FPGA Timestamp: %d\n"\
    "Channel: %d\n"\
    "MCS: %d\n"\
    "Num. Packets: %d\n"\
    "Num. Errors: %d\n"\
    "Power: %d\n"\
    "*********************************************************************\n"\
    ,phy_stat->seq_number,phy_stat->status,phy_stat->host_timestamp,phy_stat->fpga_timestamp,phy_stat->ch,phy_stat->mcs,phy_stat->num_cb_total,phy_stat->num_cb_err,phy_stat->stat.tx_stat.power);
}

void helpers_print_subframe(cf_t *subframe, int num_of_samples, bool start_of_burst, bool end_of_burst) {
  if(start_of_burst) {
    printf("************** SOB **************\n");
  }
  for(int i = 0; i < num_of_samples; i++) {
    printf("sample: %d - (%1.3f,%1.3f)\n",i,crealf(subframe[i]),cimagf(subframe[i]));
  }
  if(end_of_burst) {
    printf("************** EOB **************\n");
  } else {
    printf("\n");
  }
}
