#ifndef _TRANSCEIVER_H_
#define _TRANSCEIVER_H_

// Structure used to store parsed arguments.
typedef struct {
    bool disable_cfo;
    uint16_t rnti;
    uint32_t nof_prb;
    uint32_t nof_ports;
    char *rf_args;
    double competition_center_frequency;
    float initial_rx_gain;
    float initial_tx_gain;
    int node_operation;
    unsigned long single_log_duration;
    unsigned long logging_frequency;
    unsigned long max_number_of_dumps;
    char *path_to_start_file;
    char *path_to_log_files;
    unsigned int node_id;
    srslte_datatype_t data_type;
    float sensing_rx_gain;
    uint32_t radio_id;
    bool use_std_carrier_sep;
    float initial_agc_gain;
    double competition_bw;
    uint32_t default_tx_channel;
    uint32_t default_rx_channel;
    float lbt_threshold;
    uint64_t lbt_timeout;
    uint32_t max_turbo_decoder_noi;
    bool phy_filtering;
    srslte_datatype_t iq_dump_data_type;
    float rf_monitor_rx_sample_rate;
    size_t rf_monitor_channel;
    bool lbt_use_fft_based_pwr;
    bool send_tx_stats_to_mac;
    uint32_t max_backoff_period;
    bool iq_dumping;
    bool immediate_transmission;
    bool add_tx_timestamp;
    uint32_t rf_monitor_option;
    int initial_subframe_index;
    bool plot_rx_info;
    float rf_amp;
    uint32_t trx_filter_idx;
} transceiver_args_t;

#define DEFAULT_CFI 1 // We use only one OFDM symbol for control.

#define DEVNAME_B200 "uhd_b200"

#define DEVNAME_X300 "uhd_x300"

#define MAXIMUM_NUMBER_OF_RADIOS 503

// Define used to enable timestamps to be added to TX data and measure time it takes to receive packet on RX side.
#define ENABLE_TX_TO_RX_TIME_DIFF 0

#endif // _TRANSCEIVER_H_
