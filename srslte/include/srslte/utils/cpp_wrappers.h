#ifndef _CPP_WRAPPERS_H_
#define _CPP_WRAPPERS_H_

#ifdef __cplusplus

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <stdexcept>
#include <queue>
#include <map>

extern "C" {
#endif

#include <inttypes.h>

#include "srslte/ue/ue_sync.h"
#include "../intf/intf.h"

SRSLTE_API void push_ue_sync_to_queue(short_ue_sync_t *short_ue_sync);

SRSLTE_API void pop_ue_sync_from_queue(short_ue_sync_t *short_ue_sync);

SRSLTE_API bool is_ue_queue_empty();

SRSLTE_API int ue_queue_size();

SRSLTE_API void push_tx_basic_control_to_container(uint64_t timestamp, basic_ctrl_t *basic_ctrl);

SRSLTE_API void pop_tx_basic_control_from_container(uint64_t timestamp, basic_ctrl_t *basic_ctrl);

SRSLTE_API bool is_tx_basic_control_container_empty();

SRSLTE_API int tx_basic_control_container_size();

#ifdef __cplusplus
}
#endif

#endif /* _CPP_WRAPPERS_H_ */
