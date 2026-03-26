/*
 * SPDX-FileCopyrightText: Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMEpNT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* Source file for device part of packet processing sample.
 * Contain functions for initialize contexts of internal queues,
 * read, check, change and resend the packet and wait for another.
 */

/* Shared header file with utilities for samples.
 * The file also have includes to flexio_dev_ver.h and flexio_dev.h
 * The include must be placed first to correctly handle the version.
 */
#include "com_dev.h"
#include <libflexio-dev/flexio_dev_err.h>
#include <libflexio-dev/flexio_dev_queue_access.h>
#include <libflexio-dev/flexio_dev_debug.h>
#include <libflexio-libc/string.h>
#include <stddef.h>
#include <dpaintrin.h>
/* Shared header file for packet processor sample */
#include "../flexio_packet_processor_com.h"

/* Mask for CQ index */
#define CQ_IDX_MASK ((1 << LOG_CQ_DEPTH) - 1)
/* Mask for RQ index */
#define RQ_IDX_MASK ((1 << LOG_RQ_DEPTH) - 1)
/* Mask for SQ index */
#define SQ_IDX_MASK ((1 << (LOG_SQ_DEPTH + LOG_SQE_NUM_SEGS)) - 1)
/* Mask for data index */
#define DATA_IDX_MASK ((1 << (LOG_SQ_DEPTH)) - 1)

/* The structure of the sample DPA application contains global data that the application uses */
static struct dpa_thread_context {
	/* Packet count - used for debug message */
	uint64_t packets_count;
	/* lkey - local memory key */
	uint32_t sq_lkey;
	uint32_t rq_lkey;
	int buffer_location;
	uint32_t window_id;
	uint32_t idx;

	cq_ctx_t rq_cq_ctx;     /* RQ CQ */
	rq_ctx_t rq_ctx;        /* RQ */
	sq_ctx_t sq_ctx;        /* SQ */
	cq_ctx_t sq_cq_ctx;     /* SQ CQ */
	dt_ctx_t dt_ctx;        /* SQ Data ring */
} thd_ctx[190];

/* Initialize the app_ctx structure from the host data.
 *  data_from_host - pointer host2dev_packet_processor_data from host.
 */
static void thd_ctx_init(struct host2dev_packet_processor_data *data_from_host)
{
	int i = data_from_host->thd_id;
	thd_ctx[i].packets_count = 0;
	thd_ctx[i].sq_lkey = data_from_host->sq_transf.wqd_mkey_id;
	thd_ctx[i].rq_lkey = data_from_host->rq_transf.wqd_mkey_id;
	thd_ctx[i].buffer_location = data_from_host->buffer_location;
	thd_ctx[i].window_id = data_from_host->window_id;
	thd_ctx[i].idx = i;

	/* Set context for RQ's CQ */
	com_cq_ctx_init(&(thd_ctx[i].rq_cq_ctx),
			data_from_host->rq_cq_transf.cq_num,
			data_from_host->rq_cq_transf.log_cq_depth,
			data_from_host->rq_cq_transf.cq_ring_daddr,
			data_from_host->rq_cq_transf.cq_dbr_daddr);

	/* Set context for RQ */
	com_rq_ctx_init(&(thd_ctx[i].rq_ctx),
			data_from_host->rq_transf.wq_num,
			data_from_host->rq_transf.wq_ring_daddr,
			data_from_host->rq_transf.wq_dbr_daddr);

	/* Set context for SQ */
	com_sq_ctx_init(&(thd_ctx[i].sq_ctx),
			data_from_host->sq_transf.wq_num,
			data_from_host->sq_transf.wq_ring_daddr);

	/* Set context for SQ's CQ */
	com_cq_ctx_init(&(thd_ctx[i].sq_cq_ctx),
			data_from_host->sq_cq_transf.cq_num,
			data_from_host->sq_cq_transf.log_cq_depth,
			data_from_host->sq_cq_transf.cq_ring_daddr,
			data_from_host->sq_cq_transf.cq_dbr_daddr);

	/* Set context for data */
	com_dt_ctx_init(&(thd_ctx[i].dt_ctx), data_from_host->sq_transf.wqd_daddr);


	for (int a = 0; a < (1UL << LOG_SQ_DEPTH); a++) {

		union flexio_dev_sqe_seg *swqe;
        swqe = get_next_sqe(&(thd_ctx[i].sq_ctx), SQ_IDX_MASK);
		flexio_dev_swqe_seg_ctrl_set(swqe, a, thd_ctx[i].sq_ctx.sq_number,
				     MLX5_CTRL_SEG_CE_CQE_ON_CQE_ERROR, FLEXIO_CTRL_SEG_SEND_EN);

		swqe = get_next_sqe(&(thd_ctx[i].sq_ctx), SQ_IDX_MASK);
		flexio_dev_swqe_seg_eth_set(swqe, 0, 0, 0, NULL);

        swqe = get_next_sqe(&(thd_ctx[i].sq_ctx), SQ_IDX_MASK);
		flexio_dev_swqe_seg_mem_ptr_data_set(swqe, 0, thd_ctx->sq_lkey, 0);

        swqe = get_next_sqe(&(thd_ctx[i].sq_ctx), SQ_IDX_MASK);
	}
    thd_ctx[i].sq_ctx.sq_wqe_seg_idx = 0;
}


#define DPA_FREQ_HZ 1800000000ULL  // 1.8GHz


int pkt_size_to_delay_cycles(uint64_t pkt_size){
	// 1024 + 42
	if(pkt_size == 1066){
		return 1400*3;
	}else{
		return 0;
	}

}
void dpa_delay_ns(uint64_t nsec) {
    uint64_t start = __dpa_thread_cycles();
    uint64_t wait_cycles = (9ULL * nsec / 5ULL);

    while ((__dpa_thread_cycles() - start) < wait_cycles) {
        // busy wait
    }
}
void dpa_delay_cycles(uint64_t cycles) {
    uint64_t start = __dpa_thread_cycles();
	uint64_t end = __dpa_thread_cycles();
    // uint64_t wait_cycles = (DPA_FREQ_HZ / 1000000000ULL) * nsec;

    while ((end - start) < cycles) {
        // busy wait
		end = __dpa_thread_cycles();
    }
}

uint16_t calculate_checksum(uint16_t *data, int length) {
    uint32_t sum = 0;
    int i;

    // 逐块相加
    for (i = 0; i < length; i++) {
        sum += data[i];
        // 处理溢出
        if (sum & 0xFFFF0000) {
            sum &= 0xFFFF;
            sum++;
        }
    }

    // 取一的补码
    return ~sum;
}

// 只swap mac 需要 1400 cycle 左右+-10 
static void process_packet(struct flexio_dev_thread_ctx *dtctx, struct dpa_thread_context* thd_ctx, size_t* cycle_interval)
{
	// *cycle_interval = __dpa_thread_cycles();
	(void)cycle_interval;
	/* RX packet handling variables */
	struct flexio_dev_wqe_rcv_data_seg *rwqe;
	/* RQ WQE index */
	uint32_t rq_wqe_idx;
	/* Pointer to RQ data */
	char *rq_data, *sq_data;

	/* TX packet handling variables */
	union flexio_dev_sqe_seg *swqe;

	/* Size of the data */
	uint32_t data_sz;

	/* Extract relevant data from the CQE */
	rq_wqe_idx = be16_to_cpu((volatile __be16)thd_ctx->rq_cq_ctx.cqe->wqe_counter);
	data_sz = be32_to_cpu((volatile __be32)thd_ctx->rq_cq_ctx.cqe->byte_cnt);
	// flexio_dev_print("data_sz  %d\n", data_sz );

	// uint64_t need_cycles = pkt_size_to_delay_cycles(data_sz);
	// dpa_delay_cycles(need_cycles);
	// dpa_delay_ns(200000);

	/* Get the RQ WQE pointed to by the CQE */
	rwqe = &(thd_ctx->rq_ctx.rq_ring[rq_wqe_idx & RQ_IDX_MASK]);

	/* Extract data (whole packet) pointed to by the RQ WQE */
	rq_data = (void *)be64_to_cpu((volatile __be64)rwqe->addr);
	// sq_data = get_next_dte(&(thd_ctx->dt_ctx), DATA_IDX_MASK, LOG_WQD_CHUNK_BSIZE);

	/* swap mac address */
	uint64_t src_mac = *((uint64_t *)rq_data);
	uint64_t dst_mac = *((uint64_t *)(rq_data + 6));
	*((uint64_t *)rq_data) = dst_mac;
	*((uint64_t *)(rq_data + 6)) = (src_mac & 0x0000FFFFFFFFFFFF) | (dst_mac & 0xFFFF000000000000);

	swqe = &(thd_ctx->sq_ctx.sq_ring[(thd_ctx->sq_ctx.sq_wqe_seg_idx + 2) & SQ_IDX_MASK]);
	thd_ctx->sq_ctx.sq_wqe_seg_idx += 4;
	flexio_dev_swqe_seg_mem_ptr_data_set(swqe, data_sz, thd_ctx->rq_lkey, (uint64_t)rq_data);

	/* Ring DB */
	__dpa_thread_memory_writeback();
	flexio_dev_qp_sq_ring_db(dtctx, ++thd_ctx->sq_ctx.sq_pi, thd_ctx->sq_ctx.sq_number);
	flexio_dev_dbr_rq_inc_pi(thd_ctx->rq_ctx.rq_dbr);
	// *cycle_interval = __dpa_thread_cycles() - *cycle_interval;
}

inline static char* get_dpa_host_rq_data_addr(char* host_addr, struct dpa_thread_context* thd_ctx) {
	return (char*)((flexio_uintptr_t)host_addr - (thd_ctx->rq_ctx.rqd_host_addr) + (thd_ctx->rq_ctx.rqd_dpa_addr));
}

inline static char* get_dpa_host_sq_data_addr(char* host_addr, struct dpa_thread_context* thd_ctx) {
	return (char*)((flexio_uintptr_t)host_addr - (thd_ctx->sq_ctx.sqd_host_addr) + (thd_ctx->sq_ctx.sqd_dpa_addr));
}

static void process_packet_host(struct flexio_dev_thread_ctx *dtctx, struct dpa_thread_context* thd_ctx)
{
	/* RX packet handling variables */
	struct flexio_dev_wqe_rcv_data_seg *rwqe;
	/* RQ WQE index */
	uint32_t rq_wqe_idx;
	/* Pointer to RQ data */
	char *rq_data_host, *rq_data_dpa;

	/* TX packet handling variables */
	union flexio_dev_sqe_seg *swqe;

	/* Size of the data */
	uint32_t data_sz;

	/* Extract relevant data from the CQE */
	rq_wqe_idx = be16_to_cpu((volatile __be16)thd_ctx->rq_cq_ctx.cqe->wqe_counter);
	data_sz = be32_to_cpu((volatile __be32)thd_ctx->rq_cq_ctx.cqe->byte_cnt);

	/* Get the RQ WQE pointed to by the CQE */
	rwqe = &(thd_ctx->rq_ctx.rq_ring[rq_wqe_idx & RQ_IDX_MASK]);

	/* Extract data (whole packet) pointed to by the RQ WQE */
	rq_data_host = (void *)be64_to_cpu((volatile __be64)rwqe->addr);

	rq_data_dpa = (char*)((flexio_uintptr_t)rq_data_host - (thd_ctx->rq_ctx.rqd_host_addr) + (thd_ctx->rq_ctx.rqd_dpa_addr));
	
	/* swap mac address */
	uint64_t src_mac = *((uint64_t *)rq_data_dpa);
	uint64_t dst_mac = *((uint64_t *)(rq_data_dpa + 6));
	*((uint64_t *)rq_data_dpa) = dst_mac;
	*((uint64_t *)(rq_data_dpa + 6)) = (src_mac & 0x0000FFFFFFFFFFFF) | (dst_mac & 0xFFFF000000000000);

	swqe = &(thd_ctx->sq_ctx.sq_ring[(thd_ctx->sq_ctx.sq_wqe_seg_idx + 2) & SQ_IDX_MASK]);
	thd_ctx->sq_ctx.sq_wqe_seg_idx += 4;
	flexio_dev_swqe_seg_mem_ptr_data_set(swqe, data_sz, thd_ctx->sq_lkey, (uint64_t)rq_data_host);
	
	/* Ring DB */
	__dpa_thread_memory_writeback();
    __dpa_thread_window_writeback();
	flexio_dev_qp_sq_ring_db(dtctx, ++thd_ctx->sq_ctx.sq_pi, thd_ctx->sq_ctx.sq_number);
	flexio_dev_dbr_rq_inc_pi(thd_ctx->rq_ctx.rq_dbr);
}


#define test_size_in_bytes 102400000
#define test_packet_count 100000

flexio_dev_event_handler_t flexio_pp_dev;
__dpa_global__ void flexio_pp_dev(uint64_t thread_arg)
{
	struct host2dev_packet_processor_data *data_from_host = (void *)thread_arg;
	struct flexio_dev_thread_ctx *dtctx;
	int i = data_from_host->thd_id;
	register int buffer_location = data_from_host->buffer_location;
	register int use_copy = data_from_host->use_copy;
	flexio_uintptr_t result = 0;
	struct dpa_thread_context* this_thd_ctx = &(thd_ctx[i]);

	flexio_dev_get_thread_ctx(&dtctx);

	if (!data_from_host->not_first_run) {
		flexio_dev_print("start thread %d %d\n", i, buffer_location);
		thd_ctx_init(data_from_host);
		data_from_host->not_first_run = 1;
	}
	if (buffer_location == 0) {
		thd_ctx[i].rq_ctx.rqd_dpa_addr = data_from_host->rq_transf.wqd_daddr;
		thd_ctx[i].sq_ctx.sqd_dpa_addr = data_from_host->sq_transf.wqd_daddr;
		flexio_dev_status_t ret;
		ret = flexio_dev_window_config(dtctx, (uint16_t)thd_ctx[i].window_id, data_from_host->result_buffer_mkey_id);
		if (ret != FLEXIO_DEV_STATUS_SUCCESS) {
			flexio_dev_print("failed to config rq window, thread %d\n", i);
		}
		ret = flexio_dev_window_ptr_acquire(dtctx, (uint64_t)(data_from_host->result_buffer), &(result));
		if (ret != FLEXIO_DEV_STATUS_SUCCESS) {
			flexio_dev_print("failed to acquire result ptr, thread %d\n", i);
		}
		// uint64_t *data = NULL;
		// data = data_from_host->dpa_buffer_result;
	}
	else {
		thd_ctx[i].rq_ctx.rqd_host_addr = data_from_host->rq_transf.wqd_daddr;
		thd_ctx[i].sq_ctx.sqd_host_addr = data_from_host->sq_transf.wqd_daddr;
		flexio_dev_status_t ret;
		ret = flexio_dev_window_config(dtctx, (uint16_t)thd_ctx[i].window_id, thd_ctx[i].rq_lkey);
		if (ret != FLEXIO_DEV_STATUS_SUCCESS) {
			flexio_dev_print("failed to config rq window, thread %d\n", i);
		}
		ret = flexio_dev_window_ptr_acquire(dtctx, (uint64_t)data_from_host->rq_transf.wqd_daddr, &(thd_ctx[i].rq_ctx.rqd_dpa_addr));
		if (ret != FLEXIO_DEV_STATUS_SUCCESS) {
			flexio_dev_print("failed to acquire rq host ptr, thread %d\n", i);
		}
		// ret = flexio_dev_window_config((uint16_t)thd_ctx[i].window_id, thd_ctx[i].sq_lkey);
		// if (ret != FLEXIO_DEV_STATUS_SUCCESS) {
		// 	flexio_dev_print("failed to config sq window, thread %d\n", i);
		// }
		ret = flexio_dev_window_ptr_acquire(dtctx, (uint64_t)data_from_host->sq_transf.wqd_daddr, &(thd_ctx[i].sq_ctx.sqd_dpa_addr));
		if (ret != FLEXIO_DEV_STATUS_SUCCESS) {
			flexio_dev_print("failed to acquire sq host ptr, thread %d\n", i);
		}		
		// flexio_dev_print("before host addr: 0x%llx, dpa addr: 0x%llx\n", data_from_host->result_buffer, result);	
		ret = flexio_dev_window_ptr_acquire(dtctx, (uint64_t)(data_from_host->result_buffer), &(result));
		if (ret != FLEXIO_DEV_STATUS_SUCCESS) {
			flexio_dev_print("failed to acquire result ptr, thread %d\n", i);
			// while(1);
		}
		// flexio_dev_print("after host addr: 0x%llx, dpa addr: 0x%llx\n", data_from_host->result_buffer, result);	
	}

	register size_t pkt_count = 0;
	register size_t start;
	register size_t end;
	register size_t total_cycles = 0;
	size_t cycle_interval = 0;
#if 0
	register size_t cycle_fdpacket = 0;
	register size_t packet_num = 0;
#endif
	while (dtctx != NULL) {
		while (flexio_dev_cqe_get_owner(this_thd_ctx->rq_cq_ctx.cqe) != this_thd_ctx->rq_cq_ctx.cq_hw_owner_bit) {
			// __dpa_thread_fence(__DPA_MEMORY, __DPA_R, __DPA_R);

			// start = __dpa_thread_cycles();
			if (buffer_location) {
				process_packet_host(dtctx, this_thd_ctx);
			}
			else {
				process_packet(dtctx, this_thd_ctx, &cycle_interval);
			}
			com_step_cq(&(this_thd_ctx->rq_cq_ctx));
			// end = __dpa_thread_cycles();
#if 0
			cycle_fdpacket += cycle_interval;
			packet_num++;
#endif
			// total_cycles += end-start;


			// flexio_dev_print("total_cycles per packet %ld \n", total_cycles);

			pkt_count++;
			if (pkt_count > 1000000) {
				// flexio_dev_print("total_cycles per packet %ld \n", total_cycles/pkt_count);
				// flexio_dev_print("cycle_fdpacket: %ld, packet_num: %ld, avg_cycle_per_packet: %ld\n", cycle_fdpacket, packet_num, cycle_fdpacket/packet_num);
				pkt_count = 0;
				total_cycles = 0;
				flexio_dev_print("reschedule\n");
				__dpa_thread_fence(__DPA_MEMORY, __DPA_W, __DPA_W);
				flexio_dev_cq_arm(dtctx, this_thd_ctx->rq_cq_ctx.cq_idx, this_thd_ctx->rq_cq_ctx.cq_number);
				flexio_dev_thread_reschedule();
				return;
			}
		}
	}

	__dpa_thread_fence(__DPA_MEMORY, __DPA_W, __DPA_W);
	flexio_dev_cq_arm(dtctx, thd_ctx[i].rq_cq_ctx.cq_idx, thd_ctx[i].rq_cq_ctx.cq_number);
	flexio_dev_thread_reschedule();
}


			// pkt_count++;
			// if (pkt_count > 100000) {
			// 	pkt_count = 0;
			// 	// flexio_dev_print("reschedule\n");
			// 	__dpa_thread_fence(__DPA_MEMORY, __DPA_W, __DPA_W);
			// 	flexio_dev_cq_arm(dtctx, thd_ctx[i].rq_cq_ctx.cq_idx, thd_ctx[i].rq_cq_ctx.cq_number);
			// 	flexio_dev_thread_reschedule();
			// 	return;
			// }
			// __dpa_thread_fence(__DPA_MEMORY, __DPA_R, __DPA_R);
			// if (buffer_location == 0) {
				// process_packet(dtctx, this_thd_ctx, use_copy);
			// }
			// else {
			// 	process_packet_host(dtctx, &thd_ctx[i], use_copy);
			// }
			// com_step_cq(&(this_thd_ctx->rq_cq_ctx));
			// pkt_count++;
			// if (__builtin_expect((pkt_count == test_packet_count), 0)) {
			// 	// trigger_time++;
			// 	// if (i == 0) {
			// 	end = __dpa_thread_cycles();
			// 	uint64_t speed = 1024ULL * 8 * (pkt_count * 1024) / (( end - start) * 10 / 18);
			// 	// flexio_dev_print("result buffer: 0x%llx\n", result);
			// 	// flexio_dev_print("thread_id: %d, speed: %llu Mbps\n", i, speed);
			// 	// uint64_t test;
			// 	// memcpy(&test,(void*)result, sizeof(uint64_t));
			// 	// flexio_dev_print("before host addr: 0x%lx, result: %lu, speed: %lu\n", data_from_host->result_buffer, test, speed);
			// 	// flexio_dev_print("before host addr: 0x%llu, result: %lu, speed: %lu\n", data_from_host->result_buffer, *((uint64_t*)result), speed);
			// 	*((uint64_t*)result) = speed;
			// 	__dpa_thread_window_writeback();
			// 	// memcpy(&test,(void*)result, sizeof(uint64_t));
			// 	// flexio_dev_print("after host addr: 0x%lx, result: %lu, speed: %lu\n", data_from_host->result_buffer, *((uint64_t*)result), speed);
			// 	// flexio_dev_print("after host addr: 0x%llu, result: %lu, speed: %lu\n", data_from_host->result_buffer, *((uint64_t*)result), speed);
			// 	// flexio_dev_print("result speed: %llu Mbps\n", *((uint64_t*)result));
			// 	// }
			// 	// struct host_to_device_config *device_cfg = (struct host_to_device_config *)data_from_host->device_cfg_p;
			// 	// result = flexio_dev_window_config(dtctx, device_cfg->window_id, device_cfg->mkey);
			// 	// if (result != FLEXIO_DEV_STATUS_SUCCESS) {
			// 	// 	flexio_dev_print("flexio_dev_window_config failed\n");
			// 	// 	return;
			// 	// }
			// 	// result = flexio_dev_window_ptr_acquire(dtctx, device_cfg->haddr, &a);
			// 	// if (result != FLEXIO_DEV_STATUS_SUCCESS) {
			// 	// 	flexio_dev_print("flexio_dev_window_ptr_acquire failed\n");
			// 	// 	return;
			// 	// }
			// 	// *((uint64_t*)a) = 1024ULL * 8 * (data_from_host->recv_sz) / used_time;
			// 	start = __dpa_thread_cycles();
			// 	pkt_count = 0;

            //     // if (trigger_time > 3) {
            //     //     flexio_dev_print("thread %ld end due to trigger-time limit\n", i);
            //     //     return;
            //     // }
            // }
            // if (__dpa_thread_cycles() - time_total > 1800ULL * 1000 * 1000 * 30) {
            //     flexio_dev_print("thread %ld end due to time limit\n", i);
            //     return;
            // }