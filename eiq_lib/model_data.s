/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
	.section .rodata
	.global model_data
	.global model_data_end
		
model_data:
	.incbin "./eiq_lib/model_quant_full_int_io.tflite"
model_data_end: