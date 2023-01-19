/**
 * SPDX-License-Identifier: MIT License
 *
 * Copyright (c) 2019 Jan Goldacker
 * Copyright (c) 2021-2022 Konrad Beckmann <konrad.beckmann@gmail.com>
 */

#pragma once

#define CIC_SEED_6102 (2)
#define CIC_SEED_6101 (1)
#define CIC_SEED_6103 (3)
#define CIC_SEED_6105 (5)
#define CIC_SEED_6106 (6)
#define CIC_SEED_7102 (7)

void n64_cic_set_seed(uint8_t seed);
void n64_cic_reset();
void n64_cic_run(uint8_t _pin_cr, uint8_t _pin_dclk, uint8_t _pin_dio);
