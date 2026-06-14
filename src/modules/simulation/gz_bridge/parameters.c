/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Simulator Gazebo bridge enable
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(SIM_GZ_EN, 0);

/**
 * Simulated GPS noise RNG seed
 *
 * Fixed seed for the gz_bridge GPS noise generator so the GPS error trace is
 * reproducible across runs (fair A/B comparison). Vary it for statistical (CEP) sweeps.
 *
 * @min 1
 * @group Simulation
 */
PARAM_DEFINE_INT32(SIM_GPS_SEED, 1);

/**
 * Simulated GPS noise scale
 *
 * Scales the gz_bridge GPS noise (white + Gauss-Markov bias). 1.0 = realistic
 * NEO-M9N (~2 m CEP). 0 = no random noise (truth + constant bias only).
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group Simulation
 */
PARAM_DEFINE_FLOAT(SIM_GPS_NSC, 1.0f);

/**
 * Simulated GPS constant bias North
 *
 * Deterministic North offset added to the simulated GPS (m, NED). Optional, for
 * controlled probes; default 0.
 *
 * @unit m
 * @decimal 2
 * @group Simulation
 */
PARAM_DEFINE_FLOAT(SIM_GPS_BIAS_N, 0.0f);

/**
 * Simulated GPS constant bias East
 *
 * Deterministic East offset added to the simulated GPS (m, NED). Optional; default 0.
 *
 * @unit m
 * @decimal 2
 * @group Simulation
 */
PARAM_DEFINE_FLOAT(SIM_GPS_BIAS_E, 0.0f);
