/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "mpu6500/MPU9250_I2C.hpp"


void MPU9250_I2C::PX4_LOG(const char *fmt, ...){
//    char buffer[256];
//    va_list args;
//    va_start(args, fmt);
//    vsnprintf(buffer, sizeof(buffer), fmt, args);
//    va_end(args);
    //HAL_UART_Transmit(_huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

#define PX4_DEBUG(fmt, ...) PX4_LOG(fmt, ##__VA_ARGS__)
#define PX4_INFO(fmt, ...) PX4_LOG(fmt, ##__VA_ARGS__)
#define PX4_ERR(fmt, ...) PX4_LOG(fmt, ##__VA_ARGS__)
#define DEVICE_DEBUG PX4_DEBUG

#define PX4_OK 0
#define PX4_ERROR 1

#define ADDRESS 0xd0

hrt_abstime MPU9250_I2C::hrt_absolute_time(){
    return __HAL_TIM_GetCounter(_tim);
}


hrt_abstime MPU9250_I2C::hrt_elapsed_time(const hrt_abstime *then)
{
    return hrt_absolute_time() - *then;
}

void MPU9250_I2C::us_delay(uint32_t us){
    uint32_t start = hrt_absolute_time();
    while (hrt_absolute_time() - start > us);

}

HAL_StatusTypeDef MPU9250_I2C::transfer(const uint8_t *send, size_t sendlen, uint8_t *recv, size_t recvlen){
//    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_hi2c, ADDRESS, (uint8_t *)send, sendlen, HAL_MAX_DELAY);
//    if (ret != HAL_OK) {
//        return ret;
//    }
//    ret = HAL_I2C_Master_Receive(_hi2c, ADDRESS, recv, recvlen, HAL_MAX_DELAY);
//    return ret;

    if (recvlen > 0){
        HAL_I2C_Mem_Read(_hi2c, ADDRESS, *(uint8_t *)send, sendlen,  recv, recvlen, 10000);
    }else{
        HAL_I2C_Master_Transmit(_hi2c, ADDRESS, (uint8_t *)send, sendlen, 10000);
    }
    return  HAL_OK;
}


static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

MPU9250_I2C::MPU9250_I2C()
{
	ConfigureSampleRate(4000);
}

MPU9250_I2C::~MPU9250_I2C()
{
}

int MPU9250_I2C::init(I2C_HandleTypeDef *hi2c  ,TIM_HandleTypeDef *tim, sensor_accel_fifo_s *accel, sensor_gyro_fifo_s *gyro)
{
    _hi2c = hi2c;
    _tim = tim;
//    _gpiox = gpio;
//    _gpio_pin = gpio_pin;
    _accel = accel;
    _gyro = gyro;
	return Reset() ? 0 : -1;
}

bool MPU9250_I2C::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	return true;
}

void MPU9250_I2C::exit_and_cleanup()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
}

uint8_t MPU9250_I2C::print_status()
{

    return static_cast<uint8_t>(_state);
}

int MPU9250_I2C::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHOAMI) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void MPU9250_I2C::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// PWR_MGMT_1: Device Reset
		RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::H_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
        us_delay(100_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		// The reset value is 0x00 for all registers other than the registers below
		//  Document Number: RM-MPU-9250A-00 Page 9 of 55
		if ((RegisterRead(Register::WHO_AM_I) == WHOAMI)
		    && (RegisterRead(Register::PWR_MGMT_1) == 0x01)) {

			// Wakeup and reset digital signal path
			RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::CLKSEL_0);
			RegisterWrite(Register::SIGNAL_PATH_RESET,
				      SIGNAL_PATH_RESET_BIT::GYRO_RESET | SIGNAL_PATH_RESET_BIT::ACCEL_RESET | SIGNAL_PATH_RESET_BIT::TEMP_RESET);
			RegisterWrite(Register::USER_CTRL, USER_CTRL_BIT::SIG_COND_RST);

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
            us_delay(100_ms);

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
                us_delay(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
                us_delay(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
                us_delay(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				//ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

            us_delay(100_ms);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample;

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
                    us_delay(100_ms);
				}

				// push backup schedule back
				//ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			// always check current FIFO count
			bool success = false;
			const uint16_t fifo_count = FIFOReadCount();

			if (fifo_count >= FIFO::SIZE) {
				FIFOReset();

			} else if (fifo_count == 0) {

			} else {
				// FIFO count (size in bytes) should be a multiple of the FIFO::DATA structure
				uint8_t samples = fifo_count / sizeof(FIFO::DATA);

				// tolerate minor jitter, leave sample to next iteration if behind by only 1
				if (samples == _fifo_gyro_samples + 1) {
					timestamp_sample -= static_cast<int>(FIFO_SAMPLE_DT);
					samples--;
				}

				if (samples > FIFO_MAX_SAMPLES) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();

				} else if (samples >= SAMPLES_PER_TRANSFER) {
					if (FIFORead(timestamp_sample, samples)) {
						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					Reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void MPU9250_I2C::ConfigureAccel()
{
	const uint8_t ACCEL_FS_SEL = RegisterRead(Register::ACCEL_CONFIG) & (Bit4 | Bit3); // [4:3] ACCEL_FS_SEL[1:0]

	switch (ACCEL_FS_SEL) {
	case ACCEL_FS_SEL_2G:
//		_px4_accel.set_scale(CONSTANTS_ONE_G / 16384.f);
//		_px4_accel.set_range(2.f * CONSTANTS_ONE_G);
        accel_scale = CONSTANTS_ONE_G / 16384.f;
        accel_range = 2.f * CONSTANTS_ONE_G;
		break;

	case ACCEL_FS_SEL_4G:
//		_px4_accel.set_scale(CONSTANTS_ONE_G / 8192.f);
//		_px4_accel.set_range(4.f * CONSTANTS_ONE_G);
        accel_scale = CONSTANTS_ONE_G / 8192.f;
        accel_range = 4.f * CONSTANTS_ONE_G;
		break;

	case ACCEL_FS_SEL_8G:
//		_px4_accel.set_scale(CONSTANTS_ONE_G / 4096.f);
//		_px4_accel.set_range(8.f * CONSTANTS_ONE_G);
        accel_scale = CONSTANTS_ONE_G / 4096.f;
        accel_range = 8.f * CONSTANTS_ONE_G;
		break;

	case ACCEL_FS_SEL_16G:
//		_px4_accel.set_scale(CONSTANTS_ONE_G / 2048.f);
//		_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
        accel_scale = CONSTANTS_ONE_G / 2048.f;
        accel_range = 16.f * CONSTANTS_ONE_G;
		break;
	}
}

void MPU9250_I2C::ConfigureGyro()
{
	const uint8_t GYRO_FS_SEL = RegisterRead(Register::GYRO_CONFIG) & (Bit4 | Bit3); // [4:3] GYRO_FS_SEL[1:0]

	float range_dps = 0.f;

	switch (GYRO_FS_SEL) {
	case GYRO_FS_SEL_250_DPS:
		range_dps = 250.f;
		break;

	case GYRO_FS_SEL_500_DPS:
		range_dps = 500.f;
		break;

	case GYRO_FS_SEL_1000_DPS:
		range_dps = 1000.f;
		break;

	case GYRO_FS_SEL_2000_DPS:
		range_dps = 2000.f;
		break;
	}

//	_px4_gyro.set_scale(math::radians(range_dps / 32768.f));
//	_px4_gyro.set_range(math::radians(range_dps));
    gyro_scale = math::radians((range_dps / 32768.f));
    gyro_range = math::radians((range_dps));
}

void MPU9250_I2C::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = FIFO_SAMPLE_DT * SAMPLES_PER_TRANSFER;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);
}

bool MPU9250_I2C::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	ConfigureAccel();
	ConfigureGyro();

	return success;
}

int MPU9250_I2C::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<MPU9250_I2C *>(arg)->DataReady();
	return 0;
}

void MPU9250_I2C::DataReady()
{
	// at least the required number of samples in the FIFO
	if (++_drdy_count >= _fifo_gyro_samples) {
		_drdy_timestamp_sample = hrt_absolute_time();
		_drdy_count -= _fifo_gyro_samples;
	}
}

bool MPU9250_I2C::DataReadyInterruptConfigure()
{
	// TODO
	return false;
}

bool MPU9250_I2C::DataReadyInterruptDisable()
{
    return false;
}

bool MPU9250_I2C::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t MPU9250_I2C::RegisterRead(Register reg)
{
	uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t value = 0;
	//set_frequency(SPI_SPEED); // low speed for regular registers
	transfer(&cmd, 1, &value, 1);
	return value;
}

void MPU9250_I2C::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2];
	cmd[0] = static_cast<uint8_t>(reg);
	cmd[1] = value;
	//set_frequency(SPI_SPEED); // low speed for regular registers
	transfer(cmd, sizeof(cmd), nullptr, 0);
}

void MPU9250_I2C::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t MPU9250_I2C::FIFOReadCount()
{
	// read FIFO count
	uint8_t cmd = static_cast<uint8_t>(Register::FIFO_COUNTH);
	uint8_t fifo_count_buf[2] {};
	//set_frequency(SPI_SPEED_SENSOR);

	if (transfer(&cmd, 1, fifo_count_buf, 2) != PX4_OK) {
		return 0;
	}

	return combine(fifo_count_buf[0], fifo_count_buf[1]);
}

bool MPU9250_I2C::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	uint8_t cmd = static_cast<uint8_t>(Register::FIFO_R_W);
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA), FIFO::SIZE);
	//set_frequency(SPI_SPEED_SENSOR);

	if (transfer(&cmd, 1, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		return false;
	}


	ProcessGyro(timestamp_sample, buffer.f, samples);
	return ProcessAccel(timestamp_sample, buffer.f, samples);
}

void MPU9250_I2C::FIFOReset()
{

	// FIFO_EN: disable FIFO
	RegisterWrite(Register::FIFO_EN, 0);

	// USER_CTRL: reset FIFO
	RegisterSetAndClearBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_RST, USER_CTRL_BIT::FIFO_EN);

	// reset while FIFO is disabled
	_drdy_count = 0;
	_drdy_timestamp_sample = (0);

	// FIFO_EN: enable both gyro and accel
	// USER_CTRL: re-enable FIFO
	for (const auto &r : _register_cfg) {
		if ((r.reg == Register::FIFO_EN) || (r.reg == Register::USER_CTRL)) {
			RegisterSetAndClearBits(r.reg, r.set_bits, r.clear_bits);
		}
	}
}

bool MPU9250_I2C::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = FIFO_SAMPLE_DT * SAMPLES_PER_TRANSFER;
    accel.scale = accel_scale;

	bool bad_data = false;

	for (int i = 0; i < samples; i = i + SAMPLES_PER_TRANSFER) {
		int16_t accel_x = combine(fifo[i].ACCEL_XOUT_H, fifo[i].ACCEL_XOUT_L);
		int16_t accel_y = combine(fifo[i].ACCEL_YOUT_H, fifo[i].ACCEL_YOUT_L);
		int16_t accel_z = combine(fifo[i].ACCEL_ZOUT_H, fifo[i].ACCEL_ZOUT_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[accel.samples] = accel_x;
		accel.y[accel.samples] = accel_y;//(accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
		accel.z[accel.samples] = accel_z;(accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
		accel.samples++;
	}


	if (accel.samples > 0) {
        memcpy(_accel, &accel, sizeof(sensor_accel_fifo_s));
	}

	return !bad_data;
}

void MPU9250_I2C::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = FIFO_SAMPLE_DT;
    gyro.scale = gyro_scale;

	for (int i = 0; i < samples; i++) {
		const int16_t gyro_x = combine(fifo[i].GYRO_XOUT_H, fifo[i].GYRO_XOUT_L);
		const int16_t gyro_y = combine(fifo[i].GYRO_YOUT_H, fifo[i].GYRO_YOUT_L);
		const int16_t gyro_z = combine(fifo[i].GYRO_ZOUT_H, fifo[i].GYRO_ZOUT_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro_x;
		gyro.y[i] = gyro_y;//(gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
		gyro.z[i] = gyro_z;//(gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
	}

    memcpy(_gyro, &gyro, sizeof(sensor_accel_fifo_s));
}

void MPU9250_I2C::UpdateTemperature()
{
	// read current temperature
	uint8_t cmd = static_cast<uint8_t>(Register::TEMP_OUT_H);
	uint8_t temperature_buf[2] {};
	//set_frequency(SPI_SPEED_SENSOR);

	if (transfer(&cmd, 1, temperature_buf, 2) != PX4_OK) {
		return;
	}

	const int16_t TEMP_OUT = combine(temperature_buf[0], temperature_buf[1]);
	const float TEMP_degC = (TEMP_OUT / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

//	if (PX4_ISFINITE(TEMP_degC)) {
//		_px4_accel.set_temperature(TEMP_degC);
//		_px4_gyro.set_temperature(TEMP_degC);
//	}
}


MPU9250_I2C* mpu6500 = nullptr; // 声明一个指向MPU9250_I2C的指针

void mpu6500_init(I2C_HandleTypeDef *i2c, TIM_HandleTypeDef *tim, sensor_accel_fifo_s *accel, sensor_gyro_fifo_s *gyro) {
	// 动态创建MPU9250_I2C对象
	if (mpu6500 == nullptr) {
		mpu6500 = new MPU9250_I2C();
	}

	mpu6500->init(i2c, tim, accel, gyro);
	while (mpu6500->print_status() != 3) {
		mpu6500->RunImpl();
	}
}

void mpu6500_exit_and_cleanup() {
	if (mpu6500 != nullptr) {
		mpu6500->exit_and_cleanup();
		delete mpu6500; // 销毁对象
		mpu6500 = nullptr; // 防止野指针
	}
}

void mpu6500_run() {
	if (mpu6500 != nullptr) {
		mpu6500->RunImpl();
	}
}
