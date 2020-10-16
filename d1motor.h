#pragma once

#include "esphome.h"
#include <cmath>

static const char *TAG = "d1motor";

//! D1Motor implements interface to motor uC
//
// See: https://github.com/fabiuz7/wemos_motor_shield
class D1Motor : public Component, public i2c::I2CDevice, public output::FloatOutput {
public:
	D1Motor(I2CComponent *parent, uint8_t address=0x30, uint8_t channel=0) :
		I2CDevice(parent, address),
		channel_(channel)
		//min_power_(-1.0), max_power_(1.0)
	{}

	void setup() override {
		ESP_LOGVV(TAG, "initializing freq");
		this->update_frequency(8000.0);
	}

	//! set controls motor channel
	//
	// @param speed +-1.0 - CW or CCW rotation with set PWM
	//              0.0 - stop
	//              NAN - standby
	//              INFINITE - brake
	void set_level(float state) {
		ESP_LOGD(TAG, "set_level: %f", state);

		uint8_t buf[4];
		uint8_t dir;
		uint16_t sp = 0;

		if (this->is_inverted()) {
			state *= -1.0f;
		}

		if (std::isinf(state)) {
			dir = 0x00; // BREAK
		}
		else if (std::isnan(state)) {
			dir = 0x04; // STANDBY
		}
		else if (state > 0) {
			dir = 0x02; // CW
			sp = state * 10000;
		}
		else if (state < 0) {
			dir = 0x01; // CCW
			sp = state * 10000;
		}
		else {
			dir = 0x03; // STOP
		}

		buf[0] = 0x10 | channel_;
		buf[1] = dir;
		buf[2] = sp >> 8;
		buf[3] = sp & 0xff;

		auto res = this->write_bytes_raw(buf, sizeof(buf));
		if (!res) ESP_LOGE(TAG, "write error");
	}

	//! set_freq sets PWM frequency
	void update_frequency(float freq) override {
		uint8_t buf[4];
		uint32_t f = freq;

		buf[0] = (f >> 24) & 0x0f;
		buf[1] = (f >> 16) & 0xff;
		buf[2] = (f >> 8) & 0xff;
		buf[3] = f & 0xff;

		auto res = this->write_bytes_raw(buf, sizeof(buf));
		if (!res) ESP_LOGE(TAG, "write error");
	}

	void dump_config() {
		ESP_LOGCONFIG(TAG, "Address: 0x%02X", address_);
		ESP_LOGCONFIG(TAG, "Channel: %d", channel_);

		uint8_t info = this->reg(0x40).get();
		ESP_LOGCONFIG(TAG, "Info reg: 0x%02X", info);
	}

	void write_state(float state) override {
		ESP_LOGE(TAG, "write_state not overriden");
		this->set_level(state);	// XXX: original set_level clamps to 0..1
	}

private:
	uint8_t channel_;
};
