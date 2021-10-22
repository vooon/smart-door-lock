#pragma once

#include "esphome.h"
#include <cmath>

static const char *TAGXGZP = "xgzp6847d";

//! XGZP6847D  implements interface to preessure sensor
//
// Datasheet: http://www.cfsensor.com/static/upload/file/20210731/XGZP6847D%20Pressure%20Sensor%20Module%20V2.pdf
class XGZP6847D : public PollingComponent, public i2c::I2CDevice {
public:
	Sensor *pressure_sensor;
	Sensor *temperature_sensor;

	XGZP6847D(I2CBus *bus, uint8_t address=0x6D, int pressure_k=64) :
		PollingComponent(1000),
		pressure_sensor(new Sensor()),
		temperature_sensor(new Sensor()),
		pressure_k_(pressure_k)
	{
		set_i2c_bus(bus);
		set_i2c_address(address);
	}

	void setup() override {
		ESP_LOGVV(TAGXGZP, "initializing sensor");

		uint8_t part_id = this->reg(0x01).get();
		// NOTE: DS says that it's OTP value set in 0xA4, in my case 0x00.
		// If (part_id!= 0xA4) {
		// 	ESP_LOGE(TAGXGZP, "wrong part id: 0x%02X", part_id);
		// 	this->mark_failed();
		// }

		// 1. Clear raw_data_on
		uint8_t sys_config = this->reg(0xA5).get();
		this->reg(0xA5) = sys_config & 0xFD;

		// 2. Setup conversion mode
		this->reg(0x30) = (0b0010 << 4 /* 125 ms sleep mode */) | \
				  (0b010 << 0 /* temp+pressure measurement mode */) | \
				  (0b1 << 3 /* start conversion by SCO bit */);
	}

	void update() override {
		uint8_t buf[5];
		uint32_t press_raw;
		uint16_t temp_raw;
		float pressure, temperature;

		// 1. read status register
		uint8_t status = this->reg(0x02).get();

		// 2. read pressure and temperature registers
		if (read_register(0x06, buf, sizeof(buf)) != ERROR_OK) {
			this->status_set_warning();
			return;
		}

		// 3. calculate pressure
		press_raw = (buf[0] << 16) | (buf[1] << 8) | (buf[2] << 0);
		if (press_raw & (1 << 23)) {
			// MSB 1 => negative pressure
			pressure = ((int)press_raw - (1 << 24)) / pressure_k_;
		} else {
			pressure = press_raw / pressure_k_;
		}

		// 4. calculate temperature
		temp_raw = (buf[3] << 8) | (buf[4] << 0);
		if (temp_raw & (1 << 15)) {
			// MSB 1 => negative temperature
			temperature = ((int)temp_raw - (1 << 16)) / 256;
		} else {
			temperature = ((int)temp_raw) / 256;
		}

		// 5. publish states
		pressure_sensor->publish_state(pressure);
		temperature_sensor->publish_state(temperature);
		this->status_clear_warning();
	}

	void dump_config() {
#define TAG TAGXGZP
		LOG_I2C_DEVICE(this);
#undef TAG

		uint8_t part_id = this->reg(0xA4).get();
		uint8_t sys_config = this->reg(0xA5).get();
		ESP_LOGCONFIG(TAGXGZP, "Part ID: 0x%02X", part_id);
		ESP_LOGCONFIG(TAGXGZP, "Sys config: 0x%02X", sys_config);
	}

private:
	int pressure_k_;
};
