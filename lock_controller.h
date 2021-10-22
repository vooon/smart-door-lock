#pragma once

#include "esphome.h"
#include "d1motor.h"

static const char *TAGLC = "lockctl";

#define get_lockctl(constructor) static_cast<LockController *> \
  (const_cast<custom_component::CustomComponentConstructor *>(&constructor)->get_component(0))


class LockController : public Component {
public:
	using ES = gpio::GPIOBinarySensor&;
	using GV32 = uint32_t&;

	static constexpr uint32_t MOTOR_TIMEOUT_MS = 10000;
	static constexpr uint32_t MOTOR_STARTUP_TIME = 500;
	static constexpr uint32_t MOTOR_MIN_UPDATE_PERIOD_MS = 10;
	static constexpr float MOTOR_START_PWM = 0.1;
	static constexpr float MOTOR_FULL_PWM = 1.0;

	enum class State {
		open = 1,
		opening = 2,
		close = 3,
		closing = 4,
		wait_to_close = 5,
		unknown = 6
	};

	LockController(D1Motor &m, script::Script &on_state_change,
			ES locked_es, ES unlocked_es, ES door_closed,
			GV32 lock_cnt, GV32 unlock_cnt) :
		_st(State::unknown), _m(m), _on_state_change(on_state_change),
		_locked_es(locked_es), _unlocked_es(unlocked_es), _door_closed(door_closed),
		_lock_cnt(lock_cnt), _unlock_cnt(unlock_cnt),
		_prev_speed(0.0), _prev_update(0)
	{}

	void setup() override {
		ESP_LOGD(TAGLC, "controller setup complete");
	}

	void loop() override {
		// ESP_LOGD(TAGLC, "state: %d", _st);
		if (!(_st == State::opening || _st == State::closing)) {
			return;
		}

		uint32_t now = millis();
		if (_prev_speed == 0.0) {
			_start_millis = now;
		}

#if 0
		uint32_t update_period = now - _prev_update;
		if (!(now < _prev_update || update_period > MOTOR_MIN_UPDATE_PERIOD_MS)) {
			//ESP_LOGD(TAGLC, "trottling");
			return;
		}
#endif

		uint32_t run_time = now - _start_millis;
#if 1	// slow speed build-up
		float new_pwm = mapf(run_time, 0, MOTOR_STARTUP_TIME, MOTOR_START_PWM, MOTOR_FULL_PWM);
		if (_st == State::closing) {
			new_pwm *= -1.0;
		}

#else  	// instant full-power
		float new_pwm = MOTOR_FULL_PWM;
		if (_st == State::closing) {
			new_pwm *= -1.0;
		}

#endif

		if (_prev_speed != new_pwm && run_time <= MOTOR_TIMEOUT_MS) {
			//ESP_LOGD(TAGLC, "state %d, t: %d/%d (%d), p: %f/%f", _st, _start_millis, now, run_time, _prev_speed, new_pwm);
			_m.set_level(new_pwm);
			_prev_speed = new_pwm;
			_prev_update = now;
			_on_state_change.execute();
		}

		if (run_time > MOTOR_TIMEOUT_MS) {
			ESP_LOGE(TAGLC, "run timeout, stop motor in unknown state");
			stop_actuation(State::unknown);
		}
	}

	inline State get_state() { return _st; }
	inline float get_motor_command() { return _prev_speed; }

	void restore_state() {
		// Restore state from endstops
		if (_locked_es.state) {
			_st = State::close;
		}
		else if (_unlocked_es.state) {
			_st = State::open;
		}

		// If state still unknown - close
		if (_st == State::unknown) {
			_st = State::closing;
		}

		//ESP_LOGI(TAGLC, "restored state: %d", _st);
		_on_state_change.execute();
	}

	void unlock() {
		ESP_LOGI(TAGLC, "requested to unlock");

		if (_unlocked_es.state) {
			ESP_LOGI(TAGLC, "already unlocked");
			return;
		}

		command_actuation(State::opening);
	}

	void lock() {
		ESP_LOGI(TAGLC, "requested to lock");

		if (_locked_es.state) {
			ESP_LOGI(TAGLC, "already locked");
			return;
		}

		if (_door_closed.state) {
			ESP_LOGI(TAGLC, "door closed, locking immediately");
			command_actuation(State::closing);
		}
		else {
			ESP_LOGI(TAGLC, "door not closed yet, waiting...");
			command_actuation(State::wait_to_close);
		}
	}

	void on_lock_endstop() {
		ESP_LOGD(TAGLC, "endstop: locked");
		_lock_cnt += 1;
		stop_actuation(State::close);
	}

	void on_unlock_endstop() {
		ESP_LOGD(TAGLC, "endstop: unlocked");
		_unlock_cnt += 1;
		stop_actuation(State::open);
	}

	void on_door_closed() {
		ESP_LOGD(TAGLC, "endstop: door closed");
		if (_st == State::wait_to_close) {
			command_actuation(State::closing);
		}
	}

private:
	State _st;
	D1Motor &_m;
	script::Script &_on_state_change;
	ES _locked_es, _unlocked_es, _door_closed;
	GV32 _lock_cnt, _unlock_cnt;

	uint32_t _start_millis;
	float _prev_speed;
	uint32_t _prev_update;

	void command_actuation(State st) {
		_st = st;
		_prev_speed = 0.0;
		_on_state_change.execute();
	}

	void stop_actuation(State st) {
		_m.set_level(INFINITY);
		// _m.set_level(0.0);
		_st = st;
		_prev_speed = 0.0;
		_on_state_change.execute();
	}

	float mapf(float val, float in_min, float in_max, float out_min, float out_max) {
		val = clamp(val, in_min, in_max);
		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
};
