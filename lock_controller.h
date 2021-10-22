#pragma once

#include "esphome.h"

static const char *TAGLC = "lockctl";

#define get_lockctl(constructor) static_cast<LockController *> \
  (const_cast<custom_component::CustomComponentConstructor *>(&constructor)->get_component(0))


class LockController : public Component {
public:
	using SW = switch_::Switch&;
	using ES = gpio::GPIOBinarySensor&;
	using GV32 = uint32_t&;

	static constexpr uint32_t ACTUATION_TIMEOUT_MS = 10000;

	enum class State {
		open = 1,
		opening = 2,
		close = 3,
		closing = 4,
		wait_to_close = 5,
		unknown = 6
	};

	LockController(SW open_sw, SW close_sw, script::Script &on_state_change,
			ES locked_es, ES unlocked_es, ES door_closed,
			GV32 lock_cnt, GV32 unlock_cnt) :
		_st(State::unknown), _prev_st(State::unknown),
		_open_sw(open_sw), _close_sw(close_sw), _on_state_change(on_state_change),
		_locked_es(locked_es), _unlocked_es(unlocked_es), _door_closed(door_closed),
		_lock_cnt(lock_cnt), _unlock_cnt(unlock_cnt)
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
		if (_prev_st != _st) {
			_start_millis = now;
			_prev_st = _st;
		}

		uint32_t run_time = now - _start_millis;

		if (run_time > ACTUATION_TIMEOUT_MS) {
			ESP_LOGE(TAGLC, "run timeout, stop motor in unknown state");
			stop_actuation(State::unknown);
		}
	}

	inline State get_state() { return _st; }

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
	State _st, _prev_st;
	SW _open_sw, _close_sw;
	script::Script &_on_state_change;
	ES _locked_es, _unlocked_es, _door_closed;
	GV32 _lock_cnt, _unlock_cnt;

	uint32_t _start_millis;

	void command_actuation(State st) {
		_st = st;
		if (st == State::opening) {
			_open_sw.turn_on();
		} else if (st == State::closing) {
			_close_sw.turn_on();
		}

		_on_state_change.execute();
	}

	void stop_actuation(State st) {
		_open_sw.turn_off();
		_close_sw.turn_off();
		_st = st;
		_on_state_change.execute();
	}
};
