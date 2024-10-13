struct Repetition_Tester {
	i64 cycles_begin;
	i64 total_cycles_spent;
	i64 min_cycles_spent;
	i64 max_cycles_spent;
	i64 min_cycles_to_spend;
	i64 iterations;
	const char* name;
};

void init(Repetition_Tester* tester, const char* name, i64 min_ms_to_spend) {
	i64 cycles_per_s = get_rdtsc_timer_freq();
	i64 cycles_per_ms = cycles_per_s / 1'000;

	tester->cycles_begin = {};
	tester->total_cycles_spent = 0;
	tester->min_cycles_spent = 0x7FFF'FFFF'FFFF'FFFF;
	tester->max_cycles_spent = 0;
	tester->min_cycles_to_spend = min_ms_to_spend * cycles_per_ms;
	tester->iterations = 0;
	tester->name = name;
}

bool is_testing(Repetition_Tester* tester) {
	bool enough_cycles_spent = tester->total_cycles_spent >= tester->min_cycles_to_spend;
	bool result = !enough_cycles_spent;
	return result;
}

void begin_time(Repetition_Tester* tester) {
	i64 rdtsc_value = rdtsc();
	tester->cycles_begin = rdtsc_value;
}

void end_time(Repetition_Tester* tester) {
	i64 rdtsc_value = rdtsc();
	i64 cycles_spent = rdtsc_value - tester->cycles_begin;
	tester->total_cycles_spent += cycles_spent;
	tester->min_cycles_spent = cycles_spent < tester->min_cycles_spent ? cycles_spent : tester->min_cycles_spent;
	tester->max_cycles_spent = cycles_spent > tester->max_cycles_spent ? cycles_spent : tester->max_cycles_spent;
	tester->iterations++;
}

void print_results(Repetition_Tester* tester) {
	f32 us_per_s = 1000000.0f;
	f32 s_per_cycle = 1.0f / f32(get_rdtsc_timer_freq());
	f32 us_per_cycle = s_per_cycle * us_per_s;

	i64 cycles_per_iteration = tester->total_cycles_spent / tester->iterations;
	f32 us_per_iteration = f32(cycles_per_iteration) * us_per_cycle;
	f32 us_total = f32(tester->total_cycles_spent) * us_per_cycle;

	f32 min_us = f32(tester->min_cycles_spent) * us_per_cycle;
	f32 max_us = f32(tester->max_cycles_spent) * us_per_cycle;

	std::cout << "Repitition test results for \"" << tester->name << "\":\n";
	if (tester->iterations) {
		std::cout << "Time/iteration: " << us_per_iteration << " us (" << us_total << " us total, " << tester->iterations << " iterations)\n";
	}
	std::cout << "Min time: " << min_us << " us\n";
	std::cout << "Max time: " << max_us << " us\n";
	std::cout << "\n";
}