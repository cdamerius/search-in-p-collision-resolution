//Compile with:
//g++-12 -mavx2 -s -flto -O3 -ftabstop=4 -Wno-maybe-uninitialized -Wno-uninitialized -o usage_example.out usage_example.cpp

#include <iostream>

#include "resolve_collision.hpp"
#include "repetition_tester.cpp"

void print_collision_resolve_result(f32_4 target, Resolve_Collision_Result resolve_collision_result) {
	std::cout << "Attempting to move to (" << target.x << ", " << target.y << ", " << target.z << ", " << target.w << ").\n";
	if (resolve_collision_result.found_new_pos) {
		f32_4 new_pos = resolve_collision_result.new_pos;
		std::cout << "Found new position at (" << new_pos.x << ", " << new_pos.y << ", " << new_pos.z << ", " << new_pos.w << ").\n";
		std::cout << "Is the cylinder grounded? ";
		if (resolve_collision_result.new_grounded) {
			std::cout << "yes.\n";
		} else {
			std::cout << "no.\n";
		}
	} else {
		std::cout << "No space to put cylinder to!\n";
	}
}

int main(int, char**) {
	//Usage example
	f32_4 vertices[3 * 16];
	i32 vertices_count = array_len(vertices);
	f32_4* out = &vertices[0];

	for (i32 y = 0; y != 3; y++) {
		for (i32 x = 0; x != 3; x++) {
			if (x == 2 && y == 2) {
				break;
			}

			f32 freq = 600.0f;
			i32 lx = 70;
			i32 ly = 70;
			f32 tau = 2.0f * 3.141592653589793f;

			f32 zbias00 = sin_f32(tau * (f32(lx * x + ly * y) / freq));
			f32 zbias01 = sin_f32(tau * (f32(lx * x + ly * (y + 1)) / freq));
			f32 zbias10 = sin_f32(tau * (f32(lx * (x + 1) + ly * y) / freq));
			f32 zbias11 = sin_f32(tau * (f32(lx * (x + 1) + ly * (y + 1)) / freq));

			*(out++) = {f32(x + 0), f32(y + 0), zbias00, 1.0f};
			*(out++) = {f32(x + 1), f32(y + 0), zbias10, 1.0f};
			*(out++) = {f32(x + 0), f32(y + 1), zbias01, 1.0f};
			*(out++) = {f32(x + 1), f32(y + 0), zbias10, 1.0f};
			*(out++) = {f32(x + 1), f32(y + 1), zbias11, 1.0f};
			*(out++) = {f32(x + 0), f32(y + 1), zbias01, 1.0f};
		}
	}

	Collision_Cylinder cylinder = {
		.leg_height = 0.3f,
		.head_height = 1.2f,
		.total_height = 1.5f,
		.radius = 0.3f
	};

	f32_4 target = {0.2f, 0.1f, -1.0f, 1.0f};

	Resolve_Collision_Result resolve_collision_result = resolve_collision(cylinder, target, vertices_count, &vertices[0]);
	print_collision_resolve_result(target, resolve_collision_result);

	//Benchmark
	Repetition_Tester tester;
	i64 min_ms_to_spend = 1'000;
	init(&tester, "resolve_collision", min_ms_to_spend);
	while (is_testing(&tester)) {
		begin_time(&tester);
		resolve_collision_result = resolve_collision(cylinder, target, vertices_count, &vertices[0]);
		end_time(&tester);
	}
	std::cout << "\n";
	print_results(&tester);

	return 0;
}