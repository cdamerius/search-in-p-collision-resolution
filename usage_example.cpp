//Compile with:
//g++-12 -mavx2 -s -flto -O3 -ftabstop=4 -Wno-maybe-uninitialized -Wno-uninitialized -o usage_example.out usage_example.cpp

#include "resolve_collision.hpp"

#include <cstdio>

void print_result(f32_4 target, Resolve_Collision_Result resolve_collision_result) {
	printf("Attempting to move to (%f, %f, %f, %f).\n", target.x, target.y, target.z, target.w);
	if (resolve_collision_result.found_new_pos) {
		f32_4 new_pos = resolve_collision_result.new_pos;
		printf("Found new position at (%f, %f, %f, %f).\n", new_pos.x, new_pos.y, new_pos.z, new_pos.w);
		printf("Is the cylinder grounded? ");
		if (resolve_collision_result.new_grounded) {
			printf("yes.\n");
		} else {
			printf("no.\n");
		}
	} else {
		printf("No space to put cylinder to!\n");
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
	print_result(target, resolve_collision_result);


	//Benchmark
	u64 rdtsc_begin = rdtsc();
	i32 exec_count = 40000;
	for (i32 i = 0; i != exec_count; i++) {
		resolve_collision_result = resolve_collision(cylinder, target, vertices_count, &vertices[0]);
	}
	u64 rdtsc_end = rdtsc();

	u64 cycles_total = rdtsc_end - rdtsc_begin;
	f64 cycles_average = f64(cycles_total) / f64(exec_count);
	u64 cycles_per_second = 3'000'000'000;
	f64 microseconds_avg = cycles_average / f64(cycles_per_second / 1'000'000);

	printf("Benchmark setting: Hot cache scenario (%d iterations), %d triangles, %lu CPU cycles per second.\n", exec_count, vertices_count / 3, cycles_per_second);
	printf("Time per resolve_collision call: %f microseconds.\n", microseconds_avg);

	return 0;
}