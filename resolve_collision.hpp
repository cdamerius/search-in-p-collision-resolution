#ifndef RESOLVE_COLLISION_HPP
#define RESOLVE_COLLISION_HPP

#include "util.hpp"

//Collision_Cylinder represents a cylinder with the z-axis as
//its vertical axis. total_height is the height of the cylinder,
//and radius the radius of its caps.
//leg_height is the maximum height that a floor can be so that
//the cylinder can still clip up to that floor. This is useful
//for games where you want the cylinder to clip up stairs etc.
//head_height is the same from the other side, i.e., the minimum
//height that a ceiling may be so that the cylinder may still
//clip under it.
struct Collision_Cylinder {
	f32 leg_height;
	f32 head_height;
	f32 total_height;
	f32 radius;
};

struct Resolve_Collision_Result {
	bool found_new_pos;//whether a new cylinder position could be found.
	bool new_grounded;//whether the cylinder is on the floor after collision resolution.
	f32_4 new_pos;//the resolved position of the cylinder.
};

//resolve_collision
//All points (target and the vertices) must be given in homogeneous coordinates (.w must be 1.0f).
//vertices points to an array of "vertices_count" many vertices. These represent triangles, so
//vertices_count must be a multiple of 3.
Resolve_Collision_Result resolve_collision(Collision_Cylinder cylinder, f32_4 target, i32 vertices_count, f32_4* vertices) {
	//Make grid with integer points (x, y) where 0 <= x,y < grid_width
	//We do an affine transform the coordinate system such that the
	//cylinder is in the central cell of the grid, target.z gets mapped
	//to z=0, the cylinder's total height becomes 1 and the cylinder's
	//disk barely fits into the grid.
	//Points (p) are mapped to (scale * p - trans).
	constexpr const i32 grid_width = 8;//NOTE(cdamerius): grid_width cannot be changed as we do AVX-256 optimizations that rely on this grid_width.
	static_assert(grid_width % 2 == 0, "Must have grid_width % 2 == 0.");
	constexpr const i32 grid_radius_i32 = grid_width / 2 - 1;
	constexpr const f32 grid_radius = f32(grid_radius_i32);
	constexpr const f32 grid_radius_sq = f32(grid_radius_i32 * grid_radius_i32);
	f32 radius_scale = grid_radius / cylinder.radius;
	f32_4 scale = {radius_scale, radius_scale, 1.0f / cylinder.total_height, 0.0f};
	f32_4 trans = {
		floor_f32_to_f32(target.x * radius_scale - grid_radius),
		floor_f32_to_f32(target.y * radius_scale - grid_radius),
		scale.z * target.z,
		-1.0f
	};


	//Each grid point stores a set of floor and ceiling values. For a grid
	//point, always the highest seen-so-far floor and lowest seen-so-far
	//ceiling are stored. We address the grid corner at (x,y) with
	//with (y * grid_width + x).

	//Each grid point stores the floor (floor_z.grid_corners) and the ceiling
	//(ceil.z.grid_corners) found at these grid points. floor_z/ceil_z.bound_x,
	//floor_z/ceil_z.bound_y store such floors and ceiling z values for points
	//on the edges between two grid corners. An entry at index (y * grid_width + x)
	//stores the value for the edge starting at that grid point towards higher
	//x value (bound_x) and higher y value (bound_y) respectively.
	//Additionally, we store the actual x and y coordinates where the floor
	//and ceiling have these z-values on the edge (floor_t, ceil_t within
	//bound_x, bound_y).

	//For homogenization reasons, all floor z values are actually negated.
	//Sentinels are added to avoid extra if-checks for writing outside of the
	//arrays.
	struct {
		f32 grid_corners[grid_width * grid_width] alignas(32);
		f32 sentinel_0[2 * grid_width] alignas(32);
		f32 bound_x[grid_width * grid_width] alignas(32);
		f32 sentinel_1[2 * grid_width] alignas(32);
		f32 bound_y[grid_width * grid_width] alignas(32);
		f32 sentinel_2[2 * grid_width] alignas(32);
	} floor_z;

	struct {
		f32 grid_corners[grid_width * grid_width] alignas(32);
		f32 sentinel_0[2 * grid_width] alignas(32);
		f32 bound_x[grid_width * grid_width] alignas(32);
		f32 sentinel_1[2 * grid_width] alignas(32);
		f32 bound_y[grid_width * grid_width] alignas(32);
		f32 sentinel_2[2 * grid_width] alignas(32);
	} ceil_z;

	struct {
		f32 floor_t[grid_width * grid_width] alignas(32);
		f32 ceil_t[grid_width * grid_width] alignas(32);
	} bound_x;

	struct {
		f32 floor_t[grid_width * grid_width] alignas(32);
		f32 ceil_t[grid_width * grid_width] alignas(32);
	} bound_y;

	f32 high_ceil = (cylinder.total_height + cylinder.leg_height) * scale.z;
	f32 neg_low_floor = (cylinder.total_height - cylinder.head_height) * scale.z;

	for (i32 i = 0; i != array_len(floor_z.grid_corners); i++) {
		floor_z.grid_corners[i] = neg_low_floor;
	}
	for (i32 i = 0; i != array_len(floor_z.sentinel_0); i++) {
		floor_z.sentinel_0[i] = neg_low_floor;
	}
	for (i32 i = 0; i != array_len(floor_z.bound_x); i++) {
		floor_z.bound_x[i] = neg_low_floor;
	}
	for (i32 i = 0; i != array_len(floor_z.sentinel_1); i++) {
		floor_z.sentinel_1[i] = neg_low_floor;
	}
	for (i32 i = 0; i != array_len(floor_z.bound_y); i++) {
		floor_z.bound_y[i] = neg_low_floor;
	}
	for (i32 i = 0; i != array_len(floor_z.sentinel_2); i++) {
		floor_z.sentinel_2[i] = neg_low_floor;
	}

	for (i32 i = 0; i != array_len(ceil_z.grid_corners); i++) {
		ceil_z.grid_corners[i] = high_ceil;
	}
	for (i32 i = 0; i != array_len(ceil_z.sentinel_0); i++) {
		ceil_z.sentinel_0[i] = high_ceil;
	}
	for (i32 i = 0; i != array_len(ceil_z.bound_x); i++) {
		ceil_z.bound_x[i] = high_ceil;
	}
	for (i32 i = 0; i != array_len(ceil_z.sentinel_1); i++) {
		ceil_z.sentinel_1[i] = high_ceil;
	}
	for (i32 i = 0; i != array_len(ceil_z.bound_y); i++) {
		ceil_z.bound_y[i] = high_ceil;
	}
	for (i32 i = 0; i != array_len(ceil_z.sentinel_2); i++) {
		ceil_z.sentinel_2[i] = high_ceil;
	}

	for (i32 y = 0; y != grid_width; y++) {
		for (i32 x = 0; x != grid_width; x++) {
			f32 xf = f32(x);
			i32 i = y * grid_width + x;
			bound_x.floor_t[i] = xf;
		}
	}
	for (i32 y = 0; y != grid_width; y++) {
		for (i32 x = 0; x != grid_width; x++) {
			f32 xf = f32(x);
			i32 i = y * grid_width + x;
			bound_x.ceil_t[i] = xf;
		}
	}
	for (i32 y = 0; y != grid_width; y++) {
		f32 yf = f32(y);
		for (i32 x = 0; x != grid_width; x++) {
			i32 i = y * grid_width + x;
			bound_y.floor_t[i] = yf;
		}
	}
	for (i32 y = 0; y != grid_width; y++) {
		f32 yf = f32(y);
		for (i32 x = 0; x != grid_width; x++) {
			i32 i = y * grid_width + x;
			bound_y.ceil_t[i] = yf;
		}
	}

	f32_4 grid_target = {scale.x * target.x - trans.x, scale.y * target.y - trans.y, 1.0f, 1.0f};

	//z_waist is used to classify a triangle as a floor or ceiling.
	//If the z-value that the triangle has at the target's xy position is smaller
	//than z_waist, then it is considered a floor, otherwise a ceiling.
	f32 z_waist = 0.5f * (cylinder.leg_height + cylinder.head_height) * scale.z;

	for (i32 i = 0; i != vertices_count; i += 3) {
		//x_min, x_max, y_min, y_max store the minimum/maximum x/y grid coordinates
		//where the current triangle affects floor_z/ceil_z.grid_corners. This is
		//used to determine points on grid edges.
		i32 x_min[grid_width] alignas(32);
		i32 y_min[grid_width] alignas(32);
		i32 x_max[grid_width] alignas(32);
		i32 y_max[grid_width] alignas(32);

		for (i32 h = 0; h != grid_width; h++) {
			x_min[h] = grid_width;
		}
		for (i32 h = 0; h != grid_width; h++) {
			y_min[h] = grid_width;
		}
		for (i32 h = 0; h != grid_width; h++) {
			x_max[h] = -1;
		}
		for (i32 h = 0; h != grid_width; h++) {
			y_max[h] = -1;
		}

		f32_4 p0 = vertices[i + 0] * scale - trans;
		f32_4 p1 = vertices[i + 1] * scale - trans;
		f32_4 p2 = vertices[i + 2] * scale - trans;

		//ensure that the XY-projection of the triangle is left-winded.
		f32 winding = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
		if (winding < 0.0f) {
			f32_4 temp = p1;
			p1 = p2;
			p2 = temp;
		}

		f32_4 v0 = p1 - p0;
		f32_4 v1 = p2 - p1;
		f32_4 v2 = p0 - p2;

		//Calculate the triangle plane P: x * z_form.x + y * z_form.y + 1 * z_form.z
		f32_4 z_form_crosses;
		z_form_crosses.x = v0.y * v2.z - v0.z * v2.y;
		z_form_crosses.y = v0.z * v2.x - v0.x * v2.z;
		z_form_crosses.z = v0.x * v2.y - v0.y * v2.x;
		z_form_crosses.w = 0.0f;
		f32 inverse_z_form_crosses_z = 1.0f / z_form_crosses.z;
		f32 z_form_z_dot = p0.x * z_form_crosses.x + p0.y * z_form_crosses.y + p0.z * z_form_crosses.z + p0.w * z_form_crosses.w;
		f32_4 z_form;
		z_form.x = inverse_z_form_crosses_z * -z_form_crosses.x;
		z_form.y = inverse_z_form_crosses_z * -z_form_crosses.y;
		z_form.z = inverse_z_form_crosses_z * z_form_z_dot;
		z_form.w = 0.0f;

		f32 z_at_target = z_form.x * grid_target.x + z_form.y * grid_target.y + z_form.z * grid_target.z + z_form.w * grid_target.w;
		i32 tri_is_ceil = i32(z_at_target >= z_waist);

		f32 upwards_dir_z_helper_v0 = v0.x * z_form.y - v0.y * z_form.x;
		f32 upwards_dir_z_helper_v2 = z_form.x * v2.y - z_form.y * v2.x;

		//negate z values for floor triangles to homogenize calculations between floors and ceilings
		f32_4 old_z_form = z_form;
		if (!tri_is_ceil) {
			p0.z = -p0.z;
			p1.z = -p1.z;
			p2.z = -p2.z;
			z_form = -z_form;
		}

		//Calculate triangle upwards vector.
		//This is the vector that lies within the triangle plane that has
		//length grid_width in XY and where upwards.z is maximized.
		f32_4 upwards;
		f32 z_form_xy_sqlen = z_form.x * z_form.x + z_form.y * z_form.y;
		f32 upwards_eps = 0.0000001f;
		f32 upwards_scale = grid_radius / sqrt_f32(z_form_xy_sqlen);//NOTE(cdamerius): this won't work for degenerate triangles or triangles where the XY projection has zero area, since we would divide by 0.
		f32_4 upwards_dir = {z_form.x, z_form.y, inverse_z_form_crosses_z * (upwards_dir_z_helper_v2 * v0.z + upwards_dir_z_helper_v0 * v2.z), 0.0f};
		f32_4 non_degenerate_upwards = {upwards_scale * upwards_dir.x, upwards_scale * upwards_dir.y, upwards_scale * upwards_dir.z, upwards_scale * upwards_dir.w};
		if (z_form_xy_sqlen < upwards_eps) {
			//the triangle is coplanar to the XY plane, so any direction is fine here.
			upwards = {grid_radius, 0.0f, 0.0f, 0.0f};
		} else {
			upwards = non_degenerate_upwards;
		}

		//Order triangle points descendingly after z value.
		f32_4 p_order0;
		f32_4 p_order1;
		f32_4 p_order2;
		if (p0.z <= p1.z) {
			if (p1.z <= p2.z) {
				p_order0 = p2;
				p_order1 = p1;
				p_order2 = p0;
			} else {
				if (p0.z <= p2.z) {
					p_order0 = p1;
					p_order1 = p2;
					p_order2 = p0;
				} else {
					p_order0 = p1;
					p_order1 = p0;
					p_order2 = p2;
				}
			}
		} else {
			if (p1.z <= p2.z) {
				if (p0.z <= p2.z) {
					p_order0 = p2;
					p_order1 = p0;
					p_order2 = p1;
				} else {
					p_order0 = p0;
					p_order1 = p2;
					p_order2 = p1;
				}
			} else {
				p_order0 = p0;
				p_order1 = p1;
				p_order2 = p2;
			}
		}

		f32_4 v_order0 = p_order1 - p_order0;
		f32_4 v_order1 = p_order2 - p_order1;
		f32_4 v_order2 = p_order2 - p_order0;

		//Calculation of values that stay constant for the calculations in the
		//grid loop below.
		f32 e0_comp = (p0.x + upwards.x) * v0.y - (p0.y + upwards.y) * v0.x;
		f32 e1_comp = (p1.x + upwards.x) * v1.y - (p1.y + upwards.y) * v1.x;
		f32 e2_comp = (p2.x + upwards.x) * v2.y - (p2.y + upwards.y) * v2.x;

		f32 edge_helper[3][6];
		for (i32 j = 0; j != 3; j++) {
			f32_4 s;
			f32_4 f;
			if (j == 0) {
				s = p_order0;
				f = v_order0;
			} else if (j == 1) {
				s = p_order0;
				f = v_order2;
			} else {
				s = p_order1;
				f = v_order1;
			}
			f32 fscale = 1.0f / (f.x * f.x + f.y * f.y);
			f32 T0 = f.y * fscale;
			f32 T1 = f.x * fscale;
			f32 T0T0 = T0 * T0;
			f32 T1T1 = T1 * T1;
			f32 T0T1 = T0 * T1;
			edge_helper[j][0] = T0;
			edge_helper[j][1] = T1;
			edge_helper[j][2] = 2.0f * T0T1 * s.x * s.y + T1T1 * (grid_radius_sq - s.y * s.y) + T0T0 * (grid_radius_sq - s.x * s.x);
			edge_helper[j][3] = s.x * T1 + s.y * T0;
			edge_helper[j][4] = 2.0f * (T0T0 * s.x - T0T1 * s.y);
			edge_helper[j][5] = 2.0f * (T1T1 * s.y - T0T1 * s.x);
		}

		f32* grid_corners;
		if (tri_is_ceil == 0) {
			grid_corners = &floor_z.grid_corners[0];
		} else {
			grid_corners = &ceil_z.grid_corners[0];
		}

		//Here we sample the highest or lowest z-value within the minkowski sum
		//of the triangle with a disk in the XY plane and radius "grid_radius" at
		//the origin, and augment the floor_z/ceil_z.grid_corners values with them.
		for (i32 y = 0; y != grid_width; y++) {
			m256 qx = _mm256_set_ps(7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f, 0.0f);
			m256 qy = _mm256_set1_ps(f32(y));
			m256 z_form_x = _mm256_set1_ps(z_form.x);
			m256 z_form_y = _mm256_set1_ps(z_form.y);
			m256 z_form_z = _mm256_set1_ps(z_form.z);
			m256 qz_x = _mm256_mul_ps(z_form_x, qx);
			m256 qz_xz = _mm256_add_ps(qz_x, z_form_z);
			m256 qz_y = _mm256_mul_ps(z_form_y, qy);
			m256 qz = _mm256_add_ps(qz_xz, qz_y);

			m256 e00 = _mm256_set1_ps(edge_helper[0][0]);
			m256 e10 = _mm256_set1_ps(edge_helper[1][0]);
			m256 e20 = _mm256_set1_ps(edge_helper[2][0]);

			m256 X00x = _mm256_mul_ps(e00, qx);
			m256 X10x = _mm256_mul_ps(e10, qx);
			m256 X20x = _mm256_mul_ps(e20, qx);

			m256 e01 = _mm256_set1_ps(edge_helper[0][1]);
			m256 e11 = _mm256_set1_ps(edge_helper[1][1]);
			m256 e21 = _mm256_set1_ps(edge_helper[2][1]);

			m256 X00y = _mm256_mul_ps(e01, qy);
			m256 X10y = _mm256_mul_ps(e11, qy);
			m256 X20y = _mm256_mul_ps(e21, qy);

			m256 X00 = _mm256_sub_ps(X00x, X00y);
			m256 X10 = _mm256_sub_ps(X10x, X10y);
			m256 X20 = _mm256_sub_ps(X20x, X20y);

			m256 e05 = _mm256_set1_ps(edge_helper[0][5]);
			m256 e15 = _mm256_set1_ps(edge_helper[1][5]);
			m256 e25 = _mm256_set1_ps(edge_helper[2][5]);

			m256 X00_sq = _mm256_mul_ps(X00, X00);
			m256 X10_sq = _mm256_mul_ps(X10, X10);
			m256 X20_sq = _mm256_mul_ps(X20, X20);

			m256 X01y = _mm256_mul_ps(e05, qy);
			m256 X11y = _mm256_mul_ps(e15, qy);
			m256 X21y = _mm256_mul_ps(e25, qy);

			m256 e04 = _mm256_set1_ps(edge_helper[0][4]);
			m256 e14 = _mm256_set1_ps(edge_helper[1][4]);
			m256 e24 = _mm256_set1_ps(edge_helper[2][4]);

			m256 X01y_e = _mm256_sub_ps(X01y, X00_sq);
			m256 X11y_e = _mm256_sub_ps(X11y, X10_sq);
			m256 X21y_e = _mm256_sub_ps(X21y, X20_sq);

			m256 e02 = _mm256_set1_ps(edge_helper[0][2]);
			m256 e12 = _mm256_set1_ps(edge_helper[1][2]);
			m256 e22 = _mm256_set1_ps(edge_helper[2][2]);

			m256 X01x = _mm256_mul_ps(e04, qx);
			m256 X11x = _mm256_mul_ps(e14, qx);
			m256 X21x = _mm256_mul_ps(e24, qx);

			m256 X02y = _mm256_mul_ps(e00, qy);
			m256 X12y = _mm256_mul_ps(e10, qy);
			m256 X22y = _mm256_mul_ps(e20, qy);

			m256 X01x_e = _mm256_add_ps(e02, X01x);
			m256 X11x_e = _mm256_add_ps(e12, X11x);
			m256 X21x_e = _mm256_add_ps(e22, X21x);

			m256 X02x = _mm256_mul_ps(e01, qx);
			m256 X12x = _mm256_mul_ps(e11, qx);
			m256 X22x = _mm256_mul_ps(e21, qx);

			m256 e03 = _mm256_set1_ps(edge_helper[0][3]);
			m256 e13 = _mm256_set1_ps(edge_helper[1][3]);
			m256 e23 = _mm256_set1_ps(edge_helper[2][3]);

			m256 X01 = _mm256_add_ps(X01x_e, X01y_e);
			m256 X11 = _mm256_add_ps(X11x_e, X11y_e);
			m256 X21 = _mm256_add_ps(X21x_e, X21y_e);

			m256 X02y_e = _mm256_sub_ps(X02y, e03);
			m256 X12y_e = _mm256_sub_ps(X12y, e13);
			m256 X22y_e = _mm256_sub_ps(X22y, e23);

			m256 X03 = _mm256_sqrt_ps(X01);
			m256 X13 = _mm256_sqrt_ps(X11);
			m256 X23 = _mm256_sqrt_ps(X21);

			m256 zero = _mm256_set1_ps(0.0f);

			m256 X02 = _mm256_add_ps(X02x, X02y_e);
			m256 X12 = _mm256_add_ps(X12x, X12y_e);
			m256 X22 = _mm256_add_ps(X22x, X22y_e);

			m256 one = _mm256_set1_ps(1.0f);

			m256 X01p = _mm256_cmp_ps(X01, zero, _CMP_GE_OQ);
			m256 X11p = _mm256_cmp_ps(X11, zero, _CMP_GE_OQ);
			m256 X21p = _mm256_cmp_ps(X21, zero, _CMP_GE_OQ);

			m256 L01 = _mm256_add_ps(X02, X03);
			m256 L11 = _mm256_add_ps(X12, X13);
			m256 L21 = _mm256_add_ps(X22, X23);

			m256 L02 = _mm256_sub_ps(X02, X03);
			m256 L12 = _mm256_sub_ps(X12, X13);
			m256 L22 = _mm256_sub_ps(X22, X23);

			m256 L03 = _mm256_min_ps(L01, one);
			m256 L13 = _mm256_min_ps(L11, one);
			m256 L23 = _mm256_min_ps(L21, one);

			m256 L032 = _mm256_cmp_ps(L03, L02, _CMP_GE_OQ);
			m256 L132 = _mm256_cmp_ps(L13, L12, _CMP_GE_OQ);
			m256 L232 = _mm256_cmp_ps(L23, L22, _CMP_GE_OQ);

			m256 L03p = _mm256_cmp_ps(L03, zero, _CMP_GE_OQ);
			m256 L13p = _mm256_cmp_ps(L13, zero, _CMP_GE_OQ);
			m256 L23p = _mm256_cmp_ps(L23, zero, _CMP_GE_OQ);

			m256 valid0_e = _mm256_and_ps(X01p, L032);
			m256 valid1_e = _mm256_and_ps(X11p, L132);
			m256 valid2_e = _mm256_and_ps(X21p, L232);

			m256 v_ord0_z = _mm256_set1_ps(v_order0.z);
			m256 v_ord2_z = _mm256_set1_ps(v_order2.z);
			m256 v_ord1_z = _mm256_set1_ps(v_order1.z);
			m256 p_ord0_z = _mm256_set1_ps(p_order0.z);
			m256 p_ord1_z = _mm256_set1_ps(p_order1.z);

			m256 valid0 = _mm256_and_ps(valid0_e, L03p);
			m256 valid1 = _mm256_and_ps(valid1_e, L13p);
			m256 valid2 = _mm256_and_ps(valid2_e, L23p);

			m256 Z1_e = _mm256_mul_ps(L03, v_ord0_z);
			m256 Z2_e = _mm256_mul_ps(L13, v_ord2_z);
			m256 Z3_e = _mm256_mul_ps(L23, v_ord1_z);

			m256 inf_v = _mm256_set1_ps(F32_INF);

			m256 Z1_p = _mm256_add_ps(p_ord0_z, Z1_e);
			m256 Z2_p = _mm256_add_ps(p_ord0_z, Z2_e);
			m256 Z3_p = _mm256_add_ps(p_ord1_z, Z3_e);

			m256 v0y = _mm256_set1_ps(v0.y);
			m256 v1y = _mm256_set1_ps(v1.y);
			m256 v2y = _mm256_set1_ps(v2.y);

			m256 Z1 = _mm256_blendv_ps(inf_v, Z1_p, valid0);
			m256 Z2 = _mm256_blendv_ps(inf_v, Z2_p, valid1);
			m256 Z3 = _mm256_blendv_ps(inf_v, Z3_p, valid2);

			m256 qx_v0y = _mm256_mul_ps(qx, v0y);
			m256 qx_v1y = _mm256_mul_ps(qx, v1y);
			m256 qx_v2y = _mm256_mul_ps(qx, v2y);

			m256 e0_comp_v = _mm256_set1_ps(e0_comp);
			m256 e1_comp_v = _mm256_set1_ps(e1_comp);
			m256 e2_comp_v = _mm256_set1_ps(e2_comp);

			m256 v0x = _mm256_set1_ps(v0.x);
			m256 v1x = _mm256_set1_ps(v1.x);
			m256 v2x = _mm256_set1_ps(v2.x);

			m256 qx_v0y_e0_comp = _mm256_sub_ps(qx_v0y, e0_comp_v);
			m256 qx_v1y_e1_comp = _mm256_sub_ps(qx_v1y, e1_comp_v);
			m256 qx_v2y_e2_comp = _mm256_sub_ps(qx_v2y, e2_comp_v);

			m256 qy_v0x = _mm256_mul_ps(qy, v0x);
			m256 qy_v1x = _mm256_mul_ps(qy, v1x);
			m256 qy_v2x = _mm256_mul_ps(qy, v2x);

			m256 left_edge_0 = _mm256_cmp_ps(qx_v0y_e0_comp, qy_v0x, _CMP_LT_OQ);
			m256 left_edge_1 = _mm256_cmp_ps(qx_v1y_e1_comp, qy_v1x, _CMP_LT_OQ);
			m256 left_edge_2 = _mm256_cmp_ps(qx_v2y_e2_comp, qy_v2x, _CMP_LT_OQ);

			m256 upwardsz = _mm256_set1_ps(upwards.z);

			m256 inside_triangle_01 = _mm256_and_ps(left_edge_0, left_edge_1);
			m256 inside_triangle = _mm256_and_ps(inside_triangle_01, left_edge_2);

			m256 qz_upwardsz = _mm256_sub_ps(qz, upwardsz);

			m256 Z0 = _mm256_blendv_ps(inf_v, qz_upwardsz, inside_triangle);

			m256 z_result_1 = _mm256_min_ps(Z1, Z2);
			m256 z_result_2 = _mm256_min_ps(Z0, Z3);
			m256 z_result = _mm256_min_ps(z_result_1, z_result_2);

			f32* grid_corners_at_index = &grid_corners[y * grid_width];

			m256 old_bound_v = _mm256_load_ps(grid_corners_at_index);
			m256 grid_corners_new = _mm256_min_ps(z_result, old_bound_v);
			_mm256_store_ps(grid_corners_at_index, grid_corners_new);

			m256 comp_v = _mm256_cmp_ps(z_result, old_bound_v, _CMP_LT_OQ);

			#pragma GCC diagnostic push
			#pragma GCC diagnostic ignored "-Wcast-align"
			m256 y_min_x = _mm256_load_si256(reinterpret_cast<m256*>(&y_min[0]));
			m256 y_max_x = _mm256_load_si256(reinterpret_cast<m256*>(&y_max[0]));
			#pragma GCC diagnostic pop

			m256 y_v = _mm256_set1_epi32(y);
			m256 y_min_x_y = _mm256_min_epi32(y_min_x, y_v);
			m256 y_max_x_y = _mm256_max_epi32(y_max_x, y_v);
			m256 y_min_x_new = _mm256_blendv_ps(y_min_x, y_min_x_y, comp_v);
			m256 y_max_x_new = _mm256_blendv_ps(y_max_x, y_max_x_y, comp_v);

			_mm256_store_ps(reinterpret_cast<f32*>(&y_min[0]), y_min_x_new);
			_mm256_store_ps(reinterpret_cast<f32*>(&y_max[0]), y_max_x_new);

			f32 comp[8] alignas(32);
			_mm256_store_ps(&comp[0], comp_v);
			i32 x_min_y = x_min[y];
			i32 x_max_y = x_max[y];
			#pragma GCC unroll 8
			for (i32 x = 0; x != 8; x++) {
				bool comp_x = bitcast_f32_to_u32(comp[x]);
				i32 x_min_y_x = min_i32(x_min_y, x);
				i32 x_max_y_x = max_i32(x_max_y, x);
				x_min_y = comp_x ? x_min_y_x : x_min_y;
				x_max_y = comp_x ? x_max_y_x : x_max_y;
			}
			x_min[y] = x_min_y;
			x_max[y] = x_max_y;
		}

		//Find the points on the grid edges, using x_min, x_max, y_min, y_max
		//that we calculated earlier.
		f32_4 w01 = p1 - p0;
		f32_4 w12 = p2 - p1;
		f32_4 w20 = p0 - p2;

		f32 w01_len = sqrt_f32(w01.x * w01.x + w01.y * w01.y);
		f32 w12_len = sqrt_f32(w12.x * w12.x + w12.y * w12.y);
		f32 w20_len = sqrt_f32(w20.x * w20.x + w20.y * w20.y);

		//NOTE(cdamerius): again, this won't work for triangles that have no area when projected onto the XY plane
		f32_4 w01_norm = w01 / w01_len;
		f32_4 w12_norm = w12 / w12_len;
		f32_4 w20_norm = w20 / w20_len;

		#pragma GCC unroll 4
		for (i32 m = 0; m != 4; m++) {
			m256 border_in_x;
			m256 border_in_y;
			if (m == 0) {
				#pragma GCC diagnostic push
				#pragma GCC diagnostic ignored "-Wcast-align"
				border_in_x = _mm256_cvtepi32_ps(_mm256_load_si256(reinterpret_cast<m256*>(&x_min[0])));
				border_in_y = _mm256_set_ps(7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f, 0.0f);
				#pragma GCC diagnostic pop
			}
			if (m == 1) {
				#pragma GCC diagnostic push
				#pragma GCC diagnostic ignored "-Wcast-align"
				border_in_x = _mm256_cvtepi32_ps(_mm256_load_si256(reinterpret_cast<m256*>(&x_max[0])));
				#pragma GCC diagnostic pop
				border_in_y = _mm256_set_ps(7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f, 0.0f);
			}
			if (m == 2) {
				border_in_x = _mm256_set_ps(7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f, 0.0f);
				#pragma GCC diagnostic push
				#pragma GCC diagnostic ignored "-Wcast-align"
				border_in_y = _mm256_cvtepi32_ps(_mm256_load_si256(reinterpret_cast<m256*>(&y_min[0])));
				#pragma GCC diagnostic pop
			}
			if (m == 3) {
				border_in_x = _mm256_set_ps(7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f, 0.0f);
				#pragma GCC diagnostic push
				#pragma GCC diagnostic ignored "-Wcast-align"
				border_in_y = _mm256_cvtepi32_ps(_mm256_load_si256(reinterpret_cast<m256*>(&y_max[0])));
				#pragma GCC diagnostic pop
			}

			m256 p0_x = _mm256_set1_ps(p0.x);
			m256 p1_x = _mm256_set1_ps(p1.x);
			m256 p2_x = _mm256_set1_ps(p2.x);

			m256 p0_y = _mm256_set1_ps(p0.y);
			m256 p1_y = _mm256_set1_ps(p1.y);
			m256 p2_y = _mm256_set1_ps(p2.y);

			m256 p0_z = _mm256_set1_ps(p0.z);
			m256 p1_z = _mm256_set1_ps(p1.z);
			m256 p2_z = _mm256_set1_ps(p2.z);

			m256 border_in_x_p0_x = _mm256_sub_ps(border_in_x, p0_x);
			m256 border_in_x_p1_x = _mm256_sub_ps(border_in_x, p1_x);
			m256 border_in_x_p2_x = _mm256_sub_ps(border_in_x, p2_x);

			m256 border_in_y_p0_y = _mm256_sub_ps(border_in_y, p0_y);
			m256 border_in_y_p1_y = _mm256_sub_ps(border_in_y, p1_y);
			m256 border_in_y_p2_y = _mm256_sub_ps(border_in_y, p2_y);

			m256 w01_norm_x = _mm256_set1_ps(w01_norm.x);
			m256 w12_norm_x = _mm256_set1_ps(w12_norm.x);
			m256 w20_norm_x = _mm256_set1_ps(w20_norm.x);

			m256 w01_norm_y = _mm256_set1_ps(w01_norm.y);
			m256 w12_norm_y = _mm256_set1_ps(w12_norm.y);
			m256 w20_norm_y = _mm256_set1_ps(w20_norm.y);

			m256 w01_norm_z = _mm256_set1_ps(w01_norm.z);
			m256 w12_norm_z = _mm256_set1_ps(w12_norm.z);
			m256 w20_norm_z = _mm256_set1_ps(w20_norm.z);

			m256 closest_p_in_0_fac_unc_x = _mm256_mul_ps(w01_norm_x, border_in_x_p0_x);
			m256 closest_p_in_1_fac_unc_x = _mm256_mul_ps(w12_norm_x, border_in_x_p1_x);
			m256 closest_p_in_2_fac_unc_x = _mm256_mul_ps(w20_norm_x, border_in_x_p2_x);

			m256 closest_p_in_0_fac_unc_y = _mm256_mul_ps(w01_norm_y, border_in_y_p0_y);
			m256 closest_p_in_1_fac_unc_y = _mm256_mul_ps(w12_norm_y, border_in_y_p1_y);
			m256 closest_p_in_2_fac_unc_y = _mm256_mul_ps(w20_norm_y, border_in_y_p2_y);

			m256 closest_p_in_0_fac_unc = _mm256_add_ps(closest_p_in_0_fac_unc_x, closest_p_in_0_fac_unc_y);
			m256 closest_p_in_1_fac_unc = _mm256_add_ps(closest_p_in_1_fac_unc_x, closest_p_in_1_fac_unc_y);
			m256 closest_p_in_2_fac_unc = _mm256_add_ps(closest_p_in_2_fac_unc_x, closest_p_in_2_fac_unc_y);

			m256 zero = _mm256_setzero_ps();

			m256 closest_p_in_0_fac_unc_max = _mm256_max_ps(closest_p_in_0_fac_unc, zero);
			m256 closest_p_in_1_fac_unc_max = _mm256_max_ps(closest_p_in_1_fac_unc, zero);
			m256 closest_p_in_2_fac_unc_max = _mm256_max_ps(closest_p_in_2_fac_unc, zero);

			m256 w01_len_v = _mm256_set1_ps(w01_len);
			m256 w12_len_v = _mm256_set1_ps(w12_len);
			m256 w20_len_v = _mm256_set1_ps(w20_len);

			m256 closest_p_in_0_fac = _mm256_min_ps(closest_p_in_0_fac_unc_max, w01_len_v);
			m256 closest_p_in_1_fac = _mm256_min_ps(closest_p_in_1_fac_unc_max, w12_len_v);
			m256 closest_p_in_2_fac = _mm256_min_ps(closest_p_in_2_fac_unc_max, w20_len_v);

			m256 closest_p_in_0_x_linear = _mm256_mul_ps(closest_p_in_0_fac, w01_norm_x);
			m256 closest_p_in_1_x_linear = _mm256_mul_ps(closest_p_in_1_fac, w12_norm_x);
			m256 closest_p_in_2_x_linear = _mm256_mul_ps(closest_p_in_2_fac, w20_norm_x);

			m256 closest_p_in_0_y_linear = _mm256_mul_ps(closest_p_in_0_fac, w01_norm_y);
			m256 closest_p_in_1_y_linear = _mm256_mul_ps(closest_p_in_1_fac, w12_norm_y);
			m256 closest_p_in_2_y_linear = _mm256_mul_ps(closest_p_in_2_fac, w20_norm_y);

			m256 closest_p_in_0_x = _mm256_add_ps(p0_x, closest_p_in_0_x_linear);
			m256 closest_p_in_0_y = _mm256_add_ps(p0_y, closest_p_in_0_y_linear);

			m256 closest_p_in_1_x = _mm256_add_ps(p1_x, closest_p_in_1_x_linear);
			m256 closest_p_in_1_y = _mm256_add_ps(p1_y, closest_p_in_1_y_linear);

			m256 closest_p_in_2_x = _mm256_add_ps(p2_x, closest_p_in_2_x_linear);
			m256 closest_p_in_2_y = _mm256_add_ps(p2_y, closest_p_in_2_y_linear);

			m256 closest_p_in_0_vec_x = _mm256_sub_ps(border_in_x, closest_p_in_0_x);
			m256 closest_p_in_1_vec_x = _mm256_sub_ps(border_in_x, closest_p_in_1_x);
			m256 closest_p_in_2_vec_x = _mm256_sub_ps(border_in_x, closest_p_in_2_x);

			m256 closest_p_in_0_vec_y = _mm256_sub_ps(border_in_y, closest_p_in_0_y);
			m256 closest_p_in_1_vec_y = _mm256_sub_ps(border_in_y, closest_p_in_1_y);
			m256 closest_p_in_2_vec_y = _mm256_sub_ps(border_in_y, closest_p_in_2_y);

			m256 closest_p_in_0_vec_x_sq = _mm256_mul_ps(closest_p_in_0_vec_x, closest_p_in_0_vec_x);
			m256 closest_p_in_1_vec_x_sq = _mm256_mul_ps(closest_p_in_1_vec_x, closest_p_in_1_vec_x);
			m256 closest_p_in_2_vec_x_sq = _mm256_mul_ps(closest_p_in_2_vec_x, closest_p_in_2_vec_x);

			m256 closest_p_in_0_vec_y_sq = _mm256_mul_ps(closest_p_in_0_vec_y, closest_p_in_0_vec_y);
			m256 closest_p_in_1_vec_y_sq = _mm256_mul_ps(closest_p_in_1_vec_y, closest_p_in_1_vec_y);
			m256 closest_p_in_2_vec_y_sq = _mm256_mul_ps(closest_p_in_2_vec_y, closest_p_in_2_vec_y);

			m256 closest_p_in_0_sqdist = _mm256_add_ps(closest_p_in_0_vec_x_sq, closest_p_in_0_vec_y_sq);
			m256 closest_p_in_1_sqdist = _mm256_add_ps(closest_p_in_1_vec_x_sq, closest_p_in_1_vec_y_sq);
			m256 closest_p_in_2_sqdist = _mm256_add_ps(closest_p_in_2_vec_x_sq, closest_p_in_2_vec_y_sq);

			m256 closest_p_in_12_sqdist = _mm256_min_ps(closest_p_in_1_sqdist, closest_p_in_2_sqdist);
			m256 closest_p_in_02_sqdist = _mm256_min_ps(closest_p_in_0_sqdist, closest_p_in_2_sqdist);

			m256 closest_0 = _mm256_cmp_ps(closest_p_in_0_sqdist, closest_p_in_12_sqdist, _CMP_LE_OQ);
			m256 closest_1 = _mm256_cmp_ps(closest_p_in_1_sqdist, closest_p_in_02_sqdist, _CMP_LE_OQ);

			m256 closest_edge_begin_x = _mm256_blendv_ps(p2_x, p1_x, closest_1);
			m256 closest_edge_begin_y = _mm256_blendv_ps(p2_y, p1_y, closest_1);
			m256 closest_edge_begin_z = _mm256_blendv_ps(p2_z, p1_z, closest_1);

			m256 closest_p_in_sqdist = _mm256_blendv_ps(closest_p_in_2_sqdist, closest_p_in_1_sqdist, closest_1);
			m256 closest_edge_len = _mm256_blendv_ps(w20_len_v, w12_len_v, closest_1);

			m256 closest_edge_norm_x = _mm256_blendv_ps(w20_norm_x, w12_norm_x, closest_1);
			m256 closest_edge_norm_y = _mm256_blendv_ps(w20_norm_y, w12_norm_y, closest_1);
			m256 closest_edge_norm_z = _mm256_blendv_ps(w20_norm_z, w12_norm_z, closest_1);

			closest_edge_begin_x = _mm256_blendv_ps(closest_edge_begin_x, p0_x, closest_0);
			closest_edge_begin_y = _mm256_blendv_ps(closest_edge_begin_y, p0_y, closest_0);
			closest_edge_begin_z = _mm256_blendv_ps(closest_edge_begin_z, p0_z, closest_0);
			closest_p_in_sqdist = _mm256_blendv_ps(closest_p_in_sqdist, closest_p_in_0_sqdist, closest_0);
			closest_edge_len = _mm256_blendv_ps(closest_edge_len, w01_len_v, closest_0);
			closest_edge_norm_x = _mm256_blendv_ps(closest_edge_norm_x, w01_norm_x, closest_0);
			closest_edge_norm_y = _mm256_blendv_ps(closest_edge_norm_y, w01_norm_y, closest_0);
			closest_edge_norm_z = _mm256_blendv_ps(closest_edge_norm_z, w01_norm_z, closest_0);

			m256 border_out_x;
			m256 border_out_y;
			m256 one = _mm256_set1_ps(1.0f);
			if (m == 0) {
				border_out_x = _mm256_sub_ps(border_in_x, one);
				border_out_y = border_in_y;
			}
			if (m == 1) {
				border_out_x = _mm256_add_ps(border_in_x, one);
				border_out_y = border_in_y;
			}
			if (m == 2) {
				border_out_x = border_in_x;
				border_out_y = _mm256_sub_ps(border_in_y, one);
			}
			if (m == 3) {
				border_out_x = border_in_x;
				border_out_y = _mm256_add_ps(border_in_y, one);
			}

			m256 closest_p_in_dist = _mm256_sqrt_ps(closest_p_in_sqdist);

			m256 closest_p_out_fac_unc_x_fac = _mm256_sub_ps(border_out_x, closest_edge_begin_x);
			m256 closest_p_out_fac_unc_y_fac = _mm256_sub_ps(border_out_y, closest_edge_begin_y);

			m256 closest_p_out_fac_unc_x = _mm256_mul_ps(closest_edge_norm_x, closest_p_out_fac_unc_x_fac);
			m256 closest_p_out_fac_unc_y = _mm256_mul_ps(closest_edge_norm_y, closest_p_out_fac_unc_y_fac);

			m256 closest_p_out_fac_unc = _mm256_add_ps(closest_p_out_fac_unc_x, closest_p_out_fac_unc_y);
			m256 closest_p_out_fac_unc_max = _mm256_max_ps(closest_p_out_fac_unc, zero);
			m256 closest_p_out_fac = _mm256_min_ps(closest_p_out_fac_unc_max, closest_edge_len);

			m256 closest_p_out_x_linear = _mm256_mul_ps(closest_p_out_fac, closest_edge_norm_x);
			m256 closest_p_out_y_linear = _mm256_mul_ps(closest_p_out_fac, closest_edge_norm_y);

			m256 closest_p_out_z_linear = _mm256_mul_ps(closest_p_out_fac, closest_edge_norm_z);
			m256 closest_p_out_z = _mm256_add_ps(closest_edge_begin_z, closest_p_out_z_linear);

			m256 closest_p_out_vec_x = _mm256_sub_ps(closest_p_out_fac_unc_x_fac, closest_p_out_x_linear);
			m256 closest_p_out_vec_y = _mm256_sub_ps(closest_p_out_fac_unc_y_fac, closest_p_out_y_linear);

			m256 closest_p_out_vec_x_sq = _mm256_mul_ps(closest_p_out_vec_x, closest_p_out_vec_x);
			m256 closest_p_out_vec_y_sq = _mm256_mul_ps(closest_p_out_vec_y, closest_p_out_vec_y);

			m256 closest_p_out_vec_sq = _mm256_add_ps(closest_p_out_vec_x_sq, closest_p_out_vec_y_sq);
			m256 closest_p_out_dist = _mm256_sqrt_ps(closest_p_out_vec_sq);

			m256 grid_radius_v = _mm256_set1_ps(grid_radius);

			m256 t_value_with_radius_dist_unc_num = _mm256_sub_ps(grid_radius_v, closest_p_in_dist);
			m256 t_value_with_radius_dist_unc_denom = _mm256_sub_ps(closest_p_out_dist, closest_p_in_dist);
			m256 t_value_with_radius_dist_unc = _mm256_div_ps(t_value_with_radius_dist_unc_num, t_value_with_radius_dist_unc_denom);

			m256 eps = _mm256_set1_ps(0.001f);
			m256 one_minus_eps = _mm256_sub_ps(one, eps);

			m256 t_value_with_radius_dist_unc_is_nan = _mm256_cmp_ps(t_value_with_radius_dist_unc, t_value_with_radius_dist_unc, _CMP_NEQ_OQ);//only true if NAN
			m256 t_value_with_radius_dist_unc_no_nan = _mm256_blendv_ps(t_value_with_radius_dist_unc, eps, t_value_with_radius_dist_unc_is_nan);
			m256 t_value_with_radius_dist_max = _mm256_max_ps(t_value_with_radius_dist_unc_no_nan, eps);
			m256 t_value_with_radius_dist = _mm256_min_ps(t_value_with_radius_dist_max, one_minus_eps);

			m256 y_new;
			if (m == 0) {
				y_new = _mm256_sub_ps(border_in_x, t_value_with_radius_dist);
			}
			if (m == 1) {
				y_new = _mm256_add_ps(border_in_x, t_value_with_radius_dist);
			}
			if (m == 2) {
				y_new = _mm256_sub_ps(border_in_y, t_value_with_radius_dist);
			}
			if (m == 3) {
				y_new = _mm256_add_ps(border_in_y, t_value_with_radius_dist);
			}

			f32 closest_p_out_z_v[8] alignas(32);
			f32 y_new_v[8] alignas(32);

			_mm256_store_ps(&closest_p_out_z_v[0], closest_p_out_z);
			_mm256_store_ps(&y_new_v[0], y_new);

			for (i32 h = 0; h != grid_width; h++) {
				i32 x;
				i32 y;
				i32 index;
				if (m == 0) {
					x = x_min[h];
					y = h;
					index = (y + 0) * grid_width + (x - 1);
				}
				if (m == 1) {
					x = x_max[h];
					y = h;
					index = (y + 0) * grid_width + (x + 0);
				}
				if (m == 2) {
					x = h;
					y = y_min[h];
					index = (y - 1) * grid_width + (x + 0);
				}
				if (m == 3) {
					x = h;
					y = y_max[h];
					index = (y + 0) * grid_width + (x + 0);
				}

				if (tri_is_ceil == 0) {
					if (m == 0 && closest_p_out_z[h] < floor_z.bound_x[index]) {
						bound_x.floor_t[index] = y_new[h];
						floor_z.bound_x[index] = closest_p_out_z[h];
					}
					if (m == 1 && closest_p_out_z[h] < floor_z.bound_x[index]) {
						bound_x.floor_t[index] = y_new[h];
						floor_z.bound_x[index] = closest_p_out_z[h];
					}
					if (m == 2 && closest_p_out_z[h] < floor_z.bound_y[index]) {
						bound_y.floor_t[index] = y_new[h];
						floor_z.bound_y[index] = closest_p_out_z[h];
					}
					if (m == 3 && closest_p_out_z[h] < floor_z.bound_y[index]) {
						bound_y.floor_t[index] = y_new[h];
						floor_z.bound_y[index] = closest_p_out_z[h];
					}
				} else {
					if (m == 0 && closest_p_out_z[h] < ceil_z.bound_x[index]) {
						bound_x.ceil_t[index] = y_new[h];
						ceil_z.bound_x[index] = closest_p_out_z[h];
					}
					if (m == 1 && closest_p_out_z[h] < ceil_z.bound_x[index]) {
						bound_x.ceil_t[index] = y_new[h];
						ceil_z.bound_x[index] = closest_p_out_z[h];
					}
					if (m == 2 && closest_p_out_z[h] < ceil_z.bound_y[index]) {
						bound_y.ceil_t[index] = y_new[h];
						ceil_z.bound_y[index] = closest_p_out_z[h];
					}
					if (m == 3 && closest_p_out_z[h] < ceil_z.bound_y[index]) {
						bound_y.ceil_t[index] = y_new[h];
						ceil_z.bound_y[index] = closest_p_out_z[h];
					}
				}
			}
		}
	}

	//Check for each grid corner whether the cylinder would be small enough to pass
	//through that corner. If not, that corner is marked as "blocked".
	bool blocked[grid_width * grid_width];
	for (i32 i = 0; i != grid_width * grid_width; i++) {
		blocked[i] = floor_z.grid_corners[i] + ceil_z.grid_corners[i] < 1.0f;
	}

	/*for (i32 i = 0; i != grid_width * grid_width; i++) {
		f32_4 p = {
			(f32(i % grid_width) + trans.x) / scale.x,
			(f32(i / grid_width) + trans.y) / scale.y,
			0.0f,
			1.0f
		};
		draw_debug_point(p, blocked[i]);
	}*/

	//Calculate non-blocked XY area. For each grid edge that separates a blocked
	//from a non-blocked corner, we calculate the corresponding x- or y-value up
	//to which it is blocked and store them in edge_x/y. This makes use of the
	//information we got for the z values at the grid corners, as well as the
	//points we determined for the grid edges.
	f32 edge_x[grid_width * grid_width] alignas(32);
	f32 edge_y[grid_width * grid_width] alignas(32);

	for (i32 j = 0; j != 2; j++) {
		for (i32 y = 0; y != grid_width; y++) {
			i32 index = 8 * y;
			f32* grid_corner_floor = &floor_z.grid_corners[index];
			f32* grid_corner_ceil = &ceil_z.grid_corners[index];

			m256 floor_0 = _mm256_load_ps(&floor_z.grid_corners[index]);
			m256 ceil_0 = _mm256_load_ps(&ceil_z.grid_corners[index]);

			m256 floor_t;
			m256 floor_z_v;
			m256 ceil_t;
			m256 ceil_z_v;
			m256 floor_1;
			m256 ceil_1;
			m256 u;
			m256 u1;
			if (j == 0) {
				m256 shuffle = _mm256_set_epi32(0, 7, 6, 5, 4, 3, 2, 1);
				floor_t = _mm256_load_ps(&bound_x.floor_t[index]);
				floor_z_v = _mm256_load_ps(&floor_z.bound_x[index]);
				ceil_t = _mm256_load_ps(&bound_x.ceil_t[index]);
				ceil_z_v = _mm256_load_ps(&ceil_z.bound_x[index]);
				floor_1 = _mm256_permutevar8x32_ps(floor_0, shuffle);
				ceil_1 = _mm256_permutevar8x32_ps(ceil_0, shuffle);
				u = _mm256_set_ps(7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f, 0.0f);
				u1 = _mm256_set_ps(8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f);
			} else {
				floor_t = _mm256_load_ps(&bound_y.floor_t[index]);
				floor_z_v = _mm256_load_ps(&floor_z.bound_y[index]);
				ceil_t = _mm256_load_ps(&bound_y.ceil_t[index]);
				ceil_z_v = _mm256_load_ps(&ceil_z.bound_y[index]);
				floor_1 = _mm256_load_ps(&floor_z.grid_corners[index + grid_width]);
				ceil_1 = _mm256_load_ps(&ceil_z.grid_corners[index + grid_width]);
				u = _mm256_set1_ps(f32(y));
				u1 = _mm256_set1_ps(f32(y + 1));
			}

			m256 neg_low_floor_v = _mm256_set1_ps(neg_low_floor);

			m256 floor_0_is_neg_low_floor = _mm256_cmp_ps(floor_0, neg_low_floor_v, _CMP_EQ_OQ);
			m256 floor_1_is_neg_low_floor = _mm256_cmp_ps(floor_1, neg_low_floor_v, _CMP_EQ_OQ);

			m256 floor_sl_0_case = _mm256_andnot_ps(floor_1_is_neg_low_floor, floor_0_is_neg_low_floor);
			m256 floor_sl_1_case = _mm256_andnot_ps(floor_0_is_neg_low_floor, floor_1_is_neg_low_floor);

			m256 floor_sl_t0 = _mm256_blendv_ps(u, floor_t, floor_sl_0_case);
			m256 floor_sl_z0 = _mm256_blendv_ps(floor_0, floor_z_v, floor_sl_0_case);
			m256 floor_sl_t1 = _mm256_blendv_ps(u1, floor_t, floor_sl_1_case);
			m256 floor_sl_z1 = _mm256_blendv_ps(floor_1, floor_z_v, floor_sl_1_case);

			m256 high_ceil_v = _mm256_set1_ps(high_ceil);

			m256 ceil_0_is_high_ceil = _mm256_cmp_ps(ceil_0, high_ceil_v, _CMP_EQ_OQ);
			m256 ceil_1_is_high_ceil = _mm256_cmp_ps(ceil_1, high_ceil_v, _CMP_EQ_OQ);

			m256 ceil_sl_0_case = _mm256_andnot_ps(ceil_1_is_high_ceil, ceil_0_is_high_ceil);
			m256 ceil_sl_1_case = _mm256_andnot_ps(ceil_0_is_high_ceil, ceil_1_is_high_ceil);

			m256 ceil_sl_t0 = _mm256_blendv_ps(u, ceil_t, ceil_sl_0_case);
			m256 ceil_sl_z0 = _mm256_blendv_ps(ceil_0, ceil_z_v, ceil_sl_0_case);
			m256 ceil_sl_t1 = _mm256_blendv_ps(u1, ceil_t, ceil_sl_1_case);
			m256 ceil_sl_z1 = _mm256_blendv_ps(ceil_1, ceil_z_v, ceil_sl_1_case);

			m256 floor_z_diff = _mm256_sub_ps(floor_sl_z1, floor_sl_z0);
			m256 floor_t_diff = _mm256_sub_ps(floor_sl_t1, floor_sl_t0);

			m256 ceil_z_diff = _mm256_sub_ps(ceil_sl_z1, ceil_sl_z0);
			m256 ceil_t_diff = _mm256_sub_ps(ceil_sl_t1, ceil_sl_t0);

			m256 floor_sl_m = _mm256_div_ps(floor_z_diff, floor_t_diff);
			m256 ceil_sl_m = _mm256_div_ps(ceil_z_diff, ceil_t_diff);
			m256 d_sl_m = _mm256_add_ps(ceil_sl_m, floor_sl_m);

			m256 sl_z0 = _mm256_add_ps(floor_sl_z0, ceil_sl_z0);
			m256 ceil_sl = _mm256_mul_ps(ceil_sl_t0, ceil_sl_m);
			m256 floor_sl = _mm256_mul_ps(floor_sl_t0, floor_sl_m);

			m256 cf_sl = _mm256_add_ps(ceil_sl, floor_sl);
			m256 one = _mm256_set1_ps(1.0f);
			m256 sl_z0_1 = _mm256_sub_ps(one, sl_z0);
			m256 t_value_num = _mm256_add_ps(sl_z0_1, cf_sl);

			m256 t_value_0 = _mm256_div_ps(t_value_num, d_sl_m);
			m256 t_lower = _mm256_max_ps(floor_sl_t0, ceil_sl_t0);
			m256 t_upper = _mm256_min_ps(floor_sl_t1, ceil_sl_t1);

			m256 cf_0 = _mm256_add_ps(ceil_0, floor_0);
			m256 cf_1 = _mm256_add_ps(ceil_1, floor_1);

			m256 t_boundary_comp = _mm256_cmp_ps(cf_0, cf_1, _CMP_LT_OQ);
			m256 t_boundary = _mm256_blendv_ps(t_lower, t_upper, t_boundary_comp);

			m256 zero = _mm256_set1_ps(0.0f);
			m256 d_sl_m_neg = _mm256_cmp_ps(d_sl_m, zero, _CMP_LT_OQ);
			m256 d_sl_m_eq = _mm256_cmp_ps(d_sl_m, zero, _CMP_EQ_OQ);

			m256 t_comp = _mm256_blendv_ps(t_lower, t_upper, d_sl_m_neg);
			m256 t_left = _mm256_min_ps(t_value_0, t_boundary);
			m256 t_right = _mm256_max_ps(t_value_0, t_boundary);

			m256 left = _mm256_cmp_ps(t_comp, t_value_0, _CMP_LT_OQ);
			m256 t_nonzero_slope = _mm256_blendv_ps(t_right, t_left, left);
			m256 t_value_1 = _mm256_blendv_ps(t_nonzero_slope, t_boundary, d_sl_m_eq);

			m256 eps = _mm256_set1_ps(0.001f);
			m256 u_eps = _mm256_add_ps(u, eps);
			m256 u1_m_eps = _mm256_sub_ps(u1, eps);

			//NOTE(cdamerius):
			//This is to ensure that all edges that delimit the non-blocked
			//XY area have at least some eps length. This is to avoid degeneracy
			//when calculating the closest edges to the target in the next step.
			m256 t_value_2 = _mm256_max_ps(t_value_1, u_eps);
			m256 t_value = _mm256_min_ps(t_value_2, u1_m_eps);

			if (j == 0) {
				_mm256_store_ps(&edge_x[index], t_value);
			} else {
				_mm256_store_ps(&edge_y[index], t_value);
			}
		}
	}

	//Calculate for each grid cell a set of edges that delimit the non-blocked
	//XY area. We want to have the closest point on all of these edges to the target.
	//That point will be our candidate XY position where the cylinder can go to.

	//blocked_edge_connector_indices is used to look up what delimiter edges to
	//create based on the blocked value of the corners of a grid cell.
	enum Dir {
		dir__,
		dir_W,
		dir_S,
		dir_E,
		dir_N
	} blocked_edge_connector_indices[64] = {
		dir__, dir__, dir__, dir__,
		dir_W, dir_S, dir__, dir__,
		dir_S, dir_E, dir__, dir__,
		dir_W, dir_E, dir__, dir__,
		dir_N, dir_W, dir__, dir__,
		dir_N, dir_S, dir__, dir__,
		dir_S, dir_W, dir_N, dir_E,
		dir_N, dir_E, dir__, dir__,
		dir_E, dir_N, dir__, dir__,
		dir_W, dir_N, dir_E, dir_S,
		dir_S, dir_N, dir__, dir__,
		dir_W, dir_N, dir__, dir__,
		dir_E, dir_W, dir__, dir__,
		dir_E, dir_S, dir__, dir__,
		dir_S, dir_W, dir__, dir__,
		dir__, dir__, dir__, dir__
	};

	f32 closest_dist_sq = F32_INF;
	f32 closest_point_x;
	f32 closest_point_y;
	i32 closest_index;
	f32 closest_cross;
	f32 closest_edge_vec_x;
	f32 closest_edge_vec_y;

	f32 blocked_edge_connectors[10];
	blocked_edge_connectors[0] = F32_NAN_0;
	blocked_edge_connectors[1] = F32_NAN_0;

	for (i32 y = 0; y != grid_width - 1; y++) {
		for (i32 x = 0; x != grid_width - 1; x++) {
			f32 xv = f32(x);
			f32 yv = f32(y);

			blocked_edge_connectors[2] = xv;
			blocked_edge_connectors[3] = edge_y[y * grid_width + x];

			blocked_edge_connectors[4] = edge_x[y * grid_width + x];
			blocked_edge_connectors[5] = yv;

			blocked_edge_connectors[6] = xv + 1.0f;
			blocked_edge_connectors[7] = edge_y[y * grid_width + (x + 1)];

			blocked_edge_connectors[8] = edge_x[(y + 1) * grid_width + x];
			blocked_edge_connectors[9] = yv + 1.0f;

			bool blocked_00 = blocked[(y + 0) * grid_width + (x + 0)];
			bool blocked_10 = blocked[(y + 0) * grid_width + (x + 1)];
			bool blocked_01 = blocked[(y + 1) * grid_width + (x + 0)];
			bool blocked_11 = blocked[(y + 1) * grid_width + (x + 1)];
			i32 idx = 4 * i32(blocked_00) + 8 * i32(blocked_10) + 16 * i32(blocked_01) + 32 * i32(blocked_11);
			Dir d0 = blocked_edge_connector_indices[idx + 0];
			Dir d1 = blocked_edge_connector_indices[idx + 1];
			Dir d2 = blocked_edge_connector_indices[idx + 2];
			Dir d3 = blocked_edge_connector_indices[idx + 3];

			f32 s0_x = blocked_edge_connectors[2 * d0 + 0];
			f32 s0_y = blocked_edge_connectors[2 * d0 + 1];
			f32 e0_x = blocked_edge_connectors[2 * d1 + 0];
			f32 e0_y = blocked_edge_connectors[2 * d1 + 1];
			f32 s1_x = blocked_edge_connectors[2 * d2 + 0];
			f32 s1_y = blocked_edge_connectors[2 * d2 + 1];
			f32 e1_x = blocked_edge_connectors[2 * d3 + 0];
			f32 e1_y = blocked_edge_connectors[2 * d3 + 1];

			f32 v0_x = e0_x - s0_x;
			f32 v0_y = e0_y - s0_y;
			f32 v1_x = e1_x - s1_x;
			f32 v1_y = e1_y - s1_y;
			f32 w0_x = grid_target.x - s0_x;
			f32 w0_y = grid_target.y - s0_y;
			f32 w1_x = grid_target.x - s1_x;
			f32 w1_y = grid_target.y - s1_y;

			f32 dot_w0_v0 = w0_x * v0_x + w0_y * v0_y;
			f32 a0 = clamp_f32(dot_w0_v0 / (v0_x * v0_x + v0_y * v0_y), 0.0f, 1.0f);
			if (is_nan(a0)) {
				a0 = 0.0f;
			}
			f32 q0_x = s0_x + a0 * v0_x;
			f32 q0_y = s0_y + a0 * v0_y;
			f32 q0_vec_x = q0_x - grid_target.x;
			f32 q0_vec_y = q0_y - grid_target.y;
			f32 q0_dist_sq = q0_vec_x * q0_vec_x + q0_vec_y * q0_vec_y;
			f32 cr0 = w0_x * v0_y - w0_y * v0_x;

			f32 dot_w1_v1 = w1_x * v1_x + w1_y * v1_y;
			f32 a1 = clamp_f32(dot_w1_v1 / (v1_x * v1_x + v1_y * v1_y), 0.0f, 1.0f);
			if (is_nan(a1)) {
				a1 = 0.0f;
			}
			f32 q1_x = s1_x + a1 * v1_x;
			f32 q1_y = s1_y + a1 * v1_y;
			f32 q1_vec_x = q1_x - grid_target.x;
			f32 q1_vec_y = q1_y - grid_target.y;
			f32 q1_dist_sq = q1_vec_x * q1_vec_x + q1_vec_y * q1_vec_y;
			f32 cr1 = w1_x * v1_y - w1_y * v1_x;

			if (d0 != dir__) {
				/*f32_4 l0 = {
					(s0_x + trans.x) / scale.x,
					(s0_y + trans.y) / scale.y,
					0.0f,
					1.0f
				};
				f32_4 l1 = {
					(e0_x + trans.x) / scale.x,
					(e0_y + trans.y) / scale.y,
					0.0f,
					1.0f
				};
				draw_debug_line(l0, l1);*/
				if (q0_dist_sq < closest_dist_sq) {
					closest_dist_sq = q0_dist_sq;
					closest_point_x = q0_x;
					closest_point_y = q0_y;
					closest_index = y * grid_width + x;
					closest_cross = cr0;
					closest_edge_vec_x = v0_x;
					closest_edge_vec_y = v0_y;
				}
			}

			if (d2 != dir__) {
				/*f32_4 l0 = {
					(s1_x + trans.x) / scale.x,
					(s1_y + trans.y) / scale.y,
					0.0f,
					1.0f
				};
				f32_4 l1 = {
					(e1_x + trans.x) / scale.x,
					(e1_y + trans.y) / scale.y,
					0.0f,
					1.0f
				};
				draw_debug_line(l0, l1);*/
				if (q1_dist_sq < closest_dist_sq) {
					closest_dist_sq = q1_dist_sq;
					closest_point_x = q1_x;
					closest_point_y = q1_y;
					closest_index = y * grid_width + x;
					closest_cross = cr1;
					closest_edge_vec_x = v1_x;
					closest_edge_vec_y = v1_y;
				}
			}
		}
	}

	Resolve_Collision_Result result;

	constexpr const i32 central_index = (grid_width / 2 - 1) * (grid_width + 1);
	bool closest_point_side = closest_cross <= 0.0f;
	if (closest_dist_sq == F32_INF) {
		//There are no delimiter edges! Whether we can find a new cylinder
		//position depends on whether any grid corner is blocked.
		result.found_new_pos = !blocked[central_index];
	} else {
		//Only accept the new position if it is not too far away.
		//If closeest_point_side == true, then the target is in the
		//area where the cylinder can already be placed.
		result.found_new_pos = closest_dist_sq < grid_radius_sq || closest_point_side;
	}
	if (!result.found_new_pos) {
		return result;
	}

	//Now we determine the z value of the solution position.
	//For that we triangulate the blocked/unblocked parts of a cell, determine
	//which of these triangles we are in, and fetch the z value of that triangle. This
	//is done once for the floor and once for the ceiling.
	i32 solution_index;

	enum Dir_8: u8 {
		dir_8_NW,
		dir_8_NE,
		dir_8_SW,
		dir_8_SE,
		dir_8__N,
		dir_8__W,
		dir_8__E,
		dir_8__S,
		dir_8_count
	};

	bool dont_perturb_wall = closest_dist_sq == F32_INF || closest_point_side;

	f32 closest_point_perturbed_x;
	f32 closest_point_perturbed_y;

	if (dont_perturb_wall) {
		closest_point_x = target.x * scale.x - trans.x;
		closest_point_y = target.y * scale.y - trans.y;
		closest_point_perturbed_x = closest_point_x;
		closest_point_perturbed_y = closest_point_y;
		solution_index = central_index;
	} else {
		f32 anorm_eps = 0.001f / sqrt_f32(closest_edge_vec_x * closest_edge_vec_x + closest_edge_vec_y * closest_edge_vec_y);
		closest_point_perturbed_x = closest_point_x - closest_edge_vec_y * anorm_eps;
		closest_point_perturbed_y = closest_point_y + closest_edge_vec_x * anorm_eps;
		solution_index = closest_index;
		f32 a_00 = floor_z.grid_corners[closest_index + 0 * grid_width + 0];
		f32 a_10 = floor_z.grid_corners[closest_index + 0 * grid_width + 1];
		f32 a_01 = floor_z.grid_corners[closest_index + 1 * grid_width + 0];
		f32 a_11 = floor_z.grid_corners[closest_index + 1 * grid_width + 1];
	}

	i32 sol_x = solution_index % grid_width;
	i32 sol_y = solution_index / grid_width;

	struct Index_Triangle {
		Dir_8 t0;
		Dir_8 t1;
		Dir_8 t2;
		bool t_ex;
	};

	struct Index_Triangle_Table {
		Dir_8 d[10];
		Index_Triangle t[6];
	} triangles[16] = {
		{{dir_8_SW, dir_8_NE, dir_8_SW, dir_8_NE, dir_8_SW, dir_8_NE, dir_8_SW, dir_8_NE, dir_8_SW, dir_8_NE}, Index_Triangle{dir_8_SW, dir_8_NE, dir_8_NW, false}, Index_Triangle{dir_8_SW, dir_8_NE, dir_8_NW, false}, Index_Triangle{dir_8_SW, dir_8_NE, dir_8_NW, false}, Index_Triangle{dir_8_SW, dir_8_NE, dir_8_NW, false}, Index_Triangle{dir_8_SW, dir_8_SE, dir_8_NE, false}, Index_Triangle{dir_8_SW, dir_8_SE, dir_8_NE, false}},
		{{dir_8__S, dir_8_NW, dir_8__S, dir_8__W, dir_8__S, dir_8__W, dir_8__S, dir_8__W, dir_8__S, dir_8_NE}, Index_Triangle{dir_8__S, dir_8__W, dir_8_SW, true }, Index_Triangle{dir_8__S, dir_8__W, dir_8_SW, true }, Index_Triangle{dir_8__S, dir_8_NW, dir_8__W, false}, Index_Triangle{dir_8__S, dir_8_NW, dir_8__W, false}, Index_Triangle{dir_8__S, dir_8_NE, dir_8_NW, false}, Index_Triangle{dir_8__S, dir_8_SE, dir_8_NE, false}},
		{{dir_8__E, dir_8_SW, dir_8__E, dir_8__S, dir_8__E, dir_8__S, dir_8__E, dir_8__S, dir_8__E, dir_8_NW}, Index_Triangle{dir_8__E, dir_8__S, dir_8_SE, true }, Index_Triangle{dir_8__E, dir_8__S, dir_8_SE, true }, Index_Triangle{dir_8__E, dir_8_SW, dir_8__S, false}, Index_Triangle{dir_8__E, dir_8_SW, dir_8__S, false}, Index_Triangle{dir_8__E, dir_8_NW, dir_8_SW, false}, Index_Triangle{dir_8__E, dir_8_NE, dir_8_NW, false}},
		{{dir_8__E, dir_8__W, dir_8_SE, dir_8__W, dir_8_SE, dir_8__W, dir_8_SE, dir_8__W, dir_8__E, dir_8_NW}, Index_Triangle{dir_8__W, dir_8_SW, dir_8_SE, true }, Index_Triangle{dir_8__W, dir_8_SW, dir_8_SE, true }, Index_Triangle{dir_8__W, dir_8_SE, dir_8__E, true }, Index_Triangle{dir_8__W, dir_8_SE, dir_8__E, true }, Index_Triangle{dir_8__E, dir_8_NW, dir_8__W, false}, Index_Triangle{dir_8__E, dir_8_NE, dir_8_NW, false}},
		{{dir_8__W, dir_8_NE, dir_8__W, dir_8__N, dir_8__W, dir_8__N, dir_8__W, dir_8__N, dir_8__W, dir_8_SE}, Index_Triangle{dir_8__W, dir_8__N, dir_8_NW, true }, Index_Triangle{dir_8__W, dir_8__N, dir_8_NW, true }, Index_Triangle{dir_8__W, dir_8_NE, dir_8__N, false}, Index_Triangle{dir_8__W, dir_8_NE, dir_8__N, false}, Index_Triangle{dir_8__W, dir_8_SE, dir_8_NE, false}, Index_Triangle{dir_8__W, dir_8_SW, dir_8_SE, false}},
		{{dir_8__S, dir_8__N, dir_8_SW, dir_8__N, dir_8_SW, dir_8__N, dir_8_SW, dir_8__N, dir_8__S, dir_8_NE}, Index_Triangle{dir_8_SW, dir_8__N, dir_8_NW, true }, Index_Triangle{dir_8_SW, dir_8__N, dir_8_NW, true }, Index_Triangle{dir_8_SW, dir_8__S, dir_8__N, true }, Index_Triangle{dir_8_SW, dir_8__S, dir_8__N, true }, Index_Triangle{dir_8__S, dir_8_NE, dir_8__N, false}, Index_Triangle{dir_8__S, dir_8_SE, dir_8_NE, false}},
		{{dir_8_SE, dir_8__N, dir_8__S, dir_8_NW, dir_8__S, dir_8__W, dir_8__S, dir_8__N, dir_8__E, dir_8__N}, Index_Triangle{dir_8__S, dir_8__W, dir_8_SW, false}, Index_Triangle{dir_8__S, dir_8_NW, dir_8__W, true }, Index_Triangle{dir_8__S, dir_8__N, dir_8_NW, true }, Index_Triangle{dir_8__S, dir_8_SE, dir_8__N, true }, Index_Triangle{dir_8__E, dir_8__N, dir_8_SE, true }, Index_Triangle{dir_8__E, dir_8_NE, dir_8__N, false}},
		{{dir_8__N, dir_8_SE, dir_8__N, dir_8__E, dir_8__N, dir_8__E, dir_8__N, dir_8__E, dir_8__N, dir_8_SW}, Index_Triangle{dir_8__N, dir_8__E, dir_8_NE, false}, Index_Triangle{dir_8__N, dir_8__E, dir_8_NE, false}, Index_Triangle{dir_8__N, dir_8_SE, dir_8__E, true }, Index_Triangle{dir_8__N, dir_8_SE, dir_8__E, true }, Index_Triangle{dir_8__N, dir_8_SW, dir_8_SE, true }, Index_Triangle{dir_8__N, dir_8_NW, dir_8_SW, true }},
		{{dir_8__N, dir_8_SE, dir_8__N, dir_8__E, dir_8__N, dir_8__E, dir_8__N, dir_8__E, dir_8__N, dir_8_SW}, Index_Triangle{dir_8__N, dir_8__E, dir_8_NE, true }, Index_Triangle{dir_8__N, dir_8__E, dir_8_NE, true }, Index_Triangle{dir_8__N, dir_8_SE, dir_8__E, false}, Index_Triangle{dir_8__N, dir_8_SE, dir_8__E, false}, Index_Triangle{dir_8__N, dir_8_SW, dir_8_SE, false}, Index_Triangle{dir_8__N, dir_8_NW, dir_8_SW, false}},
		{{dir_8_SW, dir_8__E, dir_8__W, dir_8_NE, dir_8__W, dir_8__N, dir_8__W, dir_8__E, dir_8__S, dir_8__E}, Index_Triangle{dir_8__W, dir_8__N, dir_8_NW, false}, Index_Triangle{dir_8__W, dir_8_NE, dir_8__N, true }, Index_Triangle{dir_8__W, dir_8__E, dir_8_NE, true }, Index_Triangle{dir_8__W, dir_8_SW, dir_8__E, true }, Index_Triangle{dir_8_SW, dir_8__S, dir_8__E, true }, Index_Triangle{dir_8__S, dir_8_SE, dir_8__E, false}},
		{{dir_8__S, dir_8__N, dir_8_SW, dir_8__N, dir_8_SW, dir_8__N, dir_8_SW, dir_8__N, dir_8__S, dir_8_NE}, Index_Triangle{dir_8_SW, dir_8__N, dir_8_NW, false}, Index_Triangle{dir_8_SW, dir_8__N, dir_8_NW, false}, Index_Triangle{dir_8_SW, dir_8__S, dir_8__N, false}, Index_Triangle{dir_8_SW, dir_8__S, dir_8__N, false}, Index_Triangle{dir_8__S, dir_8_NE, dir_8__N, true }, Index_Triangle{dir_8__S, dir_8_SE, dir_8_NE, true }},
		{{dir_8__W, dir_8_NE, dir_8__W, dir_8__N, dir_8__W, dir_8__N, dir_8__W, dir_8__N, dir_8__W, dir_8_SE}, Index_Triangle{dir_8__W, dir_8__N, dir_8_NW, false}, Index_Triangle{dir_8__W, dir_8__N, dir_8_NW, false}, Index_Triangle{dir_8__W, dir_8_NE, dir_8__N, true }, Index_Triangle{dir_8__W, dir_8_NE, dir_8__N, true }, Index_Triangle{dir_8__W, dir_8_SE, dir_8_NE, true }, Index_Triangle{dir_8__W, dir_8_SW, dir_8_SE, true }},
		{{dir_8__E, dir_8__W, dir_8_SE, dir_8__W, dir_8_SE, dir_8__W, dir_8_SE, dir_8__W, dir_8__E, dir_8_NW}, Index_Triangle{dir_8__W, dir_8_SW, dir_8_SE, false}, Index_Triangle{dir_8__W, dir_8_SW, dir_8_SE, false}, Index_Triangle{dir_8__W, dir_8_SE, dir_8__E, false}, Index_Triangle{dir_8__W, dir_8_SE, dir_8__E, false}, Index_Triangle{dir_8__E, dir_8_NW, dir_8__W, true }, Index_Triangle{dir_8__E, dir_8_NE, dir_8_NW, true }},
		{{dir_8__E, dir_8_SW, dir_8__E, dir_8__S, dir_8__E, dir_8__S, dir_8__E, dir_8__S, dir_8__E, dir_8_NW}, Index_Triangle{dir_8__E, dir_8__S, dir_8_SE, false}, Index_Triangle{dir_8__E, dir_8__S, dir_8_SE, false}, Index_Triangle{dir_8__E, dir_8_SW, dir_8__S, true }, Index_Triangle{dir_8__E, dir_8_SW, dir_8__S, true }, Index_Triangle{dir_8__E, dir_8_NW, dir_8_SW, true }, Index_Triangle{dir_8__E, dir_8_NE, dir_8_NW, true }},
		{{dir_8__S, dir_8_NW, dir_8__S, dir_8__W, dir_8__S, dir_8__W, dir_8__S, dir_8__W, dir_8__S, dir_8_NE}, Index_Triangle{dir_8__S, dir_8__W, dir_8_SW, false}, Index_Triangle{dir_8__S, dir_8__W, dir_8_SW, false}, Index_Triangle{dir_8__S, dir_8_NW, dir_8__W, true }, Index_Triangle{dir_8__S, dir_8_NW, dir_8__W, true }, Index_Triangle{dir_8__S, dir_8_NE, dir_8_NW, true }, Index_Triangle{dir_8__S, dir_8_SE, dir_8_NE, true }},
		{{dir_8_SW, dir_8_NE, dir_8_SW, dir_8_NE, dir_8_SW, dir_8_NE, dir_8_SW, dir_8_NE, dir_8_SW, dir_8_NE}, Index_Triangle{dir_8_SW, dir_8_NE, dir_8_NW, true }, Index_Triangle{dir_8_SW, dir_8_NE, dir_8_NW, true }, Index_Triangle{dir_8_SW, dir_8_NE, dir_8_NW, true }, Index_Triangle{dir_8_SW, dir_8_NE, dir_8_NW, true }, Index_Triangle{dir_8_SW, dir_8_SE, dir_8_NE, true }, Index_Triangle{dir_8_SW, dir_8_SE, dir_8_NE, true }}
	};
	i32 t_left_idx_table[32] = {5, 3, 5, 1, 5, 3, 5, 0, 5, 2, 5, 1, 5, 2, 5, 0, 4, 3, 4, 1, 4, 3, 4, 0, 4, 2, 4, 1, 4, 2, 4, 0};

	f32 z_bound_ws[2];

	f32 sol_x_0 = f32(sol_x + 0);
	f32 sol_y_0 = f32(sol_y + 0);
	f32 sol_x_1 = f32(sol_x + 1);
	f32 sol_y_1 = f32(sol_y + 1);

	i32 si_00 = solution_index + 0 * grid_width + 0;
	i32 si_10 = solution_index + 0 * grid_width + 1;
	i32 si_01 = solution_index + 1 * grid_width + 0;
	i32 si_11 = solution_index + 1 * grid_width + 1;

	for (i32 i = 0; i != 2; i++) {
		f32_4 bound_z[dir_8_count];
		if (i == 0) {
			bound_z[dir_8_NW] = {sol_x_0, sol_y_1, floor_z.grid_corners[si_01], 1.0f};
			bound_z[dir_8_NE] = {sol_x_1, sol_y_1, floor_z.grid_corners[si_11], 1.0f};
			bound_z[dir_8_SW] = {sol_x_0, sol_y_0, floor_z.grid_corners[si_00], 1.0f};
			bound_z[dir_8_SE] = {sol_x_1, sol_y_0, floor_z.grid_corners[si_10], 1.0f};
			bound_z[dir_8__N] = {bound_x.floor_t[si_01], sol_y_1, floor_z.bound_x[si_01], 1.0f};
			bound_z[dir_8__W] = {sol_x_0, bound_y.floor_t[si_00], floor_z.bound_y[si_00], 1.0f};
			bound_z[dir_8__E] = {sol_x_1, bound_y.floor_t[si_10], floor_z.bound_y[si_10], 1.0f};
			bound_z[dir_8__S] = {bound_x.floor_t[si_00], sol_y_0, floor_z.bound_x[si_00], 1.0f};
		} else {
			bound_z[dir_8_NW] = {sol_x_0, sol_y_1, ceil_z.grid_corners[si_01], 1.0f};
			bound_z[dir_8_NE] = {sol_x_1, sol_y_1, ceil_z.grid_corners[si_11], 1.0f};
			bound_z[dir_8_SW] = {sol_x_0, sol_y_0, ceil_z.grid_corners[si_00], 1.0f};
			bound_z[dir_8_SE] = {sol_x_1, sol_y_0, ceil_z.grid_corners[si_10], 1.0f};
			bound_z[dir_8__N] = {bound_x.ceil_t[si_01], sol_y_1, ceil_z.bound_x[si_01], 1.0f};
			bound_z[dir_8__W] = {sol_x_0, bound_y.ceil_t[si_00], ceil_z.bound_y[si_00], 1.0f};
			bound_z[dir_8__E] = {sol_x_1, bound_y.ceil_t[si_10], ceil_z.bound_y[si_10], 1.0f};
			bound_z[dir_8__S] = {bound_x.ceil_t[si_00], sol_y_0, ceil_z.bound_x[si_00], 1.0f};
		}

		f32 limit = i == 0 ? neg_low_floor : high_ceil;
		bool bound_00_ex;
		bool bound_10_ex;
		bool bound_01_ex;
		bool bound_11_ex;
		if (i == 0) {
			bound_00_ex = floor_z.grid_corners[si_00] != limit;
			bound_10_ex = floor_z.grid_corners[si_10] != limit;
			bound_01_ex = floor_z.grid_corners[si_01] != limit;
			bound_11_ex = floor_z.grid_corners[si_11] != limit;
		} else {
			bound_00_ex = ceil_z.grid_corners[si_00] != limit;
			bound_10_ex = ceil_z.grid_corners[si_10] != limit;
			bound_01_ex = ceil_z.grid_corners[si_01] != limit;
			bound_11_ex = ceil_z.grid_corners[si_11] != limit;
		}
		i32 bound_index = i32(bound_00_ex) + 2 * i32(bound_10_ex) + 4 * i32(bound_01_ex) + 8 * i32(bound_11_ex);

		Index_Triangle_Table* index_triangle_table = &triangles[bound_index];

		bool t_left[5];
		for (i32 j = 0; j != 5; j++) {
			f32_4 s = bound_z[index_triangle_table->d[2 * j + 0]];
			f32_4 e = bound_z[index_triangle_table->d[2 * j + 1]];
			t_left[j] = (closest_point_perturbed_x - s.x) * (e.y - s.y) - (closest_point_perturbed_y - s.y) * (e.x - s.x) < 0.0f;
		}

		i32 t_left_idx = 16 * i32(t_left[4]) + 8 * i32(t_left[3]) + 4 * i32(t_left[2]) + 2 * i32(t_left[1]) + i32(t_left[0]);
		i32 index_triangle_idx = t_left_idx_table[t_left_idx];
		Index_Triangle index_triangle = index_triangle_table->t[index_triangle_idx];
		f32_4 tri_p0 = bound_z[index_triangle.t0];
		f32_4 tri_p1 = bound_z[index_triangle.t1];
		f32_4 tri_p2 = bound_z[index_triangle.t2];

		f32_4 tri_v1 = {tri_p1.x - tri_p0.x, tri_p1.y - tri_p0.y, tri_p1.z - tri_p0.z, 0.0f};
		f32_4 tri_v2 = {tri_p2.x - tri_p0.x, tri_p2.y - tri_p0.y, tri_p2.z - tri_p0.z, 0.0f};

		f32 cr = tri_v1.x * tri_v2.y - tri_v1.y * tri_v2.x;
		f32 w_x = (closest_point_x - tri_p0.x) / cr;
		f32 w_y = (closest_point_y - tri_p0.y) / cr;
		f32 t_x_unc = tri_v1.x * w_y - tri_v1.y * w_x;
		f32 t_x = clamp_f32(t_x_unc, 0.0f, 1.0f);
		if (is_nan(t_x)) {
			t_x = 0.0f;
		}
		f32 t_y_unc = w_x * tri_v2.y - w_y * tri_v2.x;
		f32 t_y = clamp_f32(t_y_unc, 0.0f, 1.0f - t_x);
		f32 z_tri = t_x * tri_v2.z + t_y * tri_v1.z + tri_p0.z;

		bool choose_z_tri = index_triangle.t_ex && tri_p0.z != limit && tri_p1.z != limit && tri_p2.z != limit;
		f32 z_side = choose_z_tri ? z_tri : limit;

		if (i == 0) {
			z_side = -z_side;
		}

		f32 z_side_ws = (z_side + trans.z) / scale.z;

		z_bound_ws[i] = z_side_ws;
	}

	//For finding the final z position, we distinguish whether the cylinder should rather
	//be placed on top of the highest floor or below the lowest ceiling. The XY position
	//is the closest point, properly backtransformed.
	f32 non_grounded_z = min_f32(target.z, z_bound_ws[1] - cylinder.total_height);
	result.new_grounded = target.z < z_bound_ws[0];
	result.new_pos = {
		(closest_point_x + trans.x) / scale.x,
		(closest_point_y + trans.y) / scale.y,
		result.new_grounded ? z_bound_ws[0] : non_grounded_z,
		1.0f
	};
	return result;
}

#endif