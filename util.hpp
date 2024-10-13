#ifndef UTIL_HPP
#define UTIL_HPP

#include <cstdint>

using u8 = uint8_t;
using f32 = float;
using i32 = int32_t;
using u32 = uint32_t;
using i64 = int64_t;
using u64 = uint64_t;
using f64 = double;

#define FORCE_INLINE __attribute__((always_inline)) inline


FORCE_INLINE i32 min_i32(i32 a, i32 b) {
	i32 result = a < b ? a : b;
	return result;
}

FORCE_INLINE i32 max_i32(i32 a, i32 b) {
	i32 result = a > b ? a : b;
	return result;
}

FORCE_INLINE f32 floor_f32_to_f32(f32 value) {
	f32 result = __builtin_roundf(value - 0.5f);
	return result;
}

FORCE_INLINE f32 sqrt_f32(f32 value) {
	f32 result = __builtin_sqrtf(value);
	return result;
}

FORCE_INLINE f32 clamp_f32(f32 a, f32 min, f32 max) {
	f32 result = a < min ? min : (a > max ? max : a);
	return result;
}

FORCE_INLINE f32 min_f32(f32 a, f32 b) {
	f32 result = a < b ? a : b;
	return result;
}

FORCE_INLINE bool is_nan(f32 a) {
	return __builtin_isnan(a);
}

FORCE_INLINE u32 bitcast_f32_to_u32(f32 value) {
	return __builtin_bit_cast(u32, value);
}

FORCE_INLINE f32 bitcast_u32_to_f32(u32 value) {
	return __builtin_bit_cast(f32, value);
}

FORCE_INLINE f32 sin_f32(f32 value) {
	f32 result = __builtin_sinf(value);
	return result;
}

#define F32_INF (bitcast_u32_to_f32(u32(0x7F800000)))
#define F32_NAN_0 (bitcast_u32_to_f32(u32(0x7FC00000)))


struct alignas(16) f32_4 {
	f32 x;
	f32 y;
	f32 z;
	f32 w;
};

FORCE_INLINE f32_4 operator-(f32_4 value) {
	f32_4 result = {-value.x, -value.y, -value.z, -value.w};
	return result;
}

FORCE_INLINE f32_4 operator-(f32_4 a, f32_4 b) {
	f32_4 c = {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
	return c;
}

FORCE_INLINE f32_4 operator*(f32_4 a, f32_4 b) {
	f32_4 c = {a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w};
	return c;
}

FORCE_INLINE f32_4 operator/(f32_4 a, f32 b) {
	f32_4 result = {a.x / b, a.y / b, a.z / b, a.w / b};
	return result;
}

FORCE_INLINE u64 rdtsc() {
	//NOTE(cdamerius): relying on invariant TSC for accurate timings.
	u64 result = __builtin_ia32_rdtsc();
	return result;
}

i64 get_rdtsc_timer_freq() {
	return 3'000'000'000;//hardcoded timer frequency of 3 GHz
}

#undef FORCE_INLINE


//NOTE(cdamerius):
//builtin intrinsics are pulled out of immintrin.h for multiple reasons:
//+ improved compilation times.
//+ casting between integer and float vector types such as m256/m256i is not required.
//  All SIMD operations are rewritten so as to accept the m256 instead
//+ removed double underscore prefix in front of types, see below.
//Integer vectors such as m256i are only used for intermediate results within simd functions.
using m64 = i32 __attribute__ ((__vector_size__(8), __may_alias__));
using v2si = i32 __attribute__ ((__vector_size__(8)));
using v2sf = f32 __attribute__ ((__vector_size__(8)));
using m128 = f32 __attribute__ ((__vector_size__(16), __may_alias__));
using v4sf = f32 __attribute__ ((__vector_size__(16)));
using v2di = i64 __attribute__ ((__vector_size__(16)));
using v8sf = f32 __attribute__ ((__vector_size__(32)));
using v8si = i32 __attribute__ ((__vector_size__(32)));
using v8su = u32 __attribute__ ((__vector_size__(32)));
using m256 = f32 __attribute__ ((__vector_size__(32), __may_alias__));
using m256i = i64 __attribute__ ((__vector_size__(32), __may_alias__));

static_assert(alignof(m128) == 16);
static_assert(alignof(m256) == 32);

#define _CMP_EQ_OQ (0x00)
#define _CMP_LT_OS (0x01)
#define _CMP_LE_OS (0x02)
#define _CMP_UNORD_Q (0x03)
#define _CMP_NEQ_UQ (0x04)
#define _CMP_NLT_US (0x05)
#define _CMP_NLE_US (0x06)
#define _CMP_ORD_Q (0x07)
#define _CMP_EQ_UQ (0x08)
#define _CMP_NGE_US (0x09)
#define _CMP_NGT_US (0x0A)
#define _CMP_FALSE_OQ (0x0B)
#define _CMP_NEQ_OQ (0x0C)
#define _CMP_GE_OS (0x0D)
#define _CMP_GT_OS (0x0E)
#define _CMP_TRUE_UQ (0x0F)
#define _CMP_EQ_OS (0x10)
#define _CMP_LT_OQ (0x11)
#define _CMP_LE_OQ (0x12)
#define _CMP_UNORD_S (0x13)
#define _CMP_NEQ_US (0x14)
#define _CMP_NLT_UQ (0x15)
#define _CMP_NLE_UQ (0x16)
#define _CMP_ORD_S (0x17)
#define _CMP_EQ_US (0x18)
#define _CMP_NGE_UQ (0x19)
#define _CMP_NGT_UQ (0x1A)
#define _CMP_FALSE_OS (0x1B)
#define _CMP_NEQ_OS (0x1C)
#define _CMP_GE_OQ (0x1D)
#define _CMP_GT_OQ (0x1E)
#define _CMP_TRUE_US (0x1F)

#define SIMD_FUNC __inline __attribute__((__gnu_inline__, __always_inline__, __artificial__))

m128 SIMD_FUNC _mm_add_ps(m128 a, m128 b) {
	m128 result = m128(v4sf(a) + v4sf(b));
	return result;
}

m128 SIMD_FUNC _mm_mul_ps(m128 a, m128 b) {
	m128 result = m128(v4sf(a) * v4sf(b));
	return result;
}

m128 SIMD_FUNC _mm_sqrt_ps(m128 a) {
	m128 result = m128(__builtin_ia32_sqrtps(v4sf(a)));
	return result;
}

m64 SIMD_FUNC _mm_cvtps_pi16(m128 a) {
	v4sf hisf = v4sf(a);
	v4sf losf = __builtin_ia32_movhlps(hisf, hisf);
	v2si hisi = __builtin_ia32_cvtps2pi(hisf);
	v2si losi = __builtin_ia32_cvtps2pi(losf);
	m64 result = m64(__builtin_ia32_packssdw(hisi, losi));
	return result;
}

void SIMD_FUNC _mm_storel_pi(m64* p, m128 a) {
	__builtin_ia32_storelps(reinterpret_cast<v2sf*>(p), v4sf(a));
}

m128 SIMD_FUNC _mm_set1_ps(f32 a) {
	m128 result = m128(v4sf{a, a, a, a});
	return result;
}

m128 SIMD_FUNC _mm_load_ps(f32* p) {
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wcast-align"
	m128 result = *reinterpret_cast<m128*>(p);
	#pragma GCC diagnostic pop
	return result;
}

m128 SIMD_FUNC _mm_set_epi64x(i64 a, i64 b) {
	m128 result = m128(v2di{b, a});
	return result;
}

m128 SIMD_FUNC _mm_set_epi64(m64 a, m64 b) {
	m128 result = _mm_set_epi64x(i64(a), i64(b));
	return result;
}

m128 SIMD_FUNC _mm_movpi64_epi64(m64 a) {
	m128 result = _mm_set_epi64(m64(0LL), a);
	return result;
}

m256 SIMD_FUNC _mm256_add_ps(m256 a, m256 b) {
	m256 result = m256(v8sf(a) + v8sf(b));
	return result;
}

m256 SIMD_FUNC _mm256_and_ps(m256 a, m256 b) {
	m256 result = m256(__builtin_ia32_andps256(v8sf(a), v8sf(b)));
	return result;
}

m256 SIMD_FUNC _mm256_andnot_ps(m256 a, m256 b) {
	m256 result = m256(__builtin_ia32_andnps256(v8sf(a), v8sf(b)));
	return result;
}

m256 SIMD_FUNC _mm256_blendv_ps(m256 a, m256 b, m256 mask) {
	m256 result = m256(__builtin_ia32_blendvps256(v8sf(a), v8sf(b), v8sf(mask)));
	return result;
}

m256 SIMD_FUNC _mm256_div_ps(m256 a, m256 b) {
	m256 result = m256(v8sf(a) / v8sf(b));
	return result;
}

m256 SIMD_FUNC _mm256_max_ps(m256 a, m256 b) {
	m256 result = m256(__builtin_ia32_maxps256(v8sf(a), v8sf(b)));
	return result;
}

m256 SIMD_FUNC _mm256_min_ps(m256 a, m256 b) {
	m256 result = m256(__builtin_ia32_minps256(v8sf(a), v8sf(b)));
	return result;
}

m256 SIMD_FUNC _mm256_mul_ps(m256 a, m256 b) {
	m256 result = m256(v8sf(a) * v8sf(b));
	return result;
}

m256 SIMD_FUNC _mm256_sub_ps(m256 a, m256 b) {
	m256 result = m256(v8sf(a) - v8sf(b));
	return result;
}

m256 SIMD_FUNC _mm256_cvtepi32_ps(m256 a) {
	m256 result = m256(__builtin_ia32_cvtdq2ps256(v8si(m256i(a))));
	return result;
}

m256 SIMD_FUNC _mm256_setzero_ps() {
	m256 result = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	return result;
}

#define _mm256_cmp_ps(a,b,imm8) (m256(__builtin_ia32_cmpps256(v8sf(a), v8sf(b), i32(imm8))))

m256 SIMD_FUNC _mm256_load_ps(f32* p) {
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wcast-align"
	m256 result = *reinterpret_cast<m256*>(p);
	#pragma GCC diagnostic pop
	return result;
}

void SIMD_FUNC _mm256_store_ps(f32* p, m256 a) {
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wcast-align"
	*reinterpret_cast<m256*>(p) = a;
	#pragma GCC diagnostic pop
}

m256 SIMD_FUNC _mm256_load_si256(m256* p) {
	return *p;
}

m256 SIMD_FUNC _mm256_sqrt_ps(m256 a) {
	m256 result = m256(__builtin_ia32_sqrtps256(v8sf(a)));
	return result;
}

m256 SIMD_FUNC _mm256_set_ps(f32 a, f32 b, f32 c, f32 d, f32 e, f32 f, f32 g, f32 h) {
	m256 result{h, g, f, e, d, c, b, a};
	return result;
}

m256 SIMD_FUNC _mm256_set_epi32(i32 a, i32 b, i32 c, i32 d, i32 e, i32 f, i32 g, i32 h) {
	m256 result = m256(v8si{h, g, f, e, d, c, b, a});
	return result;
}

m256 SIMD_FUNC _mm256_set1_ps(f32 a) {
	m256 result{a, a, a, a, a, a, a, a};
	return result;
}

m256 SIMD_FUNC _mm256_set1_epi32(i32 a) {
	m256 result = m256(v8si{a, a, a, a, a, a, a, a});
	return result;
}

m256 SIMD_FUNC _mm256_max_epi32(m256 a, m256 b) {
	m256 result = m256(__builtin_ia32_pmaxsd256(v8si(a), v8si(b)));
	return result;
}

m256 SIMD_FUNC _mm256_min_epi32(m256 a, m256 b) {
	m256 result = m256(__builtin_ia32_pminsd256(v8si(a), v8si(b)));
	return result;
}

m256 SIMD_FUNC _mm256_permutevar8x32_ps(m256 a, m256 b) {
	m256 result = m256(__builtin_ia32_permvarsf256(v8sf(a), v8si(b)));
	return result;
}

m256 SIMD_FUNC _mm256_add_epi32(m256 a, m256 b) {
	m256 result = m256(m256i(v8su(m256i(a)) + v8su(m256i(b))));
	return result;
}

m256 SIMD_FUNC _mm256_mul_epi32(m256 x, m256 y) {
	m256 result = m256(m256i(__builtin_ia32_pmuldq256(v8si(m256i(x)), v8si(m256i(y)))));
	return result;
}

#undef SIMD_FUNC


template <typename T, u32 N> constexpr i32 array_len(T(&)[N]) {
	return i32(N);
}

#endif