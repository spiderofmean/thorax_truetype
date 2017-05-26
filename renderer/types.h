//===- types.h -------------------------------------------------*- C++ --*-===//
// Copyright 2017  Warren Hunt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//===----------------------------------------------------------------------===//

#pragma once

#include <cassert>
#include <intrin.h>

//==============================================================================
// Abstraction of vector hardware.
//==============================================================================

__forceinline float floatInfinity() {
    static const unsigned bits = 0x7f800000u;
    return *(float*)&bits;
}

struct float4 {
    float4() = default;
    __forceinline float4(__m128 data) : data(data) {}
    __forceinline explicit float4(float a) : data(_mm_set1_ps(a)) {}
    __forceinline float4(float v0, float v1, float v2, float v3) : data(_mm_setr_ps(v0, v1, v2, v3)) {}
    __forceinline static float4 SignMask() { return float4(-0.0f); }
    __forceinline float4 operator-() const { return *this ^ SignMask(); }
    __forceinline float4 operator+(float4 a) const { return _mm_add_ps(data, a.data); }
    __forceinline float4 operator*(float4 a) const { return _mm_mul_ps(data, a.data); }
    __forceinline float4 operator^(float4 a) const { return _mm_xor_ps(data, a.data); }
    __forceinline float& operator[](size_t i) { return data.m128_f32[i]; }
    __forceinline float operator[](size_t i) const { return data.m128_f32[i]; }
    __m128 data;
};

__forceinline float4 min(float4 a, float4 b) { return _mm_min_ps(a.data, b.data); }
__forceinline float4 max(float4 a, float4 b) { return _mm_max_ps(a.data, b.data); }
__forceinline float4 round_up(float4 a) { return _mm_round_ps(a.data, _MM_FROUND_TO_POS_INF); }
__forceinline int movemask(float4 a) { return _mm_movemask_ps(a.data); }
template <int i0, int i1, int i2, int i3> __forceinline float4 shuffle(float4 a) {
    return _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(a.data), i0 + i1 * 4 + i2 * 16 + i3 * 64));
}
template <int i0, int i1, int i2, int i3> __forceinline float4 shuffle(float4 a, float4 b) {
    return _mm_shuffle_ps(a.data, b.data, i0 + i1 * 4 + i2 * 16 + i3 * 64);
}

struct int4 {
    __forceinline int4(__m128i a) : data(a) {}
    __forceinline int4(int v0, int v1, int v2, int v3) : data(_mm_setr_epi32(v0, v1, v2, v3)) {}
    __forceinline explicit int4(float4 a) : data(_mm_cvtps_epi32(a.data)) {}
    __forceinline static int4 True(__m128i any = _mm_setzero_si128()) { return _mm_cmpeq_epi32(any, any); }
    __forceinline int4 operator+(int4 a) const { return _mm_add_epi32(data, a.data); }
    __forceinline int operator[](size_t i) const { return data.m128i_i32[i]; }
    __m128i data;
};

__forceinline int movemask(int4 a) { return _mm_movemask_ps(_mm_castsi128_ps(a.data)); }
template <int i0, int i1, int i2, int i3> __forceinline int4 shuffle(int4 a) {
    return _mm_shuffle_epi32(a.data, i0 + i1 * 4 + i2 * 16 + i3 * 64);
}

struct float8 {
    float8() = default;
    __forceinline float8(__m256 data) : data(data) {}
    __forceinline explicit float8(float v) : data(_mm256_set1_ps(v)) {}
    __forceinline float8(float4 a, float4 b) : data(_mm256_insertf128_ps(_mm256_castps128_ps256(a.data), b.data, 1)) {}
    __forceinline float8(float v0, float v1, float v2, float v3, float v4, float v5, float v6, float v7)
        : data(_mm256_setr_ps(v0, v1, v2, v3, v4, v5, v6, v7)) {}
    __forceinline static float8 Zero() { return _mm256_setzero_ps(); }
    __forceinline static float8 SignMask() { return float8(-0.0f); }
    __forceinline static float8 Load(const float* p) { return _mm256_load_ps(p); }
    __forceinline static float8 LoadU(const float* p) { return _mm256_loadu_ps(p); }
    __forceinline float8 operator-() const { return *this ^ SignMask(); }
    __forceinline float8 operator+(float8 a) const { return _mm256_add_ps(data, a.data); }
    __forceinline float8 operator-(float8 a) const { return _mm256_sub_ps(data, a.data); }
    __forceinline float8 operator*(float8 a) const { return _mm256_mul_ps(data, a.data); }
    __forceinline float8 operator/(float8 a) const { return _mm256_div_ps(data, a.data); }
    __forceinline float8& operator+=(float8 a) { return data = _mm256_add_ps(data, a.data), *this; }
    __forceinline float8& operator-=(float8 a) { return data = _mm256_sub_ps(data, a.data), *this; }
    __forceinline float8 operator&(float8 a) const { return _mm256_and_ps(data, a.data); }
    __forceinline float8 operator^(float8 a) const { return _mm256_xor_ps(data, a.data); }
    __forceinline float8 operator|(float8 a) const { return _mm256_or_ps(data, a.data); }
    __forceinline float8 operator <=(float8 a) const { return _mm256_cmp_ps(data, a.data, _CMP_LE_OQ); }
    __forceinline float operator[](size_t i) const { return data.m256_f32[i]; }
    __forceinline float4 v0123() const { return _mm256_castps256_ps128(data); }
    __forceinline float4 v4567() const { return _mm256_extractf128_ps(data, 1); }
    __m256 data;
};

__forceinline float8 madd(float8 a, float8 b, float8 c) { return _mm256_fmadd_ps(a.data, b.data, c.data); }
__forceinline float8 nmadd(float8 a, float8 b, float8 c) { return _mm256_fnmadd_ps(a.data, b.data, c.data); }
__forceinline float8 msub(float8 a, float8 b, float8 c) { return _mm256_fmsub_ps(a.data, b.data, c.data); }
__forceinline float8 nmsub(float8 a, float8 b, float8 c) { return _mm256_fnmsub_ps(a.data, b.data, c.data); }
__forceinline float8 min(float8 a, float8 b) { return _mm256_min_ps(a.data, b.data); }
__forceinline float8 max(float8 a, float8 b) { return _mm256_max_ps(a.data, b.data); }
__forceinline float8 sqrt(float8 a) { return _mm256_sqrt_ps(a.data); }
__forceinline float8 blend(float8 a, float8 b, float8 mask) { return _mm256_blendv_ps(a.data, b.data, mask.data); }
__forceinline int movemask(float8 a) { return _mm256_movemask_ps(a.data); }
template <int i0, int i1, int i2, int i3> __forceinline float8 shuffle(float8 a) {
    return _mm256_permute_ps(a.data, i0 + i1 * 4 + i2 * 16 + i3 * 64);
}
template <int i0, int i1, int i2, int i3> __forceinline float8 shuffle(float8 a, float8 b) {
    return _mm256_shuffle_ps(a.data, b.data, i0 + i1 * 4 + i2 * 16 + i3 * 64);
}
template <int i0, int i1> __forceinline float8 shuffle4(float8 a) {
    return _mm256_permute2f128_ps(a.data, a.data, i0 + i1 * 16);
}
template <int i0, int i1, int i2, int i3> __forceinline float8 shuffle2(float8 a) {
    return _mm256_castpd_ps(_mm256_permute4x64_pd(_mm256_castps_pd(a.data), i0 + i1 * 4 + i2 * 16 + i3 * 64));
}

struct int8 {
    __forceinline int8(__m256i a) : data(a) {}
    __forceinline explicit int8(int v) : data(_mm256_set1_epi32(v)) {}
    __forceinline explicit int8(float8 a) : data(_mm256_cvtps_epi32(a.data)) {}
    __forceinline int8 operator*(int8 a) const { return _mm256_mullo_epi32(data, a.data); }
    __forceinline void storeu(int* p) const { return _mm256_storeu_si256((__m256i*)p, data); }
    __m256i data;
};
__forceinline int8 unpack4lo(int8 a, int8 b) { return _mm256_permute2x128_si256(a.data, b.data, 0x20); }
__forceinline int8 unpack4hi(int8 a, int8 b) { return _mm256_permute2x128_si256(a.data, b.data, 0x31); }

struct short16 {
    explicit short16(short v) : data(_mm256_set1_epi16(v)) {}
    short16(__m256i a) : data(a) {}
    static __forceinline short16 loadu(const short* p) { return _mm256_loadu_si256((__m256i*)p); }
    __forceinline short16 operator-(short16 a) const { return _mm256_sub_epi16(data, a.data); }
    int8 ZeroExtendUnpackLo() const { return _mm256_unpacklo_epi16(data, _mm256_setzero_si256()); }
    int8 ZeroExtendUnpackHi() const { return _mm256_unpackhi_epi16(data, _mm256_setzero_si256()); }
    __m256i data;
};
__forceinline short16 abs(short16 a) { return _mm256_abs_epi16(a.data); }
template <int i> __forceinline short16 srli(short16 a) { return _mm256_srli_epi16(a.data, i); }
__forceinline short16 min(short16 a, short16 b) { return _mm256_min_epi16(a.data, b.data); }

struct short8 {
    short8(__m128i a) : data(a) {}
    short8(__int64 v0123, __int64 v4567) : data(_mm_setr_epi64x(v0123, v4567)) {}
    __forceinline static short8 Pack(int8 a) {
        auto x = float8(_mm256_castsi256_ps(_mm256_shufflehi_epi16(_mm256_shufflelo_epi16(a.data, 0xd8), 0xd8)));
        return _mm_castps_si128(shuffle<0, 2, 0, 2>(x.v0123(), x.v4567()).data);
    }
    __forceinline short8 operator+(short8 a) const { return _mm_add_epi16(data, a.data); }
    __forceinline __int64 v0123() const { return _mm_extract_epi64(data, 0); }
    __forceinline __int64 v4567() const { return _mm_extract_epi64(data, 1); }
    __m128i data;
};

//==============================================================================
// DynamicArray
//==============================================================================

template <typename T> struct Array {
    Array() : data(nullptr), size(0) {}
    Array(size_t size, size_t alignment = __alignof(T))
        : data(size ? (T*)_aligned_malloc(sizeof(T) * size, alignment) : nullptr), size(size) {}
    Array(const Array& a) : Array(a.size) {
        for (size_t i = 0; i < size; i++) data[i] = a.data[i];
    }
    Array(Array&& a) : data(a.data), size(a.size) { a.data = nullptr; }
    ~Array() {
        if (data) _aligned_free(data);
    }
    explicit operator bool() const { return data != nullptr; }
    operator T*() const { return data; }
    T* operator+(size_t i) const {
        assert(i < size);
        return data + i;
    }
    Array& operator=(Array a) {
        auto temp = data;
        data = a.data;
        size = a.size;
        a.data = temp;
        return *this;
    }
    T& operator[](size_t i) {
        assert(i < size);
        return data[i];
    }
    T operator[](size_t i) const {
        assert(i < size);
        return data[i];
    }
    T* begin() { return data; }
    T* end() { return data + size; }
    const T* begin() const { return data; }
    const T* end() const { return data + size; }
    T* data;
    size_t size;
};

// TODO: delete me and add an initializer iterator for Array.
template <typename T> struct DynamicArray {
    DynamicArray() : data(nullptr), size(0), capacity(0) {}
    DynamicArray(size_t capacity, size_t size = 0, size_t alignment = __alignof(T))
        : data(capacity ? (T*)_aligned_malloc(sizeof(T) * capacity, alignment) : nullptr),
          size(size),
          capacity(capacity) {
        assert(size <= capacity);
    }
    DynamicArray(const DynamicArray& a) : DynamicArray(a.size) {
        for (size_t i = 0; i < size; i++) data[i] = a.data[i];
    }
    DynamicArray(DynamicArray&& a) : data(a.data), size(a.size), capacity(capacity) { a.data = nullptr; }
    ~DynamicArray() {
        if (data) _aligned_free(data);
    }
    explicit operator bool() const { return data != nullptr; }
    DynamicArray& operator=(DynamicArray a) {
        auto temp = data;
        data = a.data;
        size = a.size;
        capacity = a.capacity;
        a.data = temp;
        return *this;
    }
    T& operator[](size_t i) {
        assert(i < size);
        return data[i];
    }
    T operator[](size_t i) const {
        assert(i < size);
        return data[i];
    }
    void Resize(size_t new_size) {
        assert(new_size <= capacity);
        size = new_size;
    }
    template <typename... Args> void Push(const Args&... args) {
        assert(size < capacity);
        data[size++] = T(args...);
    }
    T* begin() { return data; }
    T* end() { return data + size; }
    const T* begin() const { return data; }
    const T* end() const { return data + size; }
    T* data;
    size_t size, capacity;
};

//==============================================================================
// Vector Types
//==============================================================================

struct Vector {
    Vector() = default;
    Vector(float x, float y) : x(x), y(y) {}
    Vector operator+(const Vector& a) const { return Vector(x + a.x, y + a.y); }
    Vector operator-(const Vector& a) const { return Vector(x - a.x, y - a.y); }
    Vector operator*(float a) const { return Vector(x * a, y * a); }
    float x, y;
};

struct Point {
    Point() = default;
    Point(float x, float y) : x(x), y(y) {}
    explicit Point(const Vector& a) : x(a.x), y(a.y) {}
    Vector operator-(const Point& a) const { return Vector(x - a.x, y - a.y); }
    float x, y;
};

struct __declspec(align(8)) Matrix2x3 {
    Matrix2x3() = default;
    Matrix2x3(float x_x, float y_x, float w_x, float x_y, float y_y, float w_y)
        : x(x_x, x_y), y(y_x, y_y), w(w_x, w_y) {}
    Matrix2x3(const Vector& x, const Vector& y, const Point& w) : x(x), y(y), w(w) {}
    Vector operator*(const Vector& a) const { return Vector(x.x * a.x + y.x * a.y, x.y * a.x + y.y * a.y); };
    Point operator*(const Point& a) const { return Point(x.x * a.x + y.x * a.y + w.x, x.y * a.x + y.y * a.y + w.y); };
    Matrix2x3 operator*(const Matrix2x3& a) const { return Matrix2x3((*this) * a.x, (*this) * a.y, (*this) * a.w); }
    Vector x, y;
    Point w;
};

static inline Matrix2x3 invert(const Matrix2x3& a) {
    auto scale = 1.0f / (a.x.x * a.y.y - a.x.y * a.y.x);
    auto x = Vector(a.y.y, -a.x.y) * scale;
    auto y = Vector(-a.y.x, a.x.x) * scale;
    auto w = Point(x * -a.w.x - y * a.w.y);
    return Matrix2x3(x, y, w);
}

//==============================================================================
// Basic Types
//==============================================================================

struct short2 {
    short x, y;
};

struct __declspec(align(8)) Segment {
    Segment() = default;
    Segment(short2 vert, short2 knot) : vert(vert), knot(knot) {}

    short2 vert;
    short2 knot;
};

//==============================================================================
// Packed Types
//==============================================================================

struct Box {
    Box() = default;
    explicit Box(float4 data) : data(data) {}
    Box(float lower_x, float lower_y, float upper_x, float upper_y) : data(-lower_x, -lower_y, upper_x, upper_y) {}
    Box(const Point& a) : data(-a.x, -a.y, a.x, a.y) {}
    Box operator+(const Box& a) const { return Box(data + a.data); }
    Box operator&(const Box& a) const { return Box(min(data, a.data)); }
    Box operator|(const Box& a) const { return Box(max(data, a.data)); }
    Box& operator|=(const Box& a) {
        data = max(data, a.data);
        return *this;
    }
    static Box Empty() { return Box(float4(-floatInfinity())); }
    float4 data; // -lower_x, -lower_y, upper_x, upper_y
};

struct Shape {
    explicit Shape(float8 p0v1p2v3) : p0v1p2v3(p0v1p2v3) {}
    Shape(float x0, float x1, float x2, float x3, float y0, float y1)
        : p0v1p2v3(x0, y0, x1 - x0, y1 - y0, x2, y1, x3 - x2, y0 - y1) {}
    Shape(const Point& p0, const Vector& v1, const Vector& v2)
        : p0v1p2v3(p0.x, p0.y, v1.x, v1.y, p0.x, p0.y, v2.x, v2.y) {}
    bool IsTrapazoid() const { return p0v1p2v3[1] != p0v1p2v3[5]; }
    bool IsBox4NodeRef() const { return (((const unsigned*)this)[7] >> 23) == 0; } // sign and exponent will be 0 only for ref?? TODO: make me NAN instead
    Box Bound() const {
        auto p1__p3__ = p0v1p2v3 + shuffle<2, 3, 2, 3>(p0v1p2v3);
        auto temp0 = -min(p0v1p2v3, p1__p3__);
        auto temp1 = max(p0v1p2v3, p1__p3__);
        auto lower = max(temp0.v0123(), temp0.v4567());
        auto upper = max(temp1.v0123(), temp1.v4567());
        return Box(shuffle<0, 1, 0, 1>(lower, upper));
    }
    float8 p0v1p2v3;
};

struct __declspec(align(32)) Box4NodeRef {
    Matrix2x3 objectFromParent;
    int meshIndex; // Relative from the current mesh.
    unsigned nodeIndex; // Relative to the base shape of the current mesh.
};

struct __declspec(align(32)) Box4Node {
    const Box4Node* Child(size_t i) const { return (const Box4Node*)((const char*)this + data[1 + i]); }
    Box4Node* Child(size_t i) { return (Box4Node*)((char*)this + data[1 + i]); }
    const Shape* ShapesAt(const Shape* shapes, size_t i) const { return (const Shape*)((const char*)shapes + data[i]); }
    const Box4NodeRef* Box4NodeRefAt(const Shape* shapes, size_t i) const { return (const Box4NodeRef*)((const char*)shapes + data[i]); }
    unsigned IsLeaf(size_t i) const { return leafMask & (1u << i); }
    Box Bound(size_t i) const { return Box(float4(-min_x[i], -min_y[i], max_x[i], max_y[i])); }

    Box Bound() const { return Bound(0) | Bound(1) | Bound(2) | Bound(3); }

    Box Bound(const Matrix2x3& m) const {
        Box box = Box::Empty();
        for (size_t i = 0; i < 4; i++) {
            if (data[i] == data[i + 1]) continue;
            box |= Box(m * Point(min_x[i], min_y[i]));
            box |= Box(m * Point(min_x[i], max_y[i]));
            box |= Box(m * Point(max_x[i], min_y[i]));
            box |= Box(m * Point(max_x[i], max_y[i]));
        }
        return box;
    }

    float min_x[4], min_y[4];
    float max_x[4], max_y[4];
    unsigned data[5];
    unsigned char leafMask;
    char PADDING[10];
};

struct Mesh {
    const Shape* shapes;
    const Box4Node* nodes;
};
