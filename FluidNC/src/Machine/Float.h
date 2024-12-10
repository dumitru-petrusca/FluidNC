// Copyright (c) 2025 -  Dumitru Petrusca
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#ifndef FLUIDNC_FLOAT_H
#define FLUIDNC_FLOAT_H
#include <algorithm>
#include <cstdint>

// SIGN (1), INTEGER (19), FRACTION (12)
// Range -524288 to +524288
#define FRACTION_BITS 12
#define MULTIPLIER (1 << FRACTION_BITS)
#define INLINEOP __attribute__((always_inline)) inline
#define i64(x) static_cast<int64_t>((x))
#define i32(x) static_cast<int32_t>((x))

struct Float {
    std::int32_t v;

    constexpr Float() : v(0) {}
    // This is not ISR-safe
    constexpr Float(const double v) : v(static_cast<int32_t>(v * MULTIPLIER)) {}
    // This is ISR-safe
    constexpr Float(const int32_t raw, int) : v(raw) {}

    INLINEOP static Float wrap(const int32_t raw) { return { raw, 0 }; }

    INLINEOP Float& operator=(const int32_t rhs) {  // assignment
        v = rhs << FRACTION_BITS;
        return *this;
    }

    INLINEOP Float operator-() const { return wrap(-v); }  // unary -

    [[nodiscard]] INLINEOP Float abs() const { return v < 0 ? wrap(-v) : wrap(v); }
    // This is ISR-safe
    [[nodiscard]] INLINEOP int32_t toInt() const { return v >> FRACTION_BITS; }
    // This is not ISR-safe
    [[nodiscard]] INLINEOP float toFloat() const { return static_cast<float>(v) / MULTIPLIER; }
};

// Float - Float operators (* / + - < > <= >= !=)

INLINEOP Float operator*(const Float lhs, const Float rhs) {
    return Float::wrap((i64(lhs.v) * rhs.v) >> FRACTION_BITS);
}
INLINEOP Float operator/(const Float lhs, const Float rhs) {
    return Float::wrap((i64(lhs.v) << FRACTION_BITS) / rhs.v);
}
INLINEOP Float operator+(const Float lhs, const Float rhs) {
    return Float::wrap(i64(lhs.v) + rhs.v);
}
INLINEOP Float operator-(const Float lhs, const Float rhs) {
    return Float::wrap(i64(lhs.v) - rhs.v);
}
INLINEOP bool operator<(const Float lhs, const Float rhs) {
    return lhs.v < rhs.v;
}
INLINEOP bool operator>(const Float lhs, const Float rhs) {
    return lhs.v > rhs.v;
}
INLINEOP bool operator<=(const Float lhs, const Float rhs) {
    return lhs.v <= rhs.v;
}
INLINEOP bool operator>=(const Float lhs, const Float rhs) {
    return lhs.v >= rhs.v;
}
INLINEOP bool operator!=(const Float lhs, const Float rhs) {
    return lhs.v != rhs.v;
}

// Float - int operators

INLINEOP Float operator*(const Float lhs, const int32_t rhs) {
    return Float::wrap(i64(lhs.v) * rhs);
}
INLINEOP Float operator/(const Float lhs, const int32_t rhs) {
    return Float::wrap(i64(lhs.v) / rhs);
}
INLINEOP Float operator+(const Float lhs, const int32_t rhs) {
    return Float::wrap(i64(lhs.v) + (rhs << FRACTION_BITS));
}
INLINEOP Float operator-(const Float lhs, const int32_t rhs) {
    return Float::wrap(i64(lhs.v) - (rhs << FRACTION_BITS));
}
INLINEOP bool operator<(const Float lhs, const int32_t rhs) {
    return i64(lhs.v) < rhs << FRACTION_BITS;
}
INLINEOP bool operator>(const Float lhs, const int32_t rhs) {
    return i64(lhs.v) > rhs << FRACTION_BITS;
}
INLINEOP bool operator<=(const Float lhs, const int32_t rhs) {
    return i64(lhs.v) <= rhs << FRACTION_BITS;
}
INLINEOP bool operator>=(const Float lhs, const int32_t rhs) {
    return i64(lhs.v) >= rhs << FRACTION_BITS;
}
INLINEOP bool operator!=(const Float lhs, const int32_t rhs) {
    return i64(lhs.v) != rhs << FRACTION_BITS;
}

// int - Float operators

INLINEOP Float operator*(const int32_t lhs, const Float rhs) {
    return rhs * lhs;
}
INLINEOP Float operator/(const int32_t lhs, const Float rhs) {
    return Float::wrap(((i64(lhs) << FRACTION_BITS) << FRACTION_BITS) / rhs.v);
}
INLINEOP Float operator+(const int32_t lhs, const Float rhs) {
    return rhs + lhs;
}
INLINEOP Float operator-(const int32_t lhs, const Float rhs) {
    return Float::wrap((lhs << FRACTION_BITS) - rhs.v);
}

// Assignment Operators

INLINEOP Float operator+=(Float& lhs, const Float rhs) {
    lhs.v += rhs.v;
    return lhs;
}
INLINEOP Float operator-=(Float& lhs, const Float rhs) {
    lhs.v -= rhs.v;
    return lhs;
}
INLINEOP Float operator+=(Float& lhs, const int32_t rhs) {
    lhs.v += rhs << FRACTION_BITS;
    return lhs;
}
INLINEOP Float operator-=(Float& lhs, const int32_t rhs) {
    lhs.v -= rhs << FRACTION_BITS;
    return lhs;
}

#endif  //FLUIDNC_FLOAT_H
