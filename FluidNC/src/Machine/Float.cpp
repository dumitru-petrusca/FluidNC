//
// Created by Dumitru Petrusca on 1/4/26.
//

#include "Float.h"

#include <cmath>
#include <cstdio>
#include <string>

// Test Helpers
int g_tests_passed = 0;
int g_tests_failed = 0;

void assertEqual(const Float& actual, double expected, const std::string& msg = "") {
    if (std::abs(actual.toFloat() - expected) < 0.001) {
        g_tests_passed++;
        // printf(".");
    } else {
        g_tests_failed++;
        printf("FAIL: %s Expected %f, got %f\n", msg.c_str(), expected, actual.toFloat());
    }
}

void assertEqualRaw(const Float& actual, int32_t expected_raw, const std::string& msg = "") {
    if (actual.v == expected_raw) {
        g_tests_passed++;
    } else {
        g_tests_failed++;
        printf("FAIL: %s Expected raw %d, got %d\n", msg.c_str(), expected_raw, actual.v);
    }
}

void assertTrue(bool result, const std::string& msg = "") {
    if (result) {
        g_tests_passed++;
    } else {
        g_tests_failed++;
        printf("FAIL: %s Expected true\n", msg.c_str());
    }
}

void assertFalse(bool result, const std::string& msg = "") {
    if (!result) {
        g_tests_passed++;
    } else {
        g_tests_failed++;
        printf("FAIL: %s Expected false\n", msg.c_str());
    }
}

// Tests
void testConstructorsAndConversions() {
    printf("Testing Constructors & Conversions...\n");

    // Default constructor
    Float f0;
    assertEqual(f0, 0.0, "Default ctor");
    assertEqualRaw(f0, 0, "Default ctor raw");

    // Double constructor
    Float f1(1.0);
    assertEqual(f1, 1.0, "Double ctor 1.0");
    assertEqualRaw(f1, 4096, "Double ctor 1.0 raw");  // 1 * 4096

    Float f1_5(1.5);
    assertEqual(f1_5, 1.5, "Double ctor 1.5");

    Float f_neg(-2.5);
    assertEqual(f_neg, -2.5, "Double ctor -2.5");

    // Wrap
    Float wrapped = Float::wrap(8192);  // 2.0
    assertEqual(wrapped, 2.0, "Wrap 8192");

    // toInt
    assertTrue(f1.toInt() == 1, "toInt 1.0");
    assertTrue(f1_5.toInt() == 1, "toInt 1.5 (floor)");
    assertTrue(Float(1.99).toInt() == 1, "toInt 1.99");

    // Negative toInt behavior (bitwise shift is floor for negative numbers)
    // -1.5 is -6144 raw. -6144 >> 12 = -2.
    assertTrue(Float(-1.5).toInt() == -2, "toInt -1.5 (floor to -2)");
    // -0.5 is -2048 raw. -2048 >> 12 = -1.
    assertTrue(Float(-0.5).toInt() == -1, "toInt -0.5 (floor to -1)");

    // toFloat is implicitly tested by assertEqual
}

void testUnaryOperators() {
    printf("Testing Unary Operators...\n");

    Float a(1.5);
    Float neg_a = -a;
    assertEqual(neg_a, -1.5, "Unary minus");

    Float b(-2.0);
    Float abs_b = b.abs();
    assertEqual(abs_b, 2.0, "abs(-2.0)");

    Float c(3.0);
    Float abs_c = c.abs();
    assertEqual(abs_c, 3.0, "abs(3.0)");
}

void testFloatFloatArithmetic() {
    printf("Testing Float-Float Arithmetic...\n");

    Float f2(2.0);
    Float f3(3.0);
    Float f4(4.0);
    Float f0_5(0.5);

    // Add
    assertEqual(f2 + f3, 5.0, "2 + 3");
    assertEqual(f2 + Float(-1.0), 1.0, "2 + (-1)");

    // Subtract
    assertEqual(f4 - f2, 2.0, "4 - 2");
    assertEqual(f2 - f4, -2.0, "2 - 4");

    // Multiply
    assertEqual(f2 * f3, 6.0, "2 * 3");
    assertEqual(f2 * f0_5, 1.0, "2 * 0.5");
    assertEqual(Float(-2.0) * f3, -6.0, "-2 * 3");

    // Divide
    assertEqual(f4 / f2, 2.0, "4 / 2");
    assertEqual(f3 / f2, 1.5, "3 / 2");
    assertEqual(f4 / f0_5, 8.0, "4 / 0.5");
}

void testMixedArithmetic() {  // Float and int
    printf("Testing Mixed Arithmetic...\n");

    Float f2(2.0);
    Float f3(3.0);
    int   i2 = 2;
    int   i3 = 3;

    // Float op int
    assertEqual(f2 * i3, 6.0, "Float(2) * 3");
    assertEqual(f3 / i2, 1.5, "Float(3) / 2");  // 3.0 / 2 = 1.5
    assertEqual(f2 + i3, 5.0, "Float(2) + 3");
    assertEqual(f2 - i3, -1.0, "Float(2) - 3");

    // int op Float
    assertEqual(i3 * f2, 6.0, "3 * Float(2)");
    assertEqual(i3 / f2, 1.5, "3 / Float(2)");  // 3 / 2.0 = 1.5
    assertEqual(i3 + f2, 5.0, "3 + Float(2)");
    assertEqual(i3 - f2, 1.0, "3 - Float(2)");
}

void testComparisons() {
    printf("Testing Comparisons...\n");

    Float f1(1.0);
    Float f2(2.0);
    Float f1_dup(1.0);

    // Float vs Float
    assertTrue(f1 < f2, "1 < 2");
    assertFalse(f2 < f1, "2 < 1");
    assertTrue(f2 > f1, "2 > 1");
    assertTrue(f1 <= f2, "1 <= 2");
    assertTrue(f1 <= f1_dup, "1 <= 1");
    assertTrue(f1 >= f1_dup, "1 >= 1");
    assertTrue(f1 != f2, "1 != 2");
    assertFalse(f1 != f1_dup, "1 != 1");

    // Float vs Int
    assertTrue(f1 < 2, "1.0 < 2");
    assertTrue(f1 > 0, "1.0 > 0");
    assertTrue(f2 <= 2, "2.0 <= 2");
    assertTrue(f2 >= 2, "2.0 >= 2");
    assertTrue(f1 != 2, "1.0 != 2");
    assertFalse(f2 != 2, "2.0 != 2");
}

void testCompoundAssignment() {
    printf("Testing Compound Assignment...\n");

    Float f(1.0);
    f += Float(2.0);
    assertEqual(f, 3.0, "1 += 2");

    f -= Float(1.0);
    assertEqual(f, 2.0, "3 -= 1");

    f += 2;  // + int
    assertEqual(f, 4.0, "2 += 2");

    f -= 1;  // - int
    assertEqual(f, 3.0, "4 -= 1");

    // operator= (assignment from raw int shifted?)
    // Float& operator=(const int32_t rhs) { v = rhs << FRACTION_BITS; ... }
    Float f2;
    f2 = 5;
    assertEqual(f2, 5.0, "op= 5");
}

int main(int argc, char* argv[]) {
    testConstructorsAndConversions();
    testUnaryOperators();
    testFloatFloatArithmetic();
    testMixedArithmetic();
    testComparisons();
    testCompoundAssignment();

    printf("\nTest Summary: %d Passed, %d Failed\n", g_tests_passed, g_tests_failed);

    return g_tests_failed > 0 ? 1 : 0;
}