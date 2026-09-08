#include <gtest/gtest.h>

#include "../include/function.h"

#include <stdlib.h>
#include <cmath>
#include <stdexcept>
#include <limits>

TEST(FunctionTests, InvalidSamplesPreserveCurve) {
    Function f;
    f.initialize(0, 1.0);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(f.closestSample(nan), -1);
    for (double invalid : {nan, std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()}) {
        EXPECT_THROW(f.addSample(invalid, 1), std::invalid_argument);
        EXPECT_THROW(f.addSample(0, invalid), std::invalid_argument);
    }
    EXPECT_EQ(f.closestSample(0), -1);
    f.addSample(0, 2);
    f.addSample(1, 4);
    EXPECT_THROW(f.addSample(0.5, nan), std::invalid_argument);
    EXPECT_THROW(f.addSample(nan, 100), std::invalid_argument);
    EXPECT_DOUBLE_EQ(f.sampleTriangle(0.5), 3);
    double low, high;
    f.getRange(&low, &high);
    EXPECT_DOUBLE_EQ(low, 2);
    EXPECT_DOUBLE_EQ(high, 4);
    f.destroy();
}

TEST(FunctionTests, ReinitializeSmallerCurve) {
    Function f;
    f.initialize(8, 1.0);
    for (int i = 0; i < 8; ++i) f.addSample(i, i + 2.0);
    double low, high;
    f.getRange(&low, &high);
    EXPECT_DOUBLE_EQ(low, 2.0);
    EXPECT_DOUBLE_EQ(high, 9.0);
    EXPECT_THROW(f.resize(1), std::invalid_argument);
    EXPECT_DOUBLE_EQ(f.sampleTriangle(7), 9.0);

    f.initialize(1, 1.0);
    f.getRange(&low, &high);
    EXPECT_DOUBLE_EQ(low, 0.0);
    EXPECT_DOUBLE_EQ(high, 0.0);
    f.addSample(0, -3);
    f.getRange(&low, &high);
    EXPECT_DOUBLE_EQ(low, -3.0);
    EXPECT_DOUBLE_EQ(high, -3.0);
    EXPECT_DOUBLE_EQ(f.sampleTriangle(0), -3.0);
    f.destroy();
}

TEST(FunctionTests, FunctionSanityCheck) {
    Function f;
    f.initialize(16, 1.0);
    f.destroy();
}

TEST(FunctionTests, FunctionTriangleFilterTest) {
    Function f;
    f.initialize(0, 1.0);
    for (int i = 0; i < 10; ++i) {
        f.addSample((double)i, (double)i * 2);
    }

    EXPECT_NEAR(f.sampleTriangle(-1.0), 0.0, 1E-6);
    EXPECT_NEAR(f.sampleTriangle(11.0), 18.0, 1E-6);

    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(f.sampleTriangle((double)i), (double)i * 2, 1E-6);
    }

    f.destroy();
}

TEST(FunctionTests, FunctionClosestTest) {
    Function f;
    f.initialize(0, 1.0);
    f.addSample(0.0, 1.0);
    f.addSample(2.0, 1.0);
    f.addSample(3.0, 1.1);
    f.addSample(1.0, 1.0);
    f.addSample(5.0, 10.0);
    f.addSample(4.0, 9.0);

    EXPECT_EQ(f.closestSample(2.4), 2);
    EXPECT_EQ(f.closestSample(6.0), 5);

    f.destroy();
}

TEST(FunctionTests, FunctionRandomAddTest) {
    Function f;
    f.initialize(0, 1.0);

    for (int i = 0; i < 1000; ++i) {
        f.addSample(rand() % 1000, i);
    }

    EXPECT_TRUE(f.isOrdered());

    f.destroy();
}

TEST(FunctionTests, FunctionGaussianTest) {
    Function f;
    f.initialize(0, 1.0);
    f.addSample(0.0, 1.0);
    f.addSample(2.0, 1.0);
    f.addSample(3.0, 5.0);
    f.addSample(1.0, 1.0);
    f.addSample(5.0, 10.0);
    f.addSample(4.0, 9.0);

    // Gaussian smoothing blends neighboring samples; it need not interpolate
    // the original data. Compare with the analytic, truncated Gaussian kernel.
    const double values[] = {1.0, 1.0, 1.0, 5.0, 9.0, 10.0};
    for (double x : {2.0, 2.5, 4.0}) {
        double sum = 0.0, weight = 0.0;
        for (int i = 0; i < 6; ++i) {
            const double d = i - x;
            const double w = std::fmax(0.0, std::exp(-d * d) - std::exp(-9.0));
            sum += w * values[i];
            weight += w;
        }
        EXPECT_NEAR(f.sampleGaussian(x), sum / weight, 1E-4);
    }
    EXPECT_NEAR(f.sampleGaussian(100.0), 10.0, 1E-3);
    EXPECT_NEAR(f.sampleGaussian(-100.0), 1.0, 1E-3);

    f.destroy();
}
