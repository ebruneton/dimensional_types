/**
 * Copyright (c) 2016 Eric Bruneton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "math/scalar_function.h"

#include <string>

#include "test/test_case.h"

namespace dimensional {

class ScalarFunctionTest : public TestCase {
 public:
  template<typename T>
  ScalarFunctionTest(const std::string& name, T test)
      : TestCase("ScalarFunctionTest " + name, static_cast<Test>(test)) {}

  void TestSamples() {
    ScalarFunction<0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 40, 360, 830> f;
    ExpectEquals(40, f.size());
    ExpectEquals(360.0, f.GetSample(0)());
    ExpectEquals(454.0, f.GetSample(8)());
    ExpectEquals(548.0, f.GetSample(16)());
    ExpectEquals(830.0, f.GetSample(40)());
  }

  void TestNewFunction() {
    typedef Scalar<1, 3, 5, 7, 9> out;
    std::vector<float> values;
    values.push_back(10.0);
    values.push_back(20.0);
    values.push_back(30.0);
    values.push_back(40.0);
    ScalarFunction<0, 2, 4, 6, 8, 1, 3, 5, 7, 9, 40, 360, 830>
        f(values, out::Unit());
    ExpectEquals(10.0 * out::Unit(), f[0]);
    ExpectEquals(20.0 * out::Unit(), f[1]);
    ExpectEquals(30.0 * out::Unit(), f[2]);
    ExpectEquals(40.0 * out::Unit(), f[3]);
    ExpectEquals(0.0 * out::Unit(), f[4]);

    std::vector<double> outputs = f.to(out::Unit());
    ExpectEquals(40, outputs.size());
    ExpectEquals(10.0, outputs[0]);
    ExpectEquals(20.0, outputs[1]);
    ExpectEquals(30.0, outputs[2]);
    ExpectEquals(40.0, outputs[3]);
    ExpectEquals(0.0, outputs[4]);
  }

  void TestNewInterpolatedFunction() {
    typedef Scalar<0, 2, 4, 6, 8> in;
    typedef Scalar<1, 3, 5, 7, 9> out;
    std::vector<in> inputs;
    std::vector<out> outputs;
    inputs.push_back(454.0 * in::Unit());
    inputs.push_back(548.0 * in::Unit());
    outputs.push_back(10.0 * out::Unit());
    outputs.push_back(20.0 * out::Unit());
    ScalarFunction<0, 2, 4, 6, 8, 1, 3, 5, 7, 9, 40, 360, 830>
        f(inputs, outputs);
    ExpectEquals(10.0 * out::Unit(), f[0]);
    ExpectEquals(10.0 * out::Unit(), f[8]);
    ExpectEquals(15.0 * out::Unit(), f[12]);
    ExpectEquals(20.0 * out::Unit(), f[16]);
    ExpectEquals(20.0 * out::Unit(), f[39]);
  }

  void TestNewUniformFunction() {
    typedef Scalar<0, 2, 4, 6, 8> in;
    typedef Scalar<1, 3, 5, 7, 9> out;
    std::vector<out> values;
    values.push_back(0.0 * out::Unit());
    values.push_back(10.0 * out::Unit());
    values.push_back(30.0 * out::Unit());
    values.push_back(0.0 * out::Unit());
    ScalarFunction<0, 2, 4, 6, 8, 1, 3, 5, 7, 9, 40, 360, 830>
        f(300 * in::Unit(), 600.0 * in::Unit(), values);
    ExpectNear(6.0, f[0].to(out::Unit()), 1e-6);
    ExpectNear(15.6, f[16].to(out::Unit()), 1e-6);
    ExpectEquals(0.0 * out::Unit(), f[39]);
    for (unsigned int i = 0; i < f.size(); ++i) {
      ExpectEquals(f[i], f(f.GetSample(i)));
    }
  }

  void TestOperators() {
    ScalarFunction<0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 100> f;
    ScalarFunction<0, 1, 2, 3, 4, 9, 8, 7, 6, 5, 10, 0, 100> g;
    for (unsigned int i = 0; i < f.size(); ++i) {
      f[i] = i * Scalar<5, 6, 7, 8, 9>::Unit();
      g[i] = i * Scalar<9, 8, 7, 6, 5>::Unit();
    }
    f += f;
    ScalarFunction<0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 100> h = -f;
    ScalarFunction<0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 100> u = f + h;
    ScalarFunction<0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 100> v = u - f;
    ScalarFunction<0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 100> w = v * 2.0;
    ScalarFunction<0, 1, 2, 3, 4, 18, 16, 14, 12, 10, 10, 0, 100> x =
        g * (2.0 * Scalar<9, 8, 7, 6, 5>::Unit());
    ScalarFunction<0, 1, 2, 3, 4, 18, 16, 14, 12, 10, 10, 0, 100> y = g * g;
    ScalarFunction<0, 1, 2, 3, 4, 0, 0, 0, 0, 0, 10, 0, 100> z = y / x;
    for (unsigned int i = 0; i < f.size(); ++i) {
      ExpectEquals(2.0 * i * Scalar<5, 6, 7, 8, 9>::Unit(), f[i]);
      ExpectEquals(-2.0 * i * Scalar<5, 6, 7, 8, 9>::Unit(), h[i]);
      ExpectEquals(0.0 * Scalar<5, 6, 7, 8, 9>::Unit(), u[i]);
      ExpectEquals(-2.0 * i * Scalar<5, 6, 7, 8, 9>::Unit(), v[i]);
      ExpectEquals(-4.0 * i * Scalar<5, 6, 7, 8, 9>::Unit(), w[i]);
      ExpectEquals(2.0 * i * Scalar<18, 16, 14, 12, 10>::Unit(), x[i]);
      ExpectEquals(i * i * Scalar<18, 16, 14, 12, 10>::Unit(), y[i]);
      if (i > 0) {
        ExpectEquals(i / 2.0, z[i]());
      }
    }
  }

  void TestFunctions() {
    ScalarFunction<0, 1, 2, 3, 4, 0, 0, 0, 0, 0, 10, 0, 100> f, g;
    for (unsigned int i = 0; i < f.size(); ++i) {
      f[i] = i;
      g[i] = f.size() - 1 - i;
    }
    ScalarFunction<0, 1, 2, 3, 4, 0, 0, 0, 0, 0, 10, 0, 100> u = exp(f * 1e-2);
    ScalarFunction<0, 1, 2, 3, 4, 0, 0, 0, 0, 0, 10, 0, 100> v = min(f, g);
    for (unsigned int i = 0; i < f.size(); ++i) {
      ExpectEquals(std::exp(i * 1e-2), u[i]());
      ExpectEquals(std::min(i, f.size() - 1 - i), v[i]());
    }
  }

  void TestIntegral() {
    ScalarFunction<0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 40, 360, 830> f;
    for (unsigned int i = 0; i < f.size(); ++i) {
      f[i] = (i < 8 ? 1.0 : 0.0) * Scalar<5, 6, 7, 8, 9>::Unit();
    }
    ExpectEquals(94.0 * Scalar<5, 7, 9, 11, 13>::Unit(), Integral(f));
  }
};

namespace {

ScalarFunctionTest samples("samples", &ScalarFunctionTest::TestSamples);
ScalarFunctionTest newfunction(
    "newfunction", &ScalarFunctionTest::TestNewFunction);
ScalarFunctionTest newinterpolatedfunction(
    "newinterpolatedfunction",
    &ScalarFunctionTest::TestNewInterpolatedFunction);
ScalarFunctionTest newuniformspectrum(
    "newuniformfunction", &ScalarFunctionTest::TestNewUniformFunction);
ScalarFunctionTest operators("operators", &ScalarFunctionTest::TestOperators);
ScalarFunctionTest functions("functions", &ScalarFunctionTest::TestFunctions);
ScalarFunctionTest integral("integral", &ScalarFunctionTest::TestIntegral);

}  // anonymous namespace

}  // namespace dimensional
