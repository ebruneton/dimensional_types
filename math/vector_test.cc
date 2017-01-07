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

#include "math/vector.h"

#include <string>

#include "test/test_case.h"

namespace dimensional {

class VectorTest : public TestCase {
 public:
  template<typename T>
  VectorTest(const std::string& name, T test)
      : TestCase("VectorTest " + name, static_cast<Test>(test)) {}

  void VectorTest2() {
    vec2 v(1.0, 2.0);
    ExpectEquals(1.0, v.x());
    ExpectEquals(2.0, v.y());

    Scalar<1, 2, 3, 4, 5> u = Scalar<1, 2, 3, 4, 5>::Unit();
    Scalar<10, 20, 30, 40, 50> t = Scalar<10, 20, 30, 40, 50>::Unit();
    Vector2<Scalar<1, 2, 3, 4, 5>> v1(2.0 * u, 1.0 * u);
    Vector2<Scalar<10, 20, 30, 40, 50>> v2(1.0 * t, 2.0 * t);
    Vector2<Scalar<9, 18, 27, 36, 45>> v3 = v2 / v1;
    ExpectEquals(0.5 * Scalar<9, 18, 27, 36, 45>::Unit(), v3.x);
    ExpectEquals(2.0 * Scalar<9, 18, 27, 36, 45>::Unit(), v3.y);
  }

  void VectorTest3() {
    Scalar<1, 2, 3, 4, 5> u = Scalar<1, 2, 3, 4, 5>::Unit();
    Vector3<Scalar<1, 2, 3, 4, 5>> v(1.0 * u, 2.0 * u, 3.0 * u);
    Vector3<Scalar<1, 2, 3, 4, 5>> v2 = v + v;
    Vector3<Scalar<1, 2, 3, 4, 5>> v3 = v - v * 2.0;
    Vector3<Scalar<11, 22, 33, 44, 55>> v4 =
        v2 * (2.0 * Scalar<10, 20, 30, 40, 50>::Unit());
    Vector3<Scalar<1, 2, 3, 4, 5>> v5 =
        v4 / (4.0 * Scalar<10, 20, 30, 40, 50>::Unit());
    Vector3<Scalar<1, 2, 3, 4, 5>> v6 = -v5;
    vec3 v7 = normalize(v6);

    ExpectEquals(1.0 * u, v.x);
    ExpectEquals(2.0 * u, v.y);
    ExpectEquals(3.0 * u, v.z);
    ExpectEquals(2.0 * u, v2.x);
    ExpectEquals(4.0 * u, v2.y);
    ExpectEquals(6.0 * u, v2.z);
    v2 += v3;
    ExpectEquals(1.0 * u, v2.x);
    ExpectEquals(2.0 * u, v2.y);
    ExpectEquals(3.0 * u, v2.z);
    ExpectEquals(-1.0 * u, v3.x);
    ExpectEquals(-2.0 * u, v3.y);
    ExpectEquals(-3.0 * u, v3.z);
    ExpectEquals(4.0 * Scalar<11, 22, 33, 44, 55>::Unit(), v4.x);
    ExpectEquals(8.0 * Scalar<11, 22, 33, 44, 55>::Unit(), v4.y);
    ExpectEquals(12.0 * Scalar<11, 22, 33, 44, 55>::Unit(), v4.z);
    ExpectEquals(1.0 * u, v5.x);
    ExpectEquals(2.0 * u, v5.y);
    ExpectEquals(3.0 * u, v5.z);
    ExpectEquals(-1.0 * u, v6.x);
    ExpectEquals(-2.0 * u, v6.y);
    ExpectEquals(-3.0 * u, v6.z);
    ExpectEquals(14.0 * Scalar<2, 4, 6, 8, 10>::Unit(), dot(v6, v6));
    ExpectEquals(sqrt(14.0) * u, length(v6));
    ExpectEquals(-1.0 / std::sqrt(14.0), v7.x());
    ExpectEquals(-2.0 / std::sqrt(14.0), v7.y());
    ExpectEquals(-3.0 / std::sqrt(14.0), v7.z());
  }

  void VectorTest4() {
    vec4 v(1.0, 2.0, 3.0, 4.0);
    ExpectEquals(1.0, v.x());
    ExpectEquals(2.0, v.y());
    ExpectEquals(3.0, v.z());
    ExpectEquals(4.0, v.w());

    Scalar<1, 2, 3, 4, 5> u = Scalar<1, 2, 3, 4, 5>::Unit();
    Scalar<10, 20, 30, 40, 50> t = Scalar<10, 20, 30, 40, 50>::Unit();
    Vector4<Scalar<1, 2, 3, 4, 5>> v1(8.0 * u, 4.0 * u, 2.0 * u, 1.0 * u);
    Vector4<Scalar<10, 20, 30, 40, 50>> v2(1.0 * t, 2.0 * t, 4.0 * t, 8.0 * t);
    Vector4<Scalar<9, 18, 27, 36, 45>> v3 = v2 / v1;
    ExpectEquals(0.125 * Scalar<9, 18, 27, 36, 45>::Unit(), v3.x);
    ExpectEquals(0.5 * Scalar<9, 18, 27, 36, 45>::Unit(), v3.y);
    ExpectEquals(2.0 * Scalar<9, 18, 27, 36, 45>::Unit(), v3.z);
    ExpectEquals(8.0 * Scalar<9, 18, 27, 36, 45>::Unit(), v3.w);
  }
};

namespace {

VectorTest vector2("vector2", &VectorTest::VectorTest2);
VectorTest vector3("vector3", &VectorTest::VectorTest3);
VectorTest vector4("vector4", &VectorTest::VectorTest4);

}  // anonymous namespace

}  // namespace dimensional
