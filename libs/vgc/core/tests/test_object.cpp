// Copyright 2022 The VGC Developers
// See the COPYRIGHT file at the top-level directory of this distribution
// and at https://github.com/vgc/vgc/blob/master/COPYRIGHT
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <vgc/core/object.h>

using vgc::core::format;
using vgc::core::Object;
using vgc::core::detail::ConstructibleTestObject;
using vgc::core::detail::ConstructibleTestObjectPtr;

TEST(TestObject, ClassName) {
    ConstructibleTestObjectPtr obj = ConstructibleTestObject::create();
    ASSERT_EQ(obj->className(), "ConstructibleTestObject");
}

TEST(TestObject, Format) {
    ConstructibleTestObjectPtr obj = ConstructibleTestObject::create();
    Object* parent = obj->parentObject();

    std::string objAddress = format("{}", fmt::ptr(obj.get()));
    ASSERT_GT(objAddress.size(), 2);
    ASSERT_EQ(objAddress.substr(0, 2), "0x");

    std::string s = format("The parent of {} is {}", ptr(obj), ptr(parent));

    std::string expectedResult = "The parent of <ConstructibleTestObject @ ";
    expectedResult += objAddress;
    expectedResult += "> is <Null Object>";

    ASSERT_EQ(s, expectedResult);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
