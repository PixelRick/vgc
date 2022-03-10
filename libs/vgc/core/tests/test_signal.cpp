// Copyright 2021 The VGC Developers
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
#include <memory>
#include <vgc/core/signal.h>
#include <vgc/core/object.h>

TEST(TestSignal, TestConnectSlot)
{
    vgc::core::internal::TestSignalObject<> t;

    VGC_CONNECT(&t, signalIntDouble, &t, slotUInt);
    VGC_DISCONNECT(&t, signalIntDouble, &t, slotUInt);
    t.signalIntDouble.emit(0, 1);
    ASSERT_FALSE(t.slotUIntCalled);

    auto h = VGC_CONNECT(&t, signalIntDouble, &t, slotUInt);
    VGC_DISCONNECT(&t, signalIntDouble, h);
    t.signalIntDouble.emit(0, 1);
    ASSERT_FALSE(t.slotUIntCalled);

    VGC_CONNECT(&t, signalIntDouble, &t, slotUInt);
    t.signalIntDouble.emit(0, 1);
    ASSERT_TRUE(t.slotUIntCalled);

    t.selfConnect();
    t.signalIntDouble.emit(0, 1);
    ASSERT_TRUE(t.slotIntDoubleCalled);
    ASSERT_TRUE(t.slotIntCalled);
    ASSERT_TRUE(t.slotUIntCalled);
    ASSERT_TRUE(t.sfnIntCalled);
    ASSERT_TRUE(t.fnIntDoubleCalled);
    ASSERT_TRUE(t.fnUIntCalled);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
