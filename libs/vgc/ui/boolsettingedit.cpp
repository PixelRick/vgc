// Copyright 2023 The VGC Developers
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

#include <vgc/ui/boolsettingedit.h>

#include <vgc/ui/strings.h>

namespace vgc::ui {

BoolSettingEdit::BoolSettingEdit(CreateKey key, BoolSettingPtr setting)

    : SettingEdit(key, setting)
    , boolSetting_(setting) {

    addStyleClass(strings::BoolSettingEdit);

    toggle_ = createChild<Toggle>();
    toggle_->setState(setting->value());
    toggle_->toggled().connect(onToggleToggledSlot_());

    boolSetting_->valueChanged().connect(onBoolSettingValueChangedSlot_());
}

BoolSettingEditPtr BoolSettingEdit::create(BoolSettingPtr setting) {

    return core::createObject<BoolSettingEdit>(setting);
}

void BoolSettingEdit::onToggleToggled_(bool state) {
    boolSetting_->setValue(state);
}

void BoolSettingEdit::onBoolSettingValueChanged_(bool value) {
    toggle_->setState(value);
}

} // namespace vgc::ui
