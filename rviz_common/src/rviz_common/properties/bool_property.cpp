// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2017, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "rviz_common/properties/bool_property.hpp"

#include <QColor>

namespace rviz_common
{
namespace properties
{

BoolProperty::BoolProperty(
  const QString & name,
  bool default_value,
  const QString & description,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: Property(name, default_value, description, parent, changed_slot, receiver),
  disable_children_if_false_(false)
{
}

BoolProperty::~BoolProperty() = default;

bool BoolProperty::getBool() const
{
  return getValue().toBool();
}

void BoolProperty::setDisableChildrenIfFalse(bool disable)
{
  disable_children_if_false_ = disable;
}

bool BoolProperty::getDisableChildrenIfFalse()
{
  return disable_children_if_false_;
}

bool BoolProperty::getDisableChildren()
{
  if (disable_children_if_false_) {
    return !getBool() || Property::getDisableChildren();
  }
  return Property::getDisableChildren();
}

bool BoolProperty::setBool(bool value)
{
  return setValue(value);
}

}  // namespace properties
}  // namespace rviz_common
