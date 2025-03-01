// Copyright (c) 2012, Willow Garage, Inc.
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

/*
 * display_visibility_manager.cpp
 *
 *  Created on: Aug 27, 2012
 *      Author: gossow
 */

#include <map>

#include "rviz_common/properties/display_group_visibility_property.hpp"

#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/bit_allocator.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_group.hpp"

namespace rviz_common
{

namespace properties
{

DisplayGroupVisibilityProperty::DisplayGroupVisibilityProperty(
  uint32_t vis_bit,
  DisplayGroup * display_group,
  Display * parent_display,
  const QString & name,
  bool default_value,
  const QString & description,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: DisplayVisibilityProperty(
    vis_bit, display_group, name, default_value, description, parent, changed_slot, receiver),
  display_group_(display_group),
  parent_display_(parent_display)
{
  connect(
    display_group, SIGNAL(displayAdded(Display*)),
    this, SLOT(onDisplayAdded(Display*)));
  connect(
    display_group, SIGNAL(displayRemoved(Display*)),
    this, SLOT(onDisplayRemoved(Display*)));

  for (int i = 0; i < display_group->numDisplays(); i++) {
    rviz_common::Display * display = display_group->getDisplayAt(i);
    if (display != parent_display) {
      onDisplayAdded(display);
    }
  }

  setDisableChildrenIfFalse(true);
}

void DisplayGroupVisibilityProperty::update()
{
  DisplayVisibilityProperty::update();
  for (auto const & it : disp_vis_props_) {
    it.second->update();
  }
}

void DisplayGroupVisibilityProperty::sortDisplayList()
{
  // remove and re-add everything in our property list
  // in the same order as it appears in the display group
  for (int i = 0; i < display_group_->numDisplays(); i++) {
    rviz_common::Display * display = display_group_->getDisplayAt(i);
    auto it = disp_vis_props_.find(display);
    if (it != disp_vis_props_.end()) {
      takeChild(it->second);
      addChild(it->second);
    }
  }
}

void DisplayGroupVisibilityProperty::onDisplayAdded(Display * display)
{
  auto display_group = qobject_cast<DisplayGroup *>(display);
  DisplayVisibilityProperty * vis_prop;
  if (display_group) {
    vis_prop = new DisplayGroupVisibilityProperty(
      vis_bit_, display_group, parent_display_, "",
      true, "Uncheck to hide everything in this Display Group", this);
  } else {
    vis_prop = new DisplayVisibilityProperty(
      vis_bit_, display, "", true,
      "Show or hide this Display", this);
  }
  disp_vis_props_[display] = vis_prop;
  sortDisplayList();
}

void DisplayGroupVisibilityProperty::onDisplayRemoved(Display * display)
{
  auto it = disp_vis_props_.find(display);
  if (it != disp_vis_props_.end()) {
    Property * child = takeChild(it->second);
    child->setParent(nullptr);
    delete child;
    disp_vis_props_.erase(display);
  }
}


DisplayGroupVisibilityProperty::~DisplayGroupVisibilityProperty() = default;

}  // namespace properties

}  // namespace rviz_common
