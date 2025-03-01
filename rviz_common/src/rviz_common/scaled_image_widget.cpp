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


#include "scaled_image_widget.hpp"

#include <QPainter>
#include <QWidget>

namespace rviz_common
{

ScaledImageWidget::ScaledImageWidget(float scale, QWidget * parent)
: QWidget(parent),
  scale_(scale)
{
}

ScaledImageWidget::~ScaledImageWidget()
{
}

void ScaledImageWidget::setImage(QPixmap image)
{
  image_ = image;
  update();
}

QSize ScaledImageWidget::sizeHint() const
{
  return image_.size() * scale_;
}

void ScaledImageWidget::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  if (!image_.isNull()) {
    QSize dest_size = image_.size();
    dest_size.scale(width(), height(), Qt::KeepAspectRatio);
    QRect dest_rect(width() / 2 - dest_size.width() / 2,
      height() / 2 - dest_size.height() / 2,
      dest_size.width(),
      dest_size.height());

    QPainter painter(this);
    painter.drawPixmap(dest_rect, image_);
  }
}

}  // namespace rviz_common
