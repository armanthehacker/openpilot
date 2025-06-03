/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/longitudinal/custom_acc_increment.h"
#include "selfdrive/ui/sunnypilot/qt/offroad/settings/settings.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/scrollview.h"


class LongitudinalPanel : public QWidget {
  Q_OBJECT

public:
  explicit LongitudinalPanel(QWidget *parent = nullptr);
  void showEvent(QShowEvent *event) override;
  void refresh();

private:
  Params params;
  QStackedLayout *main_layout = nullptr;
  ScrollViewSP *cruisePanelScroller = nullptr;
  QWidget *cruisePanelScreen = nullptr;
  CustomAccIncrement *customAccIncrement = nullptr;
};
