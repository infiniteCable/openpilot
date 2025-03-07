#pragma once

#include <QPainter>
#include "selfdrive/ui/ui.h"
#include "common/params.h"

class HudRenderer : public QObject {
  Q_OBJECT

public:
  HudRenderer();
  void updateState(const UIState &s);
  void draw(QPainter &p, const QRect &surface_rect);

private:
  void triggerParentUpdate();
  void drawSetSpeed(QPainter &p, const QRect &surface_rect);
  void drawCurrentSpeed(QPainter &p, const QRect &surface_rect);
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void drawBatteryDetailsPanel(QPainter &p, const QRect &surface_rect);

  float speed = 0;
  float set_speed = 0;
  bool is_cruise_set = false;
  bool is_cruise_available = true;
  bool is_metric = false;
  bool v_ego_cluster_seen = false;
  int status = STATUS_DISENGAGED;

  struct BatteryDetails {
    float capacity = 0.0f;       // Wh
    float charge = 0.0f;         // Wh
    float soc = 0.0f;            // %
    float temperature = 0.0f;    // °C
    bool heaterActive = false;
    float voltage = 0.0f;        // V
    float current = 0.0f;        // A
    float power = 0.0f;          // W
  };

  BatteryDetails battery_details;

};
