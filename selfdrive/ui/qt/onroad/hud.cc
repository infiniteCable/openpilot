#include "selfdrive/ui/qt/onroad/hud.h"

#include <cmath>

#include "selfdrive/ui/qt/util.h"

constexpr int SET_SPEED_NA = 255;

HudRenderer::HudRenderer() {}

void HudRenderer::updateState(const UIState &s) {
  is_metric = s.scene.is_metric;
  status = s.status;

  const SubMaster &sm = *(s.sm);
  if (sm.rcv_frame("carState") < s.scene.started_frame) {
    is_cruise_set = false;
    set_speed = SET_SPEED_NA;
    speed = 0.0;
    return;
  }

  const auto &controls_state = sm["controlsState"].getControlsState();
  const auto &car_state = sm["carState"].getCarState();
  const auto &battery_data = car_state.getBatteryDetails();

  battery_details.capacity = battery_data.getCapacity();
  battery_details.charge = battery_data.getCharge();
  battery_details.soc = battery_data.getSoc();
  battery_details.temperature = battery_data.getTemperature();
  battery_details.heaterActive = battery_data.getHeaterActive();
  battery_details.voltage = battery_data.getVoltage();
  battery_details.current = battery_data.getCurrent();
  battery_details.power = battery_data.getPower();

  // Handle older routes where vCruiseCluster is not set
  set_speed = car_state.getVCruiseCluster() == 0.0 ? controls_state.getVCruiseDEPRECATED() : car_state.getVCruiseCluster();
  is_cruise_set = set_speed > 0 && set_speed != SET_SPEED_NA;

  if (is_cruise_set && !is_metric) {
    set_speed *= KM_TO_MILE;
  }

  // Handle older routes where vEgoCluster is not set
  v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  float v_ego = v_ego_cluster_seen ? car_state.getVEgoCluster() : car_state.getVEgo();
  speed = std::max<float>(0.0f, v_ego * (is_metric ? MS_TO_KPH : MS_TO_MPH));
}

void HudRenderer::draw(QPainter &p, const QRect &surface_rect) {
  p.save();

  // Draw header gradient
  QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, surface_rect.width(), UI_HEADER_HEIGHT, bg);


  drawSetSpeed(p, surface_rect);
  drawCurrentSpeed(p, surface_rect);

  auto params = Params();
  bool display_battery_details = params.getBool("BatteryDetails");
  if (display_battery_details) {
    drawBatteryDetailsPanel(p, surface_rect);
  }

  p.restore();
}

void HudRenderer::drawSetSpeed(QPainter &p, const QRect &surface_rect) {
  // Draw outer box + border to contain set speed
  const QSize default_size = {172, 204};
  QSize set_speed_size = is_metric ? QSize(200, 204) : default_size;
  QRect set_speed_rect(QPoint(60 + (default_size.width() - set_speed_size.width()) / 2, 45), set_speed_size);

  // Draw set speed box
  p.setPen(QPen(QColor(255, 255, 255, 75), 6));
  p.setBrush(QColor(0, 0, 0, 166));
  p.drawRoundedRect(set_speed_rect, 32, 32);

  // Colors based on status
  QColor max_color = QColor(0xa6, 0xa6, 0xa6, 0xff);
  QColor set_speed_color = QColor(0x72, 0x72, 0x72, 0xff);
  if (is_cruise_set) {
    set_speed_color = QColor(255, 255, 255);
    if (status == STATUS_DISENGAGED) {
      max_color = QColor(255, 255, 255);
    } else if (status == STATUS_OVERRIDE) {
      max_color = QColor(0x91, 0x9b, 0x95, 0xff);
    } else {
      max_color = QColor(0x80, 0xd8, 0xa6, 0xff);
    }
  }

  // Draw "MAX" text
  p.setFont(InterFont(40, QFont::DemiBold));
  p.setPen(max_color);
  p.drawText(set_speed_rect.adjusted(0, 27, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("MAX"));

  // Draw set speed
  QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(set_speed)) : "–";
  p.setFont(InterFont(90, QFont::Bold));
  p.setPen(set_speed_color);
  p.drawText(set_speed_rect.adjusted(0, 77, 0, 0), Qt::AlignTop | Qt::AlignHCenter, setSpeedStr);
}

void HudRenderer::drawCurrentSpeed(QPainter &p, const QRect &surface_rect) {
  QString speedStr = QString::number(std::nearbyint(speed));

  p.setFont(InterFont(176, QFont::Bold));
  drawText(p, surface_rect.center().x(), 210, speedStr);

  p.setFont(InterFont(66));
  drawText(p, surface_rect.center().x(), 290, is_metric ? tr("km/h") : tr("mph"), 200);
}

void HudRenderer::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void HudRenderer::drawBatteryDetailsPanel(QPainter &p, const QRect &surface_rect) {
  const float scale_factor = 1.3;       // Erhöhe die Größe um 30 %
  const int panel_width = surface_rect.width() * 0.7;  // 70 % der Breite von rechts aus
  const int panel_margin = 20;          // Abstand vom unteren Rand
  const int line_height = 60;           // Größere Zeilenhöhe
  const int text_margin = 20;           // Abstand innerhalb der Spalten
  const int column_spacing = panel_width / 2 + 20;  // Abstand zwischen den beiden Spalten (+20 px zusätzliche Lücke)
  const int label_width = 200;          // Breite für Labels

  // Text styling für Labels und Werte
  QFont bold_font = InterFont(40, QFont::Bold);  // Fett für Labels
  QFont normal_font = InterFont(40, QFont::Normal);  // Normal für Werte

  QStringList labels = {
    "Capacity:", "Charge:", "SoC:", "Temperature:",
    "Heater Active:", "Voltage:", "Current:", "Power:"
  };

  QStringList values = {
    QString::number(battery_details.capacity, 'f', 2) + " Wh",
    QString::number(battery_details.charge, 'f', 2) + " Wh",
    QString::number(battery_details.soc, 'f', 2) + " %",
    QString::number(battery_details.temperature, 'f', 2) + " °C",
    battery_details.heaterActive ? "True" : "False",
    QString::number(battery_details.voltage, 'f', 2) + " V",
    QString::number(battery_details.current, 'f', 2) + " A",
    QString::number(battery_details.power, 'f', 2) + " kW",
  };

  // Zeichne die Werte in zwei Spalten
  for (int i = 0; i < labels.size(); ++i) {
    int column = i / 4; // Spalte: 0 für links, 1 für rechts
    int row = i % 4;    // Zeile innerhalb der Spalte

    // Position für die aktuelle Spalte und Zeile berechnen
    int text_x = surface_rect.width() - panel_width + column * column_spacing;
    int text_y = surface_rect.height() - panel_margin - static_cast<int>((line_height * (4 - row) * scale_factor));

    // **Label: Hintergrund und Text**
    QRect label_rect(text_x, text_y, label_width, static_cast<int>(line_height * scale_factor));

    // Hintergrund für das Label zeichnen
    p.save();
    p.setBrush(QColor(0, 0, 0, 128));  // Halbtransparenter schwarzer Hintergrund
    p.setPen(Qt::NoPen);
    p.fillRect(label_rect, p.brush());
    p.restore();  // Stellt sicher, dass der Text voll sichtbar bleibt

    // Text des Labels
    p.setFont(bold_font);
    p.setPen(Qt::white);
    p.drawText(label_rect, Qt::AlignLeft | Qt::AlignVCenter, labels[i]);

    // **Wert: Hintergrund und Text**
    QRect value_rect(text_x + label_width + text_margin, text_y, column_spacing - label_width - text_margin, static_cast<int>(line_height * scale_factor));

    // Hintergrund für den Wert zeichnen
    p.save();
    p.setBrush(QColor(0, 0, 0, 128));  // Halbtransparenter schwarzer Hintergrund
    p.setPen(Qt::NoPen);
    p.fillRect(value_rect, p.brush());
    p.restore();

    // Text des Werts
    p.setFont(normal_font);
    p.drawText(value_rect, Qt::AlignLeft | Qt::AlignVCenter, values[i]);
  }
}
