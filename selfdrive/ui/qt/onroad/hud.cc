#include "selfdrive/ui/qt/onroad/hud.h"

#include <cmath>

#include "selfdrive/ui/qt/util.h"

constexpr int SET_SPEED_NA = 255;

HudRenderer::HudRenderer() {
  img_battery_heater_enabled = loadPixmap("../assets/img_battery_heater_on.png", {img_size, img_size});
  img_battery_heater_disabled = loadPixmap("../assets/img_battery_heater_off.png", {img_size, img_size});
}

void HudRenderer::updateState(const UIState &s) {
  is_metric = s.scene.is_metric;
  status = s.status;

  const SubMaster &sm = *(s.sm);
  if (sm.rcv_frame("carState") < s.scene.started_frame) {
    is_cruise_set = false;
    set_speed = SET_SPEED_NA;
    speed = 0.0;
    battery_heater_enabled = false;
    return;
  }

  const auto &controls_state = sm["controlsState"].getControlsState();
  const auto &car_state = sm["carState"].getCarState();
  const auto &battery_data = car_state.getBatteryDetails();

  battery_details.heaterActive = battery_data.getHeaterActive();
  battery_details.capacity = battery_data.getCapacity();
  battery_details.charge = battery_data.getCharge();
  battery_details.soc = battery_data.getSoc();
  battery_details.temperature = battery_data.getTemperature();
  battery_details.cellVoltage = battery_data.getCellVoltage();
  battery_details.voltage = battery_data.getVoltage();
  battery_details.current = battery_data.getCurrent();
  battery_details.currentMax = battery_data.getCurrentMax();
  battery_details.power = battery_data.getPower();
  battery_details.powerMax = battery_data.getPowerMax();

  //bool battery_heater_state = battery_details.getHeaterActive();

  //if (battery_heater_state != battery_heater_enabled) {
  //  battery_heater_enabled = battery_heater_state;
  //  triggerParentUpdate();
  //}

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

void HudRenderer::drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity) {
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(opacity);
  p.drawPixmap(center - QPoint(img.width() / 2, img.height() / 2), img);
  p.setOpacity(1.0);
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
  
  //drawBatteryHeaterIcon(p, surface_rect);

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

void HudRenderer::drawBatteryHeaterIcon(QPainter &p, const QRect &surface_rect) {
  const int margin = 30;
  QPoint center(surface_rect.width() - margin - btn_size / 2, surface_rect.height() - margin - btn_size / 2);
  QBrush bg = QBrush(QColor(0, 0, 0, 70));
  float opacity = battery_heater_enabled ? 0.65f : 0.2f;
  QPixmap img = battery_heater_enabled ? img_battery_heater_enabled : img_battery_heater_disabled;
  drawIcon(p, center, img, bg, opacity);
}

void HudRenderer::triggerParentUpdate() {
  QWidget *widget = nullptr;
  QObject *current = parent();

  while (current) {
    widget = qobject_cast<QWidget *>(current);
    if (widget) {
      widget->update();
      break;
    }
    current = current->parent();
  }
}

void HudRenderer::drawBatteryDetailsPanel(QPainter &p, const QRect &surface_rect) {
  const float scale_factor = 0.75;
  const int panel_width = 600;
  const int panel_margin = 20;
  const int line_height = 45;
  const int text_margin = 20;
  const int label_width = 300;
  const int value_width = 250;

  int x = surface_rect.width() - panel_width - panel_margin;
  int y = surface_rect.height() - panel_margin - static_cast<int>((line_height * 11 + text_margin) * scale_factor);

  QRect panel_rect(x, y, panel_width, static_cast<int>((line_height * 11 + text_margin) * scale_factor));
  p.setBrush(QColor(0, 0, 0, 150));
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(panel_rect, 10, 10);

  p.setPen(Qt::white);
  p.setFont(InterFont(30, QFont::Bold));

  QStringList labels = {
    "Heater Active:", "Capacity:", "Charge:",
    "SoC:", "Temperature:", "Cell Voltage:",
    "Voltage:", "Current:", "Max Current:",
    "Power:", "Max Power:"
  };

  QStringList values = {
    battery_details.heaterActive ? "True" : "False",
    QString::number(battery_details.capacity, 'f', 2) + " Wh",
    QString::number(battery_details.charge, 'f', 2) + " Wh",
    QString::number(battery_details.soc, 'f', 2) + " %",
    QString::number(battery_details.temperature, 'f', 2) + " °C",
    QString::number(battery_details.cellVoltage, 'f', 2) + " V",
    QString::number(battery_details.voltage, 'f', 2) + " V",
    QString::number(battery_details.current, 'f', 2) + " A",
    QString::number(battery_details.currentMax, 'f', 2) + " A",
    QString::number(battery_details.power, 'f', 2) + " kW",
    QString::number(battery_details.powerMax, 'f', 2) + " kW"
  };

  for (int i = 0; i < labels.size(); ++i) {
    int text_y = y + text_margin + static_cast<int>(i * line_height * scale_factor);

    QRect label_rect(x + text_margin, text_y, label_width, static_cast<int>(line_height * scale_factor));
    p.drawText(label_rect, Qt::AlignLeft | Qt::AlignVCenter, labels[i]);
    
    QRect value_rect(x + label_width, text_y, value_width, static_cast<int>(line_height * scale_factor));
    p.drawText(value_rect, Qt::AlignRight | Qt::AlignVCenter, values[i]);
  }
}
