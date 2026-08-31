#include "display.h"
#include "config.h"

#include <cmath>
#include <cstdio>

namespace {

// ------------------------------------------------------------
// Géométrie de l'écran 320x240 paysage, reprise du gabarit
// "cadran rotatif" proposé initialement : barre supérieure de
// 30 px, cadran central de 130 px, bandeau inférieur de 80 px.
// ------------------------------------------------------------

constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;

constexpr int TOP_H    = 30;
constexpr int BOTTOM_H = 80;
constexpr int CENTER_H = SCREEN_H - TOP_H - BOTTOM_H; // 130 px
constexpr int BOTTOM_Y = SCREEN_H - BOTTOM_H;         // y = 160

constexpr int DIAL_CX = 160;
constexpr int DIAL_CY = 160;
constexpr int DIAL_RADIUS = 160;

constexpr int OUTER_TRIANGLE_H = 30;
constexpr int LOWER_ZONE_Y = BOTTOM_Y + OUTER_TRIANGLE_H; // 190
constexpr int LOWER_ZONE_H = 50;

constexpr int OUTER_ZONE_W = 80;
constexpr int LEFT_ZONE_X  = 0;
constexpr int RIGHT_ZONE_X = SCREEN_W - OUTER_ZONE_W; // 240
constexpr int MIDDLE_ZONE_X = OUTER_ZONE_W;           // 80
constexpr int MIDDLE_ZONE_W = SCREEN_W - 2 * OUTER_ZONE_W; // 160

constexpr int VALUE_BOX_W = 100;
constexpr int VALUE_BOX_H = 44;
constexpr int VALUE_BOX_X = (SCREEN_W - VALUE_BOX_W) / 2; // 110
constexpr int VALUE_BOX_Y = LOWER_ZONE_Y + 3;             // 193

constexpr int RED_TRIANGLE_HALF_W = 3;
constexpr int RED_TRIANGLE_HALF_H = 5;
constexpr int LEFT_TRIANGLE_X  = VALUE_BOX_X - 13;                   // 97
constexpr int RIGHT_TRIANGLE_X = VALUE_BOX_X + VALUE_BOX_W + 13;     // 223
constexpr int TRIANGLE_Y = LOWER_ZONE_Y + 25;                        // 215

constexpr int FRONT_MIN_ANGLE = 70;
constexpr int FRONT_MAX_ANGLE = 110;

constexpr uint16_t COLOR_BACKGROUND = 0x0841;
constexpr uint16_t COLOR_TOP        = 0x18E3;
// En mode simulation, la barre supérieure passe en orange sur tous les
// écrans : un rappel visuel permanent, impossible à manquer, que les
// données affichées ne viennent pas d'une vraie balise.
constexpr uint16_t COLOR_TOP_BAR    = DETECTOR_DEBUG_SIMULATION ? ILI9341_ORANGE : COLOR_TOP;
constexpr uint16_t COLOR_BOTTOM     = 0x2104;
constexpr uint16_t COLOR_ZONE_SIDE  = 0x2945;
constexpr uint16_t COLOR_BORDER     = ILI9341_DARKGREY;
constexpr uint16_t COLOR_ARC        = ILI9341_CYAN;
constexpr uint16_t COLOR_TICK       = 0xBDF7;
constexpr uint16_t COLOR_NEEDLE     = ILI9341_RED;
constexpr uint16_t COLOR_CENTER     = ILI9341_YELLOW;
constexpr uint16_t COLOR_TEXT       = ILI9341_WHITE;
constexpr uint16_t COLOR_VALUE      = ILI9341_GREEN;
constexpr uint16_t COLOR_STALE      = ILI9341_ORANGE;
constexpr uint16_t COLOR_DISPLAY    = ILI9341_BLACK;
constexpr uint16_t COLOR_TRIANGLE   = ILI9341_RED;
constexpr uint16_t COLOR_ELIMINATED = ILI9341_RED;
constexpr uint16_t COLOR_ROW_SELECT = 0x39C7;

int normalizeAngle360(int angle)
{
  int normalized = angle % 360;
  if (normalized < 0) normalized += 360;
  return normalized;
}

float degToRad(float degrees) { return degrees * static_cast<float>(M_PI) / 180.0f; }

void drawThickLine(Adafruit_GFX &gfx, int x0, int y0, int x1, int y1, uint16_t color, int thickness)
{
  const int half = thickness / 2;
  for (int offset = -half; offset <= half; offset++) {
    gfx.drawLine(x0 + offset, y0, x1 + offset, y1, color);
    gfx.drawLine(x0, y0 + offset, x1, y1 + offset, color);
  }
}

void drawSemicircle(Adafruit_GFX &gfx)
{
  for (int degree = 0; degree <= 180; degree++) {
    const float rad = degToRad(static_cast<float>(degree));
    const int x = DIAL_CX - lroundf(cosf(rad) * DIAL_RADIUS);
    const int y = DIAL_CY - lroundf(sinf(rad) * DIAL_RADIUS);
    gfx.drawPixel(x, y, COLOR_ARC);

    const int x2 = DIAL_CX - lroundf(cosf(rad) * (DIAL_RADIUS - 1));
    const int y2 = DIAL_CY - lroundf(sinf(rad) * (DIAL_RADIUS - 1));
    gfx.drawPixel(x2, y2, COLOR_ARC);
  }
}

void drawTicks(Adafruit_GFX &gfx)
{
  for (int degree = 0; degree <= 180; degree += 10) {
    const float rad = degToRad(static_cast<float>(degree));
    const int tickLength = (degree % 30 == 0) ? 12 : 7;
    const int outerRadius = DIAL_RADIUS - 3;
    const int innerRadius = outerRadius - tickLength;

    const int outerX = DIAL_CX - lroundf(cosf(rad) * outerRadius);
    const int outerY = DIAL_CY - lroundf(sinf(rad) * outerRadius);
    const int innerX = DIAL_CX - lroundf(cosf(rad) * innerRadius);
    const int innerY = DIAL_CY - lroundf(sinf(rad) * innerRadius);

    const uint16_t tickColor = (degree % 30 == 0) ? ILI9341_WHITE : COLOR_TICK;
    gfx.drawLine(innerX, innerY, outerX, outerY, tickColor);
  }
}

void drawNeedle(Adafruit_GFX &gfx, int angle)
{
  const float rad = degToRad(static_cast<float>(angle));
  const int needleLength = DIAL_RADIUS - 22;
  const int needleX = DIAL_CX - lroundf(cosf(rad) * needleLength);
  const int needleY = DIAL_CY - lroundf(sinf(rad) * needleLength);

  drawThickLine(gfx, DIAL_CX, DIAL_CY, needleX, needleY, COLOR_NEEDLE, 3);
  gfx.fillCircle(DIAL_CX, DIAL_CY, 7, COLOR_CENTER);
  gfx.drawCircle(DIAL_CX, DIAL_CY, 7, ILI9341_WHITE);
}

void drawDialLabels(Adafruit_GFX &gfx)
{
  gfx.setTextSize(1);
  gfx.setTextWrap(false);
  gfx.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);

  gfx.setCursor(6, DIAL_CY - 11);
  gfx.print("G");   // gauche

  gfx.setCursor(300, DIAL_CY - 11);
  gfx.print("D");   // droite

  gfx.setCursor(151, 10);
  gfx.print("^"); // face à la cible
}

void drawTriangleLeft(Adafruit_GFX &gfx, int centerX, int centerY)
{
  gfx.fillTriangle(
    centerX - RED_TRIANGLE_HALF_W, centerY,
    centerX + RED_TRIANGLE_HALF_W, centerY - RED_TRIANGLE_HALF_H,
    centerX + RED_TRIANGLE_HALF_W, centerY + RED_TRIANGLE_HALF_H,
    COLOR_TRIANGLE);
}

void drawTriangleRight(Adafruit_GFX &gfx, int centerX, int centerY)
{
  gfx.fillTriangle(
    centerX + RED_TRIANGLE_HALF_W, centerY,
    centerX - RED_TRIANGLE_HALF_W, centerY - RED_TRIANGLE_HALF_H,
    centerX - RED_TRIANGLE_HALF_W, centerY + RED_TRIANGLE_HALF_H,
    COLOR_TRIANGLE);
}

const char *teamLabel(uint8_t teamId)
{
  static char buffer[6];
  snprintf(buffer, sizeof(buffer), "EQ%u", teamId);
  return buffer;
}

const char *statusLabel(BeaconStatus status)
{
  switch (status) {
    case BeaconStatus::Active:     return "ACTIF";
    case BeaconStatus::Eliminated: return "HORS-JEU";
    case BeaconStatus::LowBattery: return "BATT.FAIBLE";
  }
  return "?";
}

} // namespace

Display::Display(SPIClass &spi)
  : _spi(spi),
    _tft(&spi, PIN_TFT_DC, PIN_TFT_CS, PIN_TFT_RST),
    _centerCanvas(SCREEN_W, CENTER_H)
{
}

void Display::begin()
{
  pinMode(PIN_TFT_LED, OUTPUT);
  digitalWrite(PIN_TFT_LED, HIGH);

  _spi.begin(PIN_TFT_SCLK, PIN_TFT_MISO, PIN_TFT_MOSI, PIN_TFT_CS);

  _tft.begin(40000000);
  _tft.setRotation(1); // paysage 320x240
  _tft.fillScreen(COLOR_BACKGROUND);
}

void Display::drawTopBar(bool gpsHasFix, uint8_t satellites, uint8_t batteryPercent,
                          int activeBeaconCount, const char *screenLabel)
{
  _tft.fillRect(0, 0, SCREEN_W, TOP_H, COLOR_TOP_BAR);
  _tft.drawFastHLine(0, TOP_H - 1, SCREEN_W, COLOR_BORDER);

  _tft.setTextWrap(false);
  _tft.setTextSize(1);

  char gpsText[16];
  snprintf(gpsText, sizeof(gpsText), "GPS:%s %us",
           gpsHasFix ? "OK" : "--", static_cast<unsigned>(satellites));
  _tft.setTextColor(gpsHasFix ? ILI9341_GREEN : ILI9341_RED, COLOR_TOP_BAR);
  _tft.setCursor(6, 10);
  _tft.print(gpsText);

  _tft.setTextColor(COLOR_TEXT, COLOR_TOP_BAR);
  _tft.setCursor(118, 10);
  _tft.print(screenLabel);

  char rightText[20];
  snprintf(rightText, sizeof(rightText), "B:%u%% N:%d", batteryPercent, activeBeaconCount);
  _tft.setTextColor(COLOR_TEXT, COLOR_TOP_BAR);
  _tft.setCursor(232, 10);
  _tft.print(rightText);
}

void Display::showCompassScreen(const CompassScreenData &data)
{
  char screenLabel[16];
  if (data.headingValid) {
    snprintf(screenLabel, sizeof(screenLabel), "CIBLE H:%03ld", lroundf(data.ownHeadingDeg));
  } else {
    snprintf(screenLabel, sizeof(screenLabel), "CIBLE H:---");
  }

  drawTopBar(data.gpsHasFix, data.satellites, data.batteryPercent,
             data.activeBeaconCount, screenLabel);

  // ---------------- Cadran central ----------------
  _centerCanvas.fillScreen(COLOR_BACKGROUND);

  if (data.hasTarget) {
    const int relative360 = normalizeAngle360(lroundf(data.relativeBearingDeg));
    const int displayedAngle = (relative360 <= 180) ? relative360 : (360 - relative360);

    drawSemicircle(_centerCanvas);
    drawTicks(_centerCanvas);
    drawDialLabels(_centerCanvas);
    drawNeedle(_centerCanvas, displayedAngle);
  } else {
    _centerCanvas.setTextSize(2);
    _centerCanvas.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
    _centerCanvas.setCursor(70, 55);
    _centerCanvas.print("AUCUNE CIBLE");
  }

  _tft.drawRGBBitmap(0, TOP_H, _centerCanvas.getBuffer(), SCREEN_W, CENTER_H);

  // ---------------- Bandeau inférieur ----------------
  _tft.fillRect(0, BOTTOM_Y, SCREEN_W, BOTTOM_H, COLOR_BACKGROUND);

  _tft.fillTriangle(0, BOTTOM_Y, 0, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_Y, COLOR_ZONE_SIDE);
  _tft.drawTriangle(0, BOTTOM_Y, 0, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_Y, COLOR_BORDER);
  _tft.fillRect(LEFT_ZONE_X, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_H, COLOR_ZONE_SIDE);
  _tft.drawRect(LEFT_ZONE_X, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_H, COLOR_BORDER);

  _tft.fillTriangle(SCREEN_W - 1, BOTTOM_Y, SCREEN_W - 1, LOWER_ZONE_Y, RIGHT_ZONE_X, LOWER_ZONE_Y, COLOR_ZONE_SIDE);
  _tft.drawTriangle(SCREEN_W - 1, BOTTOM_Y, SCREEN_W - 1, LOWER_ZONE_Y, RIGHT_ZONE_X, LOWER_ZONE_Y, COLOR_BORDER);
  _tft.fillRect(RIGHT_ZONE_X, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_H, COLOR_ZONE_SIDE);
  _tft.drawRect(RIGHT_ZONE_X, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_H, COLOR_BORDER);

  _tft.fillRect(MIDDLE_ZONE_X, LOWER_ZONE_Y, MIDDLE_ZONE_W, LOWER_ZONE_H, COLOR_BOTTOM);
  _tft.drawRect(MIDDLE_ZONE_X, LOWER_ZONE_Y, MIDDLE_ZONE_W, LOWER_ZONE_H, COLOR_BORDER);

  // Zone gauche : identité de la cible.
  _tft.setTextWrap(false);
  _tft.setTextSize(1);
  _tft.setTextColor(COLOR_TEXT, COLOR_ZONE_SIDE);
  if (data.hasTarget) {
    char idLine[10];
    snprintf(idLine, sizeof(idLine), "#%u", data.targetBeaconId);
    _tft.setCursor(LEFT_ZONE_X + 10, LOWER_ZONE_Y + 8);
    _tft.print(idLine);
    _tft.setCursor(LEFT_ZONE_X + 10, LOWER_ZONE_Y + 20);
    _tft.print(teamLabel(data.targetTeamId));

    const uint16_t statusColor = (data.targetStatus == BeaconStatus::Eliminated)
      ? COLOR_ELIMINATED
      : (data.targetSignalStale ? COLOR_STALE : ILI9341_GREEN);
    _tft.setTextColor(statusColor, COLOR_ZONE_SIDE);
    _tft.setCursor(LEFT_ZONE_X + 4, LOWER_ZONE_Y + 34);
    _tft.print(data.targetSignalStale ? "FAIBLE" : statusLabel(data.targetStatus));
  }

  // Zone droite : distance et niveau de signal.
  _tft.setTextColor(COLOR_TEXT, COLOR_ZONE_SIDE);
  if (data.hasTarget) {
    char distLine[12];
    if (data.targetDistanceMeters >= 1000.0) {
      snprintf(distLine, sizeof(distLine), "%.2fkm", data.targetDistanceMeters / 1000.0);
    } else {
      snprintf(distLine, sizeof(distLine), "%.0fm", data.targetDistanceMeters);
    }
    _tft.setCursor(RIGHT_ZONE_X + 6, LOWER_ZONE_Y + 8);
    _tft.print(distLine);

    char rssiLine[12];
    snprintf(rssiLine, sizeof(rssiLine), "%ddBm", data.targetRssi);
    _tft.setCursor(RIGHT_ZONE_X + 6, LOWER_ZONE_Y + 20);
    _tft.print(rssiLine);
  }

  // Zone centrale : relèvement numérique.
  _tft.fillRoundRect(VALUE_BOX_X, VALUE_BOX_Y, VALUE_BOX_W, VALUE_BOX_H, 4, COLOR_DISPLAY);
  _tft.drawRoundRect(VALUE_BOX_X, VALUE_BOX_Y, VALUE_BOX_W, VALUE_BOX_H, 4, ILI9341_LIGHTGREY);

  char valueText[8] = "---";
  int relative360 = 0;
  if (data.hasTarget) {
    relative360 = normalizeAngle360(lroundf(data.relativeBearingDeg));
    snprintf(valueText, sizeof(valueText), "%03d", relative360);
  }

  _tft.setTextSize(3);
  _tft.setTextColor(data.hasTarget ? COLOR_VALUE : ILI9341_DARKGREY, COLOR_DISPLAY);

  int16_t boundsX, boundsY;
  uint16_t textWidth, textHeight;
  _tft.getTextBounds(valueText, 0, 0, &boundsX, &boundsY, &textWidth, &textHeight);

  const int textX = VALUE_BOX_X + (VALUE_BOX_W - textWidth) / 2 - 3;
  const int textY = VALUE_BOX_Y + (VALUE_BOX_H - textHeight) / 2;
  _tft.setCursor(textX, textY);
  _tft.print(valueText);

  _tft.setTextSize(1);
  _tft.setCursor(VALUE_BOX_X + VALUE_BOX_W - 12, VALUE_BOX_Y + 6);
  _tft.print("o");

  // Petits triangles rouges indiquant si la cible est à gauche, à droite,
  // en face ou dans le dos, selon le relèvement relatif.
  if (data.hasTarget) {
    if (relative360 >= 180) {
      drawTriangleLeft(_tft, LEFT_TRIANGLE_X, TRIANGLE_Y);
      drawTriangleRight(_tft, RIGHT_TRIANGLE_X, TRIANGLE_Y);
    } else if (relative360 >= FRONT_MIN_ANGLE && relative360 <= FRONT_MAX_ANGLE) {
      drawTriangleRight(_tft, LEFT_TRIANGLE_X, TRIANGLE_Y);
      drawTriangleLeft(_tft, RIGHT_TRIANGLE_X, TRIANGLE_Y);
    } else if (relative360 < FRONT_MIN_ANGLE) {
      drawTriangleLeft(_tft, LEFT_TRIANGLE_X, TRIANGLE_Y);
      drawTriangleLeft(_tft, RIGHT_TRIANGLE_X, TRIANGLE_Y);
    } else {
      drawTriangleRight(_tft, LEFT_TRIANGLE_X, TRIANGLE_Y);
      drawTriangleRight(_tft, RIGHT_TRIANGLE_X, TRIANGLE_Y);
    }
  }
}

void Display::showBeaconList(const TrackedBeacon *beacons[], int count, int highlightIndex,
                              bool gpsHasFix, uint8_t satellites, uint8_t batteryPercent)
{
  drawTopBar(gpsHasFix, satellites, batteryPercent, count, "LISTE");

  constexpr int LIST_Y = TOP_H;
  constexpr int LIST_AREA_H = SCREEN_H - TOP_H;
  constexpr int ROW_H = 30;
  constexpr int VISIBLE_ROWS = LIST_AREA_H / ROW_H;

  _tft.fillRect(0, LIST_Y, SCREEN_W, LIST_AREA_H, COLOR_BACKGROUND);

  if (count == 0) {
    _tft.setTextSize(2);
    _tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
    _tft.setCursor(50, LIST_Y + 90);
    _tft.print("Aucune balise recue");
    return;
  }

  int firstVisible = 0;
  if (highlightIndex >= VISIBLE_ROWS) {
    firstVisible = highlightIndex - VISIBLE_ROWS + 1;
  }

  for (int row = 0; row < VISIBLE_ROWS; row++) {
    const int index = firstVisible + row;
    if (index >= count) {
      break;
    }

    const TrackedBeacon *beacon = beacons[index];
    const int rowY = LIST_Y + row * ROW_H;
    const bool selected = (index == highlightIndex);

    _tft.fillRect(0, rowY, SCREEN_W, ROW_H, selected ? COLOR_ROW_SELECT : COLOR_BACKGROUND);
    _tft.drawFastHLine(0, rowY + ROW_H - 1, SCREEN_W, COLOR_BORDER);

    const BeaconType type = static_cast<BeaconType>(beacon->lastPacket.type);
    const BeaconStatus status = static_cast<BeaconStatus>(beacon->lastPacket.status);

    char line1[24];
    snprintf(line1, sizeof(line1), "#%-3u %s %s",
             beacon->lastPacket.beaconId,
             teamLabel(beacon->lastPacket.teamId),
             (type == BeaconType::StaticBeacon) ? "BPS" : "BEM");

    char line2[24];
    if (beacon->distanceMeters >= 1000.0) {
      snprintf(line2, sizeof(line2), "%.2fkm  %ddBm", beacon->distanceMeters / 1000.0, beacon->lastRssi);
    } else {
      snprintf(line2, sizeof(line2), "%.0fm  %ddBm", beacon->distanceMeters, beacon->lastRssi);
    }

    _tft.setTextSize(1);
    _tft.setTextColor(COLOR_TEXT, selected ? COLOR_ROW_SELECT : COLOR_BACKGROUND);
    _tft.setCursor(8, rowY + 6);
    _tft.print(line1);
    _tft.setCursor(8, rowY + 18);
    _tft.print(line2);

    const uint16_t statusColor = (status == BeaconStatus::Eliminated) ? COLOR_ELIMINATED
      : (status == BeaconStatus::LowBattery ? COLOR_STALE : ILI9341_GREEN);
    _tft.setTextColor(statusColor, selected ? COLOR_ROW_SELECT : COLOR_BACKGROUND);
    _tft.setCursor(190, rowY + 12);
    _tft.print(statusLabel(status));
  }
}
