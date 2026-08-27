#include "tracker.h"

#include <cstring>

int8_t BeaconTracker::findSlotById(uint8_t beaconId) const
{
  for (int8_t i = 0; i < MAX_TRACKED_BEACONS; i++) {
    if (_beacons[i].inUse && _beacons[i].lastPacket.beaconId == beaconId) {
      return i;
    }
  }
  return -1;
}

int8_t BeaconTracker::findFreeOrOldestSlot() const
{
  for (int8_t i = 0; i < MAX_TRACKED_BEACONS; i++) {
    if (!_beacons[i].inUse) {
      return i;
    }
  }

  // Table pleine : on recycle l'entrée la plus ancienne pour laisser
  // la place à une balise active plutôt que d'ignorer silencieusement
  // un nouvel arrivant.
  int8_t oldestSlot = 0;
  uint32_t oldestSeen = _beacons[0].lastSeenMillis;

  for (int8_t i = 1; i < MAX_TRACKED_BEACONS; i++) {
    if (_beacons[i].lastSeenMillis < oldestSeen) {
      oldestSeen = _beacons[i].lastSeenMillis;
      oldestSlot = i;
    }
  }

  return oldestSlot;
}

bool BeaconTracker::handlePacket(const BeaconPacket &packet, int16_t rssi, float snr)
{
  int8_t slot = findSlotById(packet.beaconId);
  const bool isNewBeacon = (slot < 0);

  if (isNewBeacon) {
    slot = findFreeOrOldestSlot();
  }

  TrackedBeacon &tracked = _beacons[slot];
  tracked.inUse = true;
  tracked.lastPacket = packet;
  tracked.lastSeenMillis = millis();
  tracked.lastRssi = rssi;
  tracked.lastSnr = snr;

  if (_selectedSlot < 0) {
    _selectedSlot = slot; // première balise vue = cible par défaut
  }

  return isNewBeacon;
}

void BeaconTracker::updateGeometry(const GeoPoint &ownPosition)
{
  for (auto &beacon : _beacons) {
    if (!beacon.inUse) {
      continue;
    }

    const GeoPoint beaconPosition{
      beacon.lastPacket.latitudeE7 / 1e7,
      beacon.lastPacket.longitudeE7 / 1e7,
    };

    beacon.distanceMeters = haversineDistanceMeters(ownPosition, beaconPosition);
    beacon.bearingDegrees = initialBearingDegrees(ownPosition, beaconPosition);
    beacon.geometryValid = true;
  }
}

bool BeaconTracker::expireStaleBeacons()
{
  const uint32_t now = millis();
  bool selectedExpired = false;

  for (int8_t i = 0; i < MAX_TRACKED_BEACONS; i++) {
    if (_beacons[i].inUse && (now - _beacons[i].lastSeenMillis) > BEACON_TIMEOUT_MS) {
      _beacons[i] = TrackedBeacon{};

      if (_selectedSlot == i) {
        _selectedSlot = -1;
        selectedExpired = true;
      }
    }
  }

  if (_selectedSlot < 0) {
    // Reporte automatiquement la sélection sur la première balise restante.
    for (int8_t i = 0; i < MAX_TRACKED_BEACONS; i++) {
      if (_beacons[i].inUse) {
        _selectedSlot = i;
        break;
      }
    }
  }

  return selectedExpired;
}

int BeaconTracker::activeCount() const
{
  int total = 0;
  for (const auto &beacon : _beacons) {
    if (beacon.inUse) {
      total++;
    }
  }
  return total;
}

int BeaconTracker::listSortedByDistance(const TrackedBeacon *out[], int maxCount) const
{
  int written = 0;

  for (const auto &beacon : _beacons) {
    if (!beacon.inUse || written >= maxCount) {
      continue;
    }

    // Insertion triée : la table ne dépasse jamais MAX_TRACKED_BEACONS
    // entrées, un tri par insertion O(n²) est largement suffisant ici.
    int insertAt = written;
    while (insertAt > 0 && out[insertAt - 1]->distanceMeters > beacon.distanceMeters) {
      out[insertAt] = out[insertAt - 1];
      insertAt--;
    }
    out[insertAt] = &beacon;
    written++;
  }

  return written;
}

const TrackedBeacon *BeaconTracker::selectedBeacon() const
{
  if (_selectedSlot < 0 || !_beacons[_selectedSlot].inUse) {
    return nullptr;
  }
  return &_beacons[_selectedSlot];
}

void BeaconTracker::selectBeacon(const TrackedBeacon *beacon)
{
  if (beacon == nullptr) {
    return;
  }

  const auto index = beacon - &_beacons[0];
  if (index >= 0 && index < MAX_TRACKED_BEACONS) {
    _selectedSlot = static_cast<int8_t>(index);
  }
}

void BeaconTracker::selectNext()
{
  if (activeCount() == 0) {
    _selectedSlot = -1;
    return;
  }

  int8_t start = (_selectedSlot < 0) ? 0 : _selectedSlot;
  int8_t next = start;

  do {
    next = (next + 1) % MAX_TRACKED_BEACONS;
  } while (!_beacons[next].inUse && next != start);

  _selectedSlot = next;
}
