// fpvDROID radio: the pilots a scan finds, less a strong transmitter's copy, and the one it watches after.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace kp {

struct Pilot {
  double mhz = 0;        // measured carrier
  std::string name;      // nearest standard channel ("R4"), "" if none
  std::string std_name;  // NTSC / PAL
  float snr_db = 0;
  float lock = 0;
  int64_t seen_ms = 0;   // steady-clock ms of its last sighting (the status's age_s)
};

// A pilot taken as the copy of another (Pilots::add), for the log.
struct PilotCopy {
  double mhz, of_mhz;
};

// The pilots of the scan under way, or of the last one (list(): strongest first). Every frequency here is in MHz.
//
// add() takes each step's pilots. One within kSameMhz of a listed pilot is that pilot seen again (the steps overlap):
// it takes the listed one's place unless it is more than kWeakerDb weaker. Then a pilot within kCopyMhz of one at least
// kCopyDb stronger is taken as that one's copy (a strong analog VTX shows one beside itself: README) and leaves the
// list. They are weighed strongest first, each against the pilots kept before it that are not copies, so a copy hides
// nothing, and one seen before its VTX is listed until the VTX is seen. The pilot within kSameMhz of keep_mhz (the
// channel the user is on, or chose; 0: none) is never taken as a copy: whether it is one cannot be told for sure, and
// a scan does not move the user to another VTX. Kept so beside its VTX, it hides nothing either.
//
// Not thread-safe: the Radio keeps it under its lock.
class Pilots {
public:
  // Sightings this close are one VTX: two VTXs 4 MHz apart could not both give a picture.
  static constexpr double kSameMhz = 4;
  // A later sighting this much weaker is a worse view of the pilot (it sits elsewhere in the step), not news.
  static constexpr float kWeakerDb = 3;
  // FPVDesktopWiFiReceiver's: the copy it measured was 12 MHz from its VTX.
  static constexpr double kCopyMhz = 15;
  // FPVDesktopWiFiReceiver's, a dB under its copy's 3: a hump's peak moves by about half a dB from scan to scan.
  static constexpr float kCopyDb = 2;

  // A scan starts: what the last one found goes.
  void begin_scan() { _list.clear(); }
  // A step's pilots into the list (merge), then the copies out of it (drop_copies). Returns those copies.
  std::vector<PilotCopy> add(const std::vector<Pilot> &found, double keep_mhz);
  // What to watch once the scan has ended: the listed pilot within kSameMhz of resume_mhz (the channel watched before,
  // or asked for; 0: none), the last in the list if several are; else the strongest listed; else resume_mhz all the
  // same. 0: nothing to watch.
  double watch_after_scan(double resume_mhz) const;
  const std::vector<Pilot> &list() const { return _list; }

private:
  void merge(const std::vector<Pilot> &found);
  std::vector<PilotCopy> drop_copies(double keep_mhz);

  std::vector<Pilot> _list;
};

}  // namespace kp
