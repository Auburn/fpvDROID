// fpvDROID radio: the pilots the scans have found, and which of them is only a copy of another.
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
  int64_t seen_ms = 0;   // steady-clock ms of the last scan that saw it
};

// A pilot taken as the copy of another (add_sightings), for the log.
struct PilotCopy {
  double mhz, of_mhz;
};

// The list of every pilot the scans have seen (`list`, strongest first) takes a scan step's sightings (`found`, each
// seen_ms now). The scan under way began at scan_ms: what an earlier scan saw is history, which the app does not show.
//
// A sighting within kSameMhz of a listed pilot is that pilot seen again. Seen twice in one scan (two steps overlap
// by 13 MHz), the newer sighting takes the older one's place unless it is more than kWeakerDb weaker, when it only
// says the pilot is still there: the better measurement is kept. A sighting always takes the place of one from an
// earlier scan, which says nothing about now.
//
// A strong analog VTX can show a copy of itself beside it, whose video passes the sync check too. Measured by
// FPVDesktopWiFiReceiver (its PR 5): a VTX on R3 at 5731.4 MHz, 35 dB, and a copy at 5743.4 MHz, 32 dB, lock 0.75,
// listed as a second pilot, A7. The copy sat 12.0 MHz above the VTX. It was not the VTX's mirror image about the
// centre of the step that saw it, which an IQ imbalance in the chip would make: every hardware centre is on the 5
// MHz grid, and the two are mirrored about 5737.4 MHz (a hump's centroid mirrors to within 0.1 MHz). So a copy is
// told by being close to a stronger pilot, not by where it sits, and it may well be seen in another step than its
// VTX (a step sees +-16.5 MHz; they are 20 MHz apart). After every step, a pilot of this scan within kCopyMhz of one
// of this scan that is kCopyDb or more stronger is taken as that one's copy and leaves the list (two real VTXs that
// close would wreck each other's picture). kCopyDb is FPVDesktopWiFiReceiver's, a dB short of what its copy measured:
// the SNRs are hump peaks, which move by about half a dB from scan to scan, and by more with where in the step a hump
// sits. The strongest are weighed first, each against the pilots kept before it only, so that a copy does not take a
// third pilot further away with it. The order the steps come in does not matter: a copy seen before its VTX is listed
// until the VTX is seen.
// Never the pilot within kSameMhz of keep_mhz (the channel the user is on, or chose; 0: none): whether it is a copy
// cannot be told for sure, and a scan does not take the user off their channel to put them on another VTX. A copy
// kept so is listed beside its VTX.
//
// Returns the pilots taken as copies.
constexpr double kSameMhz = 4;
constexpr float kWeakerDb = 3;
constexpr double kCopyMhz = 15;
constexpr float kCopyDb = 2;
std::vector<PilotCopy> add_sightings(std::vector<Pilot> &list, const std::vector<Pilot> &found, int64_t scan_ms,
                                     double keep_mhz);

}  // namespace kp
