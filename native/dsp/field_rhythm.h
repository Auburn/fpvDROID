// fpvDROID decoder: the vertical flywheel. Lines freewheel when their hsync is not found; fields did not: without its
// vertical sync a field was not begun, and its lines, found or not, went undrawn (a weak signal that kept 86 % of
// its hsyncs showed a third of its pictures). The fields come at a fixed rhythm, though: so many lines from an odd
// field's start to the next field's, so many from an even one's, and which kind follows. The standard's interlace
// is taken as read from the first start found; a source with another rhythm shows it, and that is then learnt (seen
// twice running). With a rhythm, a field whose vertical sync has not shown kNear lines after it was due is begun
// where it was due, and a start found up to kNear lines late or as the other kind (a vertical sync damaged at its
// start) is put there. A start found early or far off is put right or passed over too, but kOff of those in a row,
// with none where the rhythm said, show that the source has another rhythm. A jump in the stream (a re-phase) loses
// the count of lines: the next start found is taken as it is.
//
// Only line numbers come in here, no samples: the decoder says where it found a start, and is told where the field
// begins.
#pragma once
#include <cstdint>
#include <optional>

namespace kp {

// Where a field begins, and which kind it is.
struct FieldStart {
  int kind;      // 0: odd (its broad pulses begin where a line does), 1: even (half a line in); -1: no field
  int64_t line;  // its first line, the first of its vertical sync, as the decoder counts lines
  bool operator==(const FieldStart &) const = default;
};

class FieldRhythm {
public:
  // Nothing is known any more (a new standard, a new acquisition).
  void reset() { *this = FieldRhythm{}; }
  // The count of lines is lost (a jump in the stream). That the source has shown a rhythm of its own is kept.
  void forget();
  // A field start has been found, s, while the field in_hand is being drawn (kind -1: there is none yet). Where the
  // field begins: at s, or where the rhythm said one was due; nothing: s is passed over.
  // field_lines: the lines in the standard's odd field (VideoStd::field_lines); its even one has one more.
  std::optional<FieldStart> found(FieldStart in_hand, FieldStart s, int64_t field_lines);
  // Line `line` has ended with no start found: the start of the field after in_hand, once that is kNear lines
  // overdue. Nothing before then, or without a rhythm.
  std::optional<FieldStart> overdue(FieldStart in_hand, int64_t line) const;

private:
  static constexpr int kNear = 2, kOff = 3;
  static constexpr int kBelieved = 2;        // times running that a step has to be seen
  static constexpr int64_t kMaxFields = 16;  // starts found further apart than this say nothing of the rhythm
  struct Step {
    int64_t len = 0;  // lines from a field's start to the next field's ...
    int next = 0;     // ... and the kind of that one
    int seen = 0;     // times running that this has been seen
  };
  Step _step[2];        // by the field's kind
  int _off = 0;         // starts found in a row that were not where the rhythm said
  bool _plain = false;  // the source has shown a rhythm that is not the standard's

  bool believed(int kind) const { return kind >= 0 && _step[kind].seen >= kBelieved; }
  FieldStart due_after(FieldStart f) const { return {_step[f.kind].next, f.line + _step[f.kind].len}; }
  void learn(FieldStart in_hand, FieldStart s, int64_t field_lines);
};

}  // namespace kp
