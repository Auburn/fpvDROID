// fpvDROID decoder: the vertical flywheel. See field_rhythm.h.
#include "field_rhythm.h"

#include <cmath>
#include <cstdlib>

#include "log.h"

namespace kp {

void FieldRhythm::forget() {
  _step[0].seen = _step[1].seen = 0;
  _off = 0;
}

std::optional<FieldStart> FieldRhythm::found(FieldStart in_hand, FieldStart s, int64_t field_lines) {
  if (!believed(in_hand.kind)) {
    // (no field is begun by the flywheel without a rhythm: the field in hand is the last one found)
    learn(in_hand, s, field_lines);
    return s;
  }
  const FieldStart due = due_after(in_hand);
  const int64_t late = s.line - due.line;
  if (s == due) {
    _off = 0;
  } else if ((late < 0 || late > kNear) && ++_off >= kOff) {
    // early, or far off, kOff times running: the source's rhythm is another
    KP_LOGD("video: the fields' rhythm is forgotten (a start %lld lines from where it was due)",
            static_cast<long long>(late));
    forget();
    _plain = true;
    return s;
  }
  if (std::llabs(late) > kNear) return std::nullopt;
  // where one was due, or within a line or two of it, or there as the other kind: what a vertical sync damaged at
  // its start looks like
  return due;
}

// A start found with no rhythm to judge it by: what it says of the rhythm.
void FieldRhythm::learn(FieldStart in_hand, FieldStart s, int64_t field_lines) {
  const int kind = in_hand.kind;
  const int64_t len = kind < 0 ? 0 : s.line - in_hand.line;  // lines since the last start found ...
  const int64_t fields = std::llround(static_cast<double>(len) / (static_cast<double>(field_lines) + 0.5));  // ... fields
  // The standard's rhythm: an odd field of field_lines lines, then an even one of one more. Does it lead from the
  // last start found to this one?
  const bool leads = kind >= 0 && fields >= 0 && fields <= kMaxFields &&
                     len == fields * field_lines + (fields + kind) / 2 && (kind + fields) % 2 == s.kind;
  if (!_plain || leads) {
    // taken as read from the first start found, until the source shows another
    _plain = false;
    for (int k = 0; k < 2; k++) _step[k] = Step{field_lines + k, 1 - k, kBelieved};
  } else if (kind >= 0 && fields >= 1 && fields <= kMaxFields && (fields == 1 || s.kind == kind) &&
             len % fields == 0 && std::llabs(len / fields - field_lines) <= 3) {
    // another, seen twice running: the next field's start found (about a field on), or a later one of the same
    // kind a whole number of fields on, which says how long each is where the fields are all alike
    const Step now{len / fields, s.kind, 1};
    Step &st = _step[kind];
    if (st.seen > 0 && st.len == now.len && st.next == now.next) st.seen = kBelieved;
    else st = now;
  }
}

std::optional<FieldStart> FieldRhythm::overdue(FieldStart in_hand, int64_t line) const {
  if (!believed(in_hand.kind)) return std::nullopt;
  const FieldStart due = due_after(in_hand);
  if (line < due.line + kNear) return std::nullopt;
  return due;
}

}  // namespace kp
