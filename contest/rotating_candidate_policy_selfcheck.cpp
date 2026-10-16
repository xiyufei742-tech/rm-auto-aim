#include "rotating_candidate_policy.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

using rotating_probe::Candidate;
using rotating_probe::CandidatePolicy;
using rotating_probe::Decision;
using rotating_probe::Event;

Candidate plate(double x, int name = 1, double confidence = 0.90) {
  return {1, name, 1, x, 540.0, confidence};
}

void check(bool condition, const char * message) {
  if (!condition) throw std::runtime_error(message);
}

void test_acquire_track_and_handoff() {
  CandidatePolicy policy;
  Decision d = policy.step(0.000, {plate(800)});
  check(d.event == Event::acquiring && d.acquisition_streak == 1 &&
        d.selected_index == -1, "first single candidate must only start acquisition");
  d = policy.step(0.033, {plate(805)});
  check(d.event == Event::acquiring && d.acquisition_streak == 2,
        "second single candidate must not lock");
  d = policy.step(0.066, {plate(810)});
  check(d.event == Event::acquired && d.locked_after && d.epoch == 1 &&
        d.selected_index == 0, "third consecutive single candidate must acquire");

  // 同一帧中远处新板的分数更高，锁仍应跟随靠近上帧的旧板。
  d = policy.step(0.099, {plate(1040, 1, 0.99), plate(815, 1, 0.85)});
  check(d.event == Event::tracked && d.selected_index == 1 && d.match_count == 1 &&
        d.step_px == 5, "higher score must not cause a far jump");
  d = policy.step(0.132, {plate(820), plate(1045, 1, 0.99)});
  check(d.event == Event::tracked && d.selected_index == 0 && d.epoch == 1,
        "candidate ordering must not affect the locked track");

  // 旧板消失时，只剩 200+ px 外的新板；不能在同一锁世代直接切过去。
  d = policy.step(0.165, {plate(1050)});
  check(d.event == Event::lost_no_match && !d.locked_after &&
        d.selected_index == -1 && d.epoch == 1,
        "far plate must cause lost lock, not a handoff");
  d = policy.step(0.198, {plate(1055)});
  check(d.event == Event::acquiring && d.selected_index == -1,
        "new plate requires a fresh acquisition streak");
  d = policy.step(0.231, {plate(1060)});
  check(d.event == Event::acquiring, "new plate needs three single frames");
  d = policy.step(0.264, {plate(1065)});
  check(d.event == Event::acquired && d.epoch == 2,
        "reacquired plate must have a new lock epoch");
}

void test_ambiguous_match_and_time_gap() {
  CandidatePolicy policy;
  policy.step(0.000, {plate(800)});
  policy.step(0.033, {plate(805)});
  policy.step(0.066, {plate(810)});
  auto d = policy.step(0.099, {plate(814), plate(820)});
  check(d.event == Event::lost_multiple_matches && d.match_count == 2 &&
        !d.locked_after && d.selected_index == -1,
        "two candidates in the gate must drop lock");
  d = policy.step(0.132, {});
  check(d.event == Event::waiting_missing, "missing candidate must keep lock clear");

  policy.step(0.165, {plate(820)});
  policy.step(0.198, {plate(825)});
  d = policy.step(0.231, {plate(830)});
  check(d.event == Event::acquired && d.epoch == 2, "reacquisition failed");
  d = policy.step(0.300, {plate(835)});
  check(d.event == Event::lost_time_gap && d.selected_index == -1 &&
        !d.locked_after, "over-60 ms frame gap must drop lock");
  d = policy.step(0.333, {plate(840)});
  check(d.event == Event::acquiring && d.acquisition_streak == 1,
        "time-gap frame must not count toward a new streak");
}

void test_class_and_invalid_inputs() {
  CandidatePolicy policy;
  policy.step(0.000, {plate(800)});
  policy.step(0.033, {plate(805)});
  auto d = policy.step(0.066, {plate(810, 2)});
  check(d.event == Event::acquiring && d.acquisition_streak == 1,
        "class change must restart acquisition");
  policy.step(0.099, {plate(815, 2)});
  d = policy.step(0.132, {plate(820, 2)});
  check(d.event == Event::acquired, "class-stable streak did not acquire");
  d = policy.step(0.165, {plate(825, 1)});
  check(d.event == Event::lost_no_match && d.selected_index == -1,
        "wrong class inside spatial gate must not match");
  d = policy.step(0.198, {plate(std::numeric_limits<double>::quiet_NaN())});
  check(d.event == Event::invalid_input && !d.locked_after,
        "nonfinite candidate must fail closed");
  d = policy.step(0.198, {plate(830)});
  check(d.event == Event::acquiring, "valid input after reset must restart cleanly");
  d = policy.step(0.198, {plate(835)});
  check(d.event == Event::invalid_input && d.selected_index == -1,
        "non-increasing capture time must fail closed");
}

}  // namespace

int main() {
  try {
    test_acquire_track_and_handoff();
    test_ambiguous_match_and_time_gap();
    test_class_and_invalid_inputs();
    std::cout << "rotating candidate policy selfcheck passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "rotating candidate policy selfcheck failed: " << error.what() << '\n';
    return 1;
  }
}
