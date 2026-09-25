"""
plc_ladder_sim.py

A small ladder-logic simulator that models the robot-cell interlock
scenario described in Section 3.4 of the internship report: a PLC
coordinating a Start push-button, a Part-Present sensor, and a safety
interlock (E-Stop / light curtain) to decide when a robot cycle is
permitted to run -- and raising an alarm if the part-present sensor
fails to trigger within an expected time window (Section 3.4.4).

Modelled on Mitsubishi MELSEC FX-style ladder logic conventions:
  - Rungs are evaluated top-to-bottom, once per scan cycle.
  - A coil is energised (TRUE) only while its logic network is TRUE.
  - NO (normally-open) contacts pass power when the referenced bit is TRUE.
  - NC (normally-closed) contacts pass power when the referenced bit is FALSE.
  - A rung's network is a list of parallel branches (OR); each branch is
    a series of contacts (AND) -- exactly how a real seal-in / latch
    circuit is drawn: a main path plus a parallel "hold" path.

Sequence modelled:
    START_PB pressed  -->  CYCLE_REQUEST latches (robot is "waiting for
    part")  -->  PART_PRESENT arrives  -->  ROBOT_RUN latches (robot is
    cycling)  -->  CYCLE_COMPLETE resets ROBOT_RUN.
    If PART_PRESENT does not arrive within `alarm_timeout_scans` scans of
    CYCLE_REQUEST latching, CYCLE_ALARM raises and cancels the request --
    the software equivalent of a TON (timer-on) instruction driving an
    alarm rung.

This is a simplified, single-file simulation for illustrative purposes,
not a production PLC runtime.
"""

from dataclasses import dataclass
from typing import Dict, List


@dataclass
class Contact:
    tag: str
    normally_closed: bool = False

    def evaluate(self, bits: Dict[str, bool]) -> bool:
        state = bits.get(self.tag, False)
        return (not state) if self.normally_closed else state


@dataclass
class Rung:
    name: str
    branches: List[List[Contact]]   # OR of ANDs, exactly like a ladder network
    coil: str

    def evaluate(self, bits: Dict[str, bool]) -> bool:
        return any(
            all(contact.evaluate(bits) for contact in branch)
            for branch in self.branches
        )


class PLC:
    """Minimal scan-cycle engine: read inputs -> evaluate rungs -> update outputs."""

    def __init__(self, rungs: List[Rung], alarm_timeout_scans: int = 3):
        self.rungs = rungs
        self.bits: Dict[str, bool] = {}
        self.alarm_timeout_scans = alarm_timeout_scans
        self._wait_scan_count = 0

    def scan(self, inputs: Dict[str, bool]) -> Dict[str, bool]:
        """Run exactly one PLC scan: merge new inputs, evaluate every rung in
        order, update the alarm timer, and return the full bit table."""
        self.bits.update(inputs)

        for rung in self.rungs:
            self.bits[rung.coil] = rung.evaluate(self.bits)

        # Alarm timer: counts scans spent "waiting for part" (request
        # latched, robot not yet running). Equivalent to a TON instruction
        # feeding an alarm rung. Once raised, the alarm latches (like a
        # real alarm coil) until CYCLE_COMPLETE acknowledges/resets it --
        # otherwise the alarm would self-clear the instant the seal-in
        # circuit it just tripped drops out.
        waiting = self.bits.get("CYCLE_REQUEST") and not self.bits.get("ROBOT_RUN")
        self._wait_scan_count = self._wait_scan_count + 1 if waiting else 0

        timed_out = self._wait_scan_count > self.alarm_timeout_scans
        newly_alarmed = timed_out and not self.bits.get("PART_PRESENT", False)
        alarm_ack = self.bits.get("CYCLE_COMPLETE", False)
        self.bits["CYCLE_ALARM"] = (self.bits.get("CYCLE_ALARM", False) or newly_alarmed) and not alarm_ack

        return dict(self.bits)


def build_robot_cell_plc(alarm_timeout_scans: int = 3) -> PLC:
    """
    Rung 1 -- CYCLE_REQUEST (latched "waiting for part"):

        --| START_PB |--| ESTOP_OK |--| LC_CLEAR |--|/ROBOT_RUN|--|/CYCLE_ALARM|--( CYCLE_REQUEST )
             |                                                          |
             +-----| CYCLE_REQUEST |------(same conditions)-------------+   <- seal-in branch

    Rung 2 -- ROBOT_RUN (latched cycle-in-progress):

        --| CYCLE_REQUEST |--| PART_PRESENT |--| ESTOP_OK |--| LC_CLEAR |--|/CYCLE_COMPLETE|--( ROBOT_RUN )
             |                                                                    |
             +-----------| ROBOT_RUN |-----(ESTOP_OK, LC_CLEAR, /CYCLE_COMPLETE)---+

    CYCLE_ALARM (see PLC.scan) is the software equivalent of a timer-driven
    alarm rung: if the part doesn't show up while CYCLE_REQUEST is latched,
    the alarm raises and its NC contact drops CYCLE_REQUEST on the next scan.
    """
    safety = [Contact("ESTOP_OK"), Contact("LC_CLEAR")]

    request_rung = Rung(
        name="CYCLE_REQUEST",
        branches=[
            [Contact("START_PB")] + safety + [Contact("ROBOT_RUN", normally_closed=True),
                                               Contact("CYCLE_ALARM", normally_closed=True)],
            [Contact("CYCLE_REQUEST")] + safety + [Contact("ROBOT_RUN", normally_closed=True),
                                                    Contact("CYCLE_ALARM", normally_closed=True)],
        ],
        coil="CYCLE_REQUEST",
    )

    run_rung = Rung(
        name="ROBOT_RUN",
        branches=[
            [Contact("CYCLE_REQUEST"), Contact("PART_PRESENT")] + safety
            + [Contact("CYCLE_COMPLETE", normally_closed=True)],
            [Contact("ROBOT_RUN")] + safety + [Contact("CYCLE_COMPLETE", normally_closed=True)],
        ],
        coil="ROBOT_RUN",
    )

    return PLC(rungs=[request_rung, run_rung], alarm_timeout_scans=alarm_timeout_scans)


# --------------------------------------------------------------------------
# Test harness -- mirrors the "Test Case / Input / Expected / Actual" table
# style used elsewhere in the report.
# --------------------------------------------------------------------------

def run_tests():
    results = []

    def case(name, scans, expect_key, expect_value):
        plc = build_robot_cell_plc()
        outputs = None
        for s in scans:
            outputs = plc.scan(s)
        actual = outputs.get(expect_key, False)
        passed = actual == expect_value
        results.append((name, expect_key, expect_value, actual, passed))

    base = dict(START_PB=False, PART_PRESENT=False, ESTOP_OK=True,
                LC_CLEAR=True, CYCLE_COMPLETE=False)

    # 1. Start pressed, part already present, safety OK -> robot should run
    case("Start pressed with part present & safety OK",
         [dict(base, START_PB=True, PART_PRESENT=True)],
         "ROBOT_RUN", True)

    # 2. Start pressed, part not yet present -> request latches, robot waits
    case("Start pressed, part not yet present -> request latches",
         [dict(base, START_PB=True)],
         "CYCLE_REQUEST", True)

    # 3. Part arrives one scan after start -> robot runs
    case("Part arrives one scan after start -> robot runs",
         [dict(base, START_PB=True),
          dict(base, PART_PRESENT=True)],
         "ROBOT_RUN", True)

    # 4. E-Stop pressed while robot is running -> robot stops immediately
    case("E-Stop pressed while robot is running",
         [dict(base, START_PB=True, PART_PRESENT=True),
          dict(base, PART_PRESENT=True, ESTOP_OK=False)],
         "ROBOT_RUN", False)

    # 5. Light curtain broken while robot is running -> robot stops
    case("Light curtain broken while robot is running",
         [dict(base, START_PB=True, PART_PRESENT=True),
          dict(base, PART_PRESENT=True, LC_CLEAR=False)],
         "ROBOT_RUN", False)

    # 6. CYCLE_COMPLETE resets ROBOT_RUN
    case("CYCLE_COMPLETE resets ROBOT_RUN",
         [dict(base, START_PB=True, PART_PRESENT=True),
          dict(base, PART_PRESENT=True, CYCLE_COMPLETE=True)],
         "ROBOT_RUN", False)

    # 7. Part sensor never triggers -> alarm should raise past the timeout
    case("Part sensor silent past timeout -> CYCLE_ALARM raised",
         [dict(base, START_PB=True)] + [dict(base)] * 5,
         "CYCLE_ALARM", True)

    # 8. Part sensor triggers well within the timeout window -> no alarm
    case("Part sensor arrives within timeout -> no alarm",
         [dict(base, START_PB=True), dict(base, PART_PRESENT=True)] + [dict(base, PART_PRESENT=True)] * 4,
         "CYCLE_ALARM", False)

    # 9. After an alarm, the stale request is cancelled (does not re-latch on its own)
    case("Request cancelled automatically after alarm fires",
         [dict(base, START_PB=True)] + [dict(base)] * 6,
         "CYCLE_REQUEST", False)

    return results


def print_results(results):
    header = f"{'Test Case':<58}{'Expected':<10}{'Actual':<9}{'Result'}"
    print(header)
    print("-" * len(header))
    for name, key, expected, actual, passed in results:
        print(f"{name:<58}{str(expected):<10}{str(actual):<9}{'PASS' if passed else 'FAIL'}")
    total = len(results)
    passed_count = sum(1 for r in results if r[-1])
    print(f"\n{passed_count}/{total} test cases passed.")


if __name__ == "__main__":
    print_results(run_tests())