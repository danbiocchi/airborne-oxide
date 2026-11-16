#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Read a ULog with pyulog, the strictest reader we know, and report what it sees.

    ulog_check.py FILE                  one line per topic: name, instance, record count
    ulog_check.py FILE --dump TOPIC     the topic's records as CSV, floats as hex bits
    ulog_check.py FILE --meta           start time, parameters, changes, strings, dropouts, syncs

Exit status 1 if pyulog flags file corruption, if a topic has no records, or if
`PX4ULog.add_roll_pitch_yaw()` (the first thing Flight Review calls) fails.
`--dump` takes `--multi N` for an instance other than 0.

CSV format: the first line is the column names as pyulog flattens them
(`name:type`, nested types as `outer[1].inner`, arrays as `name[i]`, padding in
the middle of a record included, trailing padding absent). Then one line per
record: integers in decimal (`bool` and `char` as pyulog's int8), `float` as
`0x%08x` of its bits, `double` as `0x%016x`. Used by the Rust tests of
`oxide-log`, which compare them with their own reader bit for bit.
"""
import argparse
import sys

import numpy as np
from pyulog import ULog
from pyulog.px4 import PX4ULog


def fail(msg):
    print("FAIL: " + msg, file=sys.stderr)
    sys.exit(1)


def column_text(arr, type_str):
    """One column as a list of strings."""
    if type_str == "float":
        bits = np.ascontiguousarray(arr, dtype=np.float32).view(np.uint32)
        return ["0x%08x" % int(b) for b in bits]
    if type_str == "double":
        bits = np.ascontiguousarray(arr, dtype=np.float64).view(np.uint64)
        return ["0x%016x" % int(b) for b in bits]
    return [str(int(v)) for v in arr]


def hexf(v):
    return "0x%08x" % int(np.float32(v).view(np.uint32))


def param_text(v):
    return hexf(v) if isinstance(v, float) else str(int(v))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--dump", metavar="TOPIC")
    ap.add_argument("--multi", type=int, default=0)
    ap.add_argument("--meta", action="store_true")
    a = ap.parse_args()

    ulog = ULog(a.file)
    if ulog.file_corruption:
        fail("pyulog reports file corruption")
    if not ulog.data_list:
        fail("no topic has any data")
    for d in ulog.data_list:
        if len(d.data["timestamp"]) == 0:
            fail("topic %s has no records" % d.name)
    PX4ULog(ulog).add_roll_pitch_yaw()

    if a.dump:
        d = ulog.get_dataset(a.dump, a.multi)
        cols = [(f.field_name, f.type_str) for f in d.field_data]
        print(",".join("%s:%s" % c for c in cols))
        texts = [column_text(d.data[n], t) for n, t in cols]
        for row in zip(*texts):
            print(",".join(row))
        return

    if a.meta:
        print("start %d" % ulog.start_timestamp)
        for k in sorted(ulog.initial_parameters):
            print("param %s %s" % (k, param_text(ulog.initial_parameters[k])))
        for ts, name, v in ulog.changed_parameters:
            print("changed %d %s %s" % (ts, name, param_text(v)))
        for m in ulog.logged_messages:
            print("log %d %s %s" % (m.timestamp, m.log_level_str(), m.message))
        for d in ulog.dropouts:
            print("dropout %d %d" % (d.timestamp, d.duration))
        print("sync %d" % getattr(ulog, "_sync_seq_cnt", -1))
        for k in sorted(ulog.msg_info_dict):
            print("info %s %s" % (k, ulog.msg_info_dict[k]))
        return

    for d in ulog.data_list:
        print("topic %s %d %d" % (d.name, d.multi_id, len(d.data["timestamp"])))


if __name__ == "__main__":
    main()
