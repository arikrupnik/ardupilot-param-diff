
# paramsdiff.py: compare ArduPilot param file in MissionPlanner format to another param file, log file or live MAVLink connection

# TODO:
# handle exceptions in main()
# return 0 if no diff, 1 if diff, 2 if errors

from pymavlink import mavutil
import functools
import pytest
import argparse
import math
import os.path
import sys


class Params(dict):
    """A dictionary of parsed parameters, with a name and can diff against another instance"""
    ABS_TOL = 0.00001
    def __init__(self, name):
        self.name = name
    def longest_p_len(self):
        return max(map(len, self.keys()))
    def longest_v_len(self):
        return max(map(len, map(str_value, self.values())))
    def print_diff(self, other):
        diff = Params(None)
        for p, v in self.items():
            if not math.isclose(v, other.get(p, math.nan), abs_tol=self.ABS_TOL):
                diff[p] = v
        if diff:
            print(f"   {self.name:>{diff.longest_p_len()+diff.longest_v_len()}} | "
                  f"{other.name}")
        for p, v in diff.items():
            print(f"{p:<{diff.longest_p_len()}} = "
                  f"{str_value(self[p]):<{self.longest_v_len()}} | "
                  f"{str_value(other.get(p), "")}")
def test_Params():
    p = Params("local")
    p["a"] = 1
    p["ab"] = 123
    p["b"] = 2
    assert p.name == "local"
    assert p["a"] == 1
    assert p.longest_p_len() == 2
    assert p.longest_v_len() == 3

def parse_param_line(l, n):
    head = l.rstrip("\r\n")
    if (head.startswith("#")):
        # comment (comments must start at the beginning of a line)
        return None, head
    if head.strip() != head:
        raise ValueError(f"unexpected whitespace in '{head}' (line {n})")
    try:
        p,v = head.split(",")
    except ValueError as e:
        raise ValueError(f"expecting 'PARAM,VALUE', found '{head}' (line {n})") from None
    if p.strip() != p or v.strip() != v:
        raise ValueError(f"unexpected whitespace in '{head}' (line {n})")
    # All values are numeric; Parameters have types (signed and
    # unsigned ints of various widths, floats). MissionPlanner ignores
    # these types when writing parameters to files; it writes them
    # with or without a decimal point depending on values rather than
    # types. Type information is lost in MP files. Consequently, there
    # is no advantage in trying to deduce type from format, and I
    # treat all values as floats.
    try:
        nv = float(v)
    except ValueError as e:
        raise ValueError(f"expecting numeric value, found '{v}' (line {n})") from None
    return p, nv
def test_parse_param_line():
    assert parse_param_line("FOO,100", 0) == ("FOO", 100)
    assert parse_param_line("FOO,100.5", 0) == ("FOO", 100.5)
    assert parse_param_line("# comment\r", 0) == (None, "# comment")
    with pytest.raises(ValueError) as e:
        parse_param_line(" # indented comment", 1)
    assert e.value.args[0] == "unexpected whitespace in ' # indented comment' (line 1)"
    with pytest.raises(ValueError) as e:
        parse_param_line(" FOO,BAR ", 1)
    assert e.value.args[0] == "unexpected whitespace in ' FOO,BAR ' (line 1)"
    with pytest.raises(ValueError) as e:
        parse_param_line("FOO,BAR,BAZ", 1)
    assert e.value.args[0] == "expecting 'PARAM,VALUE', found 'FOO,BAR,BAZ' (line 1)"
    with pytest.raises(ValueError) as e:
        parse_param_line("FOO", 1)
    assert e.value.args[0] == "expecting 'PARAM,VALUE', found 'FOO' (line 1)"
    with pytest.raises(ValueError) as e:
        parse_param_line("FOO ,BAR", 1)
    assert e.value.args[0] == "unexpected whitespace in 'FOO ,BAR' (line 1)"
    with pytest.raises(ValueError) as e:
        parse_param_line("FOO, BAR", 1)
    assert e.value.args[0] == "unexpected whitespace in 'FOO, BAR' (line 1)"
    with pytest.raises(ValueError) as e:
        parse_param_line("FOO,string", 1)
    assert e.value.args[0] == "expecting numeric value, found 'string' (line 1)"

def str_value(v, none_replacement=None):
    # All values are numeric; Parameters have types (signed and
    # unsigned ints of various widths, floats). MissionPlanner ignores
    # these types when writing parameters to files; it writes them
    # with or without a decimal point depending on values rather than
    # types.
    if v is None and none_replacement is not None:
        return none_replacement
    return f"{v:g}"
def test_str_value():
    assert str_value(0) == "0"
    assert str_value(0.0) == "0"
    assert str_value(0.1) == "0.1"
    with pytest.raises(TypeError):
        assert str_value(None)
    assert str_value(None, "") == ""

def write_param_line(f, param_name, value):
    return f.write(param_name + "," + str_value(value) + "\r\n")
    
def params_from_param_file(fn):
    with open(fn) as f:
        params = Params(os.path.basename(f.name))
        for n, l in enumerate(f.readlines(), 1):
            p, nv = parse_param_line(l, n)
            if p:
                params[p] = nv
        return params

def params_from_log_file(fn):
    log = mavutil.mavlink_connection(fn)
    params = Params(os.path.basename(fn))
    while (msg := log.recv_match(type="PARAM_VALUE")):
        params[msg.param_id] = msg.param_value
    return params

def params_from_mavlink(fn):
    remote = mavutil.mavlink_connection(fn)
    params = Params(fn)
    remote.wait_heartbeat()
    remote.mav.param_request_list_send(remote.target_system,
                                       remote.target_component)
    while (msg := remote.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)):
        params[msg.param_id] = msg.param_value
        if not (msg.param_index % 100):
            print(f"{msg.param_index}/{msg.param_count}", end="\r", file=sys.stderr)
        if msg.param_index == msg.param_count-1:
            print(" "*25, end="\r", file=sys.stderr)
            return params
    raise ValueError("timeout")

def paramsf_from_filename(fn):
    _, ext = os.path.splitext(fn)
    if ext.lower() in [".param", ".params"]:
        f = params_from_param_file
    elif ext.lower() in [".bin", ".log", ".tlog", ".rlog"]:
        f = params_from_log_file
    else:
        f = params_from_mavlink
    return functools.partial(f, fn)
def test_paramsf_from_filename():
    assert paramsf_from_filename("vehicle-params/32.blaze/tune.param").func == params_from_param_file
    assert paramsf_from_filename("logs/FIXED_WING/32/2026-02-07.tlog").func == params_from_log_file
    assert paramsf_from_filename("/dev/ttyACM0").func == params_from_mavlink

if __name__ == "__main__":
    argp = argparse.ArgumentParser()
    argp.add_argument("local")
    argp.add_argument("remote")
    args = argp.parse_args()
    local, remote = map(lambda fn: paramsf_from_filename(fn)(), [args.local, args.remote])
    local.print_diff(remote)
