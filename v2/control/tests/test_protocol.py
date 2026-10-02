"""SPEC §5 protokol ayristirma: gecerli/gecersiz satirlar, tampon tasmasi, bozuk bayt."""

import re
import unittest

from _helpers import CONTROL_DIR  # noqa: F401  (import yolu)
import sim
from sim import FirmwareModel, ProtocolError, parse_command

STATE_RE = re.compile(r"^STATE( \d+\.\d){6} idle=[01]$")


def feed(model, text, t=0):
    return model.feed(text.encode("latin-1"), t)


class ParseCommandTests(unittest.TestCase):
    def test_valid_s(self):
        self.assertEqual(parse_command("S 90 90 60 120 120 60"),
                         ("S", (90.0, 90.0, 60.0, 120.0, 120.0, 60.0)))
        self.assertEqual(parse_command("s  0.5\t180  1e1 +3 .5 7.")[1], (0.5, 180.0, 10.0, 3.0, 0.5, 7.0))

    def test_empty_is_ignored(self):
        self.assertIsNone(parse_command(""))
        self.assertIsNone(parse_command("   \t "))

    def test_s_arg_count(self):
        for line in ("S", "S 1 2 3 4 5", "S 1 2 3 4 5 6 7"):
            with self.assertRaisesRegex(ProtocolError, "S needs 6 angles"):
                parse_command(line)

    def test_s_bad_numbers(self):
        for bad in ("abc", "1_0", "nan", "inf", "-inf", "12abc", "0x10", "--1"):
            with self.subTest(bad=bad), self.assertRaisesRegex(ProtocolError, "bad number"):
                parse_command(f"S 90 90 90 90 90 {bad}")

    def test_s_range(self):
        for bad in ("-1", "180.01", "1000"):
            with self.subTest(bad=bad), self.assertRaisesRegex(ProtocolError, "out of 0-180"):
                parse_command(f"S {bad} 90 90 90 90 90")

    def test_other_commands(self):
        self.assertEqual(parse_command("D"), ("D", ()))
        self.assertEqual(parse_command("A 1"), ("A", (True,)))
        self.assertEqual(parse_command("a 0"), ("A", (False,)))
        self.assertEqual(parse_command("?"), ("?", ()))
        for bad, msg in (("A", "A needs"), ("A 2", "A needs"), ("A 01", "A needs"), ("A 1 1", "A needs"),
                         ("D 1", "D takes"), ("? 1", "takes no args"), ("X", "unknown"),
                         ("SS 1", "unknown"), ("P090T095", "unknown")):
            with self.subTest(bad=bad), self.assertRaisesRegex(ProtocolError, msg):
                parse_command(bad)


class FirmwareLineHandlingTests(unittest.TestCase):
    def setUp(self):
        self.m = FirmwareModel(seed=1)

    def test_ok_and_crlf(self):
        self.assertEqual(feed(self.m, "S 90 90 90 90 90 90\r\n"), ["OK"])
        self.assertEqual(feed(self.m, "D\nA 0\n"), ["OK", "OK"])

    def test_state_format(self):
        out = feed(self.m, "?\n")
        self.assertEqual(len(out), 1)
        self.assertRegex(out[0], STATE_RE)

    def test_blank_lines_silent(self):
        self.assertEqual(feed(self.m, "\n\r\n  \n"), [])

    def test_line_split_across_writes(self):
        self.assertEqual(feed(self.m, "S 100 90 9"), [])
        self.assertEqual(feed(self.m, "0 90 90 90\n"), ["OK"])
        self.assertEqual(self.m.target[0], 100.0)

    def test_bad_line_does_not_move(self):
        before = list(self.m.target)
        out = feed(self.m, "S 120 90 90 90 90 abc\n")
        self.assertEqual(out, ["ERR bad number"])
        self.assertEqual(self.m.target, before)

    def test_overflow_single_err_and_recovery(self):
        out = feed(self.m, "S" + " 1" * 200 + "\n")
        self.assertEqual(out, ["ERR line too long"])
        out = feed(self.m, "x" * 50)
        out += feed(self.m, "y" * 50)       # tasma iki yazmaya bolunmus
        out += feed(self.m, "\nS 90 90 90 90 90 90\n")
        self.assertEqual(out, ["ERR line too long", "OK"])

    def test_line_max_boundary(self):
        base = "S 90 90 90 90 90 90"
        exact = base + " " * (sim.LINE_MAX - len(base))
        self.assertEqual(len(exact), sim.LINE_MAX)
        self.assertEqual(feed(self.m, exact + "\n"), ["OK"])
        self.assertEqual(feed(self.m, exact + " \n"), ["ERR line too long"])

    def test_bad_bytes(self):
        self.assertEqual(self.m.feed(b"S 90 90\x01 90 90 90 90\n", 0), ["ERR bad character"])
        self.assertEqual(self.m.feed(b"S 90 90 \xff90 90 90 90\n", 0), ["ERR bad character"])
        self.assertEqual(feed(self.m, "?\n")[0][:5], "STATE")  # sonra toparlanir

    def test_lunar_gimbal_command_rejected(self):
        # Lunar gimbal protokolu yanlislikla bu karta gelirse hareket olmamali
        before = list(self.m.target)
        self.assertEqual(feed(self.m, "P090T095\n"), ["ERR unknown command"])
        self.assertEqual(self.m.target, before)


if __name__ == "__main__":
    unittest.main()
