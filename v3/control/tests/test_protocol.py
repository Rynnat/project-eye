"""v3 protokol ayristirma (S a0 a1 a2): gecerli/gecersiz satirlar, tasma, bozuk bayt."""

import re
import unittest

from _helpers import CONTROL_DIR  # noqa: F401  (import yolu)
import sim
from sim import FirmwareModel, ProtocolError, parse_command

STATE_RE = re.compile(r"^STATE( \d+\.\d){3} idle=[01]$")


def feed(model, text, t=0):
    return model.feed(text.encode("latin-1"), t)


class ParseCommandTests(unittest.TestCase):
    def test_valid_s(self):
        self.assertEqual(parse_command("S 90 90 67.5"), ("S", (90.0, 90.0, 67.5)))
        self.assertEqual(parse_command("s  0.5\t180  1e1")[1], (0.5, 180.0, 10.0))

    def test_empty_is_ignored(self):
        self.assertIsNone(parse_command(""))
        self.assertIsNone(parse_command("  \t "))

    def test_s_arg_count(self):
        for line in ("S", "S 1 2", "S 1 2 3 4", "S 90 90 90 90 90 90"):   # v2 bicimi de reddedilir
            with self.subTest(line=line), self.assertRaisesRegex(ProtocolError, "S needs 3 angles"):
                parse_command(line)

    def test_s_bad_numbers_and_range(self):
        for bad in ("abc", "1_0", "nan", "inf", "12abc", "0x10"):
            with self.subTest(bad=bad), self.assertRaisesRegex(ProtocolError, "bad number"):
                parse_command(f"S 90 90 {bad}")
        for bad in ("-1", "180.01"):
            with self.subTest(bad=bad), self.assertRaisesRegex(ProtocolError, "out of 0-180"):
                parse_command(f"S {bad} 90 90")

    def test_other_commands(self):
        self.assertEqual(parse_command("D"), ("D", ()))
        self.assertEqual(parse_command("A 1"), ("A", (True,)))
        self.assertEqual(parse_command("?"), ("?", ()))
        for bad in ("A", "A 2", "D 1", "? 1", "X", "P090T095"):
            with self.subTest(bad=bad), self.assertRaises(ProtocolError):
                parse_command(bad)


class FirmwareLineHandlingTests(unittest.TestCase):
    def setUp(self):
        self.m = FirmwareModel(seed=1)

    def test_ok_state_crlf(self):
        self.assertEqual(feed(self.m, "S 90 90 90\r\n"), ["OK"])
        out = feed(self.m, "?\n")
        self.assertRegex(out[0], STATE_RE)
        self.assertEqual(feed(self.m, "\n\r\n"), [])

    def test_bad_line_does_not_move(self):
        before = list(self.m.target)
        self.assertEqual(feed(self.m, "S 100 90 abc\n"), ["ERR bad number"])
        self.assertEqual(feed(self.m, "P090T095\n"), ["ERR unknown command"])
        self.assertEqual(self.m.target, before)

    def test_overflow_and_recovery(self):
        self.assertEqual(feed(self.m, "S" + " 1" * 200 + "\n"), ["ERR line too long"])
        exact = "S 90 90 90".ljust(sim.LINE_MAX)
        self.assertEqual(feed(self.m, exact + "\n"), ["OK"])
        self.assertEqual(feed(self.m, exact + " \n"), ["ERR line too long"])
        self.assertEqual(self.m.feed(b"S 90 \x01 90 90\n", 0), ["ERR bad character"])
        self.assertEqual(feed(self.m, "S 91 90 90\n"), ["OK"])


if __name__ == "__main__":
    unittest.main()
