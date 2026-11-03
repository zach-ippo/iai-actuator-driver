"""The rules of ``tests/run_backend.py``: what fails a run, the summary it prints, the choices it takes.

No hardware and no serial port: small test classes defined here, run in this process.
"""

from __future__ import annotations

import functools
import io
import os
import unittest
from unittest import mock

import run_backend
from run_backend import BackendResult, check_result, name_of

import iai_actuator_driver


def sample_classes() -> tuple[type[unittest.TestCase], ...]:
    """The tests the rules are checked on (built here so that discovery does not collect them)."""

    class Sample(unittest.TestCase):
        def test_passes(self) -> None:
            pass

        def test_fails(self) -> None:
            self.fail("fails")

        @unittest.expectedFailure
        def test_marked_and_passes(self) -> None:
            pass

        @unittest.expectedFailure
        def test_marked_and_fails(self) -> None:
            self.fail("fails, as marked")

        def test_skipped(self) -> None:
            self.skipTest("not here")

    return (Sample,)


SAMPLES = sample_classes()  # a tuple, so the loader does not collect the classes


def run(*names: str) -> tuple[BackendResult, list[str], str]:
    suite = unittest.TestSuite(SAMPLES[0](n) for n in names)
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, resultclass=functools.partial(BackendResult)).run(suite)
    assert isinstance(result, BackendResult)
    out = io.StringIO()
    run_backend.summarise(result, 0.1, out)
    return result, check_result(result), out.getvalue()


class ResultTest(unittest.TestCase):
    def test_a_marked_test_that_passes_fails_the_run(self) -> None:
        result, problems, _ = run("test_marked_and_passes")
        self.assertFalse(result.wasSuccessful())
        test = SAMPLES[0]("test_marked_and_passes")
        self.assertEqual(problems, [f"{name_of(test)}: passed, but is marked as an expected failure"])

    def test_the_summary_counts_each_outcome(self) -> None:
        result, problems, text = run("test_passes", "test_fails", "test_marked_and_fails", "test_skipped")
        self.assertEqual(problems, [])
        self.assertFalse(result.wasSuccessful())
        self.assertRegex(text, r"(?m)^ran 4 tests in [\d.]+ s: 1 passed, 1 skipped, 1 expected failures, 0 unexpected "
                               r"successes, 1 failures, 0 errors$")  # fmt: skip
        self.assertIn("skipped 1: not here", text)
        self.assertEqual(len(result.durations), 4)


class RunTest(unittest.TestCase):
    def test_a_named_run(self) -> None:
        out = io.StringIO()
        with mock.patch.dict(os.environ):
            code = run_backend.main(["--backend", "native", "--timeout", "60", "test_crc"], out=out)
        text = out.getvalue()
        self.assertEqual(code, 0, text)
        self.assertIn(f"backend {iai_actuator_driver.BACKEND} (", text)
        self.assertRegex(text, r"ran \d+ tests in [\d.]+ s: \d+ passed, 0 skipped")
        self.assertIn("OK on the native backend", text)

    def test_the_removed_choice_ends_the_run(self) -> None:
        out = io.StringIO()
        with mock.patch.dict(os.environ, {run_backend.ENV_VAR: "python"}):
            self.assertEqual(run_backend.main([], out=out), 2)
        self.assertIn("IAI_DRIVER_BACKEND=python: not a choice any more", out.getvalue())
        with self.assertRaises(SystemExit), mock.patch("sys.stderr", io.StringIO()):
            run_backend.parse_args(["--backend", "python"])

    def test_no_binding_thread_outlives_the_run(self) -> None:
        self.assertEqual(run_backend.live_threads(grace_s=0), [])


if __name__ == "__main__":
    unittest.main()
