"""Tests of checks/check_register.py: the reader, the comparison, --record and the instruments' check.

Every case runs on sources held as strings here and written to a temporary folder; two cases at the end run the reader
over the real tests/ folder. Run: python -B -m unittest tests.test_check_register
"""
import contextlib
import importlib.util
import io
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("check_register", ROOT / "checks" / "check_register.py")
cr = importlib.util.module_from_spec(_spec)
sys.modules["check_register"] = cr
_spec.loader.exec_module(cr)

A = "tests/a_test.cpp"

BASE = r'''#include "doctest.h"

namespace
{
    constexpr double kLimit = 0.001;
    struct Tolerances
    {
        double along = 1e-3;
        int windows = 5;
    };
    struct Row
    {
        const char *name;
        double tol;
    };
    const Row table[] = {{"r1", 0.01}, {"r2", 0.02}};

    void inside(double x, double limit)
    {
        CHECK(x < limit);
    }

    void outer(double x)
    {
        inside(x, 0.5);
    }
}

TEST_CASE("alpha: plain")
{
    const double tol = 0.05;
    const double measured = 0.1;
    CHECK(1.0 < 2.0);
    CHECK_FALSE(false);
    REQUIRE(tol == doctest::Approx(0.05).epsilon(1e-6));
    CHECK(measured < 0.25);
    inside(0.1, kLimit);
}

TEST_CASE("beta: " "joined name"
          * doctest::skip())
{
    SUBCASE("one") { CHECK(1 == 1); }
    SUBCASE("two")
    {
        CHECK(2 == 2);
        outer(0.2);
    }
}

TEST_CASE("gamma: strings and comments")
{
    const char *s = R"x(CHECK(not an assertion) // not a comment)x";
    const char *t = "CHECK(also not) /* nor this */";
    const char c = '"';  // CHECK(in a comment)
    /* CHECK(commented) */
    CHECK(s != nullptr);
}

TEST_CASE("delta: a known failure" * doctest::should_fail())
{
    CHECK(1 == 2);
}

TEST_CASE("epsilon: movable")
{
    CHECK(3 > 2);
}
'''

PY_BASE = '''import unittest

LIMIT = 0.5
NAMES = ["a", "b"]


class PyTests(unittest.TestCase):
    TOL = 1e-3

    def check(self, x):
        self.assertLess(x, 1)

    def test_one(self):
        eps = 0.01
        self.assertAlmostEqual(1.0, 1.0001, places=3)
        self.assertTrue(True)
        self.check(0.5)

    @unittest.expectedFailure
    def test_two(self):
        self.fail("later")

    def helper_not_a_test(self):
        pass
'''


def write_tree(root, files):
    for rel, text in files.items():
        p = Path(root) / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding="utf-8", newline="\n")


def run_main(root, *args):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = cr.main(["--root", str(root), *args])
    return code, out.getvalue(), err.getvalue()


def register_text(root):
    return cr.render_register(cr.build_register(root))


class TreeCase(unittest.TestCase):
    """A temporary tree with BASE written and its register recorded (written directly, not through --record)."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        write_tree(self.root, {A: BASE, "tests/test_py.py": PY_BASE})
        self.record()

    def record(self):
        write_tree(self.root, {cr.REGISTER_PATH: register_text(self.root)})

    def edit(self, old, new, file=A):
        text = (self.root / file).read_text(encoding="utf-8")
        self.assertIn(old, text)
        write_tree(self.root, {file: text.replace(old, new, 1)})

    def compare(self):
        return run_main(self.root)


class ReaderTest(unittest.TestCase):
    def build(self, files):
        with tempfile.TemporaryDirectory() as tmp:
            write_tree(tmp, files)
            return cr.build_register(tmp)

    def case(self, reg, name):
        found = [b for b in reg.blocks if b.name == name]
        self.assertEqual(len(found), 1, name)
        return found[0]

    def file_block(self, reg, path):
        return [b for b in reg.blocks if b.name is None and b.file == path][0]

    def test_cases_names_and_markers(self):
        reg = self.build({A: BASE})
        beta = self.case(reg, "beta: joined name")
        self.assertEqual(beta.markers, ["skip"])
        self.assertEqual(self.case(reg, "delta: a known failure").markers, ["should_fail"])
        self.assertEqual(self.case(reg, "alpha: plain").markers, [])
        self.assertEqual(sum(1 for b in reg.blocks if b.name is not None and b.file == A), 5)

    def test_decorators_with_arguments_and_several(self):
        src = 'TEST_CASE("x" * doctest::skip() * doctest::timeout(5)) { CHECK(1); }\n'
        reg = self.build({A: src})
        self.assertEqual(self.case(reg, "x").markers, ["skip", "timeout(5)"])

    def test_assertions_as_one_line_each_in_source_order(self):
        reg = self.build({A: BASE})
        lines = [t for k, t in self.case(reg, "alpha: plain").lines if k == "assert"]
        self.assertEqual(lines, ["CHECK(1.0 < 2.0)", "CHECK_FALSE(false)", "REQUIRE(tol == doctest::Approx(0.05).epsilon(1e-6))",
                                 "CHECK(measured < 0.25)"])

    def test_multiline_assertion_collapsed(self):
        src = 'TEST_CASE("m")\n{\n    CHECK(a\n          < /* c */ 0.5);\n}\n'
        reg = self.build({A: src})
        self.assertEqual(self.case(reg, "m").lines, [("assert", "CHECK(a < 0.5)")])

    def test_subcase_assertions_are_prefixed_lines_of_the_case(self):
        reg = self.build({A: BASE})
        lines = self.case(reg, "beta: joined name").lines
        self.assertIn(("assert", "[one] CHECK(1 == 1)"), lines)
        self.assertIn(("assert", "[two] CHECK(2 == 2)"), lines)
        self.assertFalse([b for b in reg.blocks if b.name in ("one", "two")])

    def test_strings_raw_strings_chars_and_comments_hide_macros(self):
        reg = self.build({A: BASE})
        gamma = self.case(reg, "gamma: strings and comments")
        self.assertEqual([t for k, t in gamma.lines if k == "assert"], ["CHECK(s != nullptr)"])

    def test_digit_separators_and_char_literals(self):
        src = "TEST_CASE(\"d\") { const int big = 1'000'000; const char q = '\\''; CHECK(big > 10'000); }\n"
        reg = self.build({A: src})
        lines = self.case(reg, "d").lines
        self.assertIn(("assert", "CHECK(big > 10'000)"), lines)
        self.assertIn(("value", "const int big = 1'000'000"), lines)

    def test_asserting_helper_closure_and_call_lines(self):
        reg = self.build({A: BASE})
        fb = self.file_block(reg, A)
        self.assertIn(("assert", "inside() :: CHECK(x < limit)"), fb.lines)
        self.assertIn(("call", "outer() :: inside(x, 0.5)"), fb.lines)
        self.assertIn(("call", "inside(0.1, kLimit)"), self.case(reg, "alpha: plain").lines)
        self.assertIn(("call", "[two] outer(0.2)"), self.case(reg, "beta: joined name").lines)

    def test_helper_defined_in_a_cpp_is_visible_in_other_files_only_through_a_header(self):
        files = {
            "tests/h.h": "#pragma once\nvoid shared(double x);\n",
            "tests/h.cpp": "void shared(double x) { CHECK(x < 1.0); }\nvoid local_only(double x) { CHECK(x < 2.0); }\n",
            "tests/u_test.cpp": 'TEST_CASE("u") { shared(0.5); local_only(0.5); }\n',
        }
        reg = self.build(files)
        self.assertEqual(self.case(reg, "u").lines, [("call", "shared(0.5)")])

    def test_values_outside_bodies_members_and_tables(self):
        reg = self.build({A: BASE})
        values = [t for k, t in self.file_block(reg, A).lines if k == "value"]
        self.assertIn("constexpr double kLimit = 0.001", values)
        self.assertIn("Tolerances :: double along = 1e-3", values)
        self.assertIn("Tolerances :: int windows = 5", values)
        self.assertIn('const Row table[] = {{"r1", 0.01}, {"r2", 0.02}}', values)

    def test_values_inside_a_case_are_const_declarations(self):
        reg = self.build({A: BASE})
        values = [t for k, t in self.case(reg, "alpha: plain").lines if k == "value"]
        self.assertEqual(values, ["const double tol = 0.05", "const double measured = 0.1"])

    def test_non_const_local_is_not_a_value(self):
        src = 'TEST_CASE("n") { double x = 0.5; for (const auto &v : list) { CHECK(v < x); } }\n'
        reg = self.build({A: src})
        self.assertEqual(self.case(reg, "n").lines, [("assert", "CHECK(v < x)")])

    def test_if_zero_is_removed_like_a_comment_and_its_else_stays(self):
        src = 'TEST_CASE("z")\n{\n#if 0\n    CHECK(false);\n#else\n    CHECK(true);\n#endif\n    CHECK(1);\n}\n'
        reg = self.build({A: src})
        self.assertEqual(self.case(reg, "z").lines, [("assert", "CHECK(true)"), ("assert", "CHECK(1)")])

    def test_other_conditionals_are_lines_at_their_place(self):
        src = 'TEST_CASE("c")\n{\n    CHECK(1);\n#ifdef NDEBUG\n    CHECK(2);\n#else\n    CHECK(3);\n#endif\n}\n'
        reg = self.build({A: src})
        self.assertEqual(self.case(reg, "c").lines, [
            ("assert", "CHECK(1)"), ("conditional", "#ifdef NDEBUG"), ("assert", "CHECK(2)"), ("conditional", "#else"),
            ("assert", "CHECK(3)"), ("conditional", "#endif")])

    def test_conditional_outside_a_case_lands_under_the_file(self):
        src = '#ifdef NDEBUG\nconstexpr double kBound = 0.5;\n#endif\nTEST_CASE("c") { CHECK(1); }\n'
        reg = self.build({A: src})
        self.assertEqual(self.file_block(reg, A).lines,
                         [("conditional", "#ifdef NDEBUG"), ("value", "constexpr double kBound = 0.5"), ("conditional", "#endif")])

    def test_other_assertion_macros_are_found(self):
        src = 'TEST_CASE("o") { CHECK_NOTHROW(f()); WARN_MESSAGE(a, "b"); REQUIRE_FALSE(x); CHECK_MESSAGE(y, "m" << 1); WARN(z); }\n'
        reg = self.build({A: src})
        names = [cr.macro_of(t) for k, t in self.case(reg, "o").lines]
        self.assertEqual(names, ["CHECK_NOTHROW", "WARN_MESSAGE", "REQUIRE_FALSE", "CHECK_MESSAGE", "WARN"])

    def test_python_tests(self):
        reg = self.build({"tests/test_py.py": PY_BASE})
        one = self.case(reg, "PyTests.test_one")
        self.assertEqual(one.markers, [])
        self.assertEqual(one.lines, [
            ("value", "eps = 0.01"),
            ("assert", "self.assertAlmostEqual(1.0, 1.0001, places=3)"),
            ("assert", "self.assertTrue(True)"),
            ("call", "self.check(0.5)")])
        two = self.case(reg, "PyTests.test_two")
        self.assertEqual(two.markers, ["expectedFailure"])
        self.assertEqual(two.lines, [("assert", "self.fail('later')")])
        fb = self.file_block(reg, "tests/test_py.py")
        self.assertIn(("value", "LIMIT = 0.5"), fb.lines)
        self.assertIn(("value", "PyTests :: TOL = 0.001"), fb.lines)
        self.assertFalse([b for b in reg.blocks if b.name and "helper_not_a_test" in b.name])

    def test_register_text_is_deterministic_and_parses_back(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_tree(tmp, {A: BASE, "tests/test_py.py": PY_BASE})
            a, b = register_text(tmp), register_text(tmp)
            self.assertEqual(a, b)
            self.assertNotIn("\r", a)
            back = cr.parse_register(a)
            self.assertEqual(cr.render_register(back), a)
            self.assertEqual(cr.compare(back, cr.build_register(tmp)), [])

    def test_totals_line(self):
        reg = self.build({A: BASE, "tests/test_py.py": PY_BASE})
        line = cr.totals_line(cr.totals(reg))
        self.assertEqual(line, "# totals: 5 cases (1 should_fail, 1 skip), 10 assertions, 2 python tests (1 expectedFailure)")


class UnreadableSourceTest(unittest.TestCase):
    def fails(self, src, fragment, line=None):
        with tempfile.TemporaryDirectory() as tmp:
            write_tree(tmp, {A: src})
            code, out, err = run_main(tmp, "--print")
        self.assertEqual(code, 2, err)
        self.assertIn(A + ":", err)
        self.assertIn(fragment, err)
        if line is not None:
            self.assertIn(f"{A}:{line}:", err)
        self.assertEqual(out, "")

    def test_unmatched_open_bracket_names_its_line(self):
        self.fails('// line 1\nTEST_CASE("u")\n{\n    CHECK(1;\n}\n', "unmatched", 4)

    def test_unmatched_close_bracket(self):
        self.fails('TEST_CASE("u")\n{\n    CHECK(1);\n}\n}\n', "unmatched '}'", 5)

    def test_unterminated_string(self):
        self.fails('TEST_CASE("u")\n{\n    const char *s = "abc;\n    CHECK(1);\n}\n', "unterminated string", 3)

    def test_unterminated_comment(self):
        self.fails('TEST_CASE("u") { CHECK(1); }\n/* never closed\n', "unterminated comment", 2)

    def test_unterminated_raw_string(self):
        self.fails('TEST_CASE("u") { const char *s = R"x(abc"; }\n', "unterminated raw string", 1)

    def test_cross_count_catches_an_assertion_the_reader_did_not_reach(self):
        self.fails('TEST_CASE("u") { CHECK(1); }\nCHECK(2);\n', "cross-count")

    def test_unterminated_if_zero(self):
        self.fails('TEST_CASE("u") { CHECK(1); }\n#if 0\nCHECK(2);\n', "unterminated #if 0", 2)

    def test_a_test_case_whose_name_is_not_a_string(self):
        self.fails("TEST_CASE(NAME) { CHECK(1); }\n", "name")

    def test_cannot_decode(self):
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, "tests").mkdir()
            Path(tmp, A).write_bytes(b"TEST_CASE(\"\xff\") {}\n")
            code, _, err = run_main(tmp, "--print")
        self.assertEqual(code, 2)
        self.assertIn(A, err)


class ComparisonTest(TreeCase):
    def assertKinds(self, code_expected, *kinds, absent=()):
        code, out, err = self.compare()
        self.assertEqual(code, code_expected, out + err)
        for kind in kinds:
            self.assertIn(kind, out)
        for kind in absent:
            self.assertNotIn(kind, out)
        return out

    def test_unchanged_is_identical(self):
        code, out, _ = self.compare()
        self.assertEqual(code, 0)
        self.assertEqual(out.strip().splitlines()[-1], "IDENTICAL")

    def test_tolerance_loosened_in_an_assertion(self):
        self.edit("doctest::Approx(0.05).epsilon(1e-6)", "doctest::Approx(0.05).epsilon(1e-3)")
        out = self.assertKinds(1, "CHANGE assertion changed: tests/a_test.cpp :: alpha: plain")
        self.assertIn("  - REQUIRE(tol == doctest::Approx(0.05).epsilon(1e-6))", out)
        self.assertIn("  + REQUIRE(tol == doctest::Approx(0.05).epsilon(1e-3))", out)
        self.assertEqual(out.strip().splitlines()[-1], "1 assertion changed")

    def test_comparison_operator_turned_round_is_a_change(self):
        self.edit("CHECK(measured < 0.25)", "CHECK(measured > 0.25)")
        self.assertKinds(1, "assertion changed")

    def test_tolerance_loosened_in_a_named_value(self):
        self.edit("kLimit = 0.001", "kLimit = 0.01")
        out = self.assertKinds(1, "CHANGE value changed: tests/a_test.cpp")
        self.assertIn("  + constexpr double kLimit = 0.01", out)

    def test_tolerance_loosened_in_a_struct_member_default(self):
        self.edit("double along = 1e-3;", "double along = 1e-2;")
        self.assertKinds(1, "value changed")

    def test_tolerance_loosened_in_a_case_table_row(self):
        self.edit('{"r2", 0.02}', '{"r2", 0.2}')
        out = self.assertKinds(1, "value changed")
        self.assertIn('{"r2", 0.2}', out)

    def test_tolerance_loosened_in_a_helpers_argument(self):
        self.edit("inside(0.1, kLimit)", "inside(0.1, 0.5)")
        self.assertKinds(1, "CHANGE call changed: tests/a_test.cpp :: alpha: plain")

    def test_tolerance_loosened_in_a_helpers_own_call(self):
        self.edit("inside(x, 0.5)", "inside(x, 5.0)")
        self.assertKinds(1, "CHANGE call changed: tests/a_test.cpp\n")

    def test_assertion_deleted(self):
        self.edit("    CHECK_FALSE(false);\n", "")
        out = self.assertKinds(1, "CHANGE assertion removed: tests/a_test.cpp :: alpha: plain")
        self.assertIn("  - CHECK_FALSE(false)", out)

    def test_assertion_commented_out(self):
        self.edit("    CHECK_FALSE(false);", "    // CHECK_FALSE(false);")
        self.assertKinds(1, "assertion removed")

    def test_assertion_in_a_block_comment(self):
        self.edit("    CHECK_FALSE(false);", "    /* CHECK_FALSE(false); */")
        self.assertKinds(1, "assertion removed")

    def test_if_zero_round_an_assertion(self):
        self.edit("    CHECK_FALSE(false);", "#if 0\n    CHECK_FALSE(false);\n#endif")
        out = self.assertKinds(1, "assertion removed")
        self.assertNotIn("conditional", out)

    def test_ifdef_round_an_assertion(self):
        self.edit("    CHECK_FALSE(false);", "#ifdef NDEBUG\n    CHECK_FALSE(false);\n#endif")
        out = self.assertKinds(1, "CHANGE conditional added: tests/a_test.cpp :: alpha: plain")
        self.assertEqual(out.count("conditional added"), 2)
        self.assertIn("+ #ifdef NDEBUG", out)

    def test_skip_added(self):
        self.edit('TEST_CASE("alpha: plain")', 'TEST_CASE("alpha: plain" * doctest::skip())')
        out = self.assertKinds(1, "CHANGE marker added: tests/a_test.cpp :: alpha: plain")
        self.assertIn("+ skip", out)
        self.assertEqual(out.strip().splitlines()[-1], "1 marker added")

    def test_should_fail_removed(self):
        self.edit('"delta: a known failure" * doctest::should_fail()', '"delta: a known failure"')
        out = self.assertKinds(1, "CHANGE marker removed: tests/a_test.cpp :: delta: a known failure")
        self.assertIn("- should_fail", out)

    def test_case_renamed_is_removed_and_added(self):
        self.edit('"gamma: strings and comments"', '"gamma: strings, renamed"')
        self.assertKinds(1, "CHANGE case removed: tests/a_test.cpp :: gamma: strings and comments",
                         "ADD case added: tests/a_test.cpp :: gamma: strings, renamed")

    def test_case_moved_to_another_file_is_an_addition(self):
        text = (self.root / A).read_text(encoding="utf-8")
        moved = 'TEST_CASE("epsilon: movable")\n{\n    CHECK(3 > 2);\n}\n'
        self.assertIn(moved, text)
        self.edit(moved, "")
        write_tree(self.root, {"tests/b_test.cpp": moved})
        out = self.assertKinds(3, "ADD case moved: tests/b_test.cpp :: epsilon: movable", absent=("CHANGE",))
        self.assertIn("from tests/a_test.cpp", out)
        self.assertEqual(out.strip().splitlines()[-1], "to record: +1 file, 1 moved")

    def test_assertion_reworded_is_an_addition(self):
        self.edit("CHECK(measured < 0.25)", "CHECK(observed < 0.25)")
        out = self.assertKinds(3, "ADD assertion reworded: tests/a_test.cpp :: alpha: plain", absent=("CHANGE",))
        self.assertIn("- CHECK(measured < 0.25)", out)
        self.assertIn("+ CHECK(observed < 0.25)", out)
        self.assertEqual(out.strip().splitlines()[-1], "to record: 1 reworded")

    def test_reworded_with_another_macro_is_a_change(self):
        self.edit("CHECK(measured < 0.25)", "WARN(measured < 0.25)")
        self.assertKinds(1, "assertion changed")

    def test_additions_only(self):
        self.edit('TEST_CASE("epsilon: movable")', 'TEST_CASE("zeta: new")\n{\n    CHECK(1 < 2);\n    CHECK(2 < 3);\n}\n\nTEST_CASE("epsilon: movable")')
        out = self.assertKinds(3, "ADD case added: tests/a_test.cpp :: zeta: new", absent=("CHANGE",))
        self.assertEqual(out.strip().splitlines()[-1], "to record: +1 case, +2 assertions")

    def test_assertion_value_and_call_added_to_an_existing_case(self):
        self.edit("    CHECK(3 > 2);", "    CHECK(3 > 2);\n    CHECK(4 > 2);\n    const double w = 2.0;\n    outer(1.0);")
        out = self.assertKinds(3, "ADD assertion added", "ADD value added", "ADD call added", absent=("CHANGE",))
        self.assertEqual(out.strip().splitlines()[-1], "to record: +1 assertion, +1 call, +1 value")

    def test_changes_are_listed_before_additions_and_both_in_the_summary(self):
        self.edit("CHECK(measured < 0.25)", "CHECK(measured < 0.5)")
        self.edit('TEST_CASE("epsilon: movable")', 'TEST_CASE("zeta: new")\n{\n    CHECK(1 < 2);\n}\n\nTEST_CASE("epsilon: movable")')
        code, out, _ = self.compare()
        self.assertEqual(code, 1)
        self.assertLess(out.index("CHANGE assertion changed"), out.index("ADD case added"))
        self.assertEqual(out.strip().splitlines()[-1], "1 assertion changed; to record: +1 case, +1 assertion")

    def test_a_removed_file_is_a_change_and_a_new_empty_file_an_addition(self):
        (self.root / "tests/test_py.py").unlink()
        self.assertKinds(1, "CHANGE case removed: tests/test_py.py :: PyTests.test_one", "CHANGE file removed: tests/test_py.py")

    def test_whitespace_and_comment_edits_change_nothing(self):
        self.edit("CHECK(1.0 < 2.0);", "CHECK(  1.0\n    <  2.0 ); // now with a comment")
        self.assertKinds(0)

    def test_python_skip_added(self):
        self.edit("    def test_one(self):", "    @unittest.skip('not now')\n    def test_one(self):", "tests/test_py.py")
        self.assertKinds(1, "CHANGE marker added: tests/test_py.py :: PyTests.test_one")

    def test_python_places_argument_changed(self):
        self.edit("places=3", "places=1", "tests/test_py.py")
        out = self.assertKinds(1, "CHANGE assertion changed: tests/test_py.py :: PyTests.test_one")
        self.assertIn("places=1", out)

    def test_python_expected_failure_removed(self):
        self.edit("    @unittest.expectedFailure\n", "", "tests/test_py.py")
        self.assertKinds(1, "CHANGE marker removed: tests/test_py.py :: PyTests.test_two")

    def test_python_literal_value_changed(self):
        self.edit("LIMIT = 0.5", "LIMIT = 5.0", "tests/test_py.py")
        self.assertKinds(1, "value changed")

    def test_unreadable_source_exits_2_with_file_and_line(self):
        self.edit('CHECK(3 > 2);', 'CHECK(3 > 2;')
        code, out, err = self.compare()
        self.assertEqual(code, 2)
        self.assertRegex(err, r"tests/a_test\.cpp:\d+: unmatched")

    def test_no_register_recorded_is_said_in_one_line_and_is_additions(self):
        (self.root / cr.REGISTER_PATH).unlink()
        code, out, _ = self.compare()
        self.assertEqual(code, 3)
        self.assertIn("no checks/tests.register recorded yet", out)

    def test_print_does_not_touch_the_file(self):
        before = (self.root / cr.REGISTER_PATH).read_bytes()
        code, out, _ = run_main(self.root, "--print")
        self.assertEqual(code, 0)
        self.assertEqual(out, before.decode("utf-8"))
        self.assertEqual((self.root / cr.REGISTER_PATH).read_bytes(), before)


def git(cwd, *args):
    res = subprocess.run(["git", "-c", "user.name=t", "-c", "user.email=t@t", "-c", "core.autocrlf=false", *args],
                         cwd=str(cwd), capture_output=True, text=True)
    if res.returncode != 0:
        raise AssertionError(f"git {args}: {res.stderr}")
    return res.stdout


class RecordTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name) / "main"
        self.root.mkdir()
        write_tree(self.root, {A: BASE, "tests/test_py.py": PY_BASE})
        git(self.root, "init", "-q")
        git(self.root, "add", "-A")
        git(self.root, "commit", "-q", "-m", "tests")

    def test_record_in_a_clean_main_tree_writes_the_register(self):
        code, out, _ = run_main(self.root, "--record")
        self.assertEqual(code, 0, out)
        target = self.root / cr.REGISTER_PATH
        self.assertEqual(target.read_text(encoding="utf-8"), register_text(self.root))
        self.assertEqual(run_main(self.root)[0], 0)

    def test_record_refuses_with_uncommitted_changes_under_tests(self):
        (self.root / A).write_text(BASE.replace("0.25", "0.5"), encoding="utf-8", newline="\n")
        code, out, _ = run_main(self.root, "--record")
        self.assertEqual(code, 4)
        self.assertIn("uncommitted", out)
        self.assertFalse((self.root / cr.REGISTER_PATH).exists())

    def test_record_refuses_with_an_untracked_new_test_file(self):
        write_tree(self.root, {"tests/new_test.cpp": 'TEST_CASE("n") { CHECK(1); }\n'})
        code, _, _ = run_main(self.root, "--record")
        self.assertEqual(code, 4)
        self.assertFalse((self.root / cr.REGISTER_PATH).exists())

    def test_record_ignores_uncommitted_changes_outside_tests(self):
        write_tree(self.root, {"notes.txt": "x"})
        self.assertEqual(run_main(self.root, "--record")[0], 0)

    def test_record_refuses_in_a_linked_worktree_unless_allowed(self):
        wt = Path(self._tmp.name) / "wt"
        git(self.root, "worktree", "add", "-q", str(wt), "-b", "other")
        code, out, _ = run_main(wt, "--record")
        self.assertEqual(code, 4)
        self.assertIn("worktree", out)
        self.assertFalse((wt / cr.REGISTER_PATH).exists())
        code, out, _ = run_main(wt, "--record", "--allow-worktree")
        self.assertEqual(code, 0, out)
        self.assertTrue((wt / cr.REGISTER_PATH).is_file())

    def test_record_refuses_outside_a_git_repository(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_tree(tmp, {A: BASE})
            code, _, _ = run_main(tmp, "--record")
            self.assertEqual(code, 4)
            self.assertFalse((Path(tmp) / cr.REGISTER_PATH).exists())


INSTR_TESTS = '''
TEST_CASE("proof: known answer") { CHECK(1 == 1); }
TEST_CASE("proof: expected to fail" * doctest::should_fail()) { CHECK(1 == 2); }
TEST_CASE("proof: measured set" * doctest::skip()) { CHECK(2 == 2); }
'''

INSTR_GOOD = '''
[sim]
what = "a simulator"
known_answer = { cases = ["proof: known answer"], tolerance = "exact" }
smallest_effect = { cases = ["proof: measured set"], measured = "2026-11-25, notes.md", says = "1.5 at 90 %" }
planted_fault = { not_established = "nothing plants a fault" }
valid_for = { says = "gaussian errors", outside = "not detected" }
guards = { not_applicable = "it takes no input" }

[tool]
known_answer = { cases = ["PyTests.test_one", "proof: known answer"] }
smallest_effect = { not_applicable = "fixed limits" }
planted_fault = { not_established = "none" }
valid_for = { not_established = "unknown" }
guards = { not_established = "none" }
'''


class InstrumentsTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        write_tree(self.root, {"tests/i_test.cpp": INSTR_TESTS, "tests/test_py.py": PY_BASE})

    def check(self, toml, *args):
        write_tree(self.root, {cr.INSTRUMENTS_PATH: toml})
        return run_main(self.root, "--instruments", *args)

    def test_a_correct_file_prints_the_table(self):
        code, out, err = self.check(INSTR_GOOD)
        self.assertEqual(code, 0, out + err)
        rows = {line.split("|")[0].strip(): [c.strip() for c in line.split("|")[1:]] for line in out.splitlines() if "|" in line}
        self.assertEqual(rows["sim"], ["established (1 case)", "established, skipped set of 2026-11-25", "NOT ESTABLISHED",
                                       "stated (no case)", "n/a"])
        self.assertEqual(rows["tool"], ["established (2 cases)", "n/a", "NOT ESTABLISHED", "NOT ESTABLISHED", "NOT ESTABLISHED"])
        self.assertIn("instruments: ok", out)

    def test_a_missing_case_names_instrument_item_and_case(self):
        code, out, _ = self.check(INSTR_GOOD.replace('"proof: known answer"], tolerance', '"proof: renamed away"], tolerance'))
        self.assertEqual(code, 1)
        self.assertIn("instrument sim: item known_answer: case not in the register: 'proof: renamed away'", out)

    def test_a_should_fail_case_is_not_proof(self):
        code, out, _ = self.check(INSTR_GOOD.replace('cases = ["proof: known answer"], tolerance', 'cases = ["proof: expected to fail"], tolerance'))
        self.assertEqual(code, 1)
        self.assertIn("instrument sim: item known_answer: case is marked should_fail and cannot be proof: 'proof: expected to fail'", out)

    def test_an_expected_failure_python_test_is_not_proof(self):
        code, out, _ = self.check(INSTR_GOOD.replace('"PyTests.test_one"', '"PyTests.test_two"'))
        self.assertEqual(code, 1)
        self.assertIn("instrument tool: item known_answer: case is marked expectedFailure", out)

    def test_a_skipped_case_needs_measured(self):
        code, out, _ = self.check(INSTR_GOOD.replace('measured = "2026-11-25, notes.md", ', ""))
        self.assertEqual(code, 1)
        self.assertIn("instrument sim: item smallest_effect: case is skipped in the regular run and the item has no 'measured'", out)

    def test_a_missing_key_is_a_fault(self):
        code, out, _ = self.check(INSTR_GOOD.replace('guards = { not_applicable = "it takes no input" }\n', ""))
        self.assertEqual(code, 1)
        self.assertIn("instrument sim: item guards is missing", out)

    def test_an_empty_item_is_a_fault(self):
        code, out, _ = self.check(INSTR_GOOD.replace('guards = { not_applicable = "it takes no input" }', "guards = {}"))
        self.assertEqual(code, 1)
        self.assertIn("instrument sim: item guards has no cases", out)

    def test_plain_comparison_checks_the_instruments_too(self):
        write_tree(self.root, {cr.REGISTER_PATH: register_text(self.root), cr.INSTRUMENTS_PATH: INSTR_GOOD})
        code, out, _ = run_main(self.root)
        self.assertEqual(code, 0, out)
        self.assertEqual(out.strip().splitlines()[-1], "IDENTICAL")
        write_tree(self.root, {cr.INSTRUMENTS_PATH: INSTR_GOOD.replace("proof: known answer", "proof: gone", 1)})
        code, out, _ = run_main(self.root)
        self.assertEqual(code, 1)
        self.assertIn("INSTRUMENTS FAULT", out)

    def test_a_renamed_proof_fails_the_instrument_and_the_register_comparison(self):
        write_tree(self.root, {cr.REGISTER_PATH: register_text(self.root), cr.INSTRUMENTS_PATH: INSTR_GOOD})
        text = (self.root / "tests/i_test.cpp").read_text(encoding="utf-8")
        write_tree(self.root, {"tests/i_test.cpp": text.replace("proof: known answer", "proof: renamed")})
        code, out, _ = run_main(self.root)
        self.assertEqual(code, 1)
        self.assertIn("CHANGE case removed", out)
        self.assertIn("instrument sim: item known_answer: case not in the register: 'proof: known answer'", out)

    def test_a_changed_case_of_a_section_is_pointed_out_for_review(self):
        write_tree(self.root, {cr.REGISTER_PATH: register_text(self.root), cr.INSTRUMENTS_PATH: INSTR_GOOD})
        text = (self.root / "tests/i_test.cpp").read_text(encoding="utf-8")
        write_tree(self.root, {"tests/i_test.cpp": text.replace("CHECK(1 == 1)", "CHECK(1 <= 1)")})
        code, out, _ = run_main(self.root)
        self.assertEqual(code, 1)
        self.assertIn("instruments: review section sim", out)

    def test_an_absent_instruments_file_is_one_line_and_not_a_failure(self):
        write_tree(self.root, {cr.REGISTER_PATH: register_text(self.root)})
        code, out, _ = run_main(self.root)
        self.assertEqual(code, 0)
        self.assertIn("no checks/instruments.toml: instruments not checked", out)

    def test_instruments_asked_for_but_absent_exits_2(self):
        code, _, err = run_main(self.root, "--instruments")
        self.assertEqual(code, 2)
        self.assertIn("instruments.toml", err)

    def test_the_last_line_of_a_plain_comparison_names_instrument_faults(self):
        write_tree(self.root, {cr.REGISTER_PATH: register_text(self.root), cr.INSTRUMENTS_PATH: INSTR_GOOD.replace("proof: known answer", "proof: gone", 1)})
        code, out, _ = run_main(self.root)
        self.assertEqual(code, 1)
        self.assertEqual(out.strip().splitlines()[-1], "IDENTICAL; instruments: 1 fault")

    def test_an_unreadable_instruments_file_exits_2(self):
        code, _, err = self.check("[sim\n")
        self.assertEqual(code, 2)
        self.assertIn("instruments.toml", err)


class RealTreeTest(unittest.TestCase):
    def test_the_reader_reads_every_real_test_file_with_the_cross_count_holding(self):
        reg = cr.build_register(ROOT)
        files = {b.file for b in reg.blocks}
        cpp, py = cr.test_files(ROOT)
        self.assertEqual(files, {p.relative_to(ROOT).as_posix() for p in cpp + py})
        cases = [b for b in reg.blocks if b.name is not None and b.file.endswith((".cpp", ".h"))]
        independent = sum(len(__import__("re").findall(r"^TEST_CASE\(", p.read_text(encoding="utf-8"), __import__("re").M))
                          for p in cpp)
        self.assertEqual(len(cases), independent)
        self.assertGreater(sum(1 for b in cases for k, _ in b.lines if k == "assert"), 1000)

    def test_two_generations_give_identical_bytes(self):
        self.assertEqual(register_text(ROOT), register_text(ROOT))

    def test_the_real_register_parses_back_to_itself(self):
        text = register_text(ROOT)
        self.assertEqual(cr.render_register(cr.parse_register(text)), text)


if __name__ == "__main__":
    unittest.main()
