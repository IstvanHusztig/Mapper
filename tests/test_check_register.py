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

    double generate()
    {
        const double sigma = 0.02;
        const auto &first = table[k];
        return sigma * first.tol;
    }
}

TEST_CASE("alpha: plain")
{
    const double tol = 0.05;
    const double measured = 0.1;
    const auto &ref = table[k];
    CHECK(ref.tol > 0);
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
        self.assertEqual(lines, ["CHECK(ref.tol > 0)", "CHECK(1.0 < 2.0)", "CHECK_FALSE(false)", "REQUIRE(tol == doctest::Approx(0.05).epsilon(1e-6))",
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

    def test_a_reference_binding_without_a_literal_is_not_a_value(self):
        reg = self.build({A: BASE})
        for b in reg.blocks:
            self.assertFalse([t for k, t in b.lines if k == "value" and ("ref =" in t or "first =" in t)])

    def test_constants_of_any_function_body_are_listed_under_the_file_with_the_function_name(self):
        reg = self.build({A: BASE})
        values = [t for k, t in self.file_block(reg, A).lines if k == "value"]
        self.assertIn("generate() :: const double sigma = 0.02", values)

    def test_a_non_const_local_with_a_literal_is_a_value_and_a_plain_assignment_is_not(self):
        src = 'TEST_CASE("n") { double x = 0.5; double y = x; x = 2.0; list.at = 3; for (const auto &v : list) { CHECK(v < x); } }\n'
        reg = self.build({A: src})
        self.assertEqual(self.case(reg, "n").lines, [("value", "double x = 0.5"), ("assert", "CHECK(v < x)")])

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

    def test_python_tests_of_a_class_derived_from_a_test_base_are_found(self):
        src = chr(10).join([
            "import unittest",
            "",
            "class Base(unittest.TestCase):",
            "    def setUp(self):",
            "        self.x = 1",
            "",
            "class Derived(Base):",
            "    def test_a(self):",
            "        self.assertEqual(self.x, 1)",
            "",
            "class NotATest:",
            "    def test_b(self):",
            "        pass",
            ""])
        reg = self.build({"tests/test_d.py": src})
        self.assertEqual([b.name for b in reg.blocks if b.name], ["Derived.test_a"])

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
        self.assertEqual(line, "# totals: 5 cases (1 should_fail, 1 skip), 11 assertions, 2 python tests (1 expectedFailure)")


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

    def test_a_renamed_reference_binding_is_no_difference(self):
        self.edit("const auto &ref = table[k];", "const auto &renamed = table[k];")
        self.edit("CHECK(ref.tol > 0);", "CHECK(ref.tol > 0);  // same")
        self.edit("const auto &first = table[k];", "const auto &other = table[k];")
        self.assertKinds(0)

    def test_a_changed_generator_constant_is_a_value_changed(self):
        self.edit("const double sigma = 0.02;", "const double sigma = 0.03;")
        out = self.assertKinds(1, "CHANGE value changed: tests/a_test.cpp"+chr(10))
        self.assertIn("  + generate() :: const double sigma = 0.03", out)

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

    def test_case_moved_to_another_file_is_a_change(self):
        text = (self.root / A).read_text(encoding="utf-8")
        moved = 'TEST_CASE("epsilon: movable")\n{\n    CHECK(3 > 2);\n}\n'
        self.assertIn(moved, text)
        self.edit(moved, "")
        write_tree(self.root, {"tests/b_test.cpp": moved})
        out = self.assertKinds(1, "CHANGE case moved: tests/b_test.cpp :: epsilon: movable")
        self.assertIn("from tests/a_test.cpp", out)
        self.assertEqual(out.strip().splitlines()[-1], "1 case moved; to record: +1 file")

    def test_any_reworded_assertion_is_a_change(self):
        self.edit("CHECK(measured < 0.25)", "CHECK(observed < 0.25)")
        out = self.assertKinds(1, "CHANGE assertion changed: tests/a_test.cpp :: alpha: plain", absent=("reworded", "ADD assertion"))
        self.assertIn("- CHECK(measured < 0.25)", out)
        self.assertIn("+ CHECK(observed < 0.25)", out)
        self.assertEqual(out.strip().splitlines()[-1], "1 assertion changed")

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

    def test_no_register_recorded_cannot_be_compared_and_exits_2(self):
        (self.root / cr.REGISTER_PATH).unlink()
        code, out, err = self.compare()
        self.assertEqual(code, 2)
        self.assertIn("no register: cannot compare", err)
        self.assertNotIn("IDENTICAL", out)

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


HOLES_CPP = r'''#include "doctest.h"
#include <sstream>

#define KEEP(...) __VA_ARGS__

namespace
{
    void inside(double x, double limit) { CHECK(x < limit); }

    const auto check_all = [](double value)
    {
        CHECK(value > 0.0);
        inside(value, 100.0);
    };

    auto not_a_helper = [](double x) { return x * 2.0; };
}

TEST_CASE("pins")
{
    const auto pin = [](const char *what, unsigned long long got, unsigned long long want)
    {
        if (got != want)
        {
            std::ostringstream message;
            message << what;
            FAIL_CHECK(message.str());
        }
    };
    pin("defaults", 1ull, 0x1ad4c1d5688ff94aull);
    pin("file", 2ull, 0x00000002ull);
}

TEST_CASE("fail macro")
{
    if (bad)
        FAIL("stop");
    CHECK(true);
}

TEST_CASE("lambda call")
{
    double buckets = 3.0;
    check_all(buckets);
    CHECK(not_a_helper(1.0) == 2.0);
}

TEST_CASE("wrapped")
{
    CHECK(1 < 2);
    CHECK(stamps_ok);
    CHECK(std::abs(equal.q_mean - 1.0) < 0.05);
    CHECK(t_begin >= 3.0);
    CHECK(flag);
    CHECK(a < b);
    REQUIRE(cell.ok);
}

TEST_CASE("loops and values")
{
    int n = 3;
    for (const double factor : {1.5, 1.0 / 1.5}) { CHECK(factor > 0.0); }
    for (int i = 0; i < 5; ++i) { CHECK(i < n); }
    for (const auto &v : list) { CHECK(v > 0); }
    CHECK(n > 0);
}

TEST_CASE("neutral skip" * doctest::skip(false)) { CHECK(1); }
TEST_CASE("plain second") { CHECK(1 < 2); }
'''

PY_HOLES = '''import sys
import unittest


def verify(tc, x):
    tc.assertLess(x, 2)


class Base(unittest.TestCase):
    def setUp(self):
        self.assertTrue(self.ready())

    def ready(self):
        return True

    def check(self, a, b):
        self.assertEqual(a, b)
        self.assertIn(a, [b])

    def indirect(self, a):
        self.check(a, a)


class Tests(Base):
    def test_a(self):
        self.check(1, 1)
        self.indirect(2)
        verify(self, 1)
        self.assertTrue(ok)

    @unittest.skipIf(sys.platform == "x", "why")
    def test_b(self):
        self.assertLess(1, 2)

    def test_c(self):
        def local(x):
            self.assertGreaterEqual(x, 0)
        local(1)
'''

H = "tests/h_test.cpp"
HP = "tests/test_h.py"


class HolesTest(unittest.TestCase):
    """Edits that weaken a test and must still not read IDENTICAL or as an addition (the reviewer's holes of check-register)."""

    def trial(self, edits, expect=1, kinds=(), absent=(), files=None):
        files = files or {H: HOLES_CPP, HP: PY_HOLES}
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_tree(root, files)
            write_tree(root, {cr.REGISTER_PATH: register_text(root)})
            self.assertEqual(run_main(root)[0], 0)
            for rel, old, new in edits:
                text = (root / rel).read_text(encoding="utf-8")
                self.assertIn(old, text)
                write_tree(root, {rel: text.replace(old, new, 1)})
            code, out, err = run_main(root)
            self.assertEqual(code, expect, out + err)
            for kind in kinds:
                self.assertIn(kind, out)
            for kind in absent:
                self.assertNotIn(kind, out)
            return out

    def each(self, cases, expect=1):
        for label, edits, kinds in cases:
            with self.subTest(label):
                self.trial(edits, expect, kinds)

    def build(self, files):
        with tempfile.TemporaryDirectory() as tmp:
            write_tree(tmp, files)
            return cr.build_register(tmp)

    # -- 1 and 2: FAIL and FAIL_CHECK, lambdas that assert

    def test_fail_and_fail_check_are_assertions_and_lambdas_that_assert_are_helpers(self):
        reg = self.build({H: HOLES_CPP})
        pins = [b for b in reg.blocks if b.name == "pins"][0].lines
        self.assertIn(("assert", "[pin] FAIL_CHECK(message.str())"), pins)
        self.assertIn(("call", 'pin("defaults", 1ull, 0x1ad4c1d5688ff94aull)'), pins)
        self.assertIn(("assert", "FAIL(\"stop\")"), [b for b in reg.blocks if b.name == "fail macro"][0].lines)
        calls = [b for b in reg.blocks if b.name == "lambda call"][0].lines
        self.assertIn(("call", "check_all(buckets)"), calls)
        self.assertFalse([t for k, t in calls if "not_a_helper" in t and k == "call"])
        fb = [b for b in reg.blocks if b.name is None][0].lines
        self.assertIn(("assert", "check_all() :: CHECK(value > 0.0)"), fb)
        self.assertIn(("call", "check_all() :: inside(value, 100.0)"), fb)
        tot = cr.totals(reg)
        self.assertEqual((tot["macros"]["FAIL_CHECK"], tot["macros"]["FAIL"]), (1, 1))

    def test_the_settings_hash_pin_cases(self):
        self.each([
            ("pinned literal 0", [(H, "0x1ad4c1d5688ff94aull", "0x0ull")], ("call changed",)),
            ("FAIL_CHECK line replaced", [(H, "            FAIL_CHECK(message.str());", "            (void)message;")], ("assertion removed",)),
            ("FAIL replaced", [(H, '        FAIL("stop");', "        (void)bad;")], ("assertion removed",)),
            ("pin call removed", [(H, '    pin("file", 2ull, 0x00000002ull);', "")], ("call removed",)),
            ("second pin literal", [(H, "0x00000002ull", "0x0ull")], ("call changed",)),
        ])

    def test_lambda_helper_cases(self):
        self.each([
            ("call removed", [(H, "    check_all(buckets);", "    (void)buckets;")], ("call removed",)),
            ("lambda body weakened", [(H, "        CHECK(value > 0.0);", "        CHECK(true);")], ("assertion changed",)),
            ("lambda body call to asserting helper removed", [(H, "        inside(value, 100.0);", "")], ("call removed",)),
            ("case-scope lambda body weakened", [(H, "            FAIL_CHECK(message.str());", "            FAIL_CHECK(\"\");")], ("assertion changed",)),
        ])

    def test_a_lambda_that_does_not_assert_is_no_helper_and_a_plain_auto_lambda_that_does_is(self):
        src = 'void h(int x) { CHECK(x); }\nauto via = [](int x) { h(x); };\nauto sq = [](int x) { return x * x; };\nTEST_CASE("t") { via(1); sq(2); }\n'
        reg = self.build({A: src})
        self.assertEqual([b for b in reg.blocks if b.name == "t"][0].lines, [("call", "via(1)")])

    # -- 3: any difference in an assertion's text is a change

    def test_reworded_assertions_are_changes(self):
        self.each([
            ("abs removed", [(H, "CHECK(std::abs(equal.q_mean - 1.0) < 0.05)", "CHECK(equal.q_mean - 1.0 < 0.05)")], ("assertion changed",)),
            ("other variable", [(H, "CHECK(std::abs(equal.q_mean - 1.0) < 0.05)", "CHECK(std::abs(other.q_mean - 1.0) < 0.05)")], ("assertion changed",)),
            ("condition to true", [(H, "CHECK(stamps_ok)", "CHECK(true)")], ("assertion changed",)),
            ("operands swapped", [(H, "CHECK(t_begin >= 3.0)", "CHECK(3.0 >= t_begin)")], ("assertion changed",)),
            ("CHECK to WARN", [(H, "CHECK(t_begin >= 3.0)", "WARN(t_begin >= 3.0)")], ("assertion changed",)),
            ("REQUIRE to CHECK", [(H, "REQUIRE(cell.ok)", "CHECK(cell.ok)")], ("assertion changed",)),
            ("|| true", [(H, "CHECK(flag)", "CHECK(flag || true)")], ("assertion changed",)),
            ("assertion removed", [(H, "    CHECK(a < b);", "")], ("assertion removed",)),
            ("python assertEqual to assertGreaterEqual", [(HP, "        self.assertEqual(a, b)", "        self.assertGreaterEqual(a, b)")], ("assertion changed",)),
            ("python assertLess to assertGreater", [(HP, "        self.assertLess(1, 2)", "        self.assertGreater(1, 2)")], ("assertion changed",)),
            ("python assertIn to assertNotIn", [(HP, "self.assertIn(a, [b])", "self.assertNotIn(a, [b])")], ("assertion changed",)),
            ("python assertTrue to assertFalse", [(HP, "        self.assertTrue(ok)", "        self.assertFalse(ok)")], ("assertion changed",)),
        ])

    def test_no_edit_reads_as_reworded_any_more(self):
        out = self.trial([(H, "CHECK(flag)", "CHECK(flagged)")])
        self.assertNotIn("reworded", out)
        self.assertEqual(out.strip().splitlines()[-1], "1 assertion changed")

    # -- 4: markers outside the case head

    def test_suite_decorators_apply_to_every_case_inside(self):
        src = 'TEST_SUITE("s" * doctest::skip()) {\nTEST_CASE("a") { CHECK(1); }\nTEST_CASE("b" * doctest::should_fail()) { CHECK(2); }\n}\nTEST_CASE("c") { CHECK(3); }\n'
        reg = self.build({A: src})
        markers = {b.name: b.markers for b in reg.blocks if b.name}
        self.assertEqual(markers, {"a": ["suite:skip"], "b": ["should_fail", "suite:skip"], "c": []})
        src = 'TEST_SUITE_BEGIN("s" * doctest::timeout(5));\nTEST_CASE("a") { CHECK(1); }\nTEST_SUITE_END();\nTEST_CASE("c") { CHECK(3); }\n'
        markers = {b.name: b.markers for b in self.build({A: src}).blocks if b.name}
        self.assertEqual(markers, {"a": ["suite:timeout(5)"], "c": []})

    def test_wrapping_cases_in_a_skipped_suite_is_a_change(self):
        self.each([
            ("TEST_SUITE skip", [(H, 'TEST_CASE("plain second") { CHECK(1 < 2); }', 'TEST_SUITE("off" * doctest::skip()) {\nTEST_CASE("plain second") { CHECK(1 < 2); }\n}')], ("marker added", "+ suite:skip")),
            ("TEST_SUITE_BEGIN skip", [(H, 'TEST_CASE("plain second")', 'TEST_SUITE_BEGIN("off" * doctest::skip());\nTEST_CASE("plain second")')], ("marker added", "+ suite:skip")),
            ("TEST_SUITE should_fail", [(H, 'TEST_CASE("plain second") { CHECK(1 < 2); }', 'TEST_SUITE("off" * doctest::should_fail()) {\nTEST_CASE("plain second") { CHECK(1 < 2); }\n}')], ("marker added", "+ suite:should_fail")),
        ])

    def test_a_suite_without_decorators_and_a_closed_begin_change_nothing(self):
        self.trial([(H, 'TEST_CASE("plain second") { CHECK(1 < 2); }', 'TEST_SUITE("plain") {\nTEST_CASE("plain second") { CHECK(1 < 2); }\n}')], expect=0)
        self.trial([(H, 'TEST_CASE("plain second")', 'TEST_SUITE_BEGIN("off" * doctest::skip());\nTEST_SUITE_END();\nTEST_CASE("plain second")')], expect=0)

    def test_skip_false_and_skip_are_told_apart(self):
        self.each([
            ("skip(false) to skip()", [(H, "doctest::skip(false)", "doctest::skip()")], ("marker added", "marker removed")),
            ("skip(false) to skip(true)", [(H, "doctest::skip(false)", "doctest::skip(true)")], ("marker added", "marker removed")),
            ("skip(false) removed", [(H, ' * doctest::skip(false)', "")], ("marker removed",)),
        ])

    def test_python_class_level_skips_and_skip_test_are_markers(self):
        reg = self.build({HP: PY_HOLES})
        self.assertEqual([b.markers for b in reg.blocks if b.name == "Tests.test_b"][0], ["skipIf(sys.platform == 'x', 'why')"])
        self.each([
            ("class skip", [(HP, "class Tests(Base):", "@unittest.skip('x')\nclass Tests(Base):")], ("marker added", "+ class:skip('x')")),
            ("base class skip reaches the derived", [(HP, "class Base(unittest.TestCase):", "@unittest.skip('x')\nclass Base(unittest.TestCase):")], ("marker added", "+ class:skip('x')")),
            ("class expectedFailure", [(HP, "class Tests(Base):", "@unittest.expectedFailure\nclass Tests(Base):")], ("marker added",)),
            ("skipTest in a test", [(HP, "    def test_a(self):\n", "    def test_a(self):\n        self.skipTest('x')\n")], ("marker added: tests/test_h.py :: Tests.test_a", "+ skipTest", "CHANGE skip added")),
            ("raise SkipTest in a test", [(HP, "    def test_b(self):\n", "    def test_b(self):\n        raise unittest.SkipTest('x')\n")], ("marker added",)),
            ("skipTest in setUp", [(HP, "    def setUp(self):\n", "    def setUp(self):\n        self.skipTest('x')\n")], ("CHANGE skip added",)),
            ("skipIf condition True", [(HP, 'sys.platform == "x"', "True")], ("marker added", "marker removed")),
            ("skipIf reason", [(HP, '"why"', '"other"')], ("marker added", "marker removed")),
        ])

    def test_a_marker_with_a_comma_survives_the_register_text(self):
        reg = self.build({HP: PY_HOLES})
        back = cr.parse_register(cr.render_register(reg))
        self.assertEqual(cr.compare(back, reg), [])

    def test_instruments_see_through_suite_and_class_markers(self):
        files = {"tests/i_test.cpp": 'TEST_SUITE("s" * doctest::skip()) {\nTEST_CASE("proof") { CHECK(1 == 1); }\n}\n'}
        with tempfile.TemporaryDirectory() as tmp:
            write_tree(tmp, files)
            toml = '[sim]\nknown_answer = { cases = ["proof"] }\nsmallest_effect = { not_applicable = "x" }\nplanted_fault = { not_applicable = "x" }\nvalid_for = { not_applicable = "x" }\nguards = { not_applicable = "x" }\n'
            write_tree(tmp, {cr.INSTRUMENTS_PATH: toml})
            code, out, _ = run_main(tmp, "--instruments")
            self.assertEqual(code, 1, out)
            self.assertIn("case is skipped in the regular run", out)

    # -- 5: helper bodies in Python

    def test_python_helpers_and_set_ups_are_listed_under_the_file(self):
        reg = self.build({HP: PY_HOLES})
        fb = [b for b in reg.blocks if b.name is None][0].lines
        self.assertIn(("assert", "verify() :: tc.assertLess(x, 2)"), fb)
        self.assertIn(("assert", "Base.setUp() :: self.assertTrue(self.ready())"), fb)
        self.assertIn(("assert", "Base.check() :: self.assertEqual(a, b)"), fb)
        self.assertIn(("call", "Base.indirect() :: self.check(a, a)"), fb)
        self.assertFalse([t for k, t in fb if "ready() ::" in t])
        a = [b for b in reg.blocks if b.name == "Tests.test_a"][0].lines
        self.assertIn(("call", "verify(self, 1)"), a)
        self.assertIn(("call", "self.indirect(2)"), a)
        c = [b for b in reg.blocks if b.name == "Tests.test_c"][0].lines
        self.assertIn(("call", "local(1)"), c)
        self.assertIn(("assert", "self.assertGreaterEqual(x, 0)"), c)

    def test_python_helper_body_cases(self):
        self.each([
            ("assertEqual deleted from a helper", [(HP, "        self.assertEqual(a, b)\n", "")], ("assertion removed",)),
            ("assertIn deleted from a helper", [(HP, "        self.assertIn(a, [b])\n", "")], ("assertion removed",)),
            ("assertion deleted from setUp", [(HP, "        self.assertTrue(self.ready())", "        pass")], ("assertion removed",)),
            ("assertion deleted from a module function", [(HP, "    tc.assertLess(x, 2)", "    pass")], ("assertion removed",)),
            ("call of a helper removed", [(HP, "        self.check(1, 1)\n", "")], ("call removed",)),
            ("call of an indirect helper removed", [(HP, "        self.indirect(2)\n", "")], ("call removed",)),
            ("call of a module function removed", [(HP, "        verify(self, 1)\n", "")], ("call removed",)),
            ("call of a local function removed", [(HP, "        local(1)\n", "")], ("call removed",)),
            ("assertion in a local function weakened", [(HP, "self.assertGreaterEqual(x, 0)", "self.assertGreaterEqual(x, -9)")], ("assertion changed",)),
        ])

    # -- 6: #define and #undef, assertions inside macro arguments

    def test_define_and_undef_are_conditionals(self):
        self.each([
            ("define added", [(H, '#include "doctest.h"', '#include "doctest.h"\n#define OFF(...)')], ("conditional added", "+ #define OFF(...)")),
            ("undef added", [(H, "#define KEEP(...) __VA_ARGS__", "#define KEEP(...) __VA_ARGS__\n#undef FLAG")], ("conditional added",)),
            ("define changed", [(H, "#define KEEP(...) __VA_ARGS__", "#define KEEP(...)")], ("conditional added", "conditional removed")),
            ("define removed", [(H, "#define KEEP(...) __VA_ARGS__\n", "")], ("conditional removed",)),
            ("neutraliser with a wrapped assertion", [(H, '#include "doctest.h"', '#include "doctest.h"\n#define OFF(...)'), (H, "    CHECK(a < b);", "    OFF(CHECK(a < b));")], ("conditional added",)),
        ])

    def test_an_assertion_inside_a_function_like_macros_arguments_is_flagged(self):
        self.each([("wrapped in an existing macro", [(H, "    CHECK(a < b);", "    KEEP(CHECK(a < b));")], ("assertion changed", "[macro KEEP] CHECK(a < b)"))])

    # -- 7: more values and loops

    def test_values_and_loops_are_lines(self):
        reg = self.build({H: HOLES_CPP})
        lines = [b for b in reg.blocks if b.name == "loops and values"][0].lines
        self.assertIn(("value", "int n = 3"), lines)
        self.assertIn(("loop", "for (const double factor : {1.5, 1.0 / 1.5})"), lines)
        self.assertIn(("loop", "for (int i = 0; i < 5; ++i)"), lines)
        self.assertFalse([t for k, t in lines if k == "loop" and "list" in t])

    def test_value_and_loop_cases(self):
        self.each([
            ("range-based list", [(H, "{1.5, 1.0 / 1.5}", "{50.0, 1.0 / 50.0}")], ("loop changed",)),
            ("classic bound", [(H, "i < 5", "i < 50")], ("loop changed",)),
            ("local non-const", [(H, "int n = 3;", "int n = 30;")], ("value changed",)),
            ("local with a call", [(H, "double buckets = 3.0;", "double buckets = 30.0;")], ("value changed",)),
        ])

    def test_brace_and_paren_initialisers_and_plain_statements(self):
        src = ('TEST_CASE("v")\n{\n    Foo f{1, 2};\n    Bar b(3.5);\n    std::array<double, 3> a{0.1, 0.2, 0.3};\n    auto g = [](int x) { return x < 7; };\n'
               '    x = 5;\n    obj.member = 6;\n    call(8);\n    return;\n}\n')
        lines = [b for b in self.build({A: src}).blocks if b.name == "v"][0].lines
        self.assertEqual([t for k, t in lines if k == "value"],
                         ["Foo f{1, 2}", "Bar b(3.5)", "std::array<double, 3> a{0.1, 0.2, 0.3}"])

    # -- 8, 9: a moved case, no register

    def test_a_moved_case_is_a_change(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_tree(root, {H: HOLES_CPP})
            write_tree(root, {cr.REGISTER_PATH: register_text(root)})
            moved = 'TEST_CASE("plain second") { CHECK(1 < 2); }\n'
            text = (root / H).read_text(encoding="utf-8")
            write_tree(root, {H: text.replace(moved, ""), "tests/other_test.cpp": moved})
            code, out, _ = run_main(root)
            self.assertEqual(code, 1, out)
            self.assertIn("CHANGE case moved", out)

    def test_without_a_register_nothing_can_be_compared(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_tree(tmp, {H: HOLES_CPP})
            code, out, err = run_main(tmp)
            self.assertEqual(code, 2)
            self.assertIn("no register: cannot compare", err)
            code, _, _ = run_main(tmp, "--record", "--allow-worktree")
            self.assertEqual(code, 4)            # not a git tree here; only --record in a clean tree writes it

    # -- what was found correct stays correct

    def test_unchanged_sources_are_identical_and_comment_or_blank_edits_change_nothing(self):
        self.trial([(H, "    CHECK(a < b);", "    CHECK(  a\n   <   b ); // now with a comment")], expect=0)


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
