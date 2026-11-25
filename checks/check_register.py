#!/usr/bin/env python3
"""The check register: what the tests assert, as data, and what proves each instrument.

Reads the test sources (tests/*.cpp, tests/*.h, tests/test_*.py) and writes / compares ONE text file,
checks/tests.register: per test case its file, name and markers, every assertion as one line, the calls it makes to
helpers that assert, and the constants and tables its limits come from. See openspec/changes/check-register
(design D1-D7).

    python checks/check_register.py                 compare the sources with checks/tests.register
    python checks/check_register.py --print         write the generated register to stdout
    python checks/check_register.py --record        write checks/tests.register (the orchestrator's; refuses with
                                                    uncommitted changes under tests/ and outside the main working
                                                    tree unless --allow-worktree)
    python checks/check_register.py --instruments   check checks/instruments.toml against the register, print the table
    --root DIR                                      another tree (default: the repository this file is in)

Exit codes of a comparison: 0 no difference, 3 additions only (cases, assertions, calls, values added; a case moved;
an assertion reworded), 1 a change (anything removed, changed, a marker or a conditional added or removed, an
instruments fault), 2 a source cannot be read (file:line on stderr). --record refuses with 4.

What it does NOT see (also in the register's header): a limit computed while a test runs from data; an assertion in a
branch that is never taken or a loop that runs zero times; limits in src/ (the verdict limits, the settings hash); the
meaning of a test; constants inside the bodies of functions that are neither cases nor asserting helpers; a value
constructed with a constructor call that has no numeric literal in its arguments.

Pure standard library; imports nothing from tools/.
"""
from __future__ import annotations

import argparse
import ast
import collections
import os
import re
import subprocess
import sys
import tomllib
from pathlib import Path

REGISTER_PATH = "checks/tests.register"
INSTRUMENTS_PATH = "checks/instruments.toml"

EXIT_SAME, EXIT_CHANGE, EXIT_UNREADABLE, EXIT_ADDITIONS, EXIT_REFUSED = 0, 1, 2, 3, 4

NOT_SEEN = [
    "# Not seen: limits computed while a test runs from data, assertions the flow never reaches (a branch not taken,",
    "# a loop that runs zero times), limits in src/ (the verdict limits, the settings hash), the meaning of a test,",
    "# constants inside functions that are not cases or asserting helpers.",
]


class ReadError(Exception):
    def __init__(self, path, line, message):
        super().__init__(f"{path}:{line}: {message}")
        self.path, self.line, self.message = path, line, message


# ---------------------------------------------------------------------------------------------------------------
# C++ lexer: comments out, literals whole, brackets matched, #if 0 out, other conditionals noted
# ---------------------------------------------------------------------------------------------------------------

class Tok:
    __slots__ = ("k", "t", "s", "e", "ln")

    def __init__(self, k, t, s, e, ln):
        self.k, self.t, self.s, self.e, self.ln = k, t, s, e, ln

    def __repr__(self):
        return f"Tok({self.k},{self.t!r},{self.ln})"


PUNCT3 = ("<<=", ">>=", "<=>", "->*", "...")
PUNCT2 = ("::", "->", "++", "--", "<<", ">>", "<=", ">=", "==", "!=", "&&", "||", "+=", "-=", "*=", "/=", "%=", "&=",
          "|=", "^=", ".*", "##")
IDENT = re.compile(r"[^\W\d]\w*")
RAW_PREFIX = {"R", "u8R", "uR", "UR", "LR"}
STR_PREFIX = {"u8", "u", "U", "L"}
OPENERS = {"(": ")", "[": "]", "{": "}"}
CLOSERS = {")": "(", "]": "[", "}": "{"}


def collapse(text):
    return re.sub(r"\s+", " ", text).strip()


class Lexed:
    def __init__(self, path, src, toks, match, conds, clean):
        self.path, self.src, self.toks, self.match, self.conds, self.clean = path, src, toks, match, conds, clean


def lex(src, path):
    return _Lexer(src, path).run()


class _Lexer:
    def __init__(self, src, path):
        self.src, self.path = src, path
        self.n = len(src)
        self.i = 0
        self.line = 1
        self.toks = []
        self.conds = []          # (token index, line, text)
        self.clean = list(src)
        self.ifstack = []        # ("live" | "if0", line)
        self.bstack = []         # token indices of open brackets
        self.match = {}

    def err(self, message, line=None):
        raise ReadError(self.path, self.line if line is None else line, message)

    def blank(self, a, b):
        c = self.clean
        for k in range(a, b):
            if c[k] != "\n":
                c[k] = " "

    def line_comment_end(self, i):
        src, n = self.src, self.n
        j = i
        while True:
            e = src.find("\n", j)
            if e < 0:
                return n
            if e > i and src[e - 1] == "\\":
                j = e + 1
                self.line += 1
                continue
            return e

    def quoted(self, j, q, what):
        src, n = self.src, self.n
        k = j + 1
        start_line = self.line
        while True:
            if k >= n or src[k] == "\n":
                self.err(f"unterminated {what} literal", start_line)
            ch = src[k]
            if ch == "\\":
                if src[k + 1:k + 2] == "\n":
                    self.line += 1
                k += 2
                continue
            if ch == q:
                return k + 1
            k += 1

    def add(self, kind, text, s, e, ln):
        self.toks.append(Tok(kind, text, s, e, ln))

    def run(self):
        src, n = self.src, self.n
        bol = True
        while self.i < n:
            i = self.i
            c = src[i]
            if c == "\n":
                self.line += 1
                self.i += 1
                bol = True
                continue
            if c in " \t\f\v":
                self.i += 1
                continue
            if c == "\\" and src.startswith("\\\n", i):
                self.line += 1
                self.i += 2
                continue
            if c == "/" and src.startswith("//", i):
                e = self.line_comment_end(i)
                self.blank(i, e)
                self.i = e
                continue
            if c == "/" and src.startswith("/*", i):
                e = src.find("*/", i + 2)
                if e < 0:
                    self.err("unterminated comment")
                self.line += src.count("\n", i, e + 2)
                self.blank(i, e + 2)
                self.i = e + 2
                continue
            if c == "#" and bol:
                self.directive()
                continue
            bol = False
            ln = self.line
            if c == '"' or c == "'":
                e = self.quoted(i, c, "string" if c == '"' else "character")
                self.blank(i, e)
                self.add("str" if c == '"' else "chr", src[i:e], i, e, ln)
                self.i = e
                continue
            m = IDENT.match(src, i)
            if m:
                word, e = m.group(), m.end()
                nxt = src[e:e + 1]
                if nxt == '"' and word in RAW_PREFIX:
                    p = src.find("(", e + 1)
                    delim = src[e + 1:p] if p >= 0 else ""
                    if p < 0 or len(delim) > 16 or re.search(r"[\s()\\]", delim):
                        self.err("cannot read the raw string literal")
                    term = ")" + delim + '"'
                    q = src.find(term, p + 1)
                    if q < 0:
                        self.err("unterminated raw string literal")
                    end = q + len(term)
                    self.line += src.count("\n", e, end)
                    self.blank(i, end)
                    self.add("str", src[i:end], i, end, ln)
                    self.i = end
                    continue
                if nxt in ('"', "'") and word in STR_PREFIX:
                    end = self.quoted(e, nxt, "string" if nxt == '"' else "character")
                    self.blank(i, end)
                    self.add("str" if nxt == '"' else "chr", src[i:end], i, end, ln)
                    self.i = end
                    continue
                self.add("id", word, i, e, ln)
                self.i = e
                continue
            if c.isdigit() or (c == "." and src[i + 1:i + 2].isdigit()):
                j = i + 1
                hexa = src.startswith(("0x", "0X"), i)
                while j < n:
                    ch = src[j]
                    if ch.isalnum() or ch in "_.":
                        j += 1
                    elif ch == "'" and src[j + 1:j + 2].isalnum():
                        j += 1
                    elif ch in "+-" and src[j - 1] in ("pP" if hexa else "eEpP"):
                        j += 1
                    else:
                        break
                self.add("num", src[i:j], i, j, ln)
                self.i = j
                continue
            if c in OPENERS:
                self.bstack.append(len(self.toks))
            elif c in CLOSERS:
                if not self.bstack:
                    self.err(f"unmatched '{c}'")
                if self.toks[self.bstack[-1]].t != CLOSERS[c]:
                    o = self.toks[self.bstack[-1]]
                    self.err(f"unmatched '{o.t}' (the '{c}' at line {self.line} does not close it)", o.ln)
                o = self.bstack.pop()
                self.match[o] = len(self.toks)
                self.match[len(self.toks)] = o
            op = None
            for cand in PUNCT3:
                if src.startswith(cand, i):
                    op = cand
                    break
            if op is None:
                for cand in PUNCT2:
                    if src.startswith(cand, i):
                        op = cand
                        break
            if op is None:
                op = c
            self.add("p", op, i, i + len(op), ln)
            self.i = i + len(op)
        if self.bstack:
            o = self.toks[self.bstack[-1]]
            self.err(f"unmatched '{o.t}'", o.ln)
        if self.ifstack:
            self.err("unterminated #if", self.ifstack[-1][1])
        match = [self.match.get(k, -1) for k in range(len(self.toks))]
        return Lexed(self.path, src, self.toks, match, self.conds, "".join(self.clean))

    def directive(self):
        src, n = self.src, self.n
        start, ln0 = self.i, self.line
        j = start + 1
        parts = []
        while j < n:
            ch = src[j]
            if ch == "\n":
                break
            if ch == "\\" and src.startswith("\\\n", j):
                j += 2
                self.line += 1
                parts.append(" ")
                continue
            if ch == "/" and src.startswith("/*", j):
                e = src.find("*/", j + 2)
                if e < 0:
                    self.err("unterminated comment", self.line)
                self.line += src.count("\n", j, e + 2)
                self.blank(j, e + 2)
                j = e + 2
                parts.append(" ")
                continue
            if ch == "/" and src.startswith("//", j):
                e = self.line_comment_end(j)
                self.blank(j, e)
                j = e
                break
            if ch == '"':
                e = j + 1
                while e < n and src[e] not in '"\n':
                    e += 2 if src[e] == "\\" else 1
                self.blank(j, min(e + 1, n))
                parts.append('""')
                j = e + 1
                continue
            parts.append(ch)
            j += 1
        end = min(j, n)
        self.i = end
        text = "".join(parts)
        m = re.match(r"\s*(\w+)\s*(.*)", text, re.S)
        if not m:
            return
        name, rest = m.group(1), collapse(m.group(2))
        if name == "if" and rest == "0":
            self.skip_if0(start, end, ln0)
            return
        shown = "#" + name + (" " + rest if rest else "")
        if name in ("if", "ifdef", "ifndef"):
            self.ifstack.append(("live", ln0))
            self.conds.append((len(self.toks), ln0, shown))
        elif name in ("else", "elif"):
            if not self.ifstack:
                self.err(f"#{name} without #if", ln0)
            self.conds.append((len(self.toks), ln0, shown))
        elif name == "endif":
            if not self.ifstack:
                self.err("#endif without #if", ln0)
            kind, _ = self.ifstack.pop()
            if kind == "live":
                self.conds.append((len(self.toks), ln0, shown))

    def skip_if0(self, start, end, ln0):
        """`#if 0` removed like a comment up to its #endif; the #else branch of it stays."""
        src, n = self.src, self.n
        self.blank(start, end)
        depth = 1
        pos = end + 1
        cur = self.line + 1
        while True:
            if pos > n:
                self.err("unterminated #if 0", ln0)
            e = src.find("\n", pos)
            e = n if e < 0 else e
            m = re.match(r"\s*#\s*(\w+)\s*(.*)", src[pos:e])
            self.blank(pos, e)
            if m:
                nm = m.group(1)
                if nm in ("if", "ifdef", "ifndef"):
                    depth += 1
                elif nm == "endif":
                    depth -= 1
                    if depth == 0:
                        break
                elif nm in ("else", "elif") and depth == 1:
                    if nm == "else":
                        self.ifstack.append(("if0", ln0))
                    else:
                        self.ifstack.append(("live", ln0))
                        self.conds.append((len(self.toks), cur, "#elif " + collapse(m.group(2))))
                    break
            if e >= n:
                self.err("unterminated #if 0", ln0)
            pos = e + 1
            cur += 1
        self.line = cur
        self.i = e


# ---------------------------------------------------------------------------------------------------------------
# C++ structure: scopes, cases, functions, values
# ---------------------------------------------------------------------------------------------------------------

ASSERT_NAME = re.compile(r"(?:CHECK|REQUIRE|WARN)(?:_[A-Za-z0-9_]+)?\Z")
ASSERT_WORD = re.compile(r"(?<![A-Za-z0-9_])(?:CHECK|REQUIRE|WARN)(?:_[A-Za-z0-9_]+)?(?![A-Za-z0-9_])")


class Func:
    __slots__ = ("name", "is_case", "case_name", "markers", "line", "lo", "hi", "cls")

    def __init__(self, name, is_case, line, lo, hi, cls, case_name="", markers=()):
        self.name, self.is_case, self.line, self.lo, self.hi, self.cls = name, is_case, line, lo, hi, cls
        self.case_name, self.markers = case_name, list(markers)


class FileScan:
    def __init__(self, lx):
        self.lx = lx
        self.path = lx.path
        self.funcs = []          # in source order
        self.values = []         # (token index, text) outside function bodies
        self.declared = set()    # names of functions declared (not defined) at scope level


def render(lx, a, b, abbreviate=False):
    """The tokens a..b-1 as one line: comments gone, white space collapsed to one blank (or none where there was none)."""
    toks, src = lx.toks, lx.src
    out = []
    k = a
    prev_end = None
    while k < b:
        t = toks[k]
        if prev_end is not None:
            out.append(" " if src[prev_end:t.s] else "")
        if abbreviate and t.t == "{" and lx.match[k] > k:
            close = lx.match[k]
            if any(toks[q].t == ";" for q in range(k + 1, close)):
                out.append("{...}")
                prev_end = toks[close].e
                k = close + 1
                continue
        out.append(t.t)
        prev_end = t.e
        k += 1
    text = collapse("".join(out))
    return re.sub(r"([(\[]) | ([)\]])| (,)", lambda m: m.group(1) or m.group(2) or m.group(3), text)


def decode_string(tok_text):
    m = re.match(r'(?:u8|u|U|L)?(R)?"', tok_text)
    body_start = m.end()
    if m.group(1):
        p = tok_text.index("(", body_start)
        delim = tok_text[body_start:p]
        return tok_text[p + 1:len(tok_text) - len(delim) - 2]
    body = tok_text[body_start:-1]
    return re.sub(r"\\(.)", lambda mm: mm.group(1) if mm.group(1) in "\"\\'" else mm.group(0), body)


def read_case_head(scan, lp):
    """TEST_CASE( name [* decorators] ): the name and the markers."""
    lx = scan.lx
    toks = lx.toks
    rp = lx.match[lp]
    parts, cur, k = [], [], lp + 1
    while k < rp:
        t = toks[k]
        if t.t in OPENERS:
            cur.append(k)
            cur.extend(range(k + 1, lx.match[k] + 1))
            k = lx.match[k] + 1
            continue
        if t.t == "*":
            parts.append(cur)
            cur = []
        else:
            cur.append(k)
        k += 1
    parts.append(cur)
    name_toks = parts[0]
    if not name_toks or any(toks[q].k != "str" for q in name_toks):
        raise ReadError(scan.path, toks[lp].ln, "TEST_CASE name is not made of string literals")
    name = collapse("".join(decode_string(toks[q].t) for q in name_toks))
    markers = []
    for part in parts[1:]:
        if not part:
            raise ReadError(scan.path, toks[lp].ln, "empty decorator in TEST_CASE")
        text = render(lx, part[0], part[-1] + 1)
        text = text.split("::")[-1].strip()
        text = re.sub(r"\(\s*\)$", "", text)
        markers.append(text)
    return name, markers


def skip_template_header(lx, i, hi):
    toks = lx.toks
    if toks[i].t == "template" and i + 1 < hi and toks[i + 1].t == "<":
        depth = 0
        k = i + 1
        while k < hi:
            t = toks[k].t
            if t in OPENERS:
                k = lx.match[k] + 1
                continue
            if t == "<":
                depth += 1
            elif t == ">":
                depth -= 1
            elif t == ">>":
                depth -= 2
            k += 1
            if depth <= 0:
                return k
    return i


def stmt_end(lx, k, hi):
    toks = lx.toks
    while k < hi:
        t = toks[k].t
        if t in OPENERS:
            k = lx.match[k] + 1
            continue
        if t == ";" or t in CLOSERS:
            return k
        k += 1
    return hi


def has_number(lx, a, b):
    return any(lx.toks[q].k == "num" for q in range(a, b))


def walk_scope(scan, lo, hi, cls):
    lx = scan.lx
    toks, match = lx.toks, lx.match
    i = lo
    while i < hi:
        t = toks[i]
        if t.t == ";":
            i += 1
            continue
        if t.t in ("public", "private", "protected") and i + 1 < hi and toks[i + 1].t == ":":
            i += 2
            continue
        start = i
        j = skip_template_header(lx, i, hi)
        paren_idx = None
        mode = None
        while j < hi:
            tj = toks[j]
            if tj.t == ";":
                mode = "semi"
                break
            if tj.t == "{":
                mode = "brace"
                break
            if tj.t in ("(", "["):
                if tj.t == "(" and paren_idx is None:
                    paren_idx = j
                j = match[j] + 1
                continue
            if tj.t in CLOSERS:
                raise ReadError(scan.path, tj.ln, f"unexpected '{tj.t}' in a declaration")
            if tj.t == "=" and not (j > start and toks[j - 1].t == "operator"):
                mode = "assign"
                break
            j += 1
        if mode is None:
            raise ReadError(scan.path, toks[start].ln, "a declaration does not end before the end of its scope")
        first = toks[start].t
        prefix = "".join(c + " :: " for c in cls)
        if mode == "assign":
            k = stmt_end(lx, j, hi)
            if first not in ("using", "typedef", "static_assert") and has_number(lx, j + 1, k):
                scan.values.append((start, prefix + render(lx, start, k, True)))
            i = k + 1
            continue
        if mode == "semi":
            if paren_idx is not None and paren_idx > start and toks[paren_idx - 1].k == "id":
                scan.declared.add(toks[paren_idx - 1].t)
            if first not in ("using", "typedef", "static_assert", "friend") and paren_idx is not None:
                close = match[paren_idx]
                inside = range(paren_idx + 1, close)
                if has_number(lx, paren_idx + 1, close) and not any(toks[q].t == "=" for q in inside):
                    scan.values.append((start, prefix + render(lx, start, j, True)))
            i = j + 1
            continue
        # mode == "brace"
        head = range(start, j)
        words = [toks[q].t for q in head]
        close = match[j]
        if first == "TEST_CASE" and j == match[start + 1] + 1 and toks[start + 1].t == "(":
            name, markers = read_case_head(scan, start + 1)
            scan.funcs.append(Func("TEST_CASE", True, toks[start].ln, j + 1, close, cls, name, markers))
            i = close + 1
            continue
        if "namespace" in words[:2] or first == "TEST_SUITE" or (first == "extern" and len(words) > 1 and toks[start + 1].k == "str"):
            walk_scope(scan, j + 1, close, cls)
            i = close + 1
            continue
        if "enum" in words and paren_idx is None:
            i = close + 1
            continue
        if paren_idx is None and any(w in ("struct", "class", "union") for w in words):
            name = ""
            for q in head:
                if toks[q].t in ("struct", "class", "union") and q + 1 < j and toks[q + 1].k == "id":
                    name = toks[q + 1].t
            walk_scope(scan, j + 1, close, cls + ((name or "(anonymous)"),))
            i = close + 1
            continue
        if paren_idx is not None:
            nt = toks[paren_idx - 1] if paren_idx > start else toks[paren_idx]
            if nt.k == "id" and nt.t != "operator" and not (paren_idx - 2 >= start and toks[paren_idx - 2].t == "operator"):
                name = nt.t
            else:
                name = "operator"
            scan.funcs.append(Func(name, False, nt.ln, j + 1, close, cls))
            i = close + 1
            continue
        # a declaration initialised with braces: Foo x{1, 2};
        k = stmt_end(lx, close + 1, hi)
        if first not in ("using", "typedef") and has_number(lx, j, k):
            scan.values.append((start, prefix + render(lx, start, k, True)))
        i = k + 1
    return scan


def scan_cpp(path, src):
    lx = lex(src, path)
    scan = FileScan(lx)
    walk_scope(scan, 0, len(lx.toks), ())
    return scan


def asserting_helpers(scans):
    """{file: set of helper names visible in it}. A function defined in a .cpp is visible in its own file only, unless a
    header declares or defines the name (then it is global). A helper asserts, or calls a helper (closure over calls)."""
    global_names = set()
    for s in scans:
        if s.path.endswith(".h"):
            global_names |= s.declared | {f.name for f in s.funcs if not f.is_case}

    def node(path, name):
        return ("*", name) if name in global_names else (path, name)

    direct, calls = set(), collections.defaultdict(set)
    for s in scans:
        toks = s.lx.toks
        for f in s.funcs:
            if f.is_case or f.name.startswith("operator"):
                continue
            me = node(s.path, f.name)
            calls[me]
            for q in range(f.lo, f.hi):
                t = toks[q]
                if t.k == "id" and q + 1 < len(toks) and toks[q + 1].t == "(":
                    if ASSERT_NAME.match(t.t):
                        direct.add(me)
                    else:
                        calls[me].add(node(s.path, t.t))
    helpers = set(direct)
    changed = True
    while changed:
        changed = False
        for me, called in calls.items():
            if me not in helpers and called & helpers:
                helpers.add(me)
                changed = True
    visible = {}
    for s in scans:
        names = {n for (scope, n) in helpers if scope == s.path or scope == "*"}
        visible[s.path] = names
    return visible


def walk_body(scan, lo, hi, helpers):
    """Items (pos, kind, text) of one body in source order; also counts per macro name and subcases."""
    lx = scan.lx
    toks, match = lx.toks, lx.match
    items = []
    macro_counts = collections.Counter()
    subcases = 0
    sub = []                 # (end index, name)
    ostack = []
    i = lo
    while i < hi:
        while sub and i >= sub[-1][0]:
            sub.pop()
        t = toks[i]
        if t.k == "p":
            if t.t in OPENERS:
                ostack.append(t.t)
            elif t.t in CLOSERS and ostack:
                ostack.pop()
            i += 1
            continue
        if t.k != "id":
            i += 1
            continue
        prefix = "[" + "/".join(n for _, n in sub) + "] " if sub else ""
        nxt = toks[i + 1] if i + 1 < len(toks) else None
        call = nxt is not None and nxt.t == "("
        if t.t == "SUBCASE" and call:
            lp = i + 1
            rp = match[lp]
            strs = [toks[q] for q in range(lp + 1, rp)]
            name = collapse("".join(decode_string(x.t) for x in strs)) if strs and all(x.k == "str" for x in strs) else render(lx, lp + 1, rp)
            if rp + 1 >= hi or toks[rp + 1].t != "{":
                raise ReadError(scan.path, t.ln, "SUBCASE without a braced body")
            sub.append((match[rp + 1], name))
            subcases += 1
        elif call and ASSERT_NAME.match(t.t):
            items.append((i, "assert", prefix + render(lx, i, match[i + 1] + 1)))
            macro_counts[t.t] += 1
        elif call and t.t in helpers:
            items.append((i, "call", prefix + render(lx, i, match[i + 1] + 1)))
        elif t.t in ("const", "constexpr"):
            prev = toks[i - 1].t if i > 0 else "{"
            if prev in ("{", "}", ";", "static", "constexpr", "const", "inline", "thread_local") and (not ostack or ostack[-1] == "{") \
                    and not (prev in ("constexpr", "const") and toks[i - 1].t == prev and t.t == prev):
                k = stmt_end(lx, i, hi)
                depth0 = []
                q = i
                while q < k:
                    if toks[q].t in OPENERS:
                        q = match[q] + 1
                        continue
                    depth0.append(q)
                    q += 1
                has_assign = any(toks[q].t == "=" and not (q > 0 and toks[q - 1].t == "operator") for q in depth0)
                last = toks[k - 1].t if k - 1 > i else ""
                brace_init = last in ("}", ")") and k - 1 > i + 2 and match[k - 1] > 0 and toks[match[k - 1] - 1].k == "id"
                if has_assign or brace_init:
                    items.append((i, "value", prefix + render(lx, i, k, True)))
        i += 1
    return items, macro_counts, subcases


# ---------------------------------------------------------------------------------------------------------------
# The register model
# ---------------------------------------------------------------------------------------------------------------

class Block:
    """A [file] block (name None) or a [case] block."""

    def __init__(self, file, name=None, markers=(), lines=None):
        self.file, self.name = file, name
        self.markers = list(markers)
        self.lines = list(lines or [])      # (kind, text)

    def key(self):
        return (self.file, self.name)


class Register:
    def __init__(self, blocks=None):
        self.blocks = list(blocks or [])


def read_text(path, shown=None):
    try:
        data = Path(path).read_bytes().decode("utf-8-sig")
    except (OSError, UnicodeDecodeError) as exc:
        raise ReadError(shown or str(path), 0, f"cannot read: {exc}")
    return data.replace("\r\n", "\n").replace("\r", "\n")


def cross_check(scan, found_macros, n_cases, n_subcases):
    clean = scan.lx.clean
    plain = collections.Counter(m.group() for m in ASSERT_WORD.finditer(clean))
    problems = []
    if plain != found_macros:
        diff = {k: (found_macros.get(k, 0), plain.get(k, 0)) for k in set(plain) | set(found_macros)
                if plain.get(k, 0) != found_macros.get(k, 0)}
        problems.append("assertion macros read vs found by plain search " + ", ".join(f"{k} {a} vs {b}" for k, (a, b) in sorted(diff.items())))
    for word, found in (("TEST_CASE", n_cases), ("SUBCASE", n_subcases)):
        plain_n = len(re.findall(r"(?<![A-Za-z0-9_])" + word + r"(?![A-Za-z0-9_])", clean))
        if plain_n != found:
            problems.append(f"{word} read {found} vs plain search {plain_n}")
    if problems:
        raise ReadError(scan.path, 1, "cross-count differs: " + "; ".join(problems))


def cpp_blocks(scan, helpers):
    """Blocks of one C++ file: its [file] block then its cases in source order; also the per-macro counts."""
    lx = scan.lx
    found = collections.Counter()
    n_cases = n_subcases = 0
    funcs = sorted(scan.funcs, key=lambda f: f.lo)

    def owner(pos):
        for f in funcs:
            if f.lo <= pos <= f.hi:
                return f
        return None

    file_items = [(pos, 1, "value", text) for pos, text in scan.values]
    case_items = {}
    for f in funcs:
        items, macros, subs = walk_body(scan, f.lo, f.hi, helpers)
        found.update(macros)
        n_subcases += subs
        if f.is_case:
            n_cases += 1
            case_items[id(f)] = [(pos, 1, kind, text) for pos, kind, text in items]
        elif f.name in helpers:
            file_items += [(pos, 1, kind, f"{f.name}() :: {text}") for pos, kind, text in items]
    for pos, ln, text in lx.conds:
        f = owner(pos)
        if f is None or not (f.is_case or f.name in helpers):
            file_items.append((pos, 0, "conditional", text))
        elif f.is_case:
            case_items[id(f)].append((pos, 0, "conditional", text))
        else:
            file_items.append((pos, 0, "conditional", f"{f.name}() :: {text}"))
    cross_check(scan, found, n_cases, n_subcases)
    blocks = [Block(scan.path, None, (), [(k, t) for _, _, k, t in sorted(file_items, key=lambda x: (x[0], x[1]))])]
    for f in funcs:
        if f.is_case:
            lines = [(k, t) for _, _, k, t in sorted(case_items[id(f)], key=lambda x: (x[0], x[1]))]
            blocks.append(Block(scan.path, f.case_name, f.markers, lines))
    return blocks


# ---------------------------------------------------------------------------------------------------------------
# Python tests
# ---------------------------------------------------------------------------------------------------------------

def _has_number(node):
    return any(isinstance(n, ast.Constant) and isinstance(n.value, (int, float, complex)) and not isinstance(n.value, bool)
               for n in ast.walk(node))


def _is_number_literal(node):
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        node = node.operand
    return isinstance(node, ast.Constant) and isinstance(node.value, (int, float, complex)) and not isinstance(node.value, bool)


def _assign_value(stmt):
    if isinstance(stmt, ast.Assign):
        return stmt.targets, stmt.value
    if isinstance(stmt, ast.AnnAssign) and stmt.value is not None:
        return [stmt.target], stmt.value
    return None, None


def _marker_of(dec):
    text = ast.unparse(dec)
    base = text.split("(")[0].split(".")[-1]
    if base.startswith("skip") or base == "expectedFailure":
        return base
    return None


def _self_call(node):
    f = node.func
    return f.attr if isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name) and f.value.id == "self" else None


def _is_assert_call(node):
    if not isinstance(node, ast.Call):
        return None
    a = _self_call(node)
    return a if a and (a.startswith("assert") or a == "fail") else None


def _ordered(node):
    out = []
    for n in ast.walk(node):
        if isinstance(n, (ast.Call, ast.Assert)):
            out.append(n)
    out.sort(key=lambda n: (n.lineno, n.col_offset))
    return out


def python_blocks(path, src):
    try:
        tree = ast.parse(src, filename=path)
    except SyntaxError as exc:
        raise ReadError(path, exc.lineno or 0, f"cannot parse: {exc.msg}")
    file_lines = []
    cases = []

    def values_of(body, into):
        for stmt in body:
            targets, value = _assign_value(stmt)
            if value is None:
                continue
            if _has_number(value):
                names = ", ".join(ast.unparse(t) for t in targets)
                into.append(("value", collapse(f"{names} = {ast.unparse(value)}")))

    values_of(tree.body, file_lines)
    for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
        if not any(ast.unparse(b).endswith("TestCase") for b in cls.bases):
            continue
        cls_values = []
        values_of(cls.body, cls_values)
        file_lines += [(k, f"{cls.name} :: {t}") for k, t in cls_values]
        methods = [n for n in cls.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]
        asserting = set()
        for m in methods:
            if not m.name.startswith("test") and any(_is_assert_call(n) for n in _ordered(m) if isinstance(n, ast.Call)):
                asserting.add(m.name)
        changed = True
        while changed:
            changed = False
            for m in methods:
                if m.name.startswith("test") or m.name in asserting:
                    continue
                if any(isinstance(n, ast.Call) and _self_call(n) in asserting for n in _ordered(m)):
                    asserting.add(m.name)
                    changed = True
        for m in methods:
            if not m.name.startswith("test"):
                continue
            markers = [x for x in (_marker_of(d) for d in m.decorator_list) if x]
            cases.append((m.lineno, Block(path, f"{cls.name}.{m.name}", markers, _python_lines_sorted(m, asserting))))
    blocks = [Block(path, None, (), file_lines)]
    blocks += [b for _, b in sorted(cases, key=lambda x: x[0])]
    return blocks


def _python_lines_sorted(method, asserting):
    items = []
    for n in ast.walk(method):
        if isinstance(n, ast.Assert):
            items.append((n.lineno, n.col_offset, "assert", collapse(ast.unparse(n))))
        elif isinstance(n, ast.Call):
            if _is_assert_call(n):
                items.append((n.lineno, n.col_offset, "assert", collapse(ast.unparse(n))))
            elif _self_call(n) in asserting:
                items.append((n.lineno, n.col_offset, "call", collapse(ast.unparse(n))))
        else:
            targets, value = _assign_value(n)
            if value is not None and _is_number_literal(value) and all(isinstance(t, ast.Name) for t in targets):
                names = ", ".join(ast.unparse(t) for t in targets)
                items.append((n.lineno, n.col_offset, "value", collapse(f"{names} = {ast.unparse(value)}")))
    items.sort(key=lambda x: (x[0], x[1], x[2]))
    return [(k, t) for _, _, k, t in items]


# ---------------------------------------------------------------------------------------------------------------
# Building, writing, reading the register
# ---------------------------------------------------------------------------------------------------------------

def test_files(root):
    tests = Path(root) / "tests"
    if not tests.is_dir():
        raise ReadError(str(tests), 0, "no tests folder")
    cpp = sorted(p for p in tests.iterdir() if p.is_file() and p.suffix in (".cpp", ".h"))
    py = sorted(p for p in tests.iterdir() if p.is_file() and p.name.startswith("test_") and p.suffix == ".py")
    return cpp, py


def build_register(root):
    root = Path(root)
    cpp, py = test_files(root)
    scans = []
    for p in cpp:
        rel = p.relative_to(root).as_posix()
        scans.append(scan_cpp(rel, read_text(p, rel)))
    helpers = asserting_helpers(scans)
    per_file = {}
    for s in scans:
        per_file[s.path] = cpp_blocks(s, helpers[s.path])
    for p in py:
        rel = p.relative_to(root).as_posix()
        per_file[rel] = python_blocks(rel, read_text(p, rel))
    blocks = []
    for path in sorted(per_file):
        blocks += per_file[path]
    return Register(blocks)


def macro_of(text):
    t = re.sub(r"^\[[^\]]*\] ", "", text)
    t = re.sub(r"^[A-Za-z_]\w*\(\) :: ", "", t)
    m = re.match(r"[A-Za-z_]\w*", t)
    return m.group() if m else "?"


def totals(reg):
    cases = [b for b in reg.blocks if b.name is not None and b.file.endswith((".cpp", ".h"))]
    pycases = [b for b in reg.blocks if b.name is not None and b.file.endswith(".py")]
    marker_counts = collections.Counter(m.split("(")[0] for b in cases for m in set(b.markers))
    py_markers = collections.Counter(m for b in pycases for m in set(b.markers))
    macros = collections.Counter()
    n_assert = n_helper = n_calls = n_values = 0
    for b in reg.blocks:
        if not b.file.endswith((".cpp", ".h")):
            continue
        for k, t in b.lines:
            if k == "assert":
                macros[macro_of(t)] += 1
                n_assert += 1
                if b.name is None:
                    n_helper += 1
            elif k == "call":
                n_calls += 1
            elif k == "value":
                n_values += 1
    py_assert = sum(1 for b in pycases for k, _ in b.lines if k == "assert")
    return dict(cases=len(cases), markers=marker_counts, assertions=n_assert, helper_assertions=n_helper, macros=macros,
                calls=n_calls, values=n_values, py_tests=len(pycases), py_markers=py_markers, py_assertions=py_assert)


def totals_line(tot):
    mk = ", ".join(f"{n} {m}" for m, n in sorted(tot["markers"].items(), key=lambda x: (x[0] not in ("skip", "should_fail"), x[0])))
    py = f"{tot['py_tests']} python tests"
    if tot["py_markers"]:
        py += " (" + ", ".join(f"{n} {m}" for m, n in sorted(tot["py_markers"].items())) + ")"
    return f"# totals: {tot['cases']} cases" + (f" ({mk})" if mk else "") + f", {tot['assertions']} assertions, {py}"


def render_register(reg):
    tot = totals(reg)
    out = ["# check register of the tests. Written by checks/check_register.py --record; do not edit."]
    out += NOT_SEEN
    out.append(totals_line(tot))
    out.append("# assertions by macro: " + ", ".join(f"{m} {n}" for m, n in sorted(tot["macros"].items()))
               + f"; {tot['helper_assertions']} of them in asserting helpers; {tot['calls']} calls of helpers, {tot['values']} values;"
               + f" {tot['py_assertions']} python assertions")
    for b in reg.blocks:
        out.append("")
        if b.name is None:
            out.append(f"[file] {b.file}")
        else:
            out.append(f"[case] {b.file} :: {b.name}")
            out.append("  markers: " + (", ".join(b.markers) if b.markers else "-"))
        for k, t in b.lines:
            out.append(f"  {k}: {t}")
    return "\n".join(out) + "\n"


def parse_register(text, path=REGISTER_PATH):
    blocks = []
    cur = None
    for n, raw in enumerate(text.split("\n"), 1):
        if not raw.strip() or raw.startswith("#"):
            continue
        if raw.startswith("[file] "):
            cur = Block(raw[7:])
            blocks.append(cur)
        elif raw.startswith("[case] "):
            body = raw[7:]
            if " :: " not in body:
                raise ReadError(path, n, "a [case] line without ' :: '")
            f, name = body.split(" :: ", 1)
            cur = Block(f, name)
            blocks.append(cur)
        elif raw.startswith("  ") and cur is not None:
            kind, sep, rest = raw[2:].partition(": ")
            if not sep:
                kind, rest = raw[2:].rstrip(":"), ""
            if kind == "markers":
                cur.markers = [] if rest == "-" else [m.strip() for m in rest.split(", ")]
            else:
                cur.lines.append((kind, rest))
        else:
            raise ReadError(path, n, "cannot read this line of the register")
    return Register(blocks)


# ---------------------------------------------------------------------------------------------------------------
# Comparison (D4)
# ---------------------------------------------------------------------------------------------------------------

ADDITION, CHANGE = "addition", "change"
SIG_RE = re.compile(
    r"0[xX][0-9a-fA-F']+|(?<![A-Za-z_0-9.])(?:\d[\d']*\.?[\d']*|\.\d+)(?:[eE][+-]?\d+)?[fFlLuU]*"
    r"|<=|>=|==|!=|&&|\|\||[<>!+\-*/%]|\b(?:Approx|epsilon|margin|scale|CHECK\w*|REQUIRE\w*|WARN\w*)\b")


def signature(kind, text):
    stripped = re.sub(r'"(?:\\.|[^"\\])*"', '""', text)
    sig = tuple(SIG_RE.findall(stripped))
    if kind == "call":
        m = re.match(r"(?:\[[^\]]*\] )?(?:\w+\(\) :: )?(\w+)", text)
        sig = (m.group(1) if m else "",) + sig
    return sig


def lcs_align(a, b):
    n, m = len(a), len(b)
    dp = [[0] * (m + 1) for _ in range(n + 1)]
    for i in range(n - 1, -1, -1):
        ai, row, nxt = a[i], dp[i], dp[i + 1]
        for j in range(m - 1, -1, -1):
            row[j] = nxt[j + 1] + 1 if ai == b[j] else max(nxt[j], row[j + 1])
    i = j = 0
    gaps = []
    go, gn = [], []
    while i < n or j < m:
        if i < n and j < m and a[i] == b[j]:
            if go or gn:
                gaps.append((go, gn))
                go, gn = [], []
            i += 1
            j += 1
        elif j >= m or (i < n and dp[i + 1][j] >= dp[i][j + 1]):
            go.append(a[i])
            i += 1
        else:
            gn.append(b[j])
            j += 1
    if go or gn:
        gaps.append((go, gn))
    return gaps


class Diff:
    def __init__(self, cls, kind, where, old=None, new=None, note=""):
        self.cls, self.kind, self.where, self.old, self.new, self.note = cls, kind, where, old, new, note


def where_of(b):
    return b.file if b.name is None else f"{b.file} :: {b.name}"


def diff_lines(where, old, new, diffs):
    for go, gn in lcs_align(old, new):
        kinds = []
        for k, _ in go + gn:
            if k not in kinds:
                kinds.append(k)
        for kind in kinds:
            o = [x for x in go if x[0] == kind]
            n = [x for x in gn if x[0] == kind]
            noun = {"assert": "assertion"}.get(kind, kind)
            paired = 0 if kind == "conditional" else min(len(o), len(n))
            for (k, ot), (_, nt) in zip(o[:paired], n[:paired]):
                if kind in ("assert", "call") and signature(kind, ot) == signature(kind, nt):
                    diffs.append(Diff(ADDITION, f"{noun} reworded", where, ot, nt))
                else:
                    diffs.append(Diff(CHANGE, f"{noun} changed", where, ot, nt))
            for k, ot in o[paired:]:
                diffs.append(Diff(CHANGE, f"{noun} removed", where, ot, None))
            for k, nt in n[paired:]:
                diffs.append(Diff(CONDITIONAL_CLASS.get(kind, ADDITION), f"{noun} added", where, None, nt))


CONDITIONAL_CLASS = {"conditional": CHANGE}


def count_kinds(b):
    c = collections.Counter(k for k, _ in b.lines)
    return f"{c['assert']} assertions, {c['call']} calls, {c['value']} values"


def compare(old, new):
    diffs = []
    old_files = {b.file: b for b in old.blocks if b.name is None}
    new_files = {b.file: b for b in new.blocks if b.name is None}
    for path in sorted(set(old_files) | set(new_files)):
        if path in old_files and path in new_files:
            diff_lines(path, old_files[path].lines, new_files[path].lines, diffs)
        elif path in new_files:
            d = Diff(ADDITION, "file added", path, None, None, count_kinds(new_files[path]))
            d.counts = collections.Counter(k for k, _ in new_files[path].lines)
            diffs.append(d)
        else:
            diffs.append(Diff(CHANGE, "file removed", path, None, None, count_kinds(old_files[path])))
    old_cases = collections.defaultdict(list)
    new_cases = collections.defaultdict(list)
    order = []
    for b in old.blocks:
        if b.name is not None:
            old_cases[b.name].append(b)
    for b in new.blocks:
        if b.name is not None:
            new_cases[b.name].append(b)
            order.append(b.name)
    names = list(dict.fromkeys(order)) + [n for n in dict.fromkeys(b.name for b in old.blocks if b.name is not None) if n not in new_cases]
    for name in names:
        olds, news = list(old_cases.get(name, [])), list(new_cases.get(name, []))
        matched = []
        for nb in list(news):
            for ob in olds:
                if ob.file == nb.file:
                    matched.append((ob, nb))
                    olds.remove(ob)
                    news.remove(nb)
                    break
        if len(olds) == 1 and len(news) == 1 and olds[0].lines == news[0].lines and sorted(olds[0].markers) == sorted(news[0].markers):
            d = Diff(ADDITION, "case moved", where_of(news[0]), None, None, f"from {olds[0].file}")
            diffs.append(d)
            olds, news = [], []
        for ob in olds:
            diffs.append(Diff(CHANGE, "case removed", where_of(ob), None, None, count_kinds(ob)))
        for nb in news:
            d = Diff(ADDITION, "case added", where_of(nb), None, None,
                     ("markers: " + ", ".join(nb.markers) + "; " if nb.markers else "") + count_kinds(nb))
            d.counts = collections.Counter(k for k, _ in nb.lines)
            diffs.append(d)
        for ob, nb in matched:
            where = where_of(nb)
            for m in nb.markers:
                if m not in ob.markers:
                    diffs.append(Diff(CHANGE, "marker added", where, None, m))
            for m in ob.markers:
                if m not in nb.markers:
                    diffs.append(Diff(CHANGE, "marker removed", where, m, None))
            diff_lines(where, ob.lines, nb.lines, diffs)
    return diffs


PLURAL = {"case": "cases", "assertion": "assertions", "call": "calls", "value": "values", "marker": "markers",
          "conditional": "conditionals", "file": "files"}


def kind_phrase(kind, n):
    noun, rest = kind.split(" ", 1)
    noun = PLURAL[noun] if n != 1 else noun
    return f"{n} {noun} {rest}"


def summarise(diffs):
    if not diffs:
        return "IDENTICAL"
    changes = [d for d in diffs if d.cls == CHANGE]
    adds = [d for d in diffs if d.cls == ADDITION]
    parts = []
    if changes:
        label = collections.Counter()
        for d in changes:
            noun, rest = d.kind.split(" ", 1)
            label[(noun, rest)] += 1
        parts.append(", ".join(kind_phrase(f"{n} {r}", c) for (n, r), c in label.items()))
    if adds:
        added = collections.Counter()
        for d in adds:
            noun, rest = d.kind.split(" ", 1)
            counts = getattr(d, "counts", None)
            if d.kind == "case added":
                added["cases"] += 1
                added["assertions"] += counts["assert"]
                added["calls"] += counts["call"]
                added["values"] += counts["value"]
            elif d.kind == "file added":
                added["files"] += 1
                added["values"] += counts["value"]
                added["assertions"] += counts["assert"]
                added["calls"] += counts["call"]
            elif d.kind == "case moved":
                added["moved"] += 1
            elif rest == "reworded":
                added["reworded"] += 1
            else:
                added[PLURAL[noun]] += 1
        singular = {v: k for k, v in PLURAL.items()}

        def phrase(k, v):
            if k in ("moved", "reworded"):
                return f"{v} {k}"
            return f"+{v} {singular[k] if v == 1 else k}"

        text = ", ".join(phrase(k, added[k]) for k in ("cases", "files", "assertions", "calls", "values", "reworded", "moved") if added[k])
        parts.append("to record: " + text)
    return "; ".join(parts)


def format_diffs(diffs):
    out = []
    for cls in (CHANGE, ADDITION):
        for d in diffs:
            if d.cls != cls:
                continue
            head = f"{'CHANGE' if cls == CHANGE else 'ADD'} {d.kind}: {d.where}"
            if d.note:
                head += f"  ({d.note})"
            out.append(head)
            if d.old is not None:
                out.append(f"  - {d.old}")
            if d.new is not None:
                out.append(f"  + {d.new}")
    return out


# ---------------------------------------------------------------------------------------------------------------
# Instruments (D7)
# ---------------------------------------------------------------------------------------------------------------

ITEMS = ("known_answer", "smallest_effect", "planted_fault", "valid_for", "guards")
SKIP_MARKERS = {"skip", "skipIf", "skipUnless"}
FAIL_MARKERS = {"should_fail", "expectedFailure"}


def case_index(reg):
    idx = collections.defaultdict(list)
    for b in reg.blocks:
        if b.name is not None:
            idx[b.name].append(b)
    return idx


def check_instruments(root, reg):
    """Returns (faults, rows, instrument names). rows: (instrument, [status per item])."""
    path = Path(root) / INSTRUMENTS_PATH
    try:
        with open(path, "rb") as fh:
            data = tomllib.load(fh)
    except (OSError, tomllib.TOMLDecodeError) as exc:
        raise ReadError(INSTRUMENTS_PATH, 0, f"cannot read: {exc}")
    idx = case_index(reg)
    faults, rows = [], []
    for inst, table in data.items():
        if not isinstance(table, dict):
            continue
        statuses = []
        for item in ITEMS:
            where = f"{inst}.{item}"
            if item not in table:
                faults.append(f"instrument {inst}: item {item} is missing (give cases, not_established or not_applicable)")
                statuses.append("MISSING")
                continue
            v = table[item]
            if not isinstance(v, dict):
                faults.append(f"instrument {inst}: item {item} is not a table")
                statuses.append("MISSING")
                continue
            if "not_applicable" in v:
                statuses.append("n/a")
                continue
            if "not_established" in v:
                statuses.append("NOT ESTABLISHED")
                continue
            cases = v.get("cases", [])
            if not isinstance(cases, list) or not all(isinstance(c, str) for c in cases):
                faults.append(f"instrument {inst}: item {item}: cases must be a list of case names")
                statuses.append("MISSING")
                continue
            measured = v.get("measured")
            bad = False
            skipped = 0
            for c in cases:
                blocks = idx.get(c)
                if not blocks:
                    faults.append(f"instrument {inst}: item {item}: case not in the register: {c!r}")
                    bad = True
                    continue
                markers = {m.split("(")[0] for b in blocks for m in b.markers if m != "skip(false)"}
                if markers & FAIL_MARKERS:
                    faults.append(f"instrument {inst}: item {item}: case is marked {sorted(markers & FAIL_MARKERS)[0]} and cannot be proof: {c!r}")
                    bad = True
                elif markers & SKIP_MARKERS:
                    skipped += 1
                    if not measured:
                        faults.append(f"instrument {inst}: item {item}: case is skipped in the regular run and the item has no 'measured' (date, file): {c!r}")
                        bad = True
            if bad:
                statuses.append("FAULT")
            elif cases and skipped == len(cases):
                statuses.append(f"established, skipped set of {str(measured).split(',')[0].strip()}")
            elif cases:
                statuses.append(f"established ({len(cases)} case{'s' if len(cases) != 1 else ''})")
            elif v.get("says"):
                statuses.append("stated (no case)")
            else:
                faults.append(f"instrument {inst}: item {item} has no cases, no says, no not_established and no not_applicable")
                statuses.append("MISSING")
        rows.append((inst, statuses, [c for item in ITEMS if isinstance(table.get(item), dict) for c in table[item].get("cases", []) if isinstance(c, str)]))
    return faults, rows


def format_table(rows):
    header = ["instrument"] + [i.replace("_", " ") for i in ITEMS]
    body = [[r[0]] + r[1] for r in rows]
    widths = [max(len(x[c]) for x in [header] + body) for c in range(len(header))]
    lines = [" | ".join(x.ljust(widths[c]) for c, x in enumerate(row)).rstrip() for row in [header] + body]
    lines.insert(1, "-+-".join("-" * w for w in widths))
    return lines


# ---------------------------------------------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------------------------------------------

def git(root, *args):
    return subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True)


def refuse_record(root, allow_worktree):
    st = git(root, "status", "--porcelain", "--", "tests")
    if st.returncode != 0:
        return "not a git repository (cannot tell whether tests/ has uncommitted changes)"
    if st.stdout.strip():
        return "tests/ has uncommitted changes:\n" + st.stdout.rstrip()
    if not allow_worktree:
        a = git(root, "rev-parse", "--absolute-git-dir")
        c = git(root, "rev-parse", "--git-common-dir")
        if a.returncode != 0 or c.returncode != 0:
            return "cannot read the git directories"
        gd = os.path.normcase(os.path.realpath(a.stdout.strip()))
        cd = os.path.normcase(os.path.realpath(os.path.join(str(root), c.stdout.strip())))
        if gd != cd:
            return "not the main working tree (a linked worktree); only the orchestrator records, or pass --allow-worktree"
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description="the check register of the tests and of the instruments")
    ap.add_argument("--root", default=str(Path(__file__).resolve().parent.parent))
    ap.add_argument("--record", action="store_true")
    ap.add_argument("--allow-worktree", action="store_true")
    ap.add_argument("--print", dest="print_", action="store_true")
    ap.add_argument("--instruments", action="store_true")
    args = ap.parse_args(argv)
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass
    root = Path(args.root)
    try:
        new = build_register(root)
        if args.print_:
            sys.stdout.write(render_register(new))
            return EXIT_SAME
        if args.record:
            why = refuse_record(root, args.allow_worktree)
            if why:
                print("check register: refused: " + why)
                return EXIT_REFUSED
            target = root / REGISTER_PATH
            target.parent.mkdir(parents=True, exist_ok=True)
            with open(target, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(render_register(new))
            print(f"check register: recorded {REGISTER_PATH}")
            print(totals_line(totals(new))[2:])
            return EXIT_SAME
        diffs = []
        code = EXIT_SAME
        reg_file = root / REGISTER_PATH
        recorded = reg_file.is_file()
        if not args.instruments:
            if recorded:
                old = parse_register(read_text(reg_file, REGISTER_PATH), REGISTER_PATH)
                diffs = compare(old, new)
                for line in format_diffs(diffs):
                    print(line)
                if any(d.cls == CHANGE for d in diffs):
                    code = EXIT_CHANGE
                elif diffs:
                    code = EXIT_ADDITIONS
            else:
                print(f"no {REGISTER_PATH} recorded yet (--record writes it)")
                code = EXIT_ADDITIONS
        faults, inst_note = [], ""
        if (root / INSTRUMENTS_PATH).is_file():
            faults, rows = check_instruments(root, new)
            if args.instruments:
                for line in format_table(rows):
                    print(line)
            for f in faults:
                print("INSTRUMENTS FAULT: " + f)
            if recorded and diffs:
                touched = {d.where.split(" :: ", 1)[1] for d in diffs if " :: " in d.where}
                for inst, _, names in rows:
                    hit = sorted(set(names) & touched)
                    if hit:
                        print(f"instruments: review section {inst}: its case differs from the register ({hit[0]!r})")
            gaps = sum(s in ("NOT ESTABLISHED", "stated (no case)") for _, st, _ in rows for s in st)
            inst_note = (f"instruments: {len(faults)} fault{'s' if len(faults) != 1 else ''}" if faults else "instruments: ok")                 + f" ({len(rows)} instruments, {gaps} items not established or only stated)"
            if faults:
                code = EXIT_CHANGE
        elif args.instruments:
            print(f"cannot read {INSTRUMENTS_PATH}: the file is absent", file=sys.stderr)
            return EXIT_UNREADABLE
        else:
            print(f"no {INSTRUMENTS_PATH}: instruments not checked")
        if inst_note:
            print(inst_note)
        if args.instruments:
            return code
        summary = summarise(diffs) if recorded else "to record: everything (no register recorded)"
        if faults:
            summary += f"; instruments: {len(faults)} fault{'s' if len(faults) != 1 else ''}"
        print(summary)
        return code
    except ReadError as exc:
        print(str(exc), file=sys.stderr)
        return EXIT_UNREADABLE


if __name__ == "__main__":
    sys.exit(main())
