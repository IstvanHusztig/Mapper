#!/usr/bin/env python3
"""Build the documentation's search index and contents page (pure standard library).

Reads every HTML and Markdown page of docs/ (not hu/; agents/ read-only), plus CLAUDE.md and
tools/README.md, splits them into sections by heading, and writes
  docs/search-index.js   window.DOCS_INDEX = [...]   (loaded by the search box of docs/index.html, works from file://)
  docs/contents.html     the indexed table of contents, grouped as in docs/README.md

    python tools/build_docs_index.py              rebuild both files
    python tools/build_docs_index.py --fix-ids    first add ids to h2/h3 headings that have none
    python tools/build_docs_index.py --check      exit 1 if the files are stale or an index/contents link is broken

Output is deterministic (pages sorted by path, sections in document order).
"""
import html
import json
import re
import sys
from html.parser import HTMLParser
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
EXTRA = [("../CLAUDE.md", ROOT / "CLAUDE.md"), ("../tools/README.md", ROOT / "tools" / "README.md")]
EXTRA_GROUP = "Outside docs/"
EXTRA_DESC = {
    "../CLAUDE.md": "How the code is built, tested and laid out; the project rules.",
    "../tools/README.md": "The analysis and verification tools and how to run them.",
}
UNLISTED_GROUP = "Not in the map"


def read(path):
    data = path.read_bytes()
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return data.decode("cp1252", errors="replace")


def slugify(text):
    s = re.sub(r"[^\w\s-]", "", text.lower(), flags=re.UNICODE)
    return re.sub(r"\s", "-", s.strip())


def norm(text):
    return re.sub(r"\s+", " ", text).strip()


# ---------- HTML ----------
class Sections(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.secs = []          # dicts: level, title, anchor, text
        self.cur = {"level": 0, "title": "", "anchor": "", "text": []}
        self.in_head = None     # (level, id, parts)
        self.skip = 0
        self.title = ""
        self.in_title = False
        self.nav_skip = 0

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if tag in ("script", "style", "svg"):
            self.skip += 1
        elif tag == "title":
            self.in_title = True
        elif tag == "nav":
            self.nav_skip += 1
        elif tag in ("h1", "h2", "h3") and not self.skip:
            self.in_head = (int(tag[1]), a.get("id", ""), [])
        if a.get("id") and not self.skip:
            self.ids.add(a["id"])

    def handle_endtag(self, tag):
        if tag in ("script", "style", "svg"):
            self.skip = max(0, self.skip - 1)
        elif tag == "title":
            self.in_title = False
        elif tag == "nav":
            self.nav_skip = max(0, self.nav_skip - 1)
        elif tag in ("h1", "h2", "h3") and self.in_head:
            level, ident, parts = self.in_head
            self.in_head = None
            self.secs.append(self.cur)
            self.cur = {"level": level, "title": norm("".join(parts)), "anchor": ident, "text": []}

    def handle_data(self, data):
        if self.in_title:
            self.title += data
        if self.skip or self.nav_skip:
            return
        if self.in_head:
            self.in_head[2].append(data)
        else:
            self.cur["text"].append(data)

    ids = None


def parse_html(path):
    p = Sections()
    p.ids = set()
    p.feed(read(path))
    p.secs.append(p.cur)
    secs = []
    for s in p.secs:
        text = norm(" ".join(s["text"]))
        if s["level"] == 0 and not text:
            continue
        secs.append({"level": s["level"], "title": s["title"], "anchor": s["anchor"], "text": text})
    title = next((s["title"] for s in secs if s["level"] == 1), "") or norm(p.title)
    return title, secs, p.ids


def fix_ids(path):
    src = read(path)
    used = set(re.findall(r'\bid="([^"]*)"', src))
    n = 0

    def repl(m):
        nonlocal n
        level, attrs, inner = m.group(1), m.group(2), m.group(3)
        if re.search(r'\bid\s*=', attrs):
            return m.group(0)
        base = slugify(norm(html.unescape(re.sub(r"<[^>]+>", "", inner)))) or "section"
        slug, k = base, 2
        while slug in used:
            slug = f"{base}-{k}"
            k += 1
        used.add(slug)
        n += 1
        return f'<h{level} id="{slug}"{attrs}>{inner}</h{level}>'

    out = re.sub(r"<h([23])((?:\s[^>]*)?)>(.*?)</h\1>", repl, src, flags=re.S)
    if n:
        path.write_text(out, encoding="utf-8", newline="")
    return n


# ---------- Markdown ----------
def md_plain(t):
    t = re.sub(r"!\[([^\]]*)\]\([^)]*\)", r"\1", t)
    t = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", t)
    t = re.sub(r"`([^`]*)`", r"\1", t)
    t = re.sub(r"(\*\*|__|\*|_)(.+?)\1", r"\2", t)
    t = re.sub(r"<[^>]+>", " ", t)
    return t


def parse_md(path):
    secs = [{"level": 0, "title": "", "anchor": "", "text": []}]
    fence = False
    used = {}
    for line in read(path).splitlines():
        if line.lstrip().startswith("```"):
            fence = not fence
            continue
        m = None if fence else re.match(r"(#{1,4})\s+(.*?)\s*#*\s*$", line)
        if m:
            title = norm(md_plain(m.group(2)))
            slug = slugify(title)
            k = used.get(slug, 0)
            used[slug] = k + 1
            secs.append({"level": len(m.group(1)), "title": title,
                         "anchor": slug if k == 0 else f"{slug}-{k}", "text": []})
        else:
            secs[-1]["text"].append(md_plain(line))
    out = []
    for s in secs:
        text = norm(" ".join(s["text"]))
        if s["level"] == 0 and not text:
            continue
        out.append({"level": s["level"], "title": s["title"], "anchor": s["anchor"], "text": text})
    title = next((s["title"] for s in out if s["level"] == 1), "") or path.stem
    return title, out, {s["anchor"] for s in out if s["anchor"]}


# ---------- pages ----------
def all_pages():
    pages = []
    for p in sorted(DOCS.rglob("*")):
        rel = p.relative_to(DOCS).as_posix()
        if rel.startswith("hu/") or p.suffix not in (".html", ".md"):
            continue
        if rel in ("contents.html", "search.html"):
            continue
        pages.append((rel, p))
    pages += EXTRA
    return pages


def read_map():
    """README.md: path -> (group, title, description), groups in order."""
    groups, cur = [], None
    for line in read(DOCS / "README.md").splitlines():
        m = re.match(r"## (.+)", line)
        if m:
            cur = {"name": m.group(1).strip(), "intro": "", "items": []}
            groups.append(cur)
            continue
        if cur is None:
            continue
        m = re.match(r"- \[([^\]]+)\]\(([^)]+)\)(?::\s*(.*))?$", line)
        if m:
            cur["items"].append((m.group(2), m.group(1), (m.group(3) or "").strip()))
        elif line.strip() and not cur["items"] and not line.startswith("A new page"):
            cur["intro"] = line.strip()
    return groups


def md_to_html(t):
    t = html.escape(t, quote=False)
    t = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", lambda m: f'<a href="{m.group(2)}">{m.group(1)}</a>', t)
    t = re.sub(r"`([^`]+)`", r"<code>\1</code>", t)
    t = re.sub(r"\*([^*]+)\*", r"<i>\1</i>", t)
    return t


def build():
    pages = all_pages()
    parsed, index, errors = {}, [], []
    for rel, p in pages:
        title, secs, ids = (parse_html if p.suffix == ".html" else parse_md)(p)
        parsed[rel] = (title, secs, ids)
    groups = read_map()
    in_map = {}
    for g in groups:
        for path, title, desc in g["items"]:
            in_map[path] = (g["name"], title, desc)
    unlisted = [rel for rel, _ in pages if rel not in in_map and rel not in EXTRA_DESC
                and not rel.startswith("agents/")]
    unlisted_agents = [rel for rel, _ in pages if rel.startswith("agents/") and rel not in in_map]
    for path in in_map:
        if path not in parsed and not (DOCS / path).exists():
            errors.append(f"README.md lists a missing page: {path}")

    def group_of(rel):
        if rel in in_map:
            return in_map[rel][0]
        if rel in EXTRA_DESC:
            return EXTRA_GROUP
        return UNLISTED_GROUP

    for rel, _ in pages:
        title, secs, _ids = parsed[rel]
        label = "Agent notes: " if rel.startswith("agents/") else ""
        for s in secs:
            index.append({"p": rel, "pt": label + title, "g": group_of(rel), "s": s["title"],
                          "a": s["anchor"], "l": s["level"], "x": s["text"]})
    index_js = ("// Generated by tools/build_docs_index.py - do not edit.\nwindow.DOCS_INDEX = [\n"
                + ",\n".join(json.dumps(e, ensure_ascii=True, separators=(",", ":")) for e in index)
                + "\n];\n")
    contents = build_contents(groups, parsed, in_map, unlisted, pages)
    # anchor checks of the index
    for e in index:
        if e["a"] and e["a"] not in parsed[e["p"]][2]:
            errors.append(f"index anchor missing: {e['p']}#{e['a']}")
    return index_js, contents, index, errors, unlisted, unlisted_agents, parsed, pages


def sub_entries(rel, secs):
    h2 = [s for s in secs if s["level"] == 2 and s["anchor"]]
    if rel.endswith(".md") and len(h2) < 3:
        return []
    return h2


def build_contents(groups, parsed, in_map, unlisted, pages):
    out = []
    names = [g["name"] for g in groups]
    layout = [(g["name"], g["intro"], [(p, t, d) for p, t, d in g["items"]]) for g in groups]
    layout.append((EXTRA_GROUP, "Outside docs/: the project rules and the analysis tools.",
                   [(p, parsed[p][0], EXTRA_DESC[p]) for p in EXTRA_DESC]))
    if unlisted:
        layout.append((UNLISTED_GROUP, "Pages that docs/README.md does not list yet.",
                       [(p, parsed[p][0], "") for p in unlisted]))
    toc = []
    for name, intro, items in layout:
        gid = "g-" + slugify(name)
        toc.append(f'<li><a href="#{gid}">{html.escape(name)}</a></li>')
        out.append(f'<h2 id="{gid}">{html.escape(name)}</h2>')
        if intro:
            out.append(f'<p class="meta">{md_to_html(intro)}</p>')
        out.append('<ul class="toc">')
        for path, title, desc in items:
            if path not in parsed and not (DOCS / path).exists():
                continue
            out.append(f'<li><a href="{path}"><b>{html.escape(title)}</b></a> <code>{html.escape(path)}</code>'
                       + (f'<div class="desc">{md_to_html(desc)}</div>' if desc else ""))
            subs = sub_entries(path, parsed[path][1]) if path in parsed else []
            if subs:
                out.append('<ul class="sub">' + "".join(
                    f'<li><a href="{path}#{s["anchor"]}">{html.escape(s["title"])}</a></li>' for s in subs) + "</ul>")
            out.append("</li>")
        out.append("</ul>")
    n_pages = sum(len(it) for _, _, it in layout)
    head = HEAD.replace("@@COUNT@@", str(n_pages))
    return head + '<ul class="groups">' + "".join(toc) + "</ul>\n" + "\n".join(out) + TAIL


HEAD = """<!doctype html>
<!-- Generated by tools/build_docs_index.py from docs/README.md and the pages' headings; do not edit by hand. -->
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Contents of the documentation</title>
<style>
  :root { --bg: #fbfbfa; --panel: #ffffff; --text: #1d2126; --muted: #5b6470; --line: #e3e5e8; --accent: #1f6fb2; --accent-soft: #e8f1fa; --code-bg: #f2f3f5; }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) { --bg: #15181c; --panel: #1c2026; --text: #e4e7eb; --muted: #9aa3ae; --line: #2c323a; --accent: #6cb0ec; --accent-soft: #1d2d3d; --code-bg: #242a31; }
  }
  :root[data-theme="dark"] { --bg: #15181c; --panel: #1c2026; --text: #e4e7eb; --muted: #9aa3ae; --line: #2c323a; --accent: #6cb0ec; --accent-soft: #1d2d3d; --code-bg: #242a31; }
  * { box-sizing: border-box; }
  body { margin: 0; background: var(--bg); color: var(--text); font: 16px/1.6 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }
  main { max-width: 860px; margin: 0 auto; padding: 40px 16px 80px; }
  h1 { font-size: 30px; line-height: 1.25; margin: 0 0 8px; }
  h2 { font-size: 22px; margin: 40px 0 8px; padding-top: 8px; border-top: 1px solid var(--line); }
  .lead { color: var(--muted); font-size: 17px; margin-top: 0; }
  .meta { color: var(--muted); font-size: 14px; margin: 0 0 8px; }
  code { font-family: ui-monospace, "Cascadia Code", Consolas, monospace; font-size: 12.5px; background: var(--code-bg); padding: 1px 5px; border-radius: 4px; color: var(--muted); }
  a { color: var(--accent); }
  ul.groups { display: flex; flex-wrap: wrap; gap: 8px 18px; list-style: none; padding: 0; margin: 18px 0; }
  ul.toc { list-style: none; padding: 0; margin: 0; }
  ul.toc > li { padding: 8px 0; border-bottom: 1px solid var(--line); }
  .desc { color: var(--muted); font-size: 14.5px; }
  ul.sub { margin: 6px 0 0; padding-left: 20px; columns: 2; column-gap: 24px; font-size: 14px; }
  ul.sub li { break-inside: avoid; }
  @media (max-width: 600px) { ul.sub { columns: 1; } }
  .search { display: flex; gap: 8px; margin: 14px 0; }
  .search input { flex: 1; font: inherit; padding: 8px 10px; border: 1px solid var(--line); border-radius: 8px; background: var(--panel); color: var(--text); }
  .search button { font: inherit; padding: 8px 14px; border: 1px solid var(--line); border-radius: 8px; background: var(--accent-soft); color: var(--text); cursor: pointer; }
</style>
</head>
<body>
<main>
<p class="docs-nav" style="font-size:13px;margin:0 0 8px;color:var(--muted)"><a href="index.html">dMapper docs</a> â€º Using dMapper Â· <a href="index.html#documents">All documents</a> Â· <a href="index.html#search">Search</a></p>
<h1>Contents of the documentation</h1>
<p class="lead">All @@COUNT@@ pages of <code>docs/</code>, grouped by purpose, with the main sections of each. The Hungarian pages (<code>hu/</code>) are not included yet.</p>
<p class="meta">Terms used here: none; the basic terms are in the guide's <a href="index.html#fundamentals">Fundamentals</a>.</p>
<form class="search" action="index.html" method="get"><input type="search" name="q" placeholder="Search all pages" aria-label="Search all pages"><button type="submit">Search</button></form>
"""

TAIL = """
<h2 id="see-also">See also</h2>
<ul class="docs-nav"><li><a href="index.html#documents">All documents</a> (the guide's map)</li><li><a href="index.html#search">Search the documentation</a></li><li><a href="README.md">docs/README.md</a></li></ul>
</main>
</body>
</html>
"""


# ---------- link check ----------
def check_links(parsed, pages):
    """Warnings for broken local links in the pages (a missing file or anchor)."""
    warns = []
    for rel, p in pages:
        if p.suffix != ".html":
            continue
        src = re.sub(r"<script\b[^>]*>.*?</script>", "", read(p), flags=re.S | re.I)  # links built in scripts are not links
        for href in re.findall(r'href="([^"]+)"', src):
            if re.match(r"[a-z]+:", href) or href.startswith("//"):
                continue
            path, _, anchor = href.partition("#")
            target = (p.parent / path).resolve() if path else p
            if path.startswith("hu/") or "/hu/" in path:
                continue
            if not target.exists():
                warns.append(f"{rel}: missing file {href}")
                continue
            if anchor and target.suffix in (".html", ".md"):
                try:
                    trel = target.relative_to(DOCS).as_posix()
                except ValueError:
                    trel = "../" + target.relative_to(ROOT).as_posix()
                if trel in parsed and anchor not in parsed[trel][2]:
                    ids = parsed[trel][2]
                    # md anchors outside our heading slugs (or non-heading ids) may exist
                    if target.suffix == ".html" and re.search(rf'\bid="{re.escape(anchor)}"', read(target)):
                        continue
                    warns.append(f"{rel}: missing anchor {href}")
    return warns


def main():
    args = sys.argv[1:]
    if "--fix-ids" in args:
        for rel, p in all_pages():
            if p.suffix == ".html" and not rel.startswith("agents/"):
                n = fix_ids(p)
                if n:
                    print(f"{rel}: added {n} heading ids")
    index_js, contents, index, errors, unlisted, unlisted_agents, parsed, pages = build()
    warns = check_links(parsed, pages)
    n_pages = len({e["p"] for e in index})
    print(f"{n_pages} pages, {len(index)} sections indexed")
    if unlisted:
        print("pages not in docs/README.md:", ", ".join(unlisted))
    if unlisted_agents:
        print("agent notes not in docs/README.md:", ", ".join(unlisted_agents))
    for w in warns:
        print("link warning:", w)
    files = {DOCS / "search-index.js": index_js, DOCS / "contents.html": contents}
    if "--check" in args:
        bad = list(errors)
        for f, text in files.items():
            if not f.exists() or f.read_bytes().decode("utf-8") != text:
                bad.append(f"stale: {f.relative_to(ROOT).as_posix()} (run python tools/build_docs_index.py)")
        if unlisted:
            bad.append("unlisted pages: " + ", ".join(unlisted))
        for b in bad:
            print("CHECK FAILED:", b)
        sys.exit(1 if bad else 0)
    for e in errors:
        print("error:", e)
    for f, text in files.items():
        f.write_bytes(text.encode("utf-8"))
        print("wrote", f.relative_to(ROOT).as_posix())
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
