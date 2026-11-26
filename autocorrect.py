"""
autocorrect.py - check and fix the user's Python code before it runs.

Only the real robot API (dobot_quad.RobotClient) is allowed, and every fix is made in the
SOURCE CODE, so the corrected program also runs on the real Rover X1.

Fixed automatically:
  1. odd characters from copy/paste: “ ” ‘ ’ used as quotes, （ ） ： ，  (only outside strings)
  2. Thai input mistakes: a line typed with the Thai keyboard layout, Thai digits (๑๒๓),
     Thai text written in the code without '#'
  3. missing ':' after if / for / while / def / else ..., unclosed '(' and bad indents
  4. mistyped SDK names / camelCase / old names
        robot.walk_forwrd(1) -> robot.walk_forward(1)
        robot.balanceStand() -> robot.balance_stand()
        robot.walk_left(1)   -> robot.move_left(1)
  5. SDK command called without the robot object:  walk_forward(1) -> robot.walk_forward(1)
  6. mistyped keyword arguments   robot.walk_forward(1, sped_ratio=50) -> speed_ratio=50
  7. text values the real SDK rejects:  rotate("lefft") -> "left", rotate("ซ้าย") -> "left",
     set_all_legs_color("green") -> Color.GREEN
  8. missing lines the real robot needs: from dobot_quad import ..., robot = RobotClient(...),
     import math / import time
  9. mistyped imports / constants / Python functions   Colour.GREN -> Color.GREEN, rnage -> range
The removed simulator-only helpers (rover.* / dog.*) are reported as errors with the SDK replacement.
"""
import ast
import builtins
import difflib
import io
import keyword
import math
import re
import tokenize

from rover_sim import thai_to_english
from sdk_compat import (COLOR_PARAMS, CASE_SENSITIVE_VALUES, Color, SDK_ALIASES, SDK_ALL_PARAMS,
                        SDK_ATTRIBUTES, SDK_CONSTS, SDK_EXPORTS, SDK_FUNCTIONS, TEXT_VALUES, resolve_sdk_name)

DEFAULT_ADDR = "192.168.5.2:50051"
SDK_DEFAULT = {"robot"}                 # the usual name of the RobotClient
REMOVED_NAMES = {"rover", "dog"}        # old simulator-only helper objects (removed)
STD_MODULES = ("math", "time", "random")
SDK_VAR_RE = re.compile(r"^\s*(\w+)\s*=\s*(?:\w+\.)?RobotClient\s*\(", re.M)
SDK_WITH_RE = re.compile(r"RobotClient\s*\(.*\)\s*as\s+(\w+)")

DQ = {"“", "”", "„"}      # “ ” „
SQ = {"‘", "’"}                # ‘ ’
CHAR_FIXES = {
    "（": "(", "）": ")", "：": ":", "，": ",", "；": ";",
    " ": " ", "​": "", "﻿": "", "　": " ",
    "＝": "=", "＋": "+", "－": "-",
}

COMMON_BUILTINS = ["print", "range", "len", "int", "float", "str", "abs", "round",
                   "min", "max", "sum", "list", "dict", "enumerate", "zip", "sorted",
                   "isinstance", "bool", "tuple", "set", "reversed", "any", "all"]

# Thai Kedmanee keyboard: English key -> Thai character at the same position
_KEDMANEE = {
    "`": "_", "1": "ๅ", "2": "/", "3": "-", "4": "ภ", "5": "ถ", "6": "ุ", "7": "ึ", "8": "ค",
    "9": "ต", "0": "จ", "-": "ข", "=": "ช", "q": "ๆ", "w": "ไ", "e": "ำ", "r": "พ", "t": "ะ",
    "y": "ั", "u": "ี", "i": "ร", "o": "น", "p": "ย", "[": "บ", "]": "ล", "\\": "ฃ", "a": "ฟ",
    "s": "ห", "d": "ก", "f": "ด", "g": "เ", "h": "้", "j": "่", "k": "า", "l": "ส", ";": "ว",
    "'": "ง", "z": "ผ", "x": "ป", "c": "แ", "v": "อ", "b": "ิ", "n": "ื", "m": "ท", ",": "ม",
    ".": "ใ", "/": "ฝ",
    "~": "%", "!": "+", "@": "๑", "#": "๒", "$": "๓", "%": "๔", "^": "ู", "&": "฿", "*": "๕",
    "(": "๖", ")": "๗", "_": "๘", "+": "๙", "Q": "๐", "W": '"', "E": "ฎ", "R": "ฑ", "T": "ธ",
    "Y": "ํ", "U": "๊", "I": "ณ", "O": "ฯ", "P": "ญ", "{": "ฐ", "}": ",", "|": "ฅ", "A": "ฤ",
    "S": "ฆ", "D": "ฏ", "F": "โ", "G": "ฌ", "H": "็", "J": "๋", "K": "ษ", "L": "ศ", ":": "ซ",
    '"': ".", "Z": "(", "X": ")", "C": "ฉ", "V": "ฮ", "B": "ฺ", "N": "์", "M": "?", "<": "ฒ",
    ">": "ฬ", "?": "ฦ",
}
THAI_TO_KEY = {thai: key for key, thai in _KEDMANEE.items()}
THAI_DIGITS = str.maketrans("๐๑๒๓๔๕๖๗๘๙", "0123456789")
KNOWN_ATTRS = {"robot_state", "pos_body", "ori_body", "vel_body", "acc_body", "omega_body",
               "success", "current_state", "current_speed_ratio", "obstacle_avoidance_enabled",
               "motions", "motion_id", "video", "sleep", "time"} | set(dir(math))
COLOR_NAMES = [c.name.lower() for c in Color]

# The removed rover.* helpers -> what to use instead (shown in the error message)
ROVER_TO_SDK = {
    "walk": "robot.walk_forward(d) / robot.walk_backward(d)",
    "backward": "robot.walk_backward(d)",
    "strafe": "robot.move_left(d) / robot.move_right(d)",
    "turn": 'robot.rotate("left" or "right", angle)',
    "turn_left": "robot.rotate_left(angle)",
    "turn_right": "robot.rotate_right(angle)",
    "turn_to": "read yaw from robot.get_state().robot_state.ori_body[2], then rotate",
    "goto": "write go_to() with get_state() + rotate + walk_forward (example 4)",
    "move": "robot.velocity_sequence([(vx, vy, vyaw_rad, sec)], stand_down_after=False)",
    "stop": "robot.ready()",
    "wait": "time.sleep(sec)",
    "stand": "robot.balance_stand()",
    "sit": "robot.stand_down()",
    "lie_down": "robot.stand_down()",
    "action": "robot.wave_hand() / robot.dance() / robot.jump() ...",
    "set_gait": "robot.walk() / robot.flying_trot() / robot.change_mode()",
    "set_body_height": "robot.balance_height(-0.05)",
    "led": "robot.set_all_legs_color(Color.GREEN)",
    "say": "print(...)",
    "get_position": "robot.get_state().robot_state.pos_body",
    "get_heading": "robot.get_state().robot_state.ori_body[2]",
    "get_imu": "robot.get_state().robot_state.ori_body",
    "get_distance": "no distance sensor in the SDK - rely on set_obstacle_avoidance(True) and .success",
    "scan": "no distance sensor in the SDK",
    "is_bumped": "check .success, e.g. robot.walk_forward(1).success",
}

BLOCK_KW = re.compile(r"^\s*(if|elif|else|for|while|def|try|except|finally|with|class)\b")


def _is_thai(ch):
    return "฀" <= ch <= "๿"


def _exact_sdk_name(name):
    """SDK command with exactly this name, ignoring case and camelCase (no fuzzy guessing)."""
    snake = "".join("_" + c.lower() if c.isupper() else c for c in name).lstrip("_")
    for cand in (name, name.lower(), snake):
        if cand in SDK_FUNCTIONS:
            return cand
    return None


# ---------------------------------------------------------------------------
# Scanner: which characters are code, and which are inside strings / comments
# ---------------------------------------------------------------------------
def _scan(code, fix=False):
    """Walk through the whole program (multi-line strings included).

    Returns (text, masks, changed_lines): masks[i][j] is True when character j of line i is
    code (not in a string literal or a comment). With fix=True, curly quotes used AS quotes and
    other odd characters in code are replaced; curly quotes inside normal strings stay."""
    out, masks, cur = [], [], []
    changed = set()
    state = None          # None or (quote, triple, curly_family_or_None)
    comment = False
    line = 0
    i, n = 0, len(code)
    while i < n:
        ch = code[i]
        if ch == "\n":
            out.append(ch)
            masks.append(cur)
            cur = []
            line += 1
            comment = False
            if state and not state[1]:
                state = None              # unterminated one-line string ends here
            i += 1
            continue
        if comment:
            out.append(ch)
            cur.append(False)
            i += 1
            continue
        if state:
            q, triple, curly = state
            if ch == "\\" and i + 1 < n:
                if code[i + 1] == "\n":       # backslash-newline inside a string
                    out.append(ch)
                    cur.append(False)
                    out.append("\n")
                    masks.append(cur)
                    cur = []
                    line += 1
                    i += 2
                    continue
                out.append(ch + code[i + 1])
                cur += [False, False]
                i += 2
                continue
            if triple and code.startswith(q * 3, i):
                out.append(q * 3)
                cur += [False] * 3
                state = None
                i += 3
                continue
            if not triple and (ch == q or (fix and curly and ch in curly)):
                if ch != q:
                    changed.add(line)
                out.append(q)
                cur.append(False)
                state = None
                i += 1
                continue
            out.append(ch)
            cur.append(False)
            i += 1
            continue
        if ch == "#":
            comment = True
            out.append(ch)
            cur.append(False)
            i += 1
            continue
        if ch in "\"'":
            triple = code.startswith(ch * 3, i)
            k = 3 if triple else 1
            out.append(ch * k)
            cur += [False] * k
            state = (ch, triple, None)
            i += k
            continue
        if fix and (ch in DQ or ch in SQ):
            q = '"' if ch in DQ else "'"
            changed.add(line)
            out.append(q)
            cur.append(False)
            state = (q, False, DQ if ch in DQ else SQ)
            i += 1
            continue
        if fix and ch in CHAR_FIXES:
            rep = CHAR_FIXES[ch]
            changed.add(line)
            out.append(rep)
            cur += [True] * len(rep)
            i += 1
            continue
        out.append(ch)
        cur.append(True)
        i += 1
    masks.append(cur)
    return "".join(out), masks, changed


def _line_mask(line):
    return _scan(line)[1][0]


# ---------------------------------------------------------------------------
# Names defined in the program
# ---------------------------------------------------------------------------
def _defined_names(code):
    names = set()
    try:
        tree = ast.parse(code)
    except SyntaxError:
        for m in re.finditer(r"^\s*(?:def|class)\s+(\w+)", code, re.M):
            names.add(m.group(1))
        for m in re.finditer(r"^\s*(\w+)\s*=[^=]", code, re.M):
            names.add(m.group(1))
        for m in re.finditer(r"\bfor\s+(\w+)", code):
            names.add(m.group(1))
        for m in re.finditer(r"import\s+(\w+)", code):
            names.add(m.group(1))
        for m in re.finditer(r"\bas\s+(\w+)", code):
            names.add(m.group(1))
        return names
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and isinstance(node.ctx, (ast.Store, ast.Del)):
            names.add(node.id)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, ast.arg):
            names.add(node.arg)
        elif isinstance(node, ast.ExceptHandler) and node.name:
            names.add(node.name)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for a in node.names:
                names.add((a.asname or a.name).split(".")[0])
    return names


def _call_name(func):
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return None


def _sdk_vars(code):
    """Variables that hold a RobotClient. `robot` counts unless it is clearly used for something else."""
    found = set(SDK_VAR_RE.findall(code)) | set(SDK_WITH_RE.findall(code))
    other = set()
    try:
        tree = ast.parse(code)
    except SyntaxError:
        tree = None
    if tree is not None:
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                # a call (RobotClient(), a misspelled RobotClinet(), make_robot()...) may give a client;
                # only plain values such as robot = "Rover" clearly mean something else
                is_call = isinstance(node.value, ast.Call)
                for tgt in node.targets:
                    if isinstance(tgt, ast.Name):
                        if is_call and _call_name(node.value.func) == "RobotClient":
                            found.add(tgt.id)
                        elif not is_call:
                            other.add(tgt.id)
    if "robot" in found or "robot" not in other:
        found.add("robot")
    return found


# ---------------------------------------------------------------------------
# 1. odd characters
# ---------------------------------------------------------------------------
def _fix_chars(code, changes):
    new, _, changed = _scan(code, fix=True)
    if new != code:
        old_lines, new_lines = code.split("\n"), new.split("\n")
        for ln in sorted(changed):
            if ln < len(old_lines) and ln < len(new_lines) and old_lines[ln] != new_lines[ln]:
                changes.append({"line": ln + 1, "before": old_lines[ln].strip(), "after": new_lines[ln].strip(),
                                "reason": "replaced special characters (curly quotes / full-width brackets)"})
    return new


# ---------------------------------------------------------------------------
# 2. Thai input mistakes
# ---------------------------------------------------------------------------
def _line_compiles(text):
    s = text.strip()
    if not s:
        return False
    if re.match(r"(elif|else|except|finally)\b", s):
        s = ("if 1:\n    pass\n" if not s.startswith(("except", "finally")) else "try:\n    pass\n") + s
    if s.rstrip().endswith(":"):
        s += "\n    pass"
    try:
        compile(s, "<line>", "exec")
        return True
    except SyntaxError:
        return False


def _names_known(text, known):
    """True if every name in the line is a known name/command (guards against bad conversions)."""
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(text.strip() + "\n").readline))
    except (tokenize.TokenError, IndentationError, SyntaxError):
        return False
    names = 0
    for i, t in enumerate(toks):
        if t.type != tokenize.NAME:
            continue
        names += 1
        if i > 0 and toks[i - 1].string == ".":   # exact names only - fuzzy would accept garbage
            ok = (t.string in KNOWN_ATTRS or t.string in SDK_FUNCTIONS or t.string in SDK_ALIASES
                  or any(t.string in v for v in SDK_CONSTS.values()))
        else:
            ok = t.string in known
        if not ok:
            return False
    return names > 0


def _convert_thai_runs(line, mask, extend):
    """Convert runs of Thai characters (plus the ASCII symbols in `extend` that touch them)
    back to the English keys at the same keyboard positions."""
    out = list(line)
    n = len(line)
    j = 0
    while j < n:
        if mask[j] and _is_thai(line[j]):
            a = j
            while a > 0 and mask[a - 1] and line[a - 1] in extend:
                a -= 1
            b = j
            while b + 1 < n and mask[b + 1] and (_is_thai(line[b + 1]) or line[b + 1] in extend):
                b += 1
            for k in range(a, b + 1):
                out[k] = THAI_TO_KEY.get(line[k], line[k])
            j = b + 1
        else:
            j += 1
    return "".join(out)


def _thai_names(text):
    try:
        toks = tokenize.generate_tokens(io.StringIO(text.strip() + "\n").readline)
        return [t.string for t in toks if t.type == tokenize.NAME and any(_is_thai(c) for c in t.string)]
    except (tokenize.TokenError, IndentationError, SyntaxError):
        return []


def _fix_thai(code, changes):
    lines = code.split("\n")
    defined = _defined_names(code)
    known = (set(dir(builtins)) | defined | set(keyword.kwlist) | SDK_DEFAULT | set(SDK_EXPORTS)
             | set(STD_MODULES))
    masks = _scan(code)[1]
    for i, line in enumerate(lines):
        mask = masks[i] if i < len(masks) and len(masks[i]) == len(line) else _line_mask(line)
        thai_in_code = [j for j, ch in enumerate(line) if _is_thai(ch) and mask[j]]
        if not thai_in_code:
            continue
        indent = line[:len(line) - len(line.lstrip())]
        body = line[len(indent):]
        compiles = _line_compiles(line)
        thai_names = _thai_names(line) if compiles else []
        if compiles and thai_names and all(nm in defined for nm in thai_names):
            continue          # valid code that uses Thai variable names, e.g. ระยะ = 2
        new, reason = None, None

        # (1) the whole line was typed with the Thai keyboard layout
        if not re.search(r"[A-Za-z]", body):
            cand = indent + "".join(THAI_TO_KEY.get(ch, ch) for ch in body)
            if _line_compiles(cand) and _names_known(cand, known):
                new, reason = cand, "line was typed with the Thai keyboard layout → converted to English keys"
        # (2) only some words were typed with the Thai keyboard layout
        if new is None:
            for extend in ("-/_%+\"(),?", ""):
                cand = _convert_thai_runs(line, mask, extend)
                if cand != line and _line_compiles(cand) and _names_known(cand, known):
                    new, reason = cand, "word typed with the Thai keyboard layout → converted to English keys"
                    break
        # (3) Thai digits ๐-๙ used as numbers
        if new is None and not compiles:
            cand = "".join(ch.translate(THAI_DIGITS) if mask[j] else ch for j, ch in enumerate(line))
            if cand != line and not any(_is_thai(ch) for j, ch in enumerate(cand) if mask[j]):
                new, reason = cand, "Thai digits → 0-9"
        # (4) Thai text written in the code without '#'
        if new is None:
            undefined_text = compiles and all(nm not in defined for nm in thai_names)
            if not compiles or undefined_text:
                cut = thai_in_code[0]
                code_part = line[:cut].rstrip()
                cand = (code_part + "  # " if code_part else indent + "# ") + line[cut:].lstrip()
                if not code_part or _line_compiles(code_part):
                    new, reason = cand, "Thai text in the code → turned into a comment (add # before notes)"
        if new is not None:
            changes.append({"line": i + 1, "before": line.strip(), "after": new.strip(), "reason": reason})
            lines[i] = new
            masks = _scan("\n".join(lines))[1]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 3. Syntax
# ---------------------------------------------------------------------------
def _split_comment(line):
    """(code part without trailing spaces, the rest of the line incl. spaces and comment)"""
    mask = _line_mask(line)
    cut = len(line)
    for j, ch in enumerate(line):
        if ch == "#" and not mask[j]:
            before = mask[:j]
            # a '#' that starts a comment: everything after it is not code
            if all(not m for m in mask[j:]) and (not before or True):
                cut = j
                break
    code_part = line[:cut].rstrip()
    return code_part, line[len(code_part):]


def _fix_syntax(code, changes):
    """Fix guessable SyntaxErrors (missing ':' etc.), possibly several times."""
    for _ in range(30):
        try:
            compile(code, "<check>", "exec")
            return code
        except SyntaxError as e:
            lines = code.split("\n")
            ln = (e.lineno or 1) - 1
            if ln >= len(lines):
                return code
            line = lines[ln]
            msg = e.msg or ""
            body, tail = _split_comment(line)
            if ("expected ':'" in msg or "invalid syntax" in msg) and BLOCK_KW.match(line) \
                    and not body.endswith(":"):
                lines[ln] = body + ":" + tail
                changes.append({"line": ln + 1, "before": line.strip(), "after": lines[ln].strip(),
                                "reason": "missing ':' at the end of the line"})
                code = "\n".join(lines)
                continue
            if "unexpected indent" in msg and ln > 0 and not lines[ln - 1].rstrip().endswith(":"):
                prev_indent = len(lines[ln - 1]) - len(lines[ln - 1].lstrip())
                lines[ln] = " " * prev_indent + line.lstrip()
                changes.append({"line": ln + 1, "before": line, "after": lines[ln],
                                "reason": "unexpected indent → aligned with the previous line"})
                code = "\n".join(lines)
                continue
            if "'(' was never closed" in msg and body.count("(") > body.count(")"):
                lines[ln] = body + ")" * (body.count("(") - body.count(")")) + tail
                changes.append({"line": ln + 1, "before": line.strip(), "after": lines[ln].strip(),
                                "reason": "missing ')'"})
                code = "\n".join(lines)
                continue
            if "\t" in code and "indent" in msg:
                code = code.replace("\t", "    ")
                changes.append({"line": ln + 1, "before": "Tab", "after": "4 spaces",
                                "reason": "tabs → 4 spaces"})
                continue
            return code
    return code


# ---------------------------------------------------------------------------
# 4. Imports
# ---------------------------------------------------------------------------
def _fix_export_name(nm):
    base = nm.split(" as ")[0].strip()
    if not base or base == "*" or base in SDK_EXPORTS:
        return nm
    c = difflib.get_close_matches(base, SDK_EXPORTS, 1, 0.6) or \
        difflib.get_close_matches(base.lower(), [e.lower() for e in SDK_EXPORTS], 1, 0.6)
    if not c:
        return nm
    fixed = next(e for e in SDK_EXPORTS if e.lower() == c[0].lower())
    return nm.replace(base, fixed, 1)


def _fix_imports(code, changes):
    """from dobotquad import RobotClinet  ->  from dobot_quad import RobotClient"""
    lines = code.split("\n")
    masks = _scan(code)[1]
    for i, line in enumerate(lines):
        if i < len(masks) and not any(masks[i]):
            continue                                  # line inside a string or a comment
        body, tail = _split_comment(line)
        m = re.match(r"^(\s*)from\s+([\w.]+)\s+import\s+(.+)$", body)
        m2 = re.match(r"^(\s*)import\s+([\w.]+)(.*)$", body)
        new = body
        if m:
            mod, names_part = m.group(2), m.group(3).strip()
            if mod != "dobot_quad" and difflib.get_close_matches(mod.lower(), ["dobot_quad"], 1, 0.75):
                mod = "dobot_quad"
            if mod == "dobot_quad":
                parens = names_part.startswith("(")
                complete = (not parens or names_part.endswith(")")) and not names_part.endswith("\\")
                if complete:
                    inner = names_part[1:-1] if parens else names_part
                    fixed = ", ".join(_fix_export_name(p.strip()) for p in inner.split(",") if p.strip())
                    names_part = f"({fixed})" if parens else fixed
            new = f"{m.group(1)}from {mod} import {names_part}"
        elif m2 and m2.group(2) != "dobot_quad" and \
                difflib.get_close_matches(m2.group(2).lower(), ["dobot_quad"], 1, 0.75):
            new = f"{m2.group(1)}import dobot_quad{m2.group(3)}"
        if new != body:
            changes.append({"line": i + 1, "before": line.strip(), "after": (new + tail).strip(),
                            "reason": "fixed the dobot_quad import line"})
            lines[i] = new + tail
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 5. Names, arguments and text values
# ---------------------------------------------------------------------------
def _fix_text_value(method, param, tok, row, col, repl, changes):
    """Check a text argument such as rotate("lefft") against the values the real SDK accepts."""
    try:
        value = ast.literal_eval(tok)
    except (ValueError, SyntaxError):
        return
    if not isinstance(value, str) or tok.lstrip("rRbBuUfF")[:3] in ('"""', "'''"):
        return
    quote = tok.lstrip("rRbBuUfF")[0]
    v = value.strip().lower()
    if (method, param) in COLOR_PARAMS:
        name = v if v in COLOR_NAMES else thai_to_english(value)
        if name not in COLOR_NAMES:
            m = difflib.get_close_matches(v, COLOR_NAMES, 1, 0.6)
            name = m[0] if m else None
        if name:
            repl.append((row, col, tok, f"Color.{name.upper()}",
                         f"text color {tok} → Color.{name.upper()} (text colors turn the light OFF on the real robot)"))
        else:
            changes.append({"line": row, "before": tok, "after": None, "warning": True,
                            "reason": "unknown color - use Color.RED, Color.GREEN, ..."})
        return
    choices = TEXT_VALUES[(method, param)]
    exact = param in CASE_SENSITIVE_VALUES
    if (value if exact else v) in choices:
        return
    target = v if (exact and v in choices) else None
    if target is None:
        en = thai_to_english(value)
        if en in choices:
            target = en
        else:
            m = difflib.get_close_matches(v, choices, 1, 0.6)
            target = m[0] if m else None
    if target:
        repl.append((row, col, tok, f"{quote}{target}{quote}",
                     f"{param} {tok} → {quote}{target}{quote} (the real SDK rejects {tok})"))
    else:
        changes.append({"line": row, "before": tok, "after": None, "warning": True,
                        "reason": f"invalid {param} for {method}() - valid: {', '.join(sorted(set(choices)))}"})


def _fix_names(code, changes):
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(code).readline))
    except (tokenize.TokenError, IndentationError, SyntaxError):
        return code
    defined = _defined_names(code)
    sdk_vars = _sdk_vars(code)
    removed = REMOVED_NAMES - defined - sdk_vars
    known = set(dir(builtins)) | defined | set(keyword.kwlist) | sdk_vars | set(SDK_EXPORTS) | set(STD_MODULES)
    lines = code.split("\n")
    repl = []   # (row, col, old, new, reason)
    stack = []  # one frame per open bracket: {"method": SDK method or None, "argpos": int, "kw": str}

    def sig(i):
        """next token that is not a newline/comment"""
        j = i + 1
        while j < len(toks) and toks[j].type in (tokenize.NL, tokenize.COMMENT):
            j += 1
        return toks[j] if j < len(toks) else None

    pending_method = None
    for i, t in enumerate(toks):
        prev = toks[i - 1] if i > 0 else None
        prev2 = toks[i - 2] if i > 1 else None
        nxt = sig(i)
        if t.type == tokenize.OP and t.string in "([{":
            stack.append({"method": pending_method if t.string == "(" else None, "argpos": 0, "kw": None})
            pending_method = None
            continue
        if t.type == tokenize.OP and t.string in ")]}":
            if stack:
                stack.pop()
            continue
        if t.type == tokenize.OP and t.string == ",":
            if stack:
                stack[-1]["argpos"] += 1
                stack[-1]["kw"] = None
            continue
        if t.type == tokenize.STRING:
            top = stack[-1] if stack else None
            if top and top["method"] and prev is not None and prev.string in ("(", ",", "=") \
                    and nxt is not None and nxt.string in (",", ")"):
                params = SDK_ALL_PARAMS.get(top["method"], [])
                param = top["kw"] or (params[top["argpos"]] if top["argpos"] < len(params) else None)
                if (top["method"], param) in TEXT_VALUES or (top["method"], param) in COLOR_PARAMS:
                    _fix_text_value(top["method"], param, t.string, t.start[0], t.start[1], repl, changes)
            continue
        if t.type != tokenize.NAME:
            if not (t.type == tokenize.OP and t.string == "."):
                pending_method = None
            continue

        name = t.string
        row, col = t.start
        is_attr = prev is not None and prev.type == tokenize.OP and prev.string == "."

        # (r) removed simulator-only helpers: rover.walk(...) / dog.turn(...)
        if not is_attr and name in removed and nxt is not None and nxt.string == ".":
            after = toks[i + 2].string if i + 2 < len(toks) else ""
            hint = ROVER_TO_SDK.get(after) or "the real SDK commands on robot"
            changes.append({"line": row, "before": f"{name}.{after}", "after": None,
                            "reason": f"'{name}.*' was removed (simulator-only). Use: {hint}",
                            "warning": True})
            continue
        if is_attr and prev2 is not None and prev2.string in removed:
            continue

        # (a) mistyped robot variable / constant class:  Robot. / robott. / colour.
        if not is_attr and nxt is not None and nxt.string == "." and name not in known \
                and name not in REMOVED_NAMES:
            m = difflib.get_close_matches(name.lower(), sorted(sdk_vars), 1, 0.7)
            if not m:
                c = difflib.get_close_matches(name.lower(), [k.lower() for k in SDK_CONSTS], 1, 0.6)
                m = [next(k for k in SDK_CONSTS if k.lower() == c[0])] if c else None
            if m:
                repl.append((row, col, name, m[0], f"name '{name}' → '{m[0]}'"))
                name = m[0]
                toks[i] = t._replace(string=m[0])

        # (a2) Color.REDD / Leg.fl
        if is_attr and prev2 is not None and prev2.string in SDK_CONSTS:
            consts = SDK_CONSTS[prev2.string]
            if name not in consts:
                m = difflib.get_close_matches(name.upper(), consts, 1, 0.5)
                if m:
                    repl.append((row, col, name, m[0], f"constant {prev2.string}.{name} → {prev2.string}.{m[0]}"))
            continue

        # (b) robot.method
        if is_attr and prev2 is not None and prev2.type == tokenize.NAME and prev2.string in sdk_vars:
            if name in SDK_ATTRIBUTES:
                continue
            target = resolve_sdk_name(name)
            if target and target != name:
                repl.append((row, col, name, target, f"SDK command '{name}' → '{target}'"))
            elif not target:
                sug = difflib.get_close_matches(name.lower(), SDK_FUNCTIONS, 3, 0.4)
                changes.append({"line": row, "before": name, "after": None,
                                "reason": "RobotClient has no such command" +
                                          (f" (did you mean {', '.join(sug)}?)" if sug else ""),
                                "warning": True})
            pending_method = target
            continue

        # (c) keyword argument of an SDK command:  walk_forward(1, sped_ratio=50)
        top = stack[-1] if stack else None
        if nxt is not None and nxt.string == "=" and top and top["method"]:
            params = SDK_ALL_PARAMS.get(top["method"], [])
            fixed_name = name
            if params and name not in params:
                m = difflib.get_close_matches(name.lower(), params, 1, 0.5)
                if m:
                    fixed_name = m[0]
                    repl.append((row, col, name, m[0], f"argument '{name}' → '{m[0]}' of {top['method']}()"))
            top["kw"] = fixed_name
            continue

        # (d) bare function calls:  walk_forward(1) -> robot.walk_forward(1),  Print("x") -> print("x")
        if not is_attr and nxt is not None and nxt.string == "(" and name not in known:
            target, reason = None, None
            m = difflib.get_close_matches(name, ["RobotClient", "LegLedConfig"], 1, 0.7)
            if m:
                target, reason = m[0], f"class '{name}' → '{m[0]}'"
            if not target:
                m = difflib.get_close_matches(name.lower(), COMMON_BUILTINS, 1, 0.8)
                if m:
                    target, reason = m[0], f"function '{name}' → '{m[0]}'"
            if not target:
                # only exact SDK names (any case / camelCase) - guessing from aliases such as
                # run() -> robot.flying_trot() would silently change what the program does
                sdk = _exact_sdk_name(name)
                if sdk:
                    target = "robot." + sdk
                    reason = f"'{name}()' is an SDK command → call it on the robot: robot.{sdk}()"
                    pending_method = sdk
            if target:
                repl.append((row, col, name, target, reason))
            continue

    # replace from the end so column positions stay valid
    for row, col, old, new, reason in sorted(repl, key=lambda r: (r[0], r[1]), reverse=True):
        line = lines[row - 1]
        if line[col:col + len(old)] != old:
            continue
        before = line
        lines[row - 1] = line[:col] + new + line[col + len(old):]
        changes.append({"line": row, "before": before.strip(), "after": lines[row - 1].strip(),
                        "reason": reason})
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 6. Lines the real robot needs: imports and robot = RobotClient(...)
# ---------------------------------------------------------------------------
def _add_missing_header(code, changes):
    try:
        tree = ast.parse(code)
    except SyntaxError:
        return code
    defined = _defined_names(code)
    used = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
    star = any(isinstance(n, ast.ImportFrom) and n.module == "dobot_quad" and any(a.name == "*" for a in n.names)
               for n in ast.walk(tree))
    need_robot = "robot" in used and "robot" not in defined
    need_sdk = [] if star else [e for e in SDK_EXPORTS if (e in used or (e == "RobotClient" and need_robot))
                                and e not in defined]
    need_mods = [m for m in STD_MODULES if m in used and m not in defined]
    if not (need_robot or need_sdk or need_mods):
        return code

    lines = code.split("\n")
    body = tree.body
    first = 0
    if body and isinstance(body[0], ast.Expr) and isinstance(getattr(body[0], "value", None), ast.Constant) \
            and isinstance(body[0].value.value, str):
        first = body[0].end_lineno                     # after the module docstring
        body = body[1:]
    else:
        first = (body[0].lineno - 1) if body else len(lines)
    after_imports = first
    for node in body:                                    # robot creation goes after the top imports
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            after_imports = node.end_lineno
        else:
            break

    inserts = []    # (index, text, reason)
    if need_sdk:
        existing = None
        for node in tree.body:
            if isinstance(node, ast.ImportFrom) and node.module == "dobot_quad" and node.lineno == node.end_lineno:
                body_part, tail = _split_comment(lines[node.lineno - 1])
                if "(" not in body_part and "\\" not in body_part:
                    existing = node
                    break
        if existing is not None:
            idx = existing.lineno - 1
            body_part, tail = _split_comment(lines[idx])
            new = body_part + ", " + ", ".join(need_sdk) + tail
            changes.append({"line": idx + 1, "before": lines[idx].strip(), "after": new.strip(),
                            "reason": f"added {', '.join(need_sdk)} to the import - needed on the real robot"})
            lines[idx] = new
        else:
            inserts.append((first, f"from dobot_quad import {', '.join(need_sdk)}",
                            f"added 'from dobot_quad import {', '.join(need_sdk)}' - needed on the real robot"))
    for mod in need_mods:
        inserts.append((first, f"import {mod}", f"added 'import {mod}' - Python needs it on the real robot too"))
    if need_robot:
        inserts.append((after_imports, f'robot = RobotClient("{DEFAULT_ADDR}")',
                        "created robot = RobotClient(...) - on the real robot put its IP address here"))

    # group the new lines by position, keeping their order (imports before robot = ...)
    groups = {}
    for idx, text, reason in inserts:
        groups.setdefault(idx, []).append((text, reason))
    for idx in sorted(groups, reverse=True):          # from the bottom: earlier indexes stay valid
        lines[idx:idx] = [text for text, _ in groups[idx]]
        for c in changes:                              # lines of earlier fixes move down
            if c["line"] > idx:
                c["line"] += len(groups[idx])
    offset = 0
    for idx in sorted(groups):
        for k, (text, reason) in enumerate(groups[idx]):
            changes.append({"line": idx + offset + k + 1, "before": "", "after": text, "reason": reason})
        offset += len(groups[idx])
    return "\n".join(lines)


def correct(code):
    """Return (fixed code, list of changes, remaining syntax error or None)."""
    changes = []
    code = code.replace("\r\n", "\n")
    code = _fix_chars(code, changes)
    code = _fix_thai(code, changes)
    code = _fix_syntax(code, changes)
    code = _fix_imports(code, changes)
    code = _fix_names(code, changes)
    code = _add_missing_header(code, changes)
    changes.sort(key=lambda c: c["line"])
    error = None
    try:
        compile(code, "<check>", "exec")
    except SyntaxError as e:
        error = {"line": e.lineno, "msg": e.msg}
    return code, changes, error


if __name__ == "__main__":
    demo = """from dobotquad import RobotClinet
robot = RobotClinet()
robot.balanceStand()
for i in rnage(2)
    robot.walk_forwrd(๓, sped_ratio=80)   # เดินหน้า
    พนินะใพนะฟะำ๘สำดะ๖ตจ๗
    robot.rotate("ซ้าย", 90)
walk_backward(1)
robot.set_all_legs_color("green")
rover.walk(2)
robot.stand_down() หมอบ
"""
    new, ch, err = correct(demo)
    print(new)
    for c in ch:
        print(c)
    print("error:", err)
