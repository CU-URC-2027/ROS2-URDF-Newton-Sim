"""
Write live design changes back into arm_params.py.

Only the blocks between  # >>> AUTO:<NAME>  and  # <<< AUTO:<NAME>  are
replaced; everything else in the file (comments, other settings) is kept.
The previous file is copied to arm_params.py.bak first.
"""
import re
import shutil
from pathlib import Path


def _num(x):
    x = float(x)
    r = round(x, 5)
    return repr(0.0 if r == 0 else r)


def _val(v):
    if isinstance(v, str):
        return repr(v)
    if isinstance(v, (tuple, list)):
        return "(" + ", ".join(_val(x) for x in v) + ")"
    if isinstance(v, (int, float)):
        return _num(v)
    raise TypeError(f"can't write {type(v)}")


def _entry(d):
    return "dict(" + ", ".join(f"{k}={_val(v)}" for k, v in d.items()) + ")"


def _dict_block(name, items):
    lines = [f"{name} = {{"]
    for key, d in items.items():
        lines.append(f"    {key!r}: {_entry(d)},")
    lines.append("}")
    return lines


def blocks_for(P):
    return {
        "PIVOTS": [f"SHOULDER_PIVOT = {_val(P.SHOULDER_PIVOT)}"] + _dict_block("LINK_GEOMETRY", P.LINK_GEOMETRY),
        "MOTORS": _dict_block("MOTORS", P.MOTORS),
        "EXTRAS": _dict_block("EXTRA_MASSES", P.EXTRA_MASSES),
    }


def save(P, path):
    path = Path(path)
    text = path.read_text()
    for name, lines in blocks_for(P).items():
        pat = re.compile(rf"(# >>> AUTO:{name}\n)(.*?)(# <<< AUTO:{name})", re.S)
        if not pat.search(text):
            raise ValueError(f"marker '# >>> AUTO:{name}' not found in {path.name}")
        text = pat.sub(lambda m: m.group(1) + "\n".join(lines) + "\n" + m.group(3), text, count=1)
    compile(text, str(path), "exec")          # never write a file that won't import
    shutil.copyfile(path, path.with_suffix(path.suffix + ".bak"))
    path.write_text(text)
    return path
