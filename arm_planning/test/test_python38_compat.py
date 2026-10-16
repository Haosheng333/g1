"""Production modules must import on the DEPLOYMENT interpreter (Python 3.8).

ROS Noetic runs Python 3.8. A PEP 585 subscript of a builtin evaluated at
RUNTIME -- `State = tuple[int, int, int, int]` rather than an annotation --
raises TypeError there, and `from __future__ import annotations` does not help
because the expression is not an annotation. That defect made
joint_space_astar, and therefore edge_checked_astar and
collision_aware_planner, unimportable on the target interpreter.

These tests are a source scan, so they catch the problem on any interpreter,
including a 3.9+ host where the import would succeed and hide it.
"""
import ast
import os
import re
import unittest
from pathlib import Path

PKG = Path(os.path.dirname(os.path.abspath(__file__))).parent
SCAN_DIRS = ("scripts", "src")
BUILTIN_GENERICS = ("tuple", "list", "dict", "set", "frozenset", "type")


def _python_files():
    for d in SCAN_DIRS:
        root = PKG / d
        if root.is_dir():
            for path in sorted(root.rglob("*.py")):
                if "__pycache__" not in str(path):
                    yield path


class RuntimeGenericsAreNotSubscriptedBuiltins(unittest.TestCase):
    def test_no_module_level_builtin_generic_assignment(self):
        """`X = tuple[...]` at module scope breaks Python 3.8 at import time."""
        offenders = []
        pattern = re.compile(
            r"^[A-Za-z_][A-Za-z0-9_]*\s*=\s*(%s)\[" % "|".join(BUILTIN_GENERICS))
        for path in _python_files():
            for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                if pattern.match(line):
                    offenders.append("%s:%d: %s" % (path.relative_to(PKG), n, line.strip()))
        self.assertEqual(offenders, [],
                         "use typing.Tuple/List/Dict for a runtime alias:\n  "
                         + "\n  ".join(offenders))

    def test_every_module_parses_as_python(self):
        """A syntax error would be a different and worse failure."""
        for path in _python_files():
            try:
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            except SyntaxError as exc:
                self.fail("%s: %s" % (path.relative_to(PKG), exc))

    def test_the_fixed_alias_uses_typing_tuple(self):
        """Pins the specific regression this file was written for."""
        src = (PKG / "scripts" / "joint_space_astar.py").read_text(encoding="utf-8")
        self.assertIn("State = Tuple[int, int, int, int]", src)
        self.assertNotIn("State = tuple[", src)


if __name__ == "__main__":
    unittest.main()
