import ast, pathlib, sys
root = pathlib.Path(".")
errors = []
count = 0
for f in sorted(root.rglob("*.py")):
    count += 1
    try:
        ast.parse(f.read_text(encoding="utf-8"))
        print("OK:", f)
    except SyntaxError as e:
        errors.append(f"{f}: {e}")
        print("FAIL:", f, e)
print(f"\n{count} files checked, {len(errors)} errors")
if errors:
    sys.exit(1)
