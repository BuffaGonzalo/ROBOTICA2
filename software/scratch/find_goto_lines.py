with open('mainwindow.ui', 'r', encoding='utf-8') as f:
    lines = f.readlines()

start = None
end = None
for i, line in enumerate(lines):
    if 'name="GOTO_PAGE"' in line:
        start = i
        break

if start is not None:
    depth = 0
    for i in range(start, len(lines)):
        line = lines[i]
        if '<widget' in line:
            depth += 1
        if '</widget>' in line:
            depth -= 1
            if depth == 0:
                end = i
                break
    print(f"GOTO_PAGE lines: {start+1} to {end+1}")
