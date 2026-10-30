import re

ui_path = r'mainwindow.ui'

with open(ui_path, 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Update QLabel in global stylesheet to explicitly have border: none; background: transparent;
old_qlabel = """QLabel {
    color: #e2e2e2;
    font-weight: bold;
}"""

new_qlabel = """QLabel {
    color: #e2e2e2;
    font-weight: bold;
    border: none;
    background: transparent;
}"""

def to_nl(s, src):
    if '\r\n' in src:
        return s.replace('\r\n', '\n').replace('\n', '\r\n')
    return s.replace('\r\n', '\n')

old_qlabel_norm = to_nl(old_qlabel, content)
new_qlabel_norm = to_nl(new_qlabel, content)

assert old_qlabel_norm in content, "old_qlabel not found"
content = content.replace(old_qlabel_norm, new_qlabel_norm, 1)

# 2. Remove the inline stylesheet from box_3d_container, box_manual_commands, box_chart_orientation, box_chart_balance, box_chart_line
target_boxes = [
    'box_3d_container',
    'box_manual_commands',
    'box_chart_orientation',
    'box_chart_balance',
    'box_chart_line'
]

pattern = r'(<widget class="QFrame" name="({0})">\s*<property name="geometry">[\s\S]*?</property>)\s*<property name="styleSheet">\s*<string notr="true">background-color: #151522; border-top: 1px solid #2d2f44; border-left: 1px solid #2d2f44; border-bottom: 1px solid #0e0e16; border-right: 1px solid #0e0e16; border-radius: 8px;</string>\s*</property>'

for box in target_boxes:
    box_pattern = pattern.format(box)
    match = re.search(box_pattern, content)
    assert match is not None, f"Pattern not matched for {box}"
    # Replace by just group 1 (widget + geometry, omitting styleSheet)
    content = content[:match.start()] + match.group(1) + content[match.end():]
    print(f"Removed inline stylesheet from {box}")

with open(ui_path, 'w', encoding='utf-8', newline='') as f:
    f.write(content)

print("SUCCESS: mainwindow.ui updated cleanly")
