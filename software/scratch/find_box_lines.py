with open('mainwindow.ui', 'r', encoding='utf-8') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    for box in ['box_3d_container', 'box_manual_commands', 'box_chart_orientation', 'box_chart_balance', 'box_chart_line']:
        if f'name="{box}"' in line:
            print(f'{box} found at line {i+1}:')
            for j in range(i, min(i+15, len(lines))):
                print(f'  {j+1}: {lines[j].rstrip()}')
