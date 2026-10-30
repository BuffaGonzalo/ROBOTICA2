import xml.etree.ElementTree as ET

tree = ET.parse('mainwindow.ui')
root = tree.getroot()

for w in root.iter('widget'):
    if w.get('name') in ['MainWindow', 'centralwidget', 'stackedWidget', 'GRAF', 'box_chart_orientation', 'box_chart_balance', 'box_chart_line']:
        print(f"=== {w.get('name')} ({w.get('class')}) ===")
        for prop in w.findall('property'):
            if prop.get('name') == 'geometry':
                for r in prop.findall('rect'):
                    x = r.find('x').text
                    y = r.find('y').text
                    width = r.find('width').text
                    height = r.find('height').text
                    print(f"  geometry: x={x}, y={y}, w={width}, h={height}")
