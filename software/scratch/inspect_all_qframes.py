import xml.etree.ElementTree as ET

tree = ET.parse('mainwindow.ui')
root = tree.getroot()

for w in root.iter('widget'):
    if w.get('class') == 'QFrame':
        name = w.get('name')
        for prop in w.findall('property'):
            if prop.get('name') == 'styleSheet':
                ss = ''.join(prop.itertext()).strip()
                print(f'{name}: {ss}')
