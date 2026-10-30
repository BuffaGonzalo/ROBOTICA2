import xml.etree.ElementTree as ET

tree = ET.parse('mainwindow.ui')
root = tree.getroot()

for w in root.iter('widget'):
    if w.get('name') == 'GOTO_PAGE':
        print('=== GOTO_PAGE widget ===')
        for child in w.iter('widget'):
            c_name = child.get('name')
            c_class = child.get('class')
            geom = None
            for prop in child.findall('property'):
                if prop.get('name') == 'geometry':
                    r = prop.find('rect')
                    if r is not None:
                        geom = (r.find('x').text, r.find('y').text, r.find('width').text, r.find('height').text)
            if geom:
                print(f"{c_name} ({c_class}): x={geom[0]}, y={geom[1]}, w={geom[2]}, h={geom[3]}")
