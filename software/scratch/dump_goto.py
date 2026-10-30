import xml.etree.ElementTree as ET

tree = ET.parse('mainwindow.ui')
root = tree.getroot()

for w in root.iter('widget'):
    if w.get('name') == 'GOTO_PAGE':
        with open('scratch/goto_page.xml', 'w', encoding='utf-8') as f:
            f.write(ET.tostring(w, encoding='utf-8').decode('utf-8'))
        print("Wrote scratch/goto_page.xml successfully")
