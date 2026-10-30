import subprocess
import os

uic_bin = r"C:\Qt\6.10.2\mingw_64\bin\uic.exe"

test_ui = """<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>TestForm</class>
 <widget class="QWidget" name="TestForm">
  <layout class="QVBoxLayout" name="mainLayout">
   <item>
    <layout class="QGridLayout" name="gridLayout_gotoContent" columnstretch="5,6" rowstretch="0,0,1">
     <property name="spacing">
      <number>16</number>
     </property>
     <item row="0" column="0">
      <widget class="QFrame" name="frame1"/>
     </item>
     <item row="0" column="1">
      <widget class="QFrame" name="frame2"/>
     </item>
     <item row="1" column="0">
      <widget class="QFrame" name="frame3"/>
     </item>
     <item row="1" column="1">
      <widget class="QFrame" name="frame4"/>
     </item>
     <item row="2" column="0">
      <spacer name="spacer1">
       <property name="orientation">
        <enum>Qt::Orientation::Vertical</enum>
       </property>
       <property name="sizeHint" stdset="0">
        <size>
         <width>20</width>
         <height>20</height>
        </size>
       </property>
      </spacer>
     </item>
     <item row="2" column="1">
      <spacer name="spacer2">
       <property name="orientation">
        <enum>Qt::Orientation::Vertical</enum>
       </property>
       <property name="sizeHint" stdset="0">
        <size>
         <width>20</width>
         <height>20</height>
        </size>
       </property>
      </spacer>
     </item>
    </layout>
   </item>
  </layout>
 </widget>
 <resources/>
 <connections/>
</ui>
"""

with open("test_grid.ui", "w", encoding="utf-8") as f:
    f.write(test_ui)

res = subprocess.run([uic_bin, "test_grid.ui", "-o", "test_grid.h"], capture_output=True, text=True)
print("Return code:", res.returncode)
if res.returncode == 0:
    print("SUCCESS! uic produced valid header:")
    with open("test_grid.h", "r") as f:
        print(f.read()[:500])
else:
    print("ERROR:\n", res.stderr)

for p in ["test_grid.ui", "test_grid.h"]:
    if os.path.exists(p):
        os.remove(p)
