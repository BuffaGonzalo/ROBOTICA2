import re
import subprocess
import os

uic_bin = r"C:\Qt\6.10.2\mingw_64\bin\uic.exe"

with open("mainwindow.ui", "r", encoding="utf-8") as f:
    content = f.read().replace("\r\n", "\n")

# 1. Update stylesheet:
# Add box_dodge_bias to box list
content = content.replace(
    "QFrame#box_linea, QFrame#box_balance, QFrame#box_motores, QFrame#box_motores_rot,",
    "QFrame#box_linea, QFrame#box_balance, QFrame#box_motores, QFrame#box_dodge_bias,"
)

# Add item_dodge_bias_time, item_dodge_bias_mult to item list
content = content.replace(
    "QFrame#item_front_kp, QFrame#item_front_kd, QFrame#item_dodgeDir, QFrame#item_dodge_bias {",
    "QFrame#item_front_kp, QFrame#item_front_kd, QFrame#item_dodgeDir, QFrame#item_dodge_bias, QFrame#item_dodge_bias_time, QFrame#item_dodge_bias_mult {"
)

# Add label_title_dodge_bias to section titles
content = content.replace(
    "QLabel#label_sensors_10, QLabel#label_sensors_9, QLabel#label_sensors_3, QLabel#label_sensors_motores_rot, QLabel#label_sensors_4 {",
    "QLabel#label_sensors_10, QLabel#label_sensors_9, QLabel#label_sensors_3, QLabel#label_title_dodge_bias, QLabel#label_sensors_4 {"
)

# 2. Remove item_dodge_bias from box_motores (si existe)
item_dodge_bias_pattern = r'\s*<item>\s*<widget class="QFrame" name="item_dodge_bias">[\s\S]*?</widget>\s*</item>(?=\s*<item>\s*<spacer name="verticalSpacer_motores">)'
if re.search(item_dodge_bias_pattern, content) is not None:
    content = re.sub(item_dodge_bias_pattern, '', content, count=1)

# 3. Replace box_motores_rot with box_dodge_bias
new_box_dodge_bias = """       <widget class="QFrame" name="box_dodge_bias">
        <property name="geometry">
         <rect>
          <x>1116</x>
          <y>20</y>
          <width>258</width>
          <height>760</height>
         </rect>
        </property>
        <layout class="QVBoxLayout" name="verticalLayout_box_dodge_bias">
         <property name="spacing">
          <number>6</number>
         </property>
         <property name="leftMargin">
          <number>10</number>
         </property>
         <property name="topMargin">
          <number>10</number>
         </property>
         <property name="rightMargin">
          <number>10</number>
         </property>
         <property name="bottomMargin">
          <number>10</number>
         </property>
         <item>
          <widget class="QLabel" name="label_title_dodge_bias">
           <property name="font">
            <font>
             <pointsize>11</pointsize>
             <bold>true</bold>
            </font>
           </property>
           <property name="text">
            <string>SESGO DE ENGANCHE</string>
           </property>
          </widget>
         </item>
         <item>
          <widget class="QFrame" name="item_dodge_bias_time">
           <layout class="QVBoxLayout" name="layout_item_dodge_bias_time">
            <property name="spacing">
             <number>2</number>
            </property>
            <property name="leftMargin">
             <number>6</number>
            </property>
            <property name="topMargin">
             <number>4</number>
            </property>
            <property name="rightMargin">
             <number>6</number>
            </property>
            <property name="bottomMargin">
             <number>4</number>
            </property>
            <item>
             <widget class="QLabel" name="label_dodgeBiasTime">
              <property name="font">
               <font>
                <bold>true</bold>
               </font>
              </property>
              <property name="text">
               <string>TIEMPO (ms)</string>
              </property>
             </widget>
            </item>
            <item>
             <layout class="QHBoxLayout" name="horizontalLayout_dodgeBiasTime">
              <item>
               <widget class="QSpinBox" name="setDodgeBiasTime">
                <property name="suffix">
                 <string> ms</string>
                </property>
                <property name="minimum">
                 <number>0</number>
                </property>
                <property name="maximum">
                 <number>30000</number>
                </property>
                <property name="singleStep">
                 <number>100</number>
                </property>
                <property name="value">
                 <number>1500</number>
                </property>
               </widget>
              </item>
              <item>
               <widget class="QPushButton" name="sendDodgeBiasTime">
                <property name="text">
                 <string>ENVIAR</string>
                </property>
               </widget>
              </item>
             </layout>
            </item>
           </layout>
          </widget>
         </item>
         <item>
          <widget class="QFrame" name="item_dodge_bias_mult">
           <layout class="QVBoxLayout" name="layout_item_dodge_bias_mult">
            <property name="spacing">
             <number>2</number>
            </property>
            <property name="leftMargin">
             <number>6</number>
            </property>
            <property name="topMargin">
             <number>4</number>
            </property>
            <property name="rightMargin">
             <number>6</number>
            </property>
            <property name="bottomMargin">
             <number>4</number>
            </property>
            <item>
             <widget class="QLabel" name="label_dodgeBiasMult">
              <property name="font">
               <font>
                <bold>true</bold>
               </font>
              </property>
              <property name="text">
               <string>MULTIPLICADOR</string>
              </property>
             </widget>
            </item>
            <item>
             <layout class="QHBoxLayout" name="horizontalLayout_dodgeBiasMult">
              <item>
               <widget class="QSpinBox" name="setDodgeBiasMult">
                <property name="minimum">
                 <number>0</number>
                </property>
                <property name="maximum">
                 <number>10000</number>
                </property>
                <property name="singleStep">
                 <number>10</number>
                </property>
                <property name="value">
                 <number>3000</number>
                </property>
               </widget>
              </item>
              <item>
               <widget class="QPushButton" name="sendDodgeBiasMult">
                <property name="text">
                 <string>ENVIAR</string>
                </property>
               </widget>
              </item>
             </layout>
            </item>
           </layout>
          </widget>
         </item>
         <item>
          <spacer name="verticalSpacer_dodge_bias">
           <property name="orientation">
            <enum>Qt::Orientation::Vertical</enum>
           </property>
           <property name="sizeHint" stdset="0">
            <size>
             <width>20</width>
             <height>10</height>
            </size>
           </property>
          </spacer>
         </item>
        </layout>
       </widget>"""

box_motores_rot_pattern = r'       <widget class="QFrame" name="box_motores_rot">[\s\S]*?</widget>\s*(?=      </widget>\s*<widget class="QWidget" name="GOTO_PAGE">)'
assert re.search(box_motores_rot_pattern, content) is not None, "box_motores_rot not matched"
content = re.sub(box_motores_rot_pattern, new_box_dodge_bias + "\n", content, count=1)

with open("mainwindow.ui", "w", encoding="utf-8", newline="") as f:
    f.write(content)

print("mainwindow.ui updated successfully! Testing uic...")
res = subprocess.run([uic_bin, "mainwindow.ui", "-o", "ui_mainwindow.h"], capture_output=True, text=True)
if res.returncode != 0:
    print("UIC ERROR:", res.stderr)
else:
    print("UIC SUCCESS!")
