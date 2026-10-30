import subprocess
import os

uic_bin = r"C:\Qt\6.10.2\mingw_64\bin\uic.exe"

with open("mainwindow.ui", "r", encoding="utf-8") as f:
    ui = f.read()

# Backup
with open("mainwindow.ui.bak", "w", encoding="utf-8") as f:
    f.write(ui)

start_str = '         <layout class="QHBoxLayout" name="horizontalLayout_gotoContent"'
start_idx = ui.find(start_str)
assert start_idx != -1, "start_str not found!"

target_closing = '</layout>\n        </item>\n       </layout>\n      </widget>\n<widget class="QWidget" name="DEBUG_PAGE">'
if target_closing not in ui:
    target_closing = '</layout>\r\n        </item>\r\n       </layout>\r\n      </widget>\r\n<widget class="QWidget" name="DEBUG_PAGE">'
close_idx = ui.find(target_closing)
assert close_idx != -1, "target_closing not found!"

end_idx = close_idx + len('</layout>')

new_grid_content = """         <layout class="QGridLayout" name="gridLayout_gotoContent" columnstretch="5,6" rowstretch="0,0,1">
          <property name="spacing">
           <number>16</number>
          </property>
          <item row="0" column="0">
           <widget class="QFrame" name="frame_gotoKeypad">
            <property name="styleSheet">
             <string notr="true">#frame_gotoKeypad { background-color: #121422; border: 1px solid #23273e; border-radius: 12px; }</string>
            </property>
            <layout class="QVBoxLayout" name="verticalLayout_keypadBox">
             <property name="spacing">
              <number>8</number>
             </property>
             <property name="leftMargin">
              <number>16</number>
             </property>
             <property name="topMargin">
              <number>14</number>
             </property>
             <property name="rightMargin">
              <number>16</number>
             </property>
             <property name="bottomMargin">
              <number>14</number>
             </property>
             <item>
              <widget class="QLabel" name="label_keypadTitle">
               <property name="font">
                <font>
                 <pointsize>10</pointsize>
                 <bold>true</bold>
                </font>
               </property>
               <property name="styleSheet">
                <string notr="true">color: #8c93a8; letter-spacing: 1px;</string>
               </property>
               <property name="text">
                <string>CRUCETA DE MOVIMIENTO (TECLADO / MOUSE)</string>
               </property>
               <property name="alignment">
                <set>Qt::AlignmentFlag::AlignCenter</set>
               </property>
              </widget>
             </item>
             <item>
              <spacer name="spacer_keypadTop">
               <property name="orientation">
                <enum>Qt::Orientation::Vertical</enum>
               </property>
               <property name="sizeHint" stdset="0">
                <size>
                 <width>20</width>
                 <height>2</height>
                </size>
               </property>
              </spacer>
             </item>
             <item>
              <layout class="QGridLayout" name="gridLayout_arrowKeys">
               <property name="spacing">
                <number>6</number>
               </property>
               <item row="0" column="1">
                <widget class="QPushButton" name="btn_goto_up">
                 <property name="focusPolicy">
                  <enum>Qt::FocusPolicy::NoFocus</enum>
                 </property>
                 <property name="minimumSize">
                  <size>
                   <width>85</width>
                   <height>48</height>
                  </size>
                 </property>
                 <property name="text">
                  <string>▲&#10;+ Setpoint</string>
                 </property>
                 <property name="toolTip">
                  <string>Flecha Arriba: Aumentar inclinación / avance</string>
                 </property>
                </widget>
               </item>
               <item row="1" column="0">
                <widget class="QPushButton" name="btn_goto_left">
                 <property name="focusPolicy">
                  <enum>Qt::FocusPolicy::NoFocus</enum>
                 </property>
                 <property name="minimumSize">
                  <size>
                   <width>85</width>
                   <height>48</height>
                  </size>
                 </property>
                 <property name="text">
                  <string>◄&#10;Girar Izq</string>
                 </property>
                 <property name="toolTip">
                  <string>Flecha Izquierda: Rotar sobre propio eje a la izquierda</string>
                 </property>
                </widget>
               </item>
               <item row="1" column="1">
                <widget class="QPushButton" name="btn_goto_down">
                 <property name="focusPolicy">
                  <enum>Qt::FocusPolicy::NoFocus</enum>
                 </property>
                 <property name="minimumSize">
                  <size>
                   <width>85</width>
                   <height>48</height>
                  </size>
                 </property>
                 <property name="text">
                  <string>▼&#10;- Setpoint</string>
                 </property>
                 <property name="toolTip">
                  <string>Flecha Abajo: Reducir inclinación</string>
                 </property>
                </widget>
               </item>
               <item row="1" column="2">
                <widget class="QPushButton" name="btn_goto_right">
                 <property name="focusPolicy">
                  <enum>Qt::FocusPolicy::NoFocus</enum>
                 </property>
                 <property name="minimumSize">
                  <size>
                   <width>85</width>
                   <height>48</height>
                  </size>
                 </property>
                 <property name="text">
                  <string>►&#10;Girar Der</string>
                 </property>
                 <property name="toolTip">
                  <string>Flecha Derecha: Rotar sobre propio eje a la derecha</string>
                 </property>
                </widget>
               </item>
               <item row="2" column="1">
                <widget class="QPushButton" name="btn_goto_center">
                 <property name="focusPolicy">
                  <enum>Qt::FocusPolicy::NoFocus</enum>
                 </property>
                 <property name="minimumSize">
                  <size>
                   <width>85</width>
                   <height>26</height>
                  </size>
                 </property>
                 <property name="styleSheet">
                  <string notr="true">font-size: 8pt; color: #00bcd4; border-color: #00bcd4;</string>
                 </property>
                 <property name="text">
                  <string>⟲ 0.0°</string>
                 </property>
                 <property name="toolTip">
                  <string>Centrar setpoint a 0.0°</string>
                 </property>
                </widget>
               </item>
              </layout>
             </item>
             <item>
              <spacer name="spacer_keypadBottom">
               <property name="orientation">
                <enum>Qt::Orientation::Vertical</enum>
               </property>
               <property name="sizeHint" stdset="0">
                <size>
                 <width>20</width>
                 <height>2</height>
                </size>
               </property>
              </spacer>
             </item>
            </layout>
           </widget>
          </item>
          <item row="0" column="1">
           <widget class="QFrame" name="frame_gotoTelemetry">
            <property name="styleSheet">
             <string notr="true">#frame_gotoTelemetry { background-color: #121422; border: 1px solid #23273e; border-radius: 12px; }</string>
            </property>
            <layout class="QVBoxLayout" name="verticalLayout_teleBox">
             <property name="spacing">
              <number>10</number>
             </property>
             <property name="leftMargin">
              <number>16</number>
             </property>
             <property name="topMargin">
              <number>14</number>
             </property>
             <property name="rightMargin">
              <number>16</number>
             </property>
             <property name="bottomMargin">
              <number>14</number>
             </property>
             <item>
              <widget class="QLabel" name="label_teleTitle">
               <property name="font">
                <font>
                 <pointsize>10</pointsize>
                 <bold>true</bold>
                </font>
               </property>
               <property name="styleSheet">
                <string notr="true">color: #8c93a8; letter-spacing: 1px;</string>
               </property>
               <property name="text">
                <string>TELEMETRÍA DE ÁNGULOS EN TIEMPO REAL</string>
               </property>
              </widget>
             </item>
             <item>
              <layout class="QHBoxLayout" name="horizontalLayout_anglesDisplay">
               <property name="spacing">
                <number>12</number>
               </property>
               <item>
                <widget class="QFrame" name="box_gotoSetpoint">
                 <property name="styleSheet">
                  <string notr="true">#box_gotoSetpoint { background-color: #171b2d; border: 1.5px solid #00bcd4; border-radius: 10px; padding: 8px; }</string>
                 </property>
                 <layout class="QVBoxLayout" name="verticalLayout_spDisplay">
                  <property name="spacing">
                   <number>4</number>
                  </property>
                  <item>
                   <widget class="QLabel" name="label_spTitle">
                    <property name="styleSheet">
                     <string notr="true">color: #00bcd4; font-size: 8pt; font-weight: bold;</string>
                    </property>
                    <property name="text">
                     <string>ÁNGULO COLOCADO (SETPOINT)</string>
                    </property>
                    <property name="alignment">
                     <set>Qt::AlignmentFlag::AlignCenter</set>
                    </property>
                   </widget>
                  </item>
                  <item>
                   <widget class="QLabel" name="lbl_goto_setpoint_deg">
                    <property name="font">
                     <font>
                      <pointsize>18</pointsize>
                      <bold>true</bold>
                     </font>
                    </property>
                    <property name="styleSheet">
                     <string notr="true">color: #00f2c3;</string>
                    </property>
                    <property name="text">
                     <string>-10.00°</string>
                    </property>
                    <property name="alignment">
                     <set>Qt::AlignmentFlag::AlignCenter</set>
                    </property>
                   </widget>
                  </item>
                  <item>
                   <widget class="QLabel" name="lbl_goto_setpoint_raw">
                    <property name="styleSheet">
                     <string notr="true">color: #7b85a3; font-size: 8pt;</string>
                    </property>
                    <property name="text">
                     <string>Valor crudo: -1000</string>
                    </property>
                    <property name="alignment">
                     <set>Qt::AlignmentFlag::AlignCenter</set>
                    </property>
                   </widget>
                  </item>
                 </layout>
                </widget>
               </item>
               <item>
                <widget class="QFrame" name="box_gotoActual">
                 <property name="styleSheet">
                  <string notr="true">#box_gotoActual { background-color: #171b2d; border: 1.5px solid #3d7bfd; border-radius: 10px; padding: 8px; }</string>
                 </property>
                 <layout class="QVBoxLayout" name="verticalLayout_actDisplay">
                  <property name="spacing">
                   <number>4</number>
                  </property>
                  <item>
                   <widget class="QLabel" name="label_actTitle">
                    <property name="styleSheet">
                     <string notr="true">color: #79a6ff; font-size: 8pt; font-weight: bold;</string>
                    </property>
                    <property name="text">
                     <string>ÁNGULO ACTUAL DEL AUTITO</string>
                    </property>
                    <property name="alignment">
                     <set>Qt::AlignmentFlag::AlignCenter</set>
                    </property>
                   </widget>
                  </item>
                  <item>
                   <widget class="QLabel" name="lbl_goto_current_deg">
                    <property name="font">
                     <font>
                      <pointsize>18</pointsize>
                      <bold>true</bold>
                     </font>
                    </property>
                    <property name="styleSheet">
                     <string notr="true">color: #ffffff;</string>
                    </property>
                    <property name="text">
                     <string>0.00°</string>
                    </property>
                    <property name="alignment">
                     <set>Qt::AlignmentFlag::AlignCenter</set>
                    </property>
                   </widget>
                  </item>
                  <item>
                   <widget class="QLabel" name="lbl_goto_error">
                    <property name="styleSheet">
                     <string notr="true">color: #ffb74d; font-size: 8pt;</string>
                    </property>
                    <property name="text">
                     <string>Error: 0.00°</string>
                    </property>
                    <property name="alignment">
                     <set>Qt::AlignmentFlag::AlignCenter</set>
                    </property>
                   </widget>
                  </item>
                 </layout>
                </widget>
               </item>
               <item>
                <widget class="QFrame" name="box_gotoYaw">
                 <property name="styleSheet">
                  <string notr="true">#box_gotoYaw { background-color: #171b2d; border: 1.5px solid #bd00ff; border-radius: 10px; padding: 8px; }</string>
                 </property>
                 <layout class="QVBoxLayout" name="verticalLayout_yawDisplay">
                  <property name="spacing">
                   <number>4</number>
                  </property>
                  <item>
                   <widget class="QLabel" name="label_yawTitle">
                    <property name="styleSheet">
                     <string notr="true">color: #bd00ff; font-size: 8pt; font-weight: bold;</string>
                    </property>
                    <property name="text">
                     <string>YAW (ROTACIÓN RELATIVA)</string>
                    </property>
                    <property name="alignment">
                     <set>Qt::AlignmentFlag::AlignCenter</set>
                    </property>
                   </widget>
                  </item>
                  <item>
                   <widget class="QLabel" name="lbl_goto_yaw_deg">
                    <property name="font">
                     <font>
                      <pointsize>18</pointsize>
                      <bold>true</bold>
                     </font>
                    </property>
                    <property name="styleSheet">
                     <string notr="true">color: #e040fb;</string>
                    </property>
                    <property name="text">
                     <string>0.00°</string>
                    </property>
                    <property name="alignment">
                     <set>Qt::AlignmentFlag::AlignCenter</set>
                    </property>
                   </widget>
                  </item>
                  <item>
                   <widget class="QLabel" name="lbl_goto_yaw_desc">
                    <property name="styleSheet">
                     <string notr="true">color: #7b85a3; font-size: 8pt;</string>
                    </property>
                    <property name="text">
                     <string>Ref. inicio Joystick (gz)</string>
                    </property>
                    <property name="alignment">
                     <set>Qt::AlignmentFlag::AlignCenter</set>
                    </property>
                   </widget>
                  </item>
                  <item>
                   <widget class="QPushButton" name="btn_goto_reset_yaw">
                    <property name="focusPolicy">
                     <enum>Qt::FocusPolicy::NoFocus</enum>
                    </property>
                    <property name="minimumSize">
                     <size>
                      <width>0</width>
                      <height>22</height>
                     </size>
                    </property>
                    <property name="styleSheet">
                     <string notr="true">font-size: 7.5pt; color: #e040fb; border: 1px solid #bd00ff; border-radius: 4px; padding: 2px;</string>
                    </property>
                    <property name="text">
                     <string>⟲ Reset Yaw</string>
                    </property>
                    <property name="toolTip">
                     <string>Poner a 0.0° la referencia del ángulo de Yaw</string>
                    </property>
                   </widget>
                  </item>
                 </layout>
                </widget>
               </item>
              </layout>
             </item>
            </layout>
           </widget>
          </item>
          <item row="1" column="0">
           <widget class="QFrame" name="frame_gotoLegend">
            <property name="styleSheet">
             <string notr="true">#frame_gotoLegend { background-color: #121422; border: 1px solid #23273e; border-radius: 12px; }</string>
            </property>
            <layout class="QVBoxLayout" name="verticalLayout_gotoLegend">
             <property name="spacing">
              <number>8</number>
             </property>
             <property name="leftMargin">
              <number>16</number>
             </property>
             <property name="topMargin">
              <number>14</number>
             </property>
             <property name="rightMargin">
              <number>16</number>
             </property>
             <property name="bottomMargin">
              <number>14</number>
             </property>
             <item>
              <widget class="QLabel" name="label_legend1">
               <property name="font">
                <font>
                 <pointsize>10</pointsize>
                 <bold>true</bold>
                </font>
               </property>
               <property name="styleSheet">
                <string notr="true">color: #8c93a8; letter-spacing: 1px;</string>
               </property>
               <property name="text">
                <string>ATAJOS ACTIVOS EN ESTA PESTAÑA</string>
               </property>
              </widget>
             </item>
             <item>
              <widget class="QFrame" name="box_legendItems">
               <property name="styleSheet">
                <string notr="true">#box_legendItems { background-color: #171b2d; border: 1px dashed #2f3758; border-radius: 8px; padding: 6px; }</string>
               </property>
               <layout class="QVBoxLayout" name="verticalLayout_legendItems">
                <property name="spacing">
                 <number>4</number>
                </property>
                <property name="leftMargin">
                 <number>8</number>
                </property>
                <property name="topMargin">
                 <number>6</number>
                </property>
                <property name="rightMargin">
                 <number>8</number>
                </property>
                <property name="bottomMargin">
                 <number>6</number>
                </property>
                <item>
                 <widget class="QLabel" name="label_legend2">
                  <property name="styleSheet">
                   <string notr="true">color: #a4adc7; font-size: 8.5pt;</string>
                  </property>
                  <property name="text">
                   <string>• &lt;b&gt;↑ Arriba:&lt;/b&gt; Suma setpoint progresivo</string>
                  </property>
                 </widget>
                </item>
                <item>
                 <widget class="QLabel" name="label_legend3">
                  <property name="styleSheet">
                   <string notr="true">color: #a4adc7; font-size: 8.5pt;</string>
                  </property>
                  <property name="text">
                   <string>• &lt;b&gt;↓ Abajo:&lt;/b&gt; Baja setpoint progresivo</string>
                  </property>
                 </widget>
                </item>
                <item>
                 <widget class="QLabel" name="label_legend4">
                  <property name="styleSheet">
                   <string notr="true">color: #a4adc7; font-size: 8.5pt;</string>
                  </property>
                  <property name="text">
                   <string>• &lt;b&gt;← / →:&lt;/b&gt; Rota sobre su propio eje (mantiene balance)</string>
                  </property>
                 </widget>
                </item>
                <item>
                 <widget class="QLabel" name="label_legend5">
                  <property name="styleSheet">
                   <string notr="true">color: #00f2c3; font-size: 8.5pt;</string>
                  </property>
                  <property name="text">
                   <string>• &lt;b&gt;R:&lt;/b&gt; Resetear setpoint a 0.0°</string>
                  </property>
                 </widget>
                </item>
               </layout>
              </widget>
             </item>
            </layout>
           </widget>
          </item>
          <item row="1" column="1">
           <widget class="QFrame" name="frame_gotoSettings">
            <property name="styleSheet">
             <string notr="true">#frame_gotoSettings { background-color: #121422; border: 1px solid #23273e; border-radius: 12px; }</string>
            </property>
            <layout class="QVBoxLayout" name="verticalLayout_settingsBox">
             <property name="spacing">
              <number>12</number>
             </property>
             <property name="leftMargin">
              <number>16</number>
             </property>
             <property name="topMargin">
              <number>14</number>
             </property>
             <property name="rightMargin">
              <number>16</number>
             </property>
             <property name="bottomMargin">
              <number>14</number>
             </property>
             <item>
              <widget class="QLabel" name="label_settingsTitle">
               <property name="font">
                <font>
                 <pointsize>10</pointsize>
                 <bold>true</bold>
                </font>
               </property>
               <property name="styleSheet">
                <string notr="true">color: #8c93a8; letter-spacing: 1px;</string>
               </property>
               <property name="text">
                <string>AJUSTES DE SENSIBILIDAD Y GIRO</string>
               </property>
              </widget>
             </item>
             <item>
              <layout class="QGridLayout" name="gridLayout_settingsInputs">
               <property name="spacing">
                <number>10</number>
               </property>
               <item row="0" column="0">
                <widget class="QLabel" name="label_stepDesc">
                 <property name="text">
                  <string>Paso de Setpoint por pulsación:</string>
                 </property>
                </widget>
               </item>
               <item row="0" column="1">
                <widget class="QDoubleSpinBox" name="spinBox_gotoStep">
                 <property name="focusPolicy">
                  <enum>Qt::FocusPolicy::ClickFocus</enum>
                 </property>
                 <property name="decimals">
                  <number>2</number>
                 </property>
                 <property name="minimum">
                  <double>0.050000000000000</double>
                 </property>
                 <property name="maximum">
                  <double>2.000000000000000</double>
                 </property>
                 <property name="singleStep">
                  <double>0.050000000000000</double>
                 </property>
                 <property name="value">
                  <double>0.250000000000000</double>
                 </property>
                 <property name="suffix">
                  <string>°</string>
                 </property>
                </widget>
               </item>
               <item row="1" column="0">
                <widget class="QLabel" name="label_turnDesc">
                 <property name="text">
                  <string>Velocidad de Giro (PWM):</string>
                 </property>
                </widget>
               </item>
               <item row="1" column="1">
                <widget class="QSpinBox" name="spinBox_gotoTurnIntensity">
                 <property name="focusPolicy">
                  <enum>Qt::FocusPolicy::ClickFocus</enum>
                 </property>
                 <property name="minimum">
                  <number>100</number>
                 </property>
                 <property name="maximum">
                  <number>600</number>
                 </property>
                 <property name="singleStep">
                  <number>25</number>
                 </property>
                 <property name="value">
                  <number>250</number>
                 </property>
                </widget>
               </item>
               <item row="2" column="0">
                <widget class="QLabel" name="label_turnDurationDesc">
                 <property name="text">
                  <string>Cantidad de Giro por pulsación:</string>
                 </property>
                </widget>
               </item>
               <item row="2" column="1">
                <widget class="QSpinBox" name="spinBox_gotoTurnDuration">
                 <property name="focusPolicy">
                  <enum>Qt::FocusPolicy::ClickFocus</enum>
                 </property>
                 <property name="minimum">
                  <number>50</number>
                 </property>
                 <property name="maximum">
                  <number>2000</number>
                 </property>
                 <property name="singleStep">
                  <number>25</number>
                 </property>
                 <property name="value">
                  <number>200</number>
                 </property>
                 <property name="suffix">
                  <string> ms</string>
                 </property>
                </widget>
               </item>
              </layout>
             </item>
            </layout>
           </widget>
          </item>
          <item row="2" column="0">
           <spacer name="spacer_gotoLeftBottom">
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
           <spacer name="spacer_gotoRightBottom">
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
         </layout>"""

if "\r\n" in ui:
    new_grid_content = new_grid_content.replace("\r\n", "\n").replace("\n", "\r\n")

updated_ui = ui[:start_idx] + new_grid_content + ui[end_idx:]

with open("mainwindow.ui", "w", encoding="utf-8") as f:
    f.write(updated_ui)

print("Updated mainwindow.ui! Testing uic...")
res = subprocess.run([uic_bin, "mainwindow.ui", "-o", "ui_mainwindow.h"], capture_output=True, text=True)
if res.returncode != 0:
    print("UIC ERROR:\n", res.stderr)
    with open("mainwindow.ui.bak", "r", encoding="utf-8") as f:
        f.write(f.read())
else:
    print("SUCCESS: mainwindow.ui compiled to ui_mainwindow.h without errors!")
