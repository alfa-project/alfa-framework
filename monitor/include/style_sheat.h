/*
 * Copyright 2025 ALFA Project. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef STYLE_SHEAT_H
#define STYLE_SHEAT_H
#include "QString"

QString black = R"(
            QWidget
            {
                background-color:#1e1d23;
            }
            QMainWindow {
                background-color:#1e1d23;
            }
            QDialog {
                background-color:#1e1d23;
            }
            QColorDialog {
                background-color:#1e1d23;
            }
            QTextEdit {
                background-color:#1e1d23;
                color: #a9b7c6;
            }
            QPlainTextEdit {
                selection-background-color:#007b50;
                background-color:#1e1d23;
                border-style: solid;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: transparent;
                border-bottom-color: transparent;
                border-width: 1px;
                color: #a9b7c6;
            }
            QPushButton{
                border-style: solid;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: transparent;
                border-bottom-color: transparent;
                border-width: 1px;
                border-style: solid;
                color: #a9b7c6;
                padding: 2px;
                background-color: #1e1d23;
            }
            QPushButton::default{
                border-style: inset;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: transparent;
                border-bottom-color: #04b97f;
                border-width: 1px;
                color: #a9b7c6;
                padding: 2px;
                background-color: #1e1d23;
            }
            QToolButton {
                border-style: solid;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: transparent;
                border-bottom-color: #04b97f;
                border-bottom-width: 1px;
                border-style: solid;
                color: #a9b7c6;
                padding: 2px;
                background-color: #1e1d23;
            }
            QToolButton:hover{
                border-style: solid;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: transparent;
                border-bottom-color: #37efba;
                border-bottom-width: 2px;
                border-style: solid;
                color: #FFFFFF;
                padding-bottom: 1px;
                background-color: #1e1d23;
            }
            QPushButton:hover{
                border-style: solid;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: transparent;
                border-bottom-color: #37efba;
                border-bottom-width: 1px;
                border-style: solid;
                color: #FFFFFF;
                padding-bottom: 2px;
                background-color: #1e1d23;
            }
            QPushButton:pressed{
                border-style: solid;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: transparent;
                border-bottom-color: #37efba;
                border-bottom-width: 2px;
                border-style: solid;
                color: #37efba;
                padding-bottom: 1px;
                background-color: #1e1d23;
            }
            QPushButton:disabled{
                border-style: solid;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: transparent;
                border-bottom-color: #808086;
                border-bottom-width: 2px;
                border-style: solid;
                color: #808086;
                padding-bottom: 1px;
                background-color: #1e1d23;
            }
            QLineEdit {
                border-width: 1px; border-radius: 4px;
                border-color: rgb(58, 58, 58);
                border-style: inset;
                padding: 0 8px;
                color: #a9b7c6;
                background:#1e1d23;
                selection-background-color:#007b50;
                selection-color: #FFFFFF;
            }
            QLabel {
                color: #a9b7c6;
            }
            QLCDNumber {
                color: #37e6b4;
            }
            QProgressBar {
                text-align: center;
                color: rgb(240, 240, 240);
                border-width: 1px;
                border-radius: 10px;
                border-color: rgb(58, 58, 58);
                border-style: inset;
                background-color:#1e1d23;
            }
            QProgressBar::chunk {
                background-color: #04b97f;
                border-radius: 5px;
            }
            QMenuBar {
                background-color: #1e1d23;
            }
            QMenuBar::item {
                color: #a9b7c6;
                spacing: 3px;
                padding: 1px 4px;
                background: #1e1d23;
            }

            QMenuBar::item:selected {
                background:#1e1d23;
                color: #FFFFFF;
            }
            QMenu::item:selected {
                border-style: solid;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: #04b97f;
                border-bottom-color: transparent;
                border-left-width: 2px;
                color: #FFFFFF;
                padding-left:15px;
                padding-top:4px;
                padding-bottom:4px;
                padding-right:7px;
                background-color: #1e1d23;
            }
            QMenu::item {
                border-style: solid;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: transparent;
                border-bottom-color: transparent;
                border-bottom-width: 1px;
                border-style: solid;
                color: #a9b7c6;
                padding-left:17px;
                padding-top:4px;
                padding-bottom:4px;
                padding-right:7px;
                background-color: #1e1d23;
            }
            QMenu{
                background-color:#1e1d23;
            }
            QTabWidget {
                color:rgb(0,0,0);
                background-color:#1e1d23;
            }
            QTabWidget::pane {
                    border-color: rgb(77,77,77);
                    background-color:#1e1d23;
                    border-style: solid;
                    border-width: 1px;
                    border-radius: 6px;
            }
            QTabBar::tab {
                border-style: solid;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: transparent;
                border-bottom-color: transparent;
                border-bottom-width: 1px;
                border-style: solid;
                color: #808086;
                padding: 3px;
                margin-left:3px;
                background-color: #1e1d23;
            }
            QTabBar::tab:selected, QTabBar::tab:last:selected, QTabBar::tab:hover {
                border-style: solid;
                border-top-color: transparent;
                border-right-color: transparent;
                border-left-color: transparent;
                border-bottom-color: #04b97f;
                border-bottom-width: 2px;
                border-style: solid;
                color: #FFFFFF;
                padding-left: 3px;
                padding-bottom: 2px;
                margin-left:3px;
                background-color: #1e1d23;
            }

            QCheckBox {
                color: #a9b7c6;
                padding: 2px;
            }
            QCheckBox:disabled {
                color: #808086;
                padding: 2px;
            }

            QCheckBox:hover {
                border-radius:4px;
                border-style:solid;
                padding-left: 1px;
                padding-right: 1px;
                padding-bottom: 1px;
                padding-top: 1px;
                border-width:1px;
                border-color: rgb(87, 97, 106);
                background-color:#1e1d23;
            }
            QCheckBox::indicator:checked {

                height: 10px;
                width: 10px;
                border-style:solid;
                border-width: 1px;
                border-color: #04b97f;
                color: #a9b7c6;
                background-color: #04b97f;
            }
            QCheckBox::indicator:unchecked {

                height: 10px;
                width: 10px;
                border-style:solid;
                border-width: 1px;
                border-color: #04b97f;
                color: #a9b7c6;
                background-color: transparent;
            }
            QRadioButton {
                color: #a9b7c6;
                background-color: #1e1d23;
                padding: 1px;
            }
            QRadioButton::indicator:checked {
                height: 10px;
                width: 10px;
                border-style:solid;
                border-radius:5px;
                border-width: 1px;
                border-color: #04b97f;
                color: #a9b7c6;
                background-color: #04b97f;
            }
            QRadioButton::indicator:!checked {
                height: 10px;
                width: 10px;
                border-style:solid;
                border-radius:5px;
                border-width: 1px;
                border-color: #04b97f;
                color: #a9b7c6;
                background-color: transparent;
            }
            QStatusBar {
                color:#027f7f;
            }
            QSpinBox {
                color: #a9b7c6;
                background-color: #1e1d23;
            }
            QDoubleSpinBox {
                color: #a9b7c6;
                background-color: #1e1d23;
            }
            QTimeEdit {
                color: #a9b7c6;
                background-color: #1e1d23;
            }
            QDateTimeEdit {
                color: #a9b7c6;
                background-color: #1e1d23;
            }
            QDateEdit {
                color: #a9b7c6;
                background-color: #1e1d23;
            }
            QComboBox {
                color: #a9b7c6;
                background: #1e1d23;
            }
            QComboBox:editable {
                background: #1e1d23;
                color: #a9b7c6;
                selection-background-color: #1e1d23;
            }
            QComboBox QAbstractItemView {
                color: #a9b7c6;
                background: #1e1d23;
                selection-color: #FFFFFF;
                selection-background-color: #1e1d23;
            }
            QComboBox:!editable:on, QComboBox::drop-down:editable:on {
                color: #a9b7c6;
                background: #1e1d23;
            }
            QFontComboBox {
                color: #a9b7c6;
                background-color: #1e1d23;
            }
            QToolBox {
                color: #a9b7c6;
                background-color: #1e1d23;
            }
            QToolBox::tab {
                color: #a9b7c6;
                background-color: #1e1d23;
            }
            QToolBox::tab:selected {
                color: #FFFFFF;
                background-color: #1e1d23;
            }
            QScrollArea {
                color: #FFFFFF;
                background-color: #1e1d23;
            }
            QSlider::groove:horizontal {
                height: 5px;
                background: #04b97f;
            }
            QSlider::groove:vertical {
                width: 5px;
                background: #04b97f;
            }
            QSlider::handle:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #b4b4b4, stop:1 #8f8f8f);
                border: 1px solid #5c5c5c;
                width: 14px;
                margin: -5px 0;
                border-radius: 7px;
            }
            QSlider::handle:vertical {
                background: qlineargradient(x1:1, y1:1, x2:0, y2:0, stop:0 #b4b4b4, stop:1 #8f8f8f);
                border: 1px solid #5c5c5c;
                height: 14px;
                margin: 0 -5px;
                border-radius: 7px;
            }
            QSlider::add-page:horizontal {
                background: white;
            }
            QSlider::add-page:vertical {
                background: white;
            }
            QSlider::sub-page:horizontal {
                background: #04b97f;
            }
            QSlider::sub-page:vertical {
                background: #04b97f;
            })";
QString diffness = R"(QWidget
{
    background-color: #242424;
    color: #fff;
    selection-background-color: #fff;
    selection-color: #000;

}


/*-----QLabel-----*/
QLabel
{
    background-color: transparent;
    color: #fff;

}


/*-----QMenuBar-----*/
QMenuBar
{
    background-color: #4a5157;
    color: #fff;

}


QMenuBar::item
{
    background-color: transparent;
    border-left: 1px solid #003333;
    padding: 5px;
    padding-left: 15px;
    padding-right: 15px;

}


QMenuBar::item:selected
{
    background-color: #003333;
    border: 1px solid #006666;
    color: #fff;

}


QMenuBar::item:pressed
{
    background-color: #006666;
    border: 1px solid #006666;
    color: #fff;

}


/*-----QMenu-----*/
QMenu
{
    background-color: #4a5157;
    border: 1px solid #4a5157;
    padding: 10px;
    color: #fff;

}


QMenu::item
{
    background-color: transparent;
    padding: 2px 20px 2px 20px;
    min-width: 200px;

}


QMenu::separator
{
    background-color: #242424;
    height: 1px;

}


QMenu::item:disabled
{
    color: #555;
    background-color: transparent;
    padding: 2px 20px 2px 20px;

}


QMenu::item:selected
{
    background-color: #003333;
    border: 1px solid #006666;
    color: #fff;

}


/*-----QToolButton-----*/
QToolButton
{
    background-color: transparent;
    color: #fff;
    padding: 3px;
    margin-left: 1px;
}


QToolButton:hover
{
    background-color: rgba(70,162,218,50%);
    border: 1px solid #46a2da;
    color: #000;

}


QToolButton:pressed
{
    background-color: #727272;
    border: 1px solid #46a2da;

}


QToolButton:checked
{
    background-color: #727272;
    border: 1px solid #222;
}


/*-----QPushButton-----*/
QPushButton
{
    background-color: #4891b4;
    color: #fff;
    min-width: 80px;
    border-radius: 4px;
    padding: 5px;

}


QPushButton::flat
{
    background-color: transparent;
    border: none;
    color: #000;

}


QPushButton::disabled
{
    background-color: #606060;
    color: #959595;
    border-color: #051a39;

}


QPushButton::hover
{
    background-color: #54aad3;
    border: 1px solid #46a2da;

}


QPushButton::pressed
{
    background-color: #2385b4;
    border: 1px solid #46a2da;

}


QPushButton::checked
{
    background-color: #bd5355;
    border: 1px solid #bd5355;

}


/*-----QLineEdit-----*/
QLineEdit
{
    background-color: #242424;
    color : #fff;
    border: 1px solid #1d1d1d;
    padding: 3px;
    padding-left: 5px;
    border-radius: 4px;

}


/*-----QPlainTExtEdit-----*/
QPlainTextEdit
{
    background-color: #242424;
    color : #fff;
    border: 1px solid #1d1d1d;
    padding: 3px;
    padding-left: 5px;
    border-radius: 4px;

}


/*-----QToolBox-----*/
QToolBox
{
    background-color: transparent;
    border: 1px solid #1d1d1d;

}


QToolBox::tab
{
    background-color: #002b2b;
    border: 1px solid #1d1d1d;

}


QToolBox::tab:hover
{
    background-color: #006d6d;
    border: 1px solid #1d1d1d;

}


/*-----QComboBox-----*/
QComboBox
{
    background-color: #4a5157;
    padding-left: 6px;
    color: #fff;
    height: 20px;
    border-radius: 4px;

}


QComboBox::disabled
{
    background-color: #404040;
    color: #656565;
    border-color: #051a39;

}


QComboBox:on
{
    background-color: #4a5157;
    color: #fff;

}


QComboBox QAbstractItemView
{
    background-color: #4a5157;
    color: #fff;
    selection-background-color: #002b2b;
    selection-color: #fff;
    outline: 0;

}


QComboBox::drop-down
{
    background-color: #4a5157;
    subcontrol-origin: padding;
    subcontrol-position: top right;
    border-radius: 4px;
    width: 15px;

}


QComboBox::down-arrow
{
    image: url(://arrow-down.png);
    width: 8px;
    height: 8px;

}


/*-----QDoubleSpinBox & QCalendarWidget-----*/
QDoubleSpinBox,
QCalendarWidget QSpinBox
{
    background-color: #242424;
    color : #fff;
    border: 1px solid #1d1d1d;
    border-radius: 4px;
    padding: 3px;
    padding-left: 5px;

}


QDoubleSpinBox::up-button,
QCalendarWidget QSpinBox::up-button
{
    background-color: #4a5157;
    width: 16px;
    border-top-right-radius: 4px;
    border-width: 1px;
    border-color: #1d1d1d;

}


QDoubleSpinBox::up-button:hover,
QCalendarWidget QSpinBox::up-button:hover
{
    background-color: #585858;

}


QDoubleSpinBox::up-button:pressed,
QCalendarWidget QSpinBox::up-button:pressed
{
    background-color: #252525;
    width: 16px;
    border-width: 1px;

}


QDoubleSpinBox::up-arrow,
QCalendarWidget QSpinBox::up-arrow
{
    image: url(://arrow-up.png);
    width: 7px;
    height: 7px;

}


QDoubleSpinBox::down-button,
QCalendarWidget QSpinBox::down-button
{
    background-color: #4a5157;
    width: 16px;
    border-width: 1px;
    border-bottom-right-radius: 4px;
    border-color: #1d1d1d;

}


QDoubleSpinBox::down-button:hover,
QCalendarWidget QSpinBox::down-button:hover
{
    background-color: #585858;

}


QDoubleSpinBox::down-button:pressed,
QCalendarWidget QSpinBox::down-button:pressed
{
    background-color: #252525;
    width: 16px;
    border-width: 1px;

}


QDoubleSpinBox::down-arrow,
QCalendarWidget QSpinBox::down-arrow
{
    image: url(://arrow-down.png);
    width: 7px;
    height: 7px;

}


/*-----QGroupBox-----*/
QGroupBox
{
    border: 1px solid;
    border-color: #1d1d1d;
    border-radius: 4px;
    margin-top: 23px;

}


QGroupBox::title
{
    background-color: #002b2b;
    color: #fff;
    subcontrol-position: top left;
    subcontrol-origin: margin;
    padding: 5px;
    min-width: 100px;
    border: 1px solid #1d1d1d;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    border-bottom: none;

}


/*-----QHeaderView-----*/
QHeaderView::section
{
    background-color: #4a5157;
    border: none;
    color: #fff;
    padding: 4px;

}


QHeaderView::section:disabled
{
    background-color: #525251;
    color: #656565;

}


QHeaderView::section:checked
{
    background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(227, 227, 227, 255),stop:1 rgba(187, 187, 187, 255));
    color: #000;

}


QHeaderView::section::vertical::first,
QHeaderView::section::vertical::only-one
{
    border-left: 1px solid #003333;

}


QHeaderView::section::vertical
{
    border-left: 1px solid #003333;
}


QHeaderView::section::horizontal::first,
QHeaderView::section::horizontal::only-one
{
    border-left: 1px solid #003333;

}


QHeaderView::section::horizontal
{
    border-left: 1px solid #003333;

}


QTableCornerButton::section
{
    background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(227, 227, 227, 255),stop:1 rgba(187, 187, 187, 255));
    border: 1px solid #000;
    color: #fff;

}


/*-----QCalendarWidget-----*/
QCalendarWidget QToolButton
{
    background-color: transparent;
    color: white;

}


QCalendarWidget QToolButton::hover
{
    background-color: #006666;
    border: 1px solid #006666;
    color: #fff;

}


QCalendarWidget QMenu
{
    width: 120px;
    left: 20px;
    color: white;

}


QCalendarWidget QWidget
{
    alternate-background-color: #4a5157;
    color: #fff;

}


QCalendarWidget QAbstractItemView:enabled
{
    color: #fff;
    background-color: #242424;
    selection-background-color: #002b2b;
    selection-color: #fff;

}


QCalendarWidget QAbstractItemView:disabled
{
    color: #404040;

}


/*-----QTreeWidget-----*/
QTreeView
{
    show-decoration-selected: 0;
    alternate-background-color: transparent;
    background-color: transparent;
    border: none;
    color: #fff;
    font: 8pt;

}


QTreeView::item:selected
{
    color:#fff;
    background-color: #002b2b;
    border-radius: 0px;

}


QTreeView::item:!selected:hover
{
    background-color: #5e5e5e;
    border: none;
    color: white;

}


QTreeView::branch:has-children:!has-siblings:closed,
QTreeView::branch:closed:has-children:has-siblings
{
    image: url(://tree-closed.png);

}


QTreeView::branch:open:has-children:!has-siblings,
QTreeView::branch:open:has-children:has-siblings
{
    image: url(://tree-open.png);

}


/*-----QListView-----*/
QListView
{
    background-color: transparent;
    alternate-background-color: transparent;
    border : none;
    color: #fff;
    show-decoration-selected: 1;
    outline: 0;
    border: 1px solid #1d1d1d;

}


QListView::disabled
{
    background-color: #656565;
    color: #1b1b1b;
    border: 1px solid #656565;

}


QListView::item
{
    background-color: transparent;
    padding: 1px;

}


QListView::item:selected
{
    background-color: #002b2b;
    border: 1px solid #002b2b;
    color: #fff;

}


QListView::item:selected:!active
{
    background-color: #002b2b;
    border: 1px solid #002b2b;
    color: #fff;

}


QListView::item:selected:active
{
    background-color: #002b2b;
    border: 1px solid #002b2b;
    color: #fff;

}


QListView::item:hover {
    background-color: #5e5e5e;
    border: none;
    color: #000;

}


/*-----QCheckBox-----*/
QCheckBox
{
    background-color: transparent;
    color: #fff;
    border: none;

}


QCheckBox::indicator
{
    background-color: lightgray;
    border: 1px solid #000;
    width: 12px;
    height: 12px;

}


QCheckBox::indicator:checked
{
    image:url("./ressources/check.png");
    background-color: #002b2b;
    border: 1px solid #3a546e;

}


QCheckBox::indicator:unchecked:hover
{
    border: 1px solid #46a2da;

}


QCheckBox::disabled
{
    color: #656565;

}


QCheckBox::indicator:disabled
{
    background-color: #656565;
    color: #656565;
    border: 1px solid #656565;

}


/*-----QRadioButton-----*/
QRadioButton
{
    color: #fff;
    background-color: transparent;

}


QRadioButton::indicator::unchecked:hover
{
    background-color: #d3d3d3;
    border: 2px solid #002b2b;
    border-radius: 6px;
}


QRadioButton::indicator::checked
{
    border: 2px solid #52beff;
    border-radius: 6px;
    background-color: #002b2b;
    width: 9px;
    height: 9px;

}


/*-----QScrollBar-----*/
QScrollBar:vertical
{
   border: none;
   width: 12px;

}


QScrollBar::handle:vertical
{
   border: none;
   border-radius : 0px;
   background-color: #7a7a7a;
   min-height: 80px;
   width : 12px;

}


QScrollBar::handle:vertical:pressed
{
   background-color: #5d5f60;

}


QScrollBar::add-line:vertical
{
   border: none;
   background: transparent;
   height: 0px;
   subcontrol-position: bottom;
   subcontrol-origin: margin;

}


QScrollBar::add-line:vertical:hover
{
   background-color: transparent;

}


QScrollBar::add-line:vertical:pressed
{
   background-color: #3f3f3f;

}


QScrollBar::sub-line:vertical
{
   border: none;
   background: transparent;
   height: 0px;

}


QScrollBar::sub-line:vertical:hover
{
   background-color: transparent;

}


QScrollBar::sub-line:vertical:pressed
{
   background-color: #3f3f3f;

}


QScrollBar::up-arrow:vertical
{
   width: 0px;
   height: 0px;
   background: transparent;

}


QScrollBar::down-arrow:vertical
{
   width: 0px;
   height: 0px;
   background: transparent;

}


QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical
{
   background-color: #222222;

}


QScrollBar:horizontal
{
   border: none;
   height: 12px;

}


QScrollBar::handle:horizontal
{
   border: none;
   border-radius : 0px;
   background-color: #7a7a7a;
   min-height: 80px;
   height : 12px;

}


QScrollBar::handle:horizontal:pressed
{
   background-color: #5d5f60;

}


QScrollBar::add-line:horizontal
{
   border: none;
   background: transparent;
   height: 0px;
   subcontrol-position: bottom;
   subcontrol-origin: margin;

}


QScrollBar::add-line:horizontal:hover
{
   background-color: transparent;

}


QScrollBar::add-line:horizontal:pressed
{
   background-color: #3f3f3f;

}


QScrollBar::sub-line:horizontal
{
   border: none;
   background: transparent;
   height: 0px;

}


QScrollBar::sub-line:horizontal:hover
{
   background-color: transparent;

}


QScrollBar::sub-line:horizontal:pressed
{
   background-color: #3f3f3f;

}


QScrollBar::up-arrow:horizontal
{
   width: 0px;
   height: 0px;
   background: transparent;

}


QScrollBar::down-arrow:horizontal
{
   width: 0px;
   height: 0px;
   background: transparent;

}


QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal
{
   background-color: #222222;

}


/*-----QProgressBar-----*/
QProgressBar
{
    border: 1px solid #1d1d1d;
    text-align: center;
    border-radius: 10px;
    color: #fff;
    font-weight: bold;

}


QProgressBar::chunk
{
    background-color: #3b86ae;
    border-radius: 9px;
    margin: 0.5px;

}


/*-----QStatusBar-----*/
QStatusBar
{
    background-color: #4a5157;
    color: #ffffff;
    border-color: #051a39;

}


/*-----QSizeGrip-----*/
QSizeGrip
{
    background-color: image("./ressources/sizegrip.png"); /*To replace*/
    border: none;

}
)";

QString combinear = R"(

                    /*-----QWidget-----*/
                    QWidget
                    {
                        background-color: #3a3a3a;
                        color: #fff;
                        selection-background-color: #b78620;
                        selection-color: #000;

                    }


                    /*-----QLabel-----*/
                    QLabel
                    {
                        background-color: transparent;
                        color: #fff;

                    }


                    /*-----QMenuBar-----*/
                    QMenuBar
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(57, 57, 57, 255),stop:1 rgba(50, 50, 50, 255));
                        border: 1px solid #000;
                        color: #fff;

                    }


                    QMenuBar::item
                    {
                        background-color: transparent;

                    }


                    QMenuBar::item:selected
                    {
                        background-color: rgba(183, 134, 32, 20%);
                        border: 1px solid #b78620;
                        color: #fff;

                    }


                    QMenuBar::item:pressed
                    {
                        background-color: rgb(183, 134, 32);
                        border: 1px solid #b78620;
                        color: #fff;

                    }


                    /*-----QMenu-----*/
                    QMenu
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(57, 57, 57, 255),stop:1 rgba(50, 50, 50, 255));
                        border: 1px solid #222;
                        padding: 4px;
                        color: #fff;

                    }


                    QMenu::item
                    {
                        background-color: transparent;
                        padding: 2px 20px 2px 20px;

                    }


                    QMenu::separator
                    {
                        background-color: rgb(183, 134, 32);
                        height: 1px;

                    }


                    QMenu::item:disabled
                    {
                        color: #555;
                        background-color: transparent;
                        padding: 2px 20px 2px 20px;

                    }


                    QMenu::item:selected
                    {
                        background-color: rgba(183, 134, 32, 20%);
                        border: 1px solid #b78620;
                        color: #fff;

                    }


                    /*-----QToolBar-----*/
                    QToolBar
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(69, 69, 69, 255),stop:1 rgba(58, 58, 58, 255));
                        border-top: none;
                        border-bottom: 1px solid #4f4f4f;
                        border-left: 1px solid #4f4f4f;
                        border-right: 1px solid #4f4f4f;

                    }


                    QToolBar::separator
                    {
                        background-color: #2e2e2e;
                        width: 1px;

                    }


                    /*-----QToolButton-----*/
                    QToolButton
                    {
                        background-color: transparent;
                        color: #fff;
                        padding: 5px;
                        padding-left: 8px;
                        padding-right: 8px;
                        margin-left: 1px;
                    }


                    QToolButton:hover
                    {
                        background-color: rgba(183, 134, 32, 20%);
                        border: 1px solid #b78620;
                        color: #fff;

                    }


                    QToolButton:pressed
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(57, 57, 57, 255),stop:1 rgba(50, 50, 50, 255));
                        border: 1px solid #b78620;

                    }


                    QToolButton:checked
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(57, 57, 57, 255),stop:1 rgba(50, 50, 50, 255));
                        border: 1px solid #222;
                    }


                    /*-----QPushButton-----*/
                    QPushButton
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(84, 84, 84, 255),stop:1 rgba(59, 59, 59, 255));
                        color: #ffffff;
                        min-width: 80px;
                        border-style: solid;
                        border-width: 1px;
                        border-radius: 3px;
                        border-color: #051a39;
                        padding: 5px;

                    }


                    QPushButton::flat
                    {
                        background-color: transparent;
                        border: none;
                        color: #fff;

                    }


                    QPushButton::disabled
                    {
                        background-color: #404040;
                        color: #656565;
                        border-color: #051a39;

                    }


                    QPushButton::hover
                    {
                        background-color: rgba(183, 134, 32, 20%);
                        border: 1px solid #b78620;

                    }


                    QPushButton::pressed
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(74, 74, 74, 255),stop:1 rgba(49, 49, 49, 255));
                        border: 1px solid #b78620;

                    }


                    QPushButton::checked
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(74, 74, 74, 255),stop:1 rgba(49, 49, 49, 255));
                        border: 1px solid #222;

                    }


                    /*-----QLineEdit-----*/
                    QLineEdit
                    {
                        background-color: #131313;
                        color : #eee;
                        border: 1px solid #343434;
                        border-radius: 2px;
                        padding: 3px;
                        padding-left: 5px;

                    }


                    /*-----QPlainTExtEdit-----*/
                    QPlainTextEdit
                    {
                        background-color: #131313;
                        color : #eee;
                        border: 1px solid #343434;
                        border-radius: 2px;
                        padding: 3px;
                        padding-left: 5px;

                    }


                    /*-----QTabBar-----*/
                    QTabBar::tab
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(84, 84, 84, 255),stop:1 rgba(59, 59, 59, 255));
                        color: #ffffff;
                        border-style: solid;
                        border-width: 1px;
                        border-color: #666;
                        border-bottom: none;
                        padding: 5px;
                        padding-left: 15px;
                        padding-right: 15px;

                    }


                    QTabWidget::pane
                    {
                        background-color: red;
                        border: 1px solid #666;
                        top: 1px;

                    }


                    QTabBar::tab:last
                    {
                        margin-right: 0;

                    }


                    QTabBar::tab:first:!selected
                    {
                        background-color: #0c0c0d;
                        margin-left: 0px;

                    }


                    QTabBar::tab:!selected
                    {
                        color: #b1b1b1;
                        border-bottom-style: solid;
                        background-color: #0c0c0d;

                    }


                    QTabBar::tab:selected
                    {
                        margin-bottom: 0px;

                    }


                    QTabBar::tab:!selected:hover
                    {
                        border-top-color: #b78620;

                    }


                    /*-----QComboBox-----*/
                    QComboBox
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(84, 84, 84, 255),stop:1 rgba(59, 59, 59, 255));
                        border: 1px solid #000;
                        padding-left: 6px;
                        color: #ffffff;
                        height: 20px;

                    }


                    QComboBox::disabled
                    {
                        background-color: #404040;
                        color: #656565;
                        border-color: #051a39;

                    }


                    QComboBox:on
                    {
                        background-color: #b78620;
                        color: #000;

                    }


                    QComboBox QAbstractItemView
                    {
                        background-color: #383838;
                        color: #ffffff;
                        border: 1px solid black;
                        selection-background-color: #b78620;
                        outline: 0;

                    }


                    QComboBox::drop-down
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(57, 57, 57, 255),stop:1 rgba(50, 50, 50, 255));
                        subcontrol-origin: padding;
                        subcontrol-position: top right;
                        width: 15px;
                        border-left-width: 1px;
                        border-left-color: black;
                        border-left-style: solid;

                    }


                    QComboBox::down-arrow
                    {
                        image: url(://arrow-down.png);
                        width: 8px;
                        height: 8px;
                    }


                    /*-----QSpinBox & QDateTimeEdit-----*/
                    QSpinBox,
                    QDateTimeEdit
                    {
                        background-color: #131313;
                        color : #eee;
                        border: 1px solid #343434;
                        padding: 3px;
                        padding-left: 5px;
                        border-radius : 2px;

                    }


                    QSpinBox::up-button,
                    QDateTimeEdit::up-button
                    {
                        border-top-right-radius:2px;
                        background-color: #777777;
                        width: 16px;
                        border-width: 1px;

                    }


                    QSpinBox::up-button:hover,
                    QDateTimeEdit::up-button:hover
                    {
                        background-color: #585858;

                    }


                    QSpinBox::up-button:pressed,
                    QDateTimeEdit::up-button:pressed
                    {
                        background-color: #252525;
                        width: 16px;
                        border-width: 1px;

                    }


                    QSpinBox::up-arrow,
                    QDateTimeEdit::up-arrow
                    {
                        image: url(://arrow-up.png);
                        width: 7px;
                        height: 7px;

                    }


                    QSpinBox::down-button,
                    QDateTimeEdit::down-button
                    {
                        border-bottom-right-radius:2px;
                        background-color: #777777;
                        width: 16px;
                        border-width: 1px;

                    }


                    QSpinBox::down-button:hover,
                    QDateTimeEdit::down-button:hover
                    {
                        background-color: #585858;

                    }


                    QSpinBox::down-button:pressed,
                    QDateTimeEdit::down-button:pressed
                    {
                        background-color: #252525;
                        width: 16px;
                        border-width: 1px;

                    }


                    QSpinBox::down-arrow,
                    QDateTimeEdit::down-arrow
                    {
                        image: url(://arrow-down.png);
                        width: 7px;
                        height: 7px;

                    }


                    /*-----QGroupBox-----*/
                    QGroupBox
                    {
                        border: 1px solid;
                        border-color: #666666;
                        border-radius: 5px;
                        margin-top: 20px;

                    }


                    QGroupBox::title
                    {
                        background-color: transparent;
                        color: #eee;
                        subcontrol-origin: margin;
                        padding: 5px;
                        border-top-left-radius: 3px;
                        border-top-right-radius: 3px;

                    }


                    /*-----QHeaderView-----*/
                    QHeaderView::section
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(60, 60, 60, 255),stop:1 rgba(50, 50, 50, 255));
                        border: 1px solid #000;
                        color: #fff;
                        text-align: left;
                        padding: 4px;

                    }


                    QHeaderView::section:disabled
                    {
                        background-color: #525251;
                        color: #656565;

                    }


                    QHeaderView::section:checked
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(60, 60, 60, 255),stop:1 rgba(50, 50, 50, 255));
                        color: #fff;

                    }


                    QHeaderView::section::vertical::first,
                    QHeaderView::section::vertical::only-one
                    {
                        border-top: 1px solid #353635;

                    }


                    QHeaderView::section::vertical
                    {
                        border-top: 1px solid #353635;

                    }


                    QHeaderView::section::horizontal::first,
                    QHeaderView::section::horizontal::only-one
                    {
                        border-left: 1px solid #353635;

                    }


                    QHeaderView::section::horizontal
                    {
                        border-left: 1px solid #353635;

                    }


                    QTableCornerButton::section
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(60, 60, 60, 255),stop:1 rgba(50, 50, 50, 255));
                        border: 1px solid #000;
                        color: #fff;

                    }


                    /*-----QTreeWidget-----*/
                    QTreeView
                    {
                        show-decoration-selected: 1;
                        alternate-background-color: #3a3a3a;
                        selection-color: #fff;
                        background-color: #2d2d2d;
                        border: 1px solid gray;
                        padding-top : 5px;
                        color: #fff;
                        font: 8pt;

                    }


                    QTreeView::item:selected
                    {
                        color:#fff;
                        background-color: #b78620;
                        border-radius: 0px;

                    }


                    QTreeView::item:!selected:hover
                    {
                        background-color: #262626;
                        border: none;
                        color: white;

                    }


                    QTreeView::branch:has-children:!has-siblings:closed,
                    QTreeView::branch:closed:has-children:has-siblings
                    {
                        image: url(://tree-closed.png);

                    }


                    QTreeView::branch:open:has-children:!has-siblings,
                    QTreeView::branch:open:has-children:has-siblings
                    {
                        image: url(://tree-open.png);

                    }


                    /*-----QListView-----*/
                    QListView
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(83, 83, 83, 255),stop:0.293269 rgba(81, 81, 81, 255),stop:0.634615 rgba(79, 79, 79, 255),stop:1 rgba(83, 83, 83, 255));
                        border : none;
                        color: white;
                        show-decoration-selected: 1;
                        outline: 0;
                        border: 1px solid gray;

                    }


                    QListView::disabled
                    {
                        background-color: #656565;
                        color: #1b1b1b;
                        border: 1px solid #656565;

                    }


                    QListView::item
                    {
                        background-color: #2d2d2d;
                        padding: 1px;

                    }


                    QListView::item:alternate
                    {
                        background-color: #3a3a3a;

                    }


                    QListView::item:selected
                    {
                        background-color: #b78620;
                        border: 1px solid #b78620;
                        color: #fff;

                    }


                    QListView::item:selected:!active
                    {
                        background-color: #b78620;
                        border: 1px solid #b78620;
                        color: #fff;

                    }


                    QListView::item:selected:active
                    {
                        background-color: #b78620;
                        border: 1px solid #b78620;
                        color: #fff;

                    }


                    QListView::item:hover {
                        background-color: #262626;
                        border: none;
                        color: white;

                    }


                    /*-----QCheckBox-----*/
                    QCheckBox
                    {
                        background-color: transparent;
                        color: lightgray;
                        border: none;

                    }


                    QCheckBox::indicator
                    {
                        background-color: #323232;
                        border: 1px solid darkgray;
                        width: 12px;
                        height: 12px;

                    }


                    QCheckBox::indicator:checked
                    {
                        image:url("./ressources/check.png");
                        background-color: #b78620;
                        border: 1px solid #3a546e;

                    }


                    QCheckBox::indicator:unchecked:hover
                    {
                        border: 1px solid #b78620;

                    }


                    QCheckBox::disabled
                    {
                        color: #656565;

                    }


                    QCheckBox::indicator:disabled
                    {
                        background-color: #656565;
                        color: #656565;
                        border: 1px solid #656565;

                    }


                    /*-----QRadioButton-----*/
                    QRadioButton
                    {
                        color: lightgray;
                        background-color: transparent;

                    }


                    QRadioButton::indicator::unchecked:hover
                    {
                        background-color: lightgray;
                        border: 2px solid #b78620;
                        border-radius: 6px;
                    }


                    QRadioButton::indicator::checked
                    {
                        border: 2px solid #b78620;
                        border-radius: 6px;
                        background-color: rgba(183,134,32,20%);
                        width: 9px;
                        height: 9px;

                    }


                    /*-----QSlider-----*/
                    QSlider::groove:horizontal
                    {
                        background-color: transparent;
                        height: 3px;

                    }


                    QSlider::sub-page:horizontal
                    {
                        background-color: #b78620;

                    }


                    QSlider::add-page:horizontal
                    {
                        background-color: #131313;

                    }


                    QSlider::handle:horizontal
                    {
                        background-color: #b78620;
                        width: 14px;
                        margin-top: -6px;
                        margin-bottom: -6px;
                        border-radius: 6px;

                    }


                    QSlider::handle:horizontal:hover
                    {
                        background-color: #d89e25;
                        border-radius: 6px;

                    }


                    QSlider::sub-page:horizontal:disabled
                    {
                        background-color: #bbb;
                        border-color: #999;

                    }


                    QSlider::add-page:horizontal:disabled
                    {
                        background-color: #eee;
                        border-color: #999;

                    }


                    QSlider::handle:horizontal:disabled
                    {
                        background-color: #eee;
                        border: 1px solid #aaa;
                        border-radius: 3px;

                    }


                    /*-----QScrollBar-----*/
                    QScrollBar:horizontal
                    {
                        border: 1px solid #222222;
                        background-color: #3d3d3d;
                        height: 15px;
                        margin: 0px 16px 0 16px;

                    }


                    QScrollBar::handle:horizontal
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(97, 97, 97, 255),stop:1 rgba(90, 90, 90, 255));
                        border: 1px solid #2d2d2d;
                        min-height: 20px;

                    }


                    QScrollBar::add-line:horizontal
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(97, 97, 97, 255),stop:1 rgba(90, 90, 90, 255));
                        border: 1px solid #2d2d2d;
                        width: 15px;
                        subcontrol-position: right;
                        subcontrol-origin: margin;

                    }


                    QScrollBar::sub-line:horizontal
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(97, 97, 97, 255),stop:1 rgba(90, 90, 90, 255));
                        border: 1px solid #2d2d2d;
                        width: 15px;
                        subcontrol-position: left;
                        subcontrol-origin: margin;

                    }


                    QScrollBar::right-arrow:horizontal
                    {
                        image: url(://arrow-right.png);
                        width: 6px;
                        height: 6px;

                    }


                    QScrollBar::left-arrow:horizontal
                    {
                        image: url(://arrow-left.png);
                        width: 6px;
                        height: 6px;

                    }


                    QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal
                    {
                        background: none;

                    }


                    QScrollBar:vertical
                    {
                        background-color: #3d3d3d;
                        width: 16px;
                        border: 1px solid #2d2d2d;
                        margin: 16px 0px 16px 0px;

                    }


                    QScrollBar::handle:vertical
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(97, 97, 97, 255),stop:1 rgba(90, 90, 90, 255));
                        border: 1px solid #2d2d2d;
                        min-height: 20px;

                    }


                    QScrollBar::add-line:vertical
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(97, 97, 97, 255),stop:1 rgba(90, 90, 90, 255));
                        border: 1px solid #2d2d2d;
                        height: 15px;
                        subcontrol-position: bottom;
                        subcontrol-origin: margin;

                    }


                    QScrollBar::sub-line:vertical
                    {
                        background-color: qlineargradient(spread:repeat, x1:1, y1:0, x2:1, y2:1, stop:0 rgba(97, 97, 97, 255),stop:1 rgba(90, 90, 90, 255));
                        border: 1px solid #2d2d2d;
                        height: 15px;
                        subcontrol-position: top;
                        subcontrol-origin: margin;

                    }


                    QScrollBar::up-arrow:vertical
                    {
                        image: url(://arrow-up.png);
                        width: 6px;
                        height: 6px;

                    }


                    QScrollBar::down-arrow:vertical
                    {
                        image: url(://arrow-down.png);
                        width: 6px;
                        height: 6px;

                    }


                    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical
                    {
                        background: none;

                    }


                    /*-----QProgressBar-----*/
                    QProgressBar
                    {
                        border: 1px solid #666666;
                        text-align: center;
                        color: #000;
                        font-weight: bold;

                    }


                    QProgressBar::chunk
                    {
                        background-color: #b78620;
                        width: 30px;
                        margin: 0.5px;

                    }


                    )";

#endif  // STYLE_SHEAT_H
