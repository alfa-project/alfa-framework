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

#include "boxsettings.h"

#include "ui/ui_boxsettings.h"
#define TRAVEL 10
#define CONSTMULT 100

BoxSettings::BoxSettings(QWidget *parent) : QDialog(parent), ui(new Ui::BoxSettings) {
  ui->setupUi(this);
}
BoxSettings::BoxSettings(QWidget *parent, box *mBox, vector<box> *mListBox)
    : QDialog(parent), ui(new Ui::BoxSettings) {
  boxlist = mListBox;
  boxseletecd = mBox;
  copy = *mBox;
  ui->setupUi(this);
  ui->spinXStart->setValue(mBox->pStart.x);
  ui->spinXEnd->setValue(mBox->pEnd.x);
  ui->spinYStart->setValue(mBox->pStart.y);
  ui->spinYEnd->setValue(mBox->pEnd.y);
  ui->spinZStart->setValue(mBox->pStart.z);
  ui->spinZEnd->setValue(mBox->pEnd.z);

  switch (boxseletecd->type) {
    case 0:
      ui->cbLabelType->setChecked(true);
      break;
    case 1:
      ui->cbNoiseRType->setChecked(true);
      break;
    case 2:
      ui->cbNoiseInject->setChecked(true);
      break;
  }
}

BoxSettings::~BoxSettings() { delete ui; }

void BoxSettings::on_spinXStart_valueChanged(double arg1) {
  (*boxlist)[boxseletecd->index].pStart.x = arg1;
  emit build_boxs();
}

void BoxSettings::on_spinXEnd_valueChanged(double arg1) {
  (*boxlist)[boxseletecd->index].pEnd.x = arg1;
  emit build_boxs();
}

void BoxSettings::on_spinYStart_valueChanged(double arg1) {
  (*boxlist)[boxseletecd->index].pStart.y = arg1;
  emit build_boxs();
}

void BoxSettings::on_spinYEnd_valueChanged(double arg1) {
  (*boxlist)[boxseletecd->index].pEnd.y = arg1;
  emit build_boxs();
}

void BoxSettings::on_spinZStart_valueChanged(double arg1) {
  (*boxlist)[boxseletecd->index].pStart.z = arg1;
  emit build_boxs();
}

void BoxSettings::on_spinZEnd_valueChanged(double arg1) {
  (*boxlist)[boxseletecd->index].pEnd.z = arg1;
  emit build_boxs();
}

void BoxSettings::on_spinMoveZ_valueChanged(double arg1) {
  if (ui->rbArrows->isChecked()) {
    boxseletecd->pStart.z += arg1;
    boxseletecd->pEnd.z += arg1;
    ui->spinMoveZ->setValue(0);
    ui->spinZStart->setValue(boxseletecd->pStart.z);
    ui->spinZEnd->setValue(boxseletecd->pEnd.z);
  }
}

void BoxSettings::on_spinMoveZ_editingFinished() {
  if (ui->rbValue->isChecked()) {
    boxseletecd->pStart.z += ui->spinMoveZ->value();
    boxseletecd->pEnd.z += ui->spinMoveZ->value();
    ui->spinMoveZ->setValue(0);
    ui->spinZStart->setValue(boxseletecd->pStart.z);
    ui->spinZEnd->setValue(boxseletecd->pEnd.z);
  }
}

void BoxSettings::on_spinMoveX_valueChanged(double arg1) {
  if (ui->rbArrows->isChecked()) {
    boxseletecd->pStart.x += arg1;
    boxseletecd->pEnd.x += arg1;
    ui->spinMoveX->setValue(0);
    ui->spinXStart->setValue(boxseletecd->pStart.x);
    ui->spinXEnd->setValue(boxseletecd->pEnd.x);
  }
}

void BoxSettings::on_spinMoveX_editingFinished() {
  if (ui->rbValue->isChecked()) {
    boxseletecd->pStart.x += ui->spinMoveX->value();
    boxseletecd->pEnd.x += ui->spinMoveX->value();
    ui->spinMoveX->setValue(0);
    ui->spinXStart->setValue(boxseletecd->pStart.x);
    ui->spinXEnd->setValue(boxseletecd->pEnd.y);
  }
}

void BoxSettings::on_spinMoveY_valueChanged(double arg1) {
  if (ui->rbArrows->isChecked()) {
    boxseletecd->pStart.y += arg1;
    boxseletecd->pEnd.y += arg1;
    ui->spinMoveY->setValue(0);
    ui->spinYStart->setValue(boxseletecd->pStart.y);
    ui->spinYEnd->setValue(boxseletecd->pEnd.y);
  }
}

void BoxSettings::on_spinMoveY_editingFinished() {
  if (ui->rbValue->isChecked()) {
    boxseletecd->pStart.y += ui->spinMoveY->value();
    boxseletecd->pEnd.y += ui->spinMoveY->value();
    ui->spinMoveY->setValue(0);
    ui->spinYStart->setValue(boxseletecd->pStart.y);
    ui->spinYEnd->setValue(boxseletecd->pEnd.y);
  }
}

void BoxSettings::on_spinSensi_valueChanged(double arg1) {
  ui->spinMoveX->setSingleStep(arg1);
  ui->spinMoveY->setSingleStep(arg1);
  ui->spinMoveZ->setSingleStep(arg1);
}

void BoxSettings::on_rbValue_clicked() {
  ui->rbValue->setChecked(true);
  ui->rbArrows->setChecked(false);
}

void BoxSettings::on_rbArrows_clicked() {
  ui->rbValue->setChecked(false);
  ui->rbArrows->setChecked(true);
}

void BoxSettings::on_cbLabelType_clicked() {
  ui->cbLabelType->setChecked(true);
  ui->cbNoiseRType->setChecked(false);
  ui->cbNoiseInject->setChecked(false);
  boxseletecd->type = 0;
}

void BoxSettings::on_cbNoiseRType_clicked() {
  ui->cbLabelType->setChecked(false);
  ui->cbNoiseRType->setChecked(true);
  ui->cbNoiseInject->setChecked(false);
  boxseletecd->type = 1;
}

void BoxSettings::on_cbNoiseInject_clicked() {
  ui->cbLabelType->setChecked(false);
  ui->cbNoiseRType->setChecked(false);
  ui->cbNoiseInject->setChecked(true);
  boxseletecd->type = 2;
}
