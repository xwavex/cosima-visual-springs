/*
 * Copyright 2015 Open Source Robotics Foundation
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
 *
*/
#include <functional>
#include <ignition/math/Helpers.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/COMVisual.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/SelectionObj.hh>
#include <gazebo/rendering/ApplyWrenchVisual.hh>

#include <gazebo/gui/Actions.hh>
#include <gazebo/gui/MainWindow.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/gui/GuiIface.hh>

#include "ConfigureVirtualElementsDialog.hh"

#include <iostream>

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ConfigureVirtualElementsDialog::ConfigureVirtualElementsDialog(QWidget *_parent)
    : QDialog(_parent), ui(new Ui::ConfigureVirtualElementsDialog)
{
    this->setObjectName("ConfigureVirtualElementsDialog");
    this->mainWindow = gui::get_main_window();

    this->setWindowTitle(tr("CoSiMA Virtual Elements Plugin"));
    this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
                         Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);
    this->setWindowModality(Qt::NonModal);
    this->setStyleSheet(
        "QPushButton {\
          border-radius: 5px;\
          border-radius: 5px;\
      }");

    std::cout << "loaded!!!" << std::endl;
    ui->setupUi(this);
    std::cout << "ui setup!!!" << std::endl;

    // this->modelLabel = new QLabel();

    // // Links list
    // QHBoxLayout *linkLayout = new QHBoxLayout();
    // QLabel *linkLabel = new QLabel(tr("<b>Apply to link:<b> "));
    // this->linksComboBox = new QComboBox();
    // this->linksComboBox->installEventFilter(this);
    // this->linksComboBox->setMinimumWidth(200);
    // connect(this->linksComboBox, SIGNAL(currentIndexChanged(QString)),
    //         this, SLOT(SetLink(QString)));

    // linkLayout->addWidget(linkLabel);
    // linkLayout->addWidget(this->linksComboBox);

    // // Force
    // QLabel *forceLabel = new QLabel(tr(
    //     "<font size=4>Force</font>"));
    // forceLabel->setObjectName("forceLabel");
    // forceLabel->setStyleSheet(
    //     "QLabel#forceLabel {\
    //       background-color: #444;\
    //       border-radius: 5px;\
    //       padding-left: 10px;\
    //       min-height: 40px;\
    //   }");

    // // Force vector layout
    // QGridLayout *forceVectorLayout = new QGridLayout();

    // // Force Vector
    // this->forceXSpin = new QDoubleSpinBox();
    // this->forceYSpin = new QDoubleSpinBox();
    // this->forceZSpin = new QDoubleSpinBox();

    // std::vector<QDoubleSpinBox *> forceSpins;
    // forceSpins.push_back(this->forceXSpin);
    // forceSpins.push_back(this->forceYSpin);
    // forceSpins.push_back(this->forceZSpin);

    // for (unsigned int i = 0; i < forceSpins.size(); ++i)
    // {
    //     QLabel *forceElementLabel = new QLabel();
    //     if (i == 0)
    //         forceElementLabel->setText(tr("X:"));
    //     else if (i == 1)
    //         forceElementLabel->setText(tr("Y:"));
    //     else if (i == 2)
    //         forceElementLabel->setText(tr("Z:"));
    //     QLabel *forceUnitLabel = new QLabel(tr("N"));

    //     forceSpins[i]->setRange(-ignition::math::MAX_D, ignition::math::MAX_D);
    //     forceSpins[i]->setSingleStep(100);
    //     forceSpins[i]->setDecimals(3);
    //     forceSpins[i]->setValue(0);
    //     forceSpins[i]->setMaximumWidth(100);
    //     forceSpins[i]->installEventFilter(this);
    //     connect(forceSpins[i], SIGNAL(valueChanged(double)), this,
    //             SLOT(OnForceChanged(double)));

    //     forceVectorLayout->addWidget(forceElementLabel, i, 0, Qt::AlignRight);
    //     forceVectorLayout->addWidget(forceSpins[i], i, 1);
    //     forceVectorLayout->addWidget(forceUnitLabel, i, 2);
    // }

    // // Force total
    // QLabel *forceMagLabel = new QLabel(tr("Mag:"));
    // QLabel *forceMagUnitLabel = new QLabel(tr("N"));

    // this->forceMagSpin = new QDoubleSpinBox();
    // this->forceMagSpin->setRange(0, ignition::math::MAX_D);
    // this->forceMagSpin->setSingleStep(100);
    // this->forceMagSpin->setDecimals(3);
    // this->forceMagSpin->setValue(0);
    // this->forceMagSpin->setMaximumWidth(100);
    // this->forceMagSpin->installEventFilter(this);
    // connect(this->forceMagSpin, SIGNAL(valueChanged(double)), this,
    //         SLOT(OnForceMagChanged(double)));

    // forceVectorLayout->addWidget(forceMagLabel, 3, 0, Qt::AlignRight);
    // forceVectorLayout->addWidget(this->forceMagSpin, 3, 1);
    // forceVectorLayout->addWidget(forceMagUnitLabel, 3, 2);

    // // Clear force
    // QPushButton *forceClearButton = new QPushButton(tr("Clear"));
    // connect(forceClearButton, SIGNAL(clicked()), this, SLOT(OnForceClear()));
    // forceVectorLayout->addWidget(forceClearButton, 4, 0, 1, 3, Qt::AlignLeft);

    // // Vertical separator
    // QFrame *separator = new QFrame();
    // separator->setFrameShape(QFrame::VLine);
    // separator->setLineWidth(10);

    // // Force Position
    // QLabel *forcePosLabel = new QLabel(tr("Application Point:"));
    // forcePosLabel->setObjectName("forcePosLabel");
    // forcePosLabel->setStyleSheet(
    //     "QLabel#forcePosLabel {\
    //       max-height: 15px;\
    //   }");

    // // CoM
    // QLabel *comLabel = new QLabel(tr("Center of mass"));

    // QLabel *comPixLabel = new QLabel();
    // QPixmap comPixmap(":images/com.png");
    // comPixmap = comPixmap.scaled(QSize(20, 20));
    // comPixLabel->setPixmap(comPixmap);
    // comPixLabel->setMask(comPixmap.mask());

    // QHBoxLayout *comLabelLayout = new QHBoxLayout();
    // comLabelLayout->addWidget(comLabel);
    // comLabelLayout->addWidget(comPixLabel);

    // this->comRadio = new QRadioButton();
    // this->forcePosRadio = new QRadioButton();
    // this->comRadio->setChecked(true);
    // connect(this->comRadio, SIGNAL(toggled(bool)), this,
    //         SLOT(ToggleComRadio(bool)));

    // // Force Position layout
    // QGridLayout *forcePosLayout = new QGridLayout();
    // forcePosLayout->setContentsMargins(0, 0, 0, 0);
    // forcePosLayout->addWidget(forcePosLabel, 0, 0, 1, 4, Qt::AlignLeft);
    // forcePosLayout->addWidget(this->comRadio, 1, 0);
    // forcePosLayout->addLayout(comLabelLayout, 1, 1, 1, 3, Qt::AlignLeft);
    // forcePosLayout->addWidget(this->forcePosRadio, 2, 0);

    // // Force Position Vector
    // this->forcePosXSpin = new QDoubleSpinBox();
    // this->forcePosYSpin = new QDoubleSpinBox();
    // this->forcePosZSpin = new QDoubleSpinBox();

    // std::vector<QDoubleSpinBox *> forcePosSpins;
    // forcePosSpins.push_back(this->forcePosXSpin);
    // forcePosSpins.push_back(this->forcePosYSpin);
    // forcePosSpins.push_back(this->forcePosZSpin);

    // for (unsigned int i = 0; i < forcePosSpins.size(); ++i)
    // {
    //     QLabel *forcePosElementLabel = new QLabel();
    //     if (i == 0)
    //         forcePosElementLabel->setText(tr("X:"));
    //     else if (i == 1)
    //         forcePosElementLabel->setText(tr("Y:"));
    //     else if (i == 2)
    //         forcePosElementLabel->setText(tr("Z:"));
    //     QLabel *forcePosUnitLabel = new QLabel(tr("m"));

    //     forcePosSpins[i]->setRange(-ignition::math::MAX_D, ignition::math::MAX_D);
    //     forcePosSpins[i]->setSingleStep(0.1);
    //     forcePosSpins[i]->setDecimals(3);
    //     forcePosSpins[i]->setValue(0);
    //     forcePosSpins[i]->setMaximumWidth(100);
    //     forcePosSpins[i]->installEventFilter(this);
    //     connect(forcePosSpins[i], SIGNAL(valueChanged(double)), this,
    //             SLOT(OnForcePosChanged(double)));

    //     forcePosLayout->addWidget(forcePosElementLabel, i + 2, 1, Qt::AlignRight);
    //     forcePosLayout->addWidget(forcePosSpins[i], i + 2, 2);
    //     forcePosLayout->addWidget(forcePosUnitLabel, i + 2, 3);
    // }

    // // Apply force
    // QPushButton *applyForceButton = new QPushButton("Apply Force");
    // connect(applyForceButton, SIGNAL(clicked()), this, SLOT(OnApplyForce()));

    // // Force layout
    // QGridLayout *forceLayout = new QGridLayout();
    // forceLayout->setContentsMargins(0, 0, 0, 0);
    // forceLayout->addWidget(forceLabel, 0, 0, 1, 5);
    // forceLayout->addItem(new QSpacerItem(10, 10), 1, 0, 1, 5);
    // forceLayout->addLayout(forceVectorLayout, 2, 1);
    // forceLayout->addWidget(separator, 2, 2);
    // forceLayout->addLayout(forcePosLayout, 2, 3);
    // forceLayout->addItem(new QSpacerItem(10, 10), 3, 0, 1, 5);
    // forceLayout->addWidget(applyForceButton, 4, 1, 1, 3, Qt::AlignRight);
    // forceLayout->addItem(new QSpacerItem(5, 10), 5, 0);
    // forceLayout->addItem(new QSpacerItem(7, 10), 5, 4);

    // QFrame *forceFrame = new QFrame();
    // forceFrame->setLayout(forceLayout);
    // forceFrame->setObjectName("forceLayout");
    // forceFrame->setFrameShape(QFrame::StyledPanel);

    // forceFrame->setStyleSheet(
    //     "QFrame#forceLayout {\
    //       background-color: #666;\
    //       border-radius: 10px;\
    //   }");

    // QGraphicsDropShadowEffect *forceEffect = new QGraphicsDropShadowEffect;
    // forceEffect->setBlurRadius(5);
    // forceEffect->setXOffset(5);
    // forceEffect->setYOffset(5);
    // forceEffect->setColor(Qt::black);
    // forceFrame->setGraphicsEffect(forceEffect);

    // // Torque
    // QLabel *torqueLabel = new QLabel(tr("<font size=4>Torque</font>"));
    // torqueLabel->setObjectName("torqueLabel");
    // torqueLabel->setStyleSheet(
    //     "QLabel#torqueLabel {\
    //       background-color: #444;\
    //       border-radius: 5px;\
    //       padding-left: 10px;\
    //       min-height: 40px;\
    //   }");

    // // Torque vector layout
    // QGridLayout *torqueVectorLayout = new QGridLayout();

    // // Torque Vector
    // this->torqueXSpin = new QDoubleSpinBox();
    // this->torqueYSpin = new QDoubleSpinBox();
    // this->torqueZSpin = new QDoubleSpinBox();

    // std::vector<QDoubleSpinBox *> torqueSpins;
    // torqueSpins.push_back(this->torqueXSpin);
    // torqueSpins.push_back(this->torqueYSpin);
    // torqueSpins.push_back(this->torqueZSpin);

    // for (unsigned int i = 0; i < torqueSpins.size(); ++i)
    // {
    //     QLabel *torqueElementLabel = new QLabel();
    //     if (i == 0)
    //         torqueElementLabel->setText(tr("X:"));
    //     else if (i == 1)
    //         torqueElementLabel->setText(tr("Y:"));
    //     else if (i == 2)
    //         torqueElementLabel->setText(tr("Z:"));
    //     QLabel *torqueUnitLabel = new QLabel(tr("Nm"));

    //     torqueSpins[i]->setRange(-ignition::math::MAX_D, ignition::math::MAX_D);
    //     torqueSpins[i]->setSingleStep(100);
    //     torqueSpins[i]->setDecimals(3);
    //     torqueSpins[i]->setValue(0);
    //     torqueSpins[i]->setMaximumWidth(100);
    //     torqueSpins[i]->installEventFilter(this);
    //     connect(torqueSpins[i], SIGNAL(valueChanged(double)), this,
    //             SLOT(OnTorqueChanged(double)));

    //     torqueVectorLayout->addWidget(torqueElementLabel, i, 0, Qt::AlignRight);
    //     torqueVectorLayout->addWidget(torqueSpins[i], i, 1);
    //     torqueVectorLayout->addWidget(torqueUnitLabel, i, 2);
    // }

    // // Torque magnitude
    // QLabel *torqueMagLabel = new QLabel(tr("Mag:"));
    // QLabel *torqueMagUnitLabel = new QLabel(tr("Nm"));

    // this->torqueMagSpin = new QDoubleSpinBox();
    // this->torqueMagSpin->setRange(0, ignition::math::MAX_D);
    // this->torqueMagSpin->setSingleStep(100);
    // this->torqueMagSpin->setDecimals(3);
    // this->torqueMagSpin->setValue(0);
    // this->torqueMagSpin->setMaximumWidth(100);
    // this->torqueMagSpin->installEventFilter(this);
    // connect(this->torqueMagSpin, SIGNAL(valueChanged(double)), this,
    //         SLOT(OnTorqueMagChanged(double)));

    // torqueVectorLayout->addWidget(torqueMagLabel, 3, 0, Qt::AlignRight);
    // torqueVectorLayout->addWidget(this->torqueMagSpin, 3, 1);
    // torqueVectorLayout->addWidget(torqueMagUnitLabel, 3, 2);

    // // Clear torque
    // QPushButton *torqueClearButton = new QPushButton(tr("Clear"));
    // connect(torqueClearButton, SIGNAL(clicked()), this, SLOT(OnTorqueClear()));
    // torqueVectorLayout->addWidget(torqueClearButton, 4, 0, 1, 3, Qt::AlignLeft);

    // // Apply torque
    // QPushButton *applyTorqueButton = new QPushButton("Apply Torque");
    // connect(applyTorqueButton, SIGNAL(clicked()), this, SLOT(OnApplyTorque()));

    // // Torque layout
    // QGridLayout *torqueLayout = new QGridLayout();
    // torqueLayout->setContentsMargins(0, 0, 0, 0);
    // torqueLayout->addWidget(torqueLabel, 0, 0, 1, 3);
    // torqueLayout->addItem(new QSpacerItem(10, 10), 1, 0, 1, 3);
    // torqueLayout->addLayout(torqueVectorLayout, 2, 1);
    // torqueLayout->addItem(new QSpacerItem(10, 10), 3, 0, 1, 3);
    // torqueLayout->addWidget(applyTorqueButton, 4, 1, 1, 1, Qt::AlignRight);
    // torqueLayout->addItem(new QSpacerItem(5, 10), 5, 0);
    // torqueLayout->addItem(new QSpacerItem(5, 10), 5, 2);

    // QFrame *torqueFrame = new QFrame();
    // torqueFrame->setLayout(torqueLayout);
    // torqueFrame->setObjectName("torqueLayout");
    // torqueFrame->setFrameShape(QFrame::StyledPanel);

    // torqueFrame->setStyleSheet(
    //     "QFrame#torqueLayout {\
    //       background-color: #666;\
    //       border-radius: 10px;\
    //   }");

    // QGraphicsDropShadowEffect *torqueEffect = new QGraphicsDropShadowEffect;
    // torqueEffect->setBlurRadius(5);
    // torqueEffect->setXOffset(5);
    // torqueEffect->setYOffset(5);
    // torqueEffect->setColor(Qt::black);
    // torqueFrame->setGraphicsEffect(torqueEffect);

    // // Buttons
    // QPushButton *cancelButton = new QPushButton(tr("Cancel"));
    // connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

    // QPushButton *applyAllButton = new QPushButton("Apply All");
    // applyAllButton->setDefault(true);
    // connect(applyAllButton, SIGNAL(clicked()), this, SLOT(OnApplyAll()));

    // QHBoxLayout *buttonsLayout = new QHBoxLayout;
    // buttonsLayout->addWidget(cancelButton);
    // buttonsLayout->addWidget(applyAllButton);

    // // Main layout
    // QGridLayout *mainLayout = new QGridLayout();
    // mainLayout->setSizeConstraint(QLayout::SetFixedSize);
    // mainLayout->addWidget(this->modelLabel, 0, 0, 1, 2, Qt::AlignLeft);
    // mainLayout->addLayout(linkLayout, 1, 0, 1, 2, Qt::AlignLeft);
    // mainLayout->addWidget(forceFrame, 2, 0);
    // mainLayout->addWidget(torqueFrame, 2, 1);
    // mainLayout->addLayout(buttonsLayout, 3, 0, 1, 2, Qt::AlignRight);

    // this->setLayout(mainLayout);

    // // Transport
    // this->node = transport::NodePtr(new transport::Node());
    // this->node->TryInit(common::Time::Maximum());

    // this->userCmdPub =
    //     this->node->Advertise<msgs::UserCmd>("~/user_cmd");

    // this->comVector = ignition::math::Vector3d::Zero;
    // this->forceVector = ignition::math::Vector3d::Zero;
    // this->torqueVector = ignition::math::Vector3d::Zero;
}

/////////////////////////////////////////////////
ConfigureVirtualElementsDialog::~ConfigureVirtualElementsDialog()
{
    this->Fini();
}

/////////////////////////////////////////////////
void ConfigureVirtualElementsDialog::Init()
{
    // if (!this->SetModel(_modelName))
    // {
    //     this->Fini();
    //     return;
    // }

    // if (!this->SetLink(_linkName))
    // {
    //     this->Fini();
    //     return;
    // }

    // connect(this, SIGNAL(rejected()), this, SLOT(OnCancel()));

    // if (g_rotateAct)
    //     connect(g_rotateAct, SIGNAL(triggered()), this, SLOT(OnManipulation()));
    // if (g_translateAct)
    //     connect(g_translateAct, SIGNAL(triggered()), this, SLOT(OnManipulation()));
    // if (g_scaleAct)
    //     connect(g_scaleAct, SIGNAL(triggered()), this, SLOT(OnManipulation()));

    this->move(QCursor::pos());
    this->show();
    this->ActivateWindow();
}

/////////////////////////////////////////////////
void ConfigureVirtualElementsDialog::Fini()
{
    if (this->mainWindow)
        this->mainWindow->removeEventFilter(this);

    // this->userCmdPub.reset();
    // if (this->node)
    //     this->node->Fini();
    // this->node.reset();
    // this->connections.clear();

    // if (this->applyWrenchVisual)
    // {
    //     MouseEventHandler::Instance()->RemoveReleaseFilter(
    //         "dialog_" + this->applyWrenchVisual->Name());
    //     MouseEventHandler::Instance()->RemovePressFilter(
    //         "dialog_" + this->applyWrenchVisual->Name());
    //     MouseEventHandler::Instance()->RemoveMoveFilter(
    //         "dialog_" + this->applyWrenchVisual->Name());

    //     this->applyWrenchVisual->Fini();
    // }
    // this->applyWrenchVisual.reset();

    this->deleteLater();
    delete ui;
}

// ConfigureVirtualElementsDialog::~ConfigureVirtualElementsDialog()
// {
//     delete ui;
// }

void ConfigureVirtualElementsDialog::on_sd_anchor_radioButton_link_clicked()
{
    ui->sd_anchor_stacked->setCurrentIndex(0);
}

void ConfigureVirtualElementsDialog::on_sd_anchor_radioButton_direction_clicked()
{
    ui->sd_anchor_stacked->setCurrentIndex(1);
}

void ConfigureVirtualElementsDialog::on_c_anchor_radioButton_link_clicked()
{
    ui->c_anchor_stacked->setCurrentIndex(0);
}

void ConfigureVirtualElementsDialog::on_c_anchor_radioButton_direction_clicked()
{
    ui->c_anchor_stacked->setCurrentIndex(1);
}

void ConfigureVirtualElementsDialog::on_btn_box_main_accepted()
{
    std::cout << "dfkljsjklfds" << std::endl;
}

void ConfigureVirtualElementsDialog::on_btn_box_main_clicked(QAbstractButton *button)
{
    if (ui->btn_box_main->button(QDialogButtonBox::Reset) == static_cast<QPushButton *>(button))
    {
        // spring-damper page
        ui->sd_anchor_stacked->setCurrentIndex(0);
        ui->sd_anchor_radioButton_link->setChecked(true);
        ui->sd_anchor_stacked_direction_doubleSpinBox_X->setValue(0.0);
        ui->sd_anchor_stacked_direction_doubleSpinBox_Y->setValue(0.0);
        ui->sd_anchor_stacked_direction_doubleSpinBox_Z->setValue(0.0);
        ui->sd_anchor_stacked_link_lineEdit_model->setText("");
        ui->sd_anchor_stacked_link_lineEdit_link->setText("");
        ui->sd_target_groupBox_lineEdit_model->setText("");
        ui->sd_target_groupBox_lineEdit_link->setText("");
        ui->sd_stiffness_groupBox_doubleSpinBox_X->setValue(0.0);
        ui->sd_stiffness_groupBox_doubleSpinBox_Y->setValue(0.0);
        ui->sd_stiffness_groupBox_doubleSpinBox_Z->setValue(0.0);
        ui->sd_stiffness_groupBox_doubleSpinBox_rR->setValue(0.0);
        ui->sd_stiffness_groupBox_doubleSpinBox_rP->setValue(0.0);
        ui->sd_stiffness_groupBox_doubleSpinBox_rY->setValue(0.0);
        ui->sd_damping_groupBox_doubleSpinBox_X->setValue(0.0);
        ui->sd_damping_groupBox_doubleSpinBox_Y->setValue(0.0);
        ui->sd_damping_groupBox_doubleSpinBox_Z->setValue(0.0);
        ui->sd_damping_groupBox_doubleSpinBox_rR->setValue(0.0);
        ui->sd_damping_groupBox_doubleSpinBox_rP->setValue(0.0);
        ui->sd_damping_groupBox_doubleSpinBox_rY->setValue(0.0);

        // constraint page
        ui->c_anchor_stacked->setCurrentIndex(0);
        ui->c_anchor_radioButton_link->setChecked(true);
        ui->c_anchor_stacked_direction_doubleSpinBox_X->setValue(0.0);
        ui->c_anchor_stacked_direction_doubleSpinBox_Y->setValue(0.0);
        ui->c_anchor_stacked_direction_doubleSpinBox_Z->setValue(0.0);
        ui->c_anchor_stacked_link_lineEdit_model->setText("");
        ui->c_anchor_stacked_link_lineEdit_link->setText("");
        ui->c_target_groupBox_lineEdit_model->setText("");
        ui->c_target_groupBox_lineEdit_link->setText("");
        ui->c_anchor_checkBox_worldframe->setChecked(false);
    }
}

// /////////////////////////////////////////////////
// bool ConfigureVirtualElementsDialog::SetModel(const std::string &_modelName)
// {
//     if (!gui::get_active_camera() || !gui::get_active_camera()->GetScene())
//         return false;

//     rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->GetVisual(_modelName);

//     if (!vis)
//     {
//         gzerr << "Model [" << _modelName << "] could not be found." << std::endl;
//         return false;
//     }

//     this->modelName = _modelName;

//     // Check if model/link hasn't been deleted on PreRender
//     this->connections.push_back(
//         event::Events::ConnectPreRender(
//             std::bind(&ConfigureVirtualElementsDialog::OnPreRender, this)));

//     this->modelLabel->setText(("<b>Model:</b> " + _modelName).c_str());

//     // Don't fire signals while inserting items
//     this->linksComboBox->blockSignals(true);
//     this->linksComboBox->clear();

//     for (unsigned int i = 0; i < vis->GetChildCount(); ++i)
//     {
//         rendering::VisualPtr childVis = vis->GetChild(i);
//         std::string linkName = childVis->Name();

//         // Issue #1553: This is failing to get real links sometimes:
//         // uint32_t flags = childVis->GetVisibilityFlags();
//         // if (!((flags != GZ_VISIBILITY_ALL) && (flags & GZ_VISIBILITY_GUI)))
//         if (linkName.find("_GL_MANIP_") == std::string::npos)
//         {
//             std::string unscopedLinkName = linkName.substr(linkName.find("::") + 2);
//             this->linksComboBox->addItem(
//                 QString::fromStdString(unscopedLinkName));

//             // Get CoM from link's COMVisual
//             for (unsigned int j = 0; j < childVis->GetChildCount(); ++j)
//             {
//                 rendering::COMVisualPtr comVis =
//                     std::dynamic_pointer_cast<rendering::COMVisual>(
//                         childVis->GetChild(j));

//                 if (comVis)
//                 {
//                     this->linkToCOMMap[linkName] = comVis->InertiaPose().Pos();
//                     break;
//                 }
//             }
//         }
//     }

//     // Sort alphabetically
//     QSortFilterProxyModel *proxy = new QSortFilterProxyModel(
//         this->linksComboBox);
//     proxy->setSourceModel(this->linksComboBox->model());
//     this->linksComboBox->model()->setParent(proxy);
//     this->linksComboBox->setModel(proxy);
//     this->linksComboBox->model()->sort(0);

//     this->linksComboBox->blockSignals(false);

//     if (this->linksComboBox->count() > 0)
//         return true;

//     gzerr << "Couldn't find links in model ' [" << _modelName << "]."
//           << std::endl;

//     return false;
// }

// /////////////////////////////////////////////////
// bool ConfigureVirtualElementsDialog::SetLink(const std::string &_linkName)
// {
//     if (!gui::get_active_camera() || !gui::get_active_camera()->GetScene())
//         return false;

//     // Select on combo box
//     std::string unscopedLinkName = _linkName.substr(_linkName.find("::") + 2);
//     int index = -1;
//     for (int i = 0; i < this->linksComboBox->count(); ++i)
//     {
//         if ((this->linksComboBox->itemText(i)).toStdString() ==
//             unscopedLinkName)
//         {
//             index = i;
//             break;
//         }
//     }
//     if (index == -1)
//     {
//         gzerr << "Link [" << _linkName << "] could not be found in the combo box."
//               << std::endl;
//         return false;
//     }
//     this->linksComboBox->setCurrentIndex(index);

//     // Visual
//     this->linkName = _linkName;
//     rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->GetVisual(this->linkName);

//     if (!vis)
//     {
//         gzerr << "A visual named [" << this->linkName
//               << "] could not be found." << std::endl;
//         return false;
//     }
//     this->linkVisual = vis;
//     this->AttachVisuals();

//     // Filter main window activate events
//     if (this->mainWindow)
//         this->mainWindow->installEventFilter(this);

//     // MouseRelease filter to gain focus
//     if (this->applyWrenchVisual)
//     {
//         MouseEventHandler::Instance()->AddReleaseFilter(
//             "dialog_" + this->applyWrenchVisual->Name(),
//             std::bind(&ConfigureVirtualElementsDialog::OnMouseRelease, this,
//                       std::placeholders::_1));
//     }

//     return true;
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::SetLink(const QString _linkName)
// {
//     // Remove previous link's filter
//     if (this->applyWrenchVisual)
//     {
//         MouseEventHandler::Instance()->RemoveReleaseFilter(
//             "dialog_" + this->applyWrenchVisual->Name());
//     }

//     if (!this->SetLink(this->modelName + "::" + _linkName.toStdString()))
//         this->Fini();
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnApplyAll()
// {
//     // Publish wrench message
//     msgs::Wrench msg;
//     msgs::Set(msg.mutable_force(), this->forceVector);
//     msgs::Set(msg.mutable_torque(), this->torqueVector);
//     msgs::Set(msg.mutable_force_offset(), this->forcePosVector);

//     // Register user command on server
//     // The wrench will be applied from the server
//     msgs::UserCmd userCmdMsg;
//     userCmdMsg.set_description("Apply wrench to [" + this->linkName +
//                                "]");
//     userCmdMsg.set_entity_name(this->linkName);
//     userCmdMsg.set_type(msgs::UserCmd::WRENCH);
//     userCmdMsg.mutable_wrench()->CopyFrom(msg);
//     this->userCmdPub->Publish(userCmdMsg);
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnApplyForce()
// {
//     // Publish wrench message
//     msgs::Wrench msg;
//     msgs::Set(msg.mutable_force(), this->forceVector);
//     msgs::Set(msg.mutable_torque(), ignition::math::Vector3d::Zero);
//     msgs::Set(msg.mutable_force_offset(), this->forcePosVector);

//     // Register user command on server
//     // The wrench will be applied from the server
//     msgs::UserCmd userCmdMsg;
//     userCmdMsg.set_description("Apply force to [" + this->linkName +
//                                "]");
//     userCmdMsg.set_entity_name(this->linkName);
//     userCmdMsg.set_type(msgs::UserCmd::WRENCH);
//     userCmdMsg.mutable_wrench()->CopyFrom(msg);
//     this->userCmdPub->Publish(userCmdMsg);
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnApplyTorque()
// {
//     // Publish wrench message
//     msgs::Wrench msg;
//     msgs::Set(msg.mutable_force(), ignition::math::Vector3d::Zero);
//     msgs::Set(msg.mutable_torque(), this->torqueVector);

//     // Register user command on server
//     // The wrench will be applied from the server
//     msgs::UserCmd userCmdMsg;
//     userCmdMsg.set_description("Apply torque to [" + this->linkName +
//                                "]");
//     userCmdMsg.set_entity_name(this->linkName);
//     userCmdMsg.set_type(msgs::UserCmd::WRENCH);
//     userCmdMsg.mutable_wrench()->CopyFrom(msg);
//     this->userCmdPub->Publish(userCmdMsg);
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnCancel()
// {
//     this->Fini();
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnForcePosChanged(double /*_value*/)
// {
//     // Update forcePos vector with values from XYZ spins
//     this->SetForcePos(
//         ignition::math::Vector3d(this->forcePosXSpin->value(),
//                                  this->forcePosYSpin->value(),
//                                  this->forcePosZSpin->value()));
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnForceMagChanged(double /*_magnitude*/)
// {
//     // Update force vector proportionally
//     // Normalize current vector
//     ignition::math::Vector3d v = this->forceVector;
//     if (v == ignition::math::Vector3d::Zero)
//         v = ignition::math::Vector3d::UnitX;
//     else
//         v.Normalize();

//     // Multiply by new magnitude
//     this->SetForce(v * this->forceMagSpin->value());
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnForceChanged(double /*_value*/)
// {
//     // Update force vector with values from XYZ spins
//     this->SetForce(ignition::math::Vector3d(this->forceXSpin->value(),
//                                             this->forceYSpin->value(),
//                                             this->forceZSpin->value()));
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnForceClear()
// {
//     this->SetForce(ignition::math::Vector3d::Zero);
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnTorqueMagChanged(double /*_magnitude*/)
// {
//     // Update torque vector proportionally
//     // Normalize current vector
//     ignition::math::Vector3d v = this->torqueVector;
//     if (v == ignition::math::Vector3d::Zero)
//         v = ignition::math::Vector3d::UnitX;
//     else
//         v.Normalize();

//     // Multiply by new magnitude
//     this->SetTorque(v * this->torqueMagSpin->value());
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnTorqueChanged(double /*_value*/)
// {
//     // Update torque vector with values from XYZ spins
//     this->SetTorque(ignition::math::Vector3d(this->torqueXSpin->value(),
//                                              this->torqueYSpin->value(),
//                                              this->torqueZSpin->value()));
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnTorqueClear()
// {
//     this->SetTorque(ignition::math::Vector3d::Zero);
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::ToggleComRadio(bool _checked)
// {
//     if (_checked)
//     {
//         this->SetForcePos(this->comVector);
//     }
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::SetSpinValue(QDoubleSpinBox *_spin, const double _value)
// {
//     _spin->blockSignals(true);
//     _spin->setValue(_value);
//     _spin->blockSignals(false);
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::SetForcePos(const ignition::math::Vector3d &_forcePos)
// {
//     this->forcePosVector = _forcePos;

//     // Spins
//     this->SetSpinValue(this->forcePosXSpin, _forcePos.X());
//     this->SetSpinValue(this->forcePosYSpin, _forcePos.Y());
//     this->SetSpinValue(this->forcePosZSpin, _forcePos.Z());

//     // Check COM box
//     if (_forcePos == this->comVector)
//     {
//         this->comRadio->setChecked(true);
//     }
//     else
//     {
//         this->forcePosRadio->setChecked(true);
//     }

//     // Visuals
//     if (!this->applyWrenchVisual)
//     {
//         gzwarn << "No wrench visual found, so it won't be updated" << std::endl;
//         return;
//     }

//     this->applyWrenchVisual->SetForcePos(
//         this->forcePosVector);
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::SetForce(const ignition::math::Vector3d &_force,
//                                               const bool _rotatedByMouse)
// {
//     // This can be called from the dialog or the mouse
//     std::lock_guard<std::mutex> lock(this->mutex);

//     this->forceVector = _force;

//     // Spins
//     this->SetSpinValue(this->forceXSpin, _force.X());
//     this->SetSpinValue(this->forceYSpin, _force.Y());
//     this->SetSpinValue(this->forceZSpin, _force.Z());
//     this->SetSpinValue(this->forceMagSpin, _force.Length());

//     // Mode
//     if (_force == ignition::math::Vector3d::Zero)
//     {
//         if (this->torqueVector == ignition::math::Vector3d::Zero)
//             this->SetMode(Mode::NONE);
//         else
//             this->SetMode(Mode::TORQUE);
//     }
//     else
//     {
//         this->SetMode(Mode::FORCE);
//     }

//     // Visuals
//     if (!this->applyWrenchVisual)
//     {
//         gzwarn << "No wrench visual found, so it won't be updated" << std::endl;
//         return;
//     }

//     this->applyWrenchVisual->SetForce(_force, _rotatedByMouse);
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::SetTorque(const ignition::math::Vector3d &_torque,
//                                                const bool _rotatedByMouse)
// {
//     // This can be called from the dialog or the mouse
//     std::lock_guard<std::mutex> lock(this->mutex);

//     this->torqueVector = _torque;

//     // Spins
//     this->SetSpinValue(this->torqueXSpin, _torque.X());
//     this->SetSpinValue(this->torqueYSpin, _torque.Y());
//     this->SetSpinValue(this->torqueZSpin, _torque.Z());
//     this->SetSpinValue(this->torqueMagSpin, _torque.Length());

//     // Mode
//     if (_torque == ignition::math::Vector3d::Zero)
//     {
//         if (this->forceVector == ignition::math::Vector3d::Zero)
//             this->SetMode(Mode::NONE);
//         else
//             this->SetMode(Mode::FORCE);
//     }
//     else
//     {
//         this->SetMode(Mode::TORQUE);
//     }

//     // Visuals
//     if (!this->applyWrenchVisual)
//     {
//         gzwarn << "No wrench visual found, so it won't be updated" << std::endl;
//         return;
//     }

//     this->applyWrenchVisual->SetTorque(_torque, _rotatedByMouse);
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::SetCoM(const ignition::math::Vector3d &_com)
// {
//     this->comVector = _com;

//     // Visuals
//     if (!this->applyWrenchVisual)
//     {
//         gzwarn << "No wrench visual found, so it won't be updated" << std::endl;
//         return;
//     }

//     this->applyWrenchVisual->SetCoM(this->comVector);
// }

/////////////////////////////////////////////////
void ConfigureVirtualElementsDialog::OnPreRender()
{
    // if (!gui::get_active_camera() || !gui::get_active_camera()->GetScene())
    //     return;

    // rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->GetVisual(this->linkName);

    // // Close dialog in case visual has been deleted
    // if (!vis)
    //     this->Fini();
}

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::AttachVisuals()
// {
//     if (!gui::get_active_camera() || !gui::get_active_camera()->GetScene())
//     {
//         gzerr << "Camera or scene missing" << std::endl;
//         return;
//     }
//     if (!this->linkVisual)
//     {
//         gzerr << "No link visual specified." << std::endl;
//         return;
//     }

//     // Attaching for the first time
//     if (!this->applyWrenchVisual)
//     {
//         // Generate unique name
//         std::string visNameBase = this->modelName + "__APPLY_WRENCH__";
//         rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->GetVisual(visNameBase);

//         std::string visName(visNameBase);
//         int count = 0;
//         while (vis)
//         {
//             visName = visNameBase + std::to_string(count);
//             vis = gui::get_active_camera()->GetScene()->GetVisual(visName);
//             ++count;
//         }

//         this->applyWrenchVisual.reset(new rendering::ApplyWrenchVisual(
//             visName, this->linkVisual));

//         this->applyWrenchVisual->Load();
//     }
//     // Different link
//     else if (!this->applyWrenchVisual->GetParent() ||
//              this->applyWrenchVisual->GetParent() !=
//                  this->linkVisual)
//     {
//         if (this->applyWrenchVisual->GetParent())
//         {
//             this->applyWrenchVisual->GetParent()->DetachVisual(
//                 this->applyWrenchVisual);
//         }
//         this->linkVisual->AttachVisual(this->applyWrenchVisual);
//         this->applyWrenchVisual->Resize();
//     }

//     if (!this->applyWrenchVisual)
//     {
//         gzwarn << "Failed to attach wrench visual. "
//                << "Dialog will work without it." << std::endl;
//     }

//     // Set COM
//     this->SetCoM(this->linkToCOMMap[this->linkName]);
//     // Apply force at com by default
//     this->SetForcePos(this->comVector);
//     this->SetTorque(this->torqueVector);
//     this->SetForce(this->forceVector);
// }

/////////////////////////////////////////////////
bool ConfigureVirtualElementsDialog::OnMousePress(const common::MouseEvent &_event)
{
    // rendering::UserCameraPtr userCamera = gui::get_active_camera();
    // if (!userCamera || !this->applyWrenchVisual)
    //     return false;

    // this->draggingTool = false;

    // rendering::VisualPtr vis = userCamera->Visual(_event.Pos(),
    //                                               this->manipState);

    // if (vis)
    //     return false;

    // // If on top of a circle handle
    // if (this->manipState == "rot_z" ||
    //     this->manipState == "rot_y")
    // {
    //     this->draggingTool = true;

    //     // Highlight dragged circle
    //     this->applyWrenchVisual->GetRotTool()->SetState(
    //         this->manipState);

    //     // Register rotTool pose at drag start
    //     this->dragStartPose =
    //         this->applyWrenchVisual->GetRotTool()->WorldPose();
    // }
    return false;
}

/////////////////////////////////////////////////
bool ConfigureVirtualElementsDialog::OnMouseRelease(const common::MouseEvent &_event)
{
    // rendering::UserCameraPtr userCamera = gui::get_active_camera();
    // if (!userCamera || !this->applyWrenchVisual)
    //     return false;

    // rendering::VisualPtr vis = userCamera->Visual(_event.Pos(),
    //                                               this->manipState);

    // if (!vis || _event.Dragging())
    //     return false;

    // // Force/torque clicked: activate dialog and prevent event propagation
    // if (vis == this->applyWrenchVisual->GetForceVisual())
    // {
    //     this->ActivateWindow();

    //     // Activate visual, can't attach rot tool with zero vector, UnitX by default
    //     if (this->forceVector == ignition::math::Vector3d::Zero)
    //         this->SetForce(ignition::math::Vector3d::UnitX);
    //     else
    //         this->SetForce(this->forceVector);

    //     return true;
    // }
    // else if (vis == this->applyWrenchVisual->GetTorqueVisual())
    // {
    //     this->ActivateWindow();

    //     // Activate visual, can't attach rot tool with zero vector, UnitX by default
    //     if (this->torqueVector == ignition::math::Vector3d::Zero)
    //         this->SetTorque(ignition::math::Vector3d::UnitX);
    //     else
    //         this->SetTorque(this->torqueVector);

    //     return true;
    // }
    return false;
}

/////////////////////////////////////////////////
bool ConfigureVirtualElementsDialog::OnMouseMove(const common::MouseEvent &_event)
{
    // rendering::UserCameraPtr userCamera = gui::get_active_camera();
    // if (!userCamera || !this->applyWrenchVisual)
    //     return false;

    // // Must make a Qt check as well because Gazebo event is not working with test
    // bool isDragging = _event.Dragging() ||
    //                   QApplication::mouseButtons() != Qt::NoButton;
    // bool isLeftButton = _event.Button() == common::MouseEvent::LEFT ||
    //                     QApplication::mouseButtons() == Qt::LeftButton;

    // // Dragging tool, adapted from ModelManipulator::RotateEntity
    // if (isDragging && isLeftButton && this->draggingTool)
    // {
    //     ignition::math::Vector3d normal;
    //     ignition::math::Vector3d axis;
    //     if (this->manipState == "rot_z")
    //     {
    //         normal = this->dragStartPose.Rot().ZAxis();
    //         axis = ignition::math::Vector3d::UnitZ;
    //     }
    //     else if (this->manipState == "rot_y")
    //     {
    //         normal = this->dragStartPose.Rot().YAxis();
    //         axis = ignition::math::Vector3d::UnitY;
    //     }
    //     else
    //     {
    //         gzerr << "Dragging tool on wrong manip state, this shouldn't happen" << std::endl;
    //         return false;
    //     }

    //     double offset = this->dragStartPose.Pos().Dot(normal);

    //     ignition::math::Vector3d pressPoint;
    //     userCamera->WorldPointOnPlane(_event.PressPos().X(),
    //                                   _event.PressPos().Y(),
    //                                   ignition::math::Planed(normal, offset), pressPoint);

    //     ignition::math::Vector3d newPoint;
    //     userCamera->WorldPointOnPlane(_event.Pos().X(), _event.Pos().Y(),
    //                                   ignition::math::Planed(normal, offset), newPoint);

    //     ignition::math::Vector3d v1 = pressPoint -
    //                                   this->dragStartPose.Pos();
    //     ignition::math::Vector3d v2 = newPoint -
    //                                   this->dragStartPose.Pos();
    //     v1 = v1.Normalize();
    //     v2 = v2.Normalize();
    //     double signTest = v1.Cross(v2).Dot(normal);
    //     double angle = atan2((v1.Cross(v2)).Length(), v1.Dot(v2));

    //     if (signTest < 0)
    //         angle *= -1;

    //     if (QApplication::keyboardModifiers() & Qt::ControlModifier)
    //         angle = rint(angle / (M_PI * 0.25)) * (M_PI * 0.25);

    //     ignition::math::Quaterniond rot(axis, angle);
    //     rot = this->dragStartPose.Rot() * rot;

    //     // Must rotate the tool here to make sure we have proper roll,
    //     // once the rotation gets transformed into a vector we lose a DOF
    //     this->applyWrenchVisual->GetRotTool()->SetWorldRotation(rot);

    //     // Get direction from tool orientation
    //     ignition::math::Vector3d vec;
    //     ignition::math::Vector3d rotEuler;
    //     rotEuler = rot.Euler();
    //     vec.X(cos(rotEuler.Z()) * cos(rotEuler.Y()));
    //     vec.Y(sin(rotEuler.Z()) * cos(rotEuler.Y()));
    //     vec.Z(-sin(rotEuler.Y()));

    //     // To local frame
    //     vec = this->linkVisual->WorldPose().Rot().RotateVectorReverse(
    //         vec);

    //     if (this->GetMode() == Mode::FORCE)
    //     {
    //         this->NewForceDirection(vec);
    //     }
    //     else if (this->GetMode() == Mode::TORQUE)
    //     {
    //         this->NewTorqueDirection(vec);
    //     }
    //     return true;
    // }
    // // Highlight hovered tools
    // else
    // {
    //     userCamera->Visual(_event.Pos(), this->manipState);

    //     if (this->manipState == "rot_z" ||
    //         this->manipState == "rot_y")
    //     {
    //         this->applyWrenchVisual->GetRotTool()->SetState(
    //             this->manipState);
    //     }
    //     else
    //     {
    //         this->applyWrenchVisual->GetRotTool()->SetState("");
    //     }
    // }

    return false;
}

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::SetMode(Mode _mode)
// {
//     this->mode = _mode;
// }

// /////////////////////////////////////////////////
// ConfigureVirtualElementsDialog::Mode ConfigureVirtualElementsDialog::GetMode() const
// {
//     return this->mode;
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::NewForceDirection(const ignition::math::Vector3d &_dir)
// {
//     // Normalize direction
//     ignition::math::Vector3d v = _dir;
//     if (v == ignition::math::Vector3d::Zero)
//         v = ignition::math::Vector3d::UnitX;
//     else
//         v.Normalize();

//     // Multiply by magnitude
//     this->SetForce(v * this->forceMagSpin->value(), true);
// }

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::NewTorqueDirection(const ignition::math::Vector3d &_dir)
// {
//     // Normalize direction
//     ignition::math::Vector3d v = _dir;
//     if (v == ignition::math::Vector3d::Zero)
//         v = ignition::math::Vector3d::UnitX;
//     else
//         v.Normalize();

//     // Multiply by magnitude
//     this->SetTorque(v * this->torqueMagSpin->value(), true);
// }

// /////////////////////////////////////////////////
void ConfigureVirtualElementsDialog::SetActive(bool _active)
{
    // if (!this->applyWrenchVisual)
    // {
    //     gzerr << "No apply wrench visual." << std::endl;
    //     this->Fini();
    //     return;
    // }
    // if (_active)
    // {
    //     // Set visible
    //     this->applyWrenchVisual->SetMode(
    //         static_cast<rendering::ApplyWrenchVisual::Mode>(this->GetMode()));

    //     // Set selected
    //     event::Events::setSelectedEntity(this->linkName, "normal");

    //     // Set arrow mode
    //     if (g_arrowAct)
    //         g_arrowAct->trigger();

    //     MouseEventHandler::Instance()->AddPressFilter(
    //         "dialog_" + this->applyWrenchVisual->Name(),
    //         std::bind(&ConfigureVirtualElementsDialog::OnMousePress, this,
    //                   std::placeholders::_1));

    //     MouseEventHandler::Instance()->AddMoveFilter(
    //         "dialog_" + this->applyWrenchVisual->Name(),
    //         std::bind(&ConfigureVirtualElementsDialog::OnMouseMove, this,
    //                   std::placeholders::_1));
    // }
    // else
    // {
    //     this->applyWrenchVisual->SetMode(
    //         rendering::ApplyWrenchVisual::Mode::NONE);

    //     MouseEventHandler::Instance()->RemovePressFilter(
    //         "dialog_" + this->applyWrenchVisual->Name());
    //     MouseEventHandler::Instance()->RemoveMoveFilter(
    //         "dialog_" + this->applyWrenchVisual->Name());
    // }
}

// /////////////////////////////////////////////////
// void ConfigureVirtualElementsDialog::OnManipulation()
// {
//     this->SetActive(false);
// }

/////////////////////////////////////////////////
void ConfigureVirtualElementsDialog::ActivateWindow()
{
    if (!this->isActiveWindow())
    {
        // Clear focus before activating not to trigger FucusIn
        QWidget *focusedWidget = this->focusWidget();
        if (focusedWidget)
            focusedWidget->clearFocus();

        this->activateWindow();
    }
}

/////////////////////////////////////////////////
bool ConfigureVirtualElementsDialog::eventFilter(QObject *_object, QEvent *_event)
{
    // // Attach rotation tool to focused mode
    // if (_event->type() == QEvent::FocusIn)
    // {
    //     if (_object == this->forceMagSpin ||
    //         _object == this->forceXSpin ||
    //         _object == this->forceYSpin ||
    //         _object == this->forceZSpin ||
    //         _object == this->forcePosXSpin ||
    //         _object == this->forcePosYSpin ||
    //         _object == this->forcePosZSpin ||
    //         (_object == this->linksComboBox &&
    //          this->mode != Mode::TORQUE))
    //     {
    //         this->SetForce(this->forceVector);
    //     }
    //     else if (_object == this->torqueMagSpin ||
    //              _object == this->torqueXSpin ||
    //              _object == this->torqueYSpin ||
    //              _object == this->torqueZSpin ||
    //              (_object == this->linksComboBox &&
    //               this->mode == Mode::TORQUE))
    //     {
    //         this->SetTorque(this->torqueVector);
    //     }
    // }
    // // Deactivate this when another dialog is focused
    // else if (_event->type() == QEvent::ActivationChange)
    // {
    //     if (!this->mainWindow)
    //         return false;

    //     if (_object == this->mainWindow)
    //     {
    //         if (!this->isActiveWindow() &&
    //             !this->mainWindow->isActiveWindow())
    //         {
    //             this->SetActive(false);
    //         }
    //     }
    // }
    // // Activate when changing spinboxes with mousewheel
    // else if (_event->type() == QEvent::Wheel)
    // {
    //     this->ActivateWindow();
    // }

    return false;
}

/////////////////////////////////////////////////
void ConfigureVirtualElementsDialog::changeEvent(QEvent *_event)
{
    // Focus in this dialog
    if (_event->type() == QEvent::ActivationChange)
    {
        // During tests it seems not to find main window, so this is true by default
        bool mainWindowActive = true;

        if (!this->mainWindow ||
            !this->mainWindow->isActiveWindow())
        {
            mainWindowActive = false;
        }

        this->SetActive(this->isActiveWindow() || mainWindowActive);
    }
}