/********************************************************************************
** Form generated from reading UI file 'configurevirtualelementsdialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONFIGUREVIRTUALELEMENTSDIALOG_H
#define UI_CONFIGUREVIRTUALELEMENTSDIALOG_H

// #include <QtCore/QVariant>
// #include <QtGui/QAction>
// #include <QtGui/QApplication>
// #include <QtGui/QButtonGroup>
// #include <QtGui/QCheckBox>
// #include <QtGui/QDialog>
// #include <QtGui/QDialogButtonBox>
// #include <QtGui/QDoubleSpinBox>
// #include <QtGui/QFormLayout>
// #include <QtGui/QGridLayout>
// #include <QtGui/QGroupBox>
// #include <QtGui/QHBoxLayout>
// #include <QtGui/QHeaderView>
// #include <QtGui/QLabel>
// #include <QtGui/QLineEdit>
// #include <QtGui/QRadioButton>
// #include <QtGui/QSpacerItem>
// #include <QtGui/QStackedWidget>
// #include <QtGui/QTabWidget>
// #include <QtGui/QVBoxLayout>
// #include <QtGui/QWidget>
#include <QVariant>
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QSpacerItem>
#include <QStackedWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

QT_BEGIN_NAMESPACE

class Ui_ConfigureVirtualElementsDialog
{
  public:
    QGridLayout *gridLayout_2;
    QVBoxLayout *verticalLayout;
    QTabWidget *tabs_main;
    QWidget *tab_spring_damper;
    QGroupBox *sd_target_groupBox;
    QVBoxLayout *verticalLayout_6;
    QGridLayout *gridLayout;
    QLabel *label;
    QLineEdit *sd_target_groupBox_lineEdit_model;
    QLabel *label_2;
    QLineEdit *sd_target_groupBox_lineEdit_link;
    QGroupBox *sd_stiffness_groupBox;
    QGridLayout *gridLayout_8;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_25;
    QDoubleSpinBox *sd_stiffness_groupBox_doubleSpinBox_X;
    QLabel *label_26;
    QDoubleSpinBox *sd_stiffness_groupBox_doubleSpinBox_Y;
    QLabel *label_27;
    QDoubleSpinBox *sd_stiffness_groupBox_doubleSpinBox_Z;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_28;
    QDoubleSpinBox *sd_stiffness_groupBox_doubleSpinBox_rR;
    QLabel *label_29;
    QDoubleSpinBox *sd_stiffness_groupBox_doubleSpinBox_rP;
    QLabel *label_30;
    QDoubleSpinBox *sd_stiffness_groupBox_doubleSpinBox_rY;
    QGroupBox *sd_damping_groupBox;
    QGridLayout *gridLayout_9;
    QVBoxLayout *verticalLayout_8;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_31;
    QDoubleSpinBox *sd_damping_groupBox_doubleSpinBox_X;
    QLabel *label_32;
    QDoubleSpinBox *sd_damping_groupBox_doubleSpinBox_Y;
    QLabel *label_33;
    QDoubleSpinBox *sd_damping_groupBox_doubleSpinBox_Z;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_34;
    QDoubleSpinBox *sd_damping_groupBox_doubleSpinBox_rR;
    QLabel *label_35;
    QDoubleSpinBox *sd_damping_groupBox_doubleSpinBox_rP;
    QLabel *label_36;
    QDoubleSpinBox *sd_damping_groupBox_doubleSpinBox_rY;
    QGroupBox *sd_anchor_groupBox;
    QVBoxLayout *verticalLayout_9;
    QStackedWidget *sd_anchor_stacked;
    QWidget *sd_anchor_stacked_link;
    QWidget *layoutWidget;
    QGridLayout *gridLayout_3;
    QLabel *label_3;
    QLineEdit *sd_anchor_stacked_link_lineEdit_model;
    QLabel *label_4;
    QLineEdit *sd_anchor_stacked_link_lineEdit_link;
    QWidget *sd_anchor_stacked_direction;
    QWidget *formLayoutWidget;
    QFormLayout *formLayout;
    QLabel *label_5;
    QDoubleSpinBox *sd_anchor_stacked_direction_doubleSpinBox_X;
    QLabel *label_6;
    QDoubleSpinBox *sd_anchor_stacked_direction_doubleSpinBox_Y;
    QLabel *label_7;
    QDoubleSpinBox *sd_anchor_stacked_direction_doubleSpinBox_Z;
    QHBoxLayout *sd_anchor_type_layout;
    QRadioButton *sd_anchor_radioButton_link;
    QRadioButton *sd_anchor_radioButton_direction;
    QSpacerItem *horizontalSpacer_3;
    QWidget *tab_constraint;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_10;
    QGridLayout *gridLayout_4;
    QLabel *label_8;
    QLineEdit *c_target_groupBox_lineEdit_model;
    QLabel *label_9;
    QLineEdit *c_target_groupBox_lineEdit_link;
    QGroupBox *groupBox_8;
    QVBoxLayout *verticalLayout_11;
    QStackedWidget *c_anchor_stacked;
    QWidget *page_3;
    QWidget *layoutWidget_3;
    QGridLayout *gridLayout_5;
    QLabel *label_10;
    QLineEdit *c_anchor_stacked_link_lineEdit_model;
    QLabel *label_11;
    QLineEdit *c_anchor_stacked_link_lineEdit_link;
    QWidget *page_4;
    QWidget *formLayoutWidget_2;
    QFormLayout *formLayout_2;
    QLabel *label_12;
    QDoubleSpinBox *c_anchor_stacked_direction_doubleSpinBox_X;
    QLabel *label_13;
    QDoubleSpinBox *c_anchor_stacked_direction_doubleSpinBox_Y;
    QLabel *label_14;
    QDoubleSpinBox *c_anchor_stacked_direction_doubleSpinBox_Z;
    QHBoxLayout *horizontalLayout_13;
    QRadioButton *c_anchor_radioButton_link;
    QRadioButton *c_anchor_radioButton_direction;
    QCheckBox *c_anchor_checkBox_worldframe;
    QHBoxLayout *horizontalLayout;
    QDialogButtonBox *btn_box_main;

    void setupUi(QDialog *ConfigureVirtualElementsDialog)
    {
        if (ConfigureVirtualElementsDialog->objectName().isEmpty())
            ConfigureVirtualElementsDialog->setObjectName(QString::fromUtf8("ConfigureVirtualElementsDialog"));
        ConfigureVirtualElementsDialog->setEnabled(true);
        ConfigureVirtualElementsDialog->resize(733, 444);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(ConfigureVirtualElementsDialog->sizePolicy().hasHeightForWidth());
        ConfigureVirtualElementsDialog->setSizePolicy(sizePolicy);
        ConfigureVirtualElementsDialog->setMinimumSize(QSize(733, 444));
        ConfigureVirtualElementsDialog->setMaximumSize(QSize(733, 444));
        gridLayout_2 = new QGridLayout(ConfigureVirtualElementsDialog);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabs_main = new QTabWidget(ConfigureVirtualElementsDialog);
        tabs_main->setObjectName(QString::fromUtf8("tabs_main"));
        tab_spring_damper = new QWidget();
        tab_spring_damper->setObjectName(QString::fromUtf8("tab_spring_damper"));
        sd_target_groupBox = new QGroupBox(tab_spring_damper);
        sd_target_groupBox->setObjectName(QString::fromUtf8("sd_target_groupBox"));
        sd_target_groupBox->setGeometry(QRect(10, 0, 271, 171));
        verticalLayout_6 = new QVBoxLayout(sd_target_groupBox);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label = new QLabel(sd_target_groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        sd_target_groupBox_lineEdit_model = new QLineEdit(sd_target_groupBox);
        sd_target_groupBox_lineEdit_model->setObjectName(QString::fromUtf8("sd_target_groupBox_lineEdit_model"));

        gridLayout->addWidget(sd_target_groupBox_lineEdit_model, 0, 1, 1, 1);

        label_2 = new QLabel(sd_target_groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        sd_target_groupBox_lineEdit_link = new QLineEdit(sd_target_groupBox);
        sd_target_groupBox_lineEdit_link->setObjectName(QString::fromUtf8("sd_target_groupBox_lineEdit_link"));

        gridLayout->addWidget(sd_target_groupBox_lineEdit_link, 1, 1, 1, 1);

        verticalLayout_6->addLayout(gridLayout);

        sd_stiffness_groupBox = new QGroupBox(tab_spring_damper);
        sd_stiffness_groupBox->setObjectName(QString::fromUtf8("sd_stiffness_groupBox"));
        sd_stiffness_groupBox->setGeometry(QRect(300, 0, 401, 171));
        gridLayout_8 = new QGridLayout(sd_stiffness_groupBox);
        gridLayout_8->setSpacing(6);
        gridLayout_8->setContentsMargins(11, 11, 11, 11);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(10);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_25 = new QLabel(sd_stiffness_groupBox);
        label_25->setObjectName(QString::fromUtf8("label_25"));

        horizontalLayout_8->addWidget(label_25);

        sd_stiffness_groupBox_doubleSpinBox_X = new QDoubleSpinBox(sd_stiffness_groupBox);
        sd_stiffness_groupBox_doubleSpinBox_X->setObjectName(QString::fromUtf8("sd_stiffness_groupBox_doubleSpinBox_X"));
        sd_stiffness_groupBox_doubleSpinBox_X->setMinimum(-9999.99);
        sd_stiffness_groupBox_doubleSpinBox_X->setMaximum(9999.99);

        horizontalLayout_8->addWidget(sd_stiffness_groupBox_doubleSpinBox_X);

        label_26 = new QLabel(sd_stiffness_groupBox);
        label_26->setObjectName(QString::fromUtf8("label_26"));

        horizontalLayout_8->addWidget(label_26);

        sd_stiffness_groupBox_doubleSpinBox_Y = new QDoubleSpinBox(sd_stiffness_groupBox);
        sd_stiffness_groupBox_doubleSpinBox_Y->setObjectName(QString::fromUtf8("sd_stiffness_groupBox_doubleSpinBox_Y"));
        sd_stiffness_groupBox_doubleSpinBox_Y->setMinimum(-9999.99);
        sd_stiffness_groupBox_doubleSpinBox_Y->setMaximum(9999.99);

        horizontalLayout_8->addWidget(sd_stiffness_groupBox_doubleSpinBox_Y);

        label_27 = new QLabel(sd_stiffness_groupBox);
        label_27->setObjectName(QString::fromUtf8("label_27"));

        horizontalLayout_8->addWidget(label_27);

        sd_stiffness_groupBox_doubleSpinBox_Z = new QDoubleSpinBox(sd_stiffness_groupBox);
        sd_stiffness_groupBox_doubleSpinBox_Z->setObjectName(QString::fromUtf8("sd_stiffness_groupBox_doubleSpinBox_Z"));
        sd_stiffness_groupBox_doubleSpinBox_Z->setMinimum(-9999.99);
        sd_stiffness_groupBox_doubleSpinBox_Z->setMaximum(9999.99);

        horizontalLayout_8->addWidget(sd_stiffness_groupBox_doubleSpinBox_Z);

        verticalLayout_7->addLayout(horizontalLayout_8);

        gridLayout_8->addLayout(verticalLayout_7, 0, 1, 1, 1);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(10);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_28 = new QLabel(sd_stiffness_groupBox);
        label_28->setObjectName(QString::fromUtf8("label_28"));

        horizontalLayout_9->addWidget(label_28);

        sd_stiffness_groupBox_doubleSpinBox_rR = new QDoubleSpinBox(sd_stiffness_groupBox);
        sd_stiffness_groupBox_doubleSpinBox_rR->setObjectName(QString::fromUtf8("sd_stiffness_groupBox_doubleSpinBox_rR"));
        sd_stiffness_groupBox_doubleSpinBox_rR->setMinimum(-9999.99);
        sd_stiffness_groupBox_doubleSpinBox_rR->setMaximum(9999.99);

        horizontalLayout_9->addWidget(sd_stiffness_groupBox_doubleSpinBox_rR);

        label_29 = new QLabel(sd_stiffness_groupBox);
        label_29->setObjectName(QString::fromUtf8("label_29"));

        horizontalLayout_9->addWidget(label_29);

        sd_stiffness_groupBox_doubleSpinBox_rP = new QDoubleSpinBox(sd_stiffness_groupBox);
        sd_stiffness_groupBox_doubleSpinBox_rP->setObjectName(QString::fromUtf8("sd_stiffness_groupBox_doubleSpinBox_rP"));
        sd_stiffness_groupBox_doubleSpinBox_rP->setMinimum(-9999.99);
        sd_stiffness_groupBox_doubleSpinBox_rP->setMaximum(9999.99);

        horizontalLayout_9->addWidget(sd_stiffness_groupBox_doubleSpinBox_rP);

        label_30 = new QLabel(sd_stiffness_groupBox);
        label_30->setObjectName(QString::fromUtf8("label_30"));

        horizontalLayout_9->addWidget(label_30);

        sd_stiffness_groupBox_doubleSpinBox_rY = new QDoubleSpinBox(sd_stiffness_groupBox);
        sd_stiffness_groupBox_doubleSpinBox_rY->setObjectName(QString::fromUtf8("sd_stiffness_groupBox_doubleSpinBox_rY"));
        sd_stiffness_groupBox_doubleSpinBox_rY->setMinimum(-9999.99);
        sd_stiffness_groupBox_doubleSpinBox_rY->setMaximum(9999.99);

        horizontalLayout_9->addWidget(sd_stiffness_groupBox_doubleSpinBox_rY);

        gridLayout_8->addLayout(horizontalLayout_9, 1, 1, 1, 1);

        sd_damping_groupBox = new QGroupBox(tab_spring_damper);
        sd_damping_groupBox->setObjectName(QString::fromUtf8("sd_damping_groupBox"));
        sd_damping_groupBox->setGeometry(QRect(300, 170, 401, 171));
        gridLayout_9 = new QGridLayout(sd_damping_groupBox);
        gridLayout_9->setSpacing(6);
        gridLayout_9->setContentsMargins(11, 11, 11, 11);
        gridLayout_9->setObjectName(QString::fromUtf8("gridLayout_9"));
        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(10);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_31 = new QLabel(sd_damping_groupBox);
        label_31->setObjectName(QString::fromUtf8("label_31"));

        horizontalLayout_10->addWidget(label_31);

        sd_damping_groupBox_doubleSpinBox_X = new QDoubleSpinBox(sd_damping_groupBox);
        sd_damping_groupBox_doubleSpinBox_X->setObjectName(QString::fromUtf8("sd_damping_groupBox_doubleSpinBox_X"));
        sd_damping_groupBox_doubleSpinBox_X->setMinimum(-9999.99);
        sd_damping_groupBox_doubleSpinBox_X->setMaximum(9999.99);

        horizontalLayout_10->addWidget(sd_damping_groupBox_doubleSpinBox_X);

        label_32 = new QLabel(sd_damping_groupBox);
        label_32->setObjectName(QString::fromUtf8("label_32"));

        horizontalLayout_10->addWidget(label_32);

        sd_damping_groupBox_doubleSpinBox_Y = new QDoubleSpinBox(sd_damping_groupBox);
        sd_damping_groupBox_doubleSpinBox_Y->setObjectName(QString::fromUtf8("sd_damping_groupBox_doubleSpinBox_Y"));
        sd_damping_groupBox_doubleSpinBox_Y->setMinimum(-9999.99);
        sd_damping_groupBox_doubleSpinBox_Y->setMaximum(9999.99);

        horizontalLayout_10->addWidget(sd_damping_groupBox_doubleSpinBox_Y);

        label_33 = new QLabel(sd_damping_groupBox);
        label_33->setObjectName(QString::fromUtf8("label_33"));

        horizontalLayout_10->addWidget(label_33);

        sd_damping_groupBox_doubleSpinBox_Z = new QDoubleSpinBox(sd_damping_groupBox);
        sd_damping_groupBox_doubleSpinBox_Z->setObjectName(QString::fromUtf8("sd_damping_groupBox_doubleSpinBox_Z"));
        sd_damping_groupBox_doubleSpinBox_Z->setMinimum(-9999.99);
        sd_damping_groupBox_doubleSpinBox_Z->setMaximum(9999.99);

        horizontalLayout_10->addWidget(sd_damping_groupBox_doubleSpinBox_Z);

        verticalLayout_8->addLayout(horizontalLayout_10);

        gridLayout_9->addLayout(verticalLayout_8, 0, 1, 1, 1);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(10);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_34 = new QLabel(sd_damping_groupBox);
        label_34->setObjectName(QString::fromUtf8("label_34"));

        horizontalLayout_11->addWidget(label_34);

        sd_damping_groupBox_doubleSpinBox_rR = new QDoubleSpinBox(sd_damping_groupBox);
        sd_damping_groupBox_doubleSpinBox_rR->setObjectName(QString::fromUtf8("sd_damping_groupBox_doubleSpinBox_rR"));
        sd_damping_groupBox_doubleSpinBox_rR->setMinimum(-9999.99);
        sd_damping_groupBox_doubleSpinBox_rR->setMaximum(9999.99);

        horizontalLayout_11->addWidget(sd_damping_groupBox_doubleSpinBox_rR);

        label_35 = new QLabel(sd_damping_groupBox);
        label_35->setObjectName(QString::fromUtf8("label_35"));

        horizontalLayout_11->addWidget(label_35);

        sd_damping_groupBox_doubleSpinBox_rP = new QDoubleSpinBox(sd_damping_groupBox);
        sd_damping_groupBox_doubleSpinBox_rP->setObjectName(QString::fromUtf8("sd_damping_groupBox_doubleSpinBox_rP"));
        sd_damping_groupBox_doubleSpinBox_rP->setMinimum(-9999.99);
        sd_damping_groupBox_doubleSpinBox_rP->setMaximum(9999.99);

        horizontalLayout_11->addWidget(sd_damping_groupBox_doubleSpinBox_rP);

        label_36 = new QLabel(sd_damping_groupBox);
        label_36->setObjectName(QString::fromUtf8("label_36"));

        horizontalLayout_11->addWidget(label_36);

        sd_damping_groupBox_doubleSpinBox_rY = new QDoubleSpinBox(sd_damping_groupBox);
        sd_damping_groupBox_doubleSpinBox_rY->setObjectName(QString::fromUtf8("sd_damping_groupBox_doubleSpinBox_rY"));
        sd_damping_groupBox_doubleSpinBox_rY->setMinimum(-9999.99);
        sd_damping_groupBox_doubleSpinBox_rY->setMaximum(9999.99);

        horizontalLayout_11->addWidget(sd_damping_groupBox_doubleSpinBox_rY);

        gridLayout_9->addLayout(horizontalLayout_11, 1, 1, 1, 1);

        sd_anchor_groupBox = new QGroupBox(tab_spring_damper);
        sd_anchor_groupBox->setObjectName(QString::fromUtf8("sd_anchor_groupBox"));
        sd_anchor_groupBox->setGeometry(QRect(10, 170, 271, 181));
        verticalLayout_9 = new QVBoxLayout(sd_anchor_groupBox);
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setContentsMargins(11, 11, 11, 11);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        sd_anchor_stacked = new QStackedWidget(sd_anchor_groupBox);
        sd_anchor_stacked->setObjectName(QString::fromUtf8("sd_anchor_stacked"));
        sd_anchor_stacked_link = new QWidget();
        sd_anchor_stacked_link->setObjectName(QString::fromUtf8("sd_anchor_stacked_link"));
        layoutWidget = new QWidget(sd_anchor_stacked_link);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(0, 30, 241, 58));
        gridLayout_3 = new QGridLayout(layoutWidget);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_3->addWidget(label_3, 0, 0, 1, 1);

        sd_anchor_stacked_link_lineEdit_model = new QLineEdit(layoutWidget);
        sd_anchor_stacked_link_lineEdit_model->setObjectName(QString::fromUtf8("sd_anchor_stacked_link_lineEdit_model"));

        gridLayout_3->addWidget(sd_anchor_stacked_link_lineEdit_model, 0, 1, 1, 1);

        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_3->addWidget(label_4, 1, 0, 1, 1);

        sd_anchor_stacked_link_lineEdit_link = new QLineEdit(layoutWidget);
        sd_anchor_stacked_link_lineEdit_link->setObjectName(QString::fromUtf8("sd_anchor_stacked_link_lineEdit_link"));

        gridLayout_3->addWidget(sd_anchor_stacked_link_lineEdit_link, 1, 1, 1, 1);

        sd_anchor_stacked->addWidget(sd_anchor_stacked_link);
        sd_anchor_stacked_direction = new QWidget();
        sd_anchor_stacked_direction->setObjectName(QString::fromUtf8("sd_anchor_stacked_direction"));
        formLayoutWidget = new QWidget(sd_anchor_stacked_direction);
        formLayoutWidget->setObjectName(QString::fromUtf8("formLayoutWidget"));
        formLayoutWidget->setGeometry(QRect(9, 9, 111, 92));
        formLayout = new QFormLayout(formLayoutWidget);
        formLayout->setSpacing(6);
        formLayout->setContentsMargins(11, 11, 11, 11);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        label_5 = new QLabel(formLayoutWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_5);

        sd_anchor_stacked_direction_doubleSpinBox_X = new QDoubleSpinBox(formLayoutWidget);
        sd_anchor_stacked_direction_doubleSpinBox_X->setObjectName(QString::fromUtf8("sd_anchor_stacked_direction_doubleSpinBox_X"));

        formLayout->setWidget(0, QFormLayout::FieldRole, sd_anchor_stacked_direction_doubleSpinBox_X);

        label_6 = new QLabel(formLayoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_6);

        sd_anchor_stacked_direction_doubleSpinBox_Y = new QDoubleSpinBox(formLayoutWidget);
        sd_anchor_stacked_direction_doubleSpinBox_Y->setObjectName(QString::fromUtf8("sd_anchor_stacked_direction_doubleSpinBox_Y"));

        formLayout->setWidget(1, QFormLayout::FieldRole, sd_anchor_stacked_direction_doubleSpinBox_Y);

        label_7 = new QLabel(formLayoutWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_7);

        sd_anchor_stacked_direction_doubleSpinBox_Z = new QDoubleSpinBox(formLayoutWidget);
        sd_anchor_stacked_direction_doubleSpinBox_Z->setObjectName(QString::fromUtf8("sd_anchor_stacked_direction_doubleSpinBox_Z"));

        formLayout->setWidget(2, QFormLayout::FieldRole, sd_anchor_stacked_direction_doubleSpinBox_Z);

        sd_anchor_stacked->addWidget(sd_anchor_stacked_direction);

        verticalLayout_9->addWidget(sd_anchor_stacked);

        sd_anchor_type_layout = new QHBoxLayout();
        sd_anchor_type_layout->setSpacing(10);
        sd_anchor_type_layout->setObjectName(QString::fromUtf8("sd_anchor_type_layout"));
        sd_anchor_radioButton_link = new QRadioButton(sd_anchor_groupBox);
        sd_anchor_radioButton_link->setObjectName(QString::fromUtf8("sd_anchor_radioButton_link"));
        sd_anchor_radioButton_link->setChecked(true);

        sd_anchor_type_layout->addWidget(sd_anchor_radioButton_link);

        sd_anchor_radioButton_direction = new QRadioButton(sd_anchor_groupBox);
        sd_anchor_radioButton_direction->setObjectName(QString::fromUtf8("sd_anchor_radioButton_direction"));

        sd_anchor_type_layout->addWidget(sd_anchor_radioButton_direction);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        sd_anchor_type_layout->addItem(horizontalSpacer_3);

        verticalLayout_9->addLayout(sd_anchor_type_layout);

        tabs_main->addTab(tab_spring_damper, QString());
        tab_constraint = new QWidget();
        tab_constraint->setObjectName(QString::fromUtf8("tab_constraint"));
        groupBox_2 = new QGroupBox(tab_constraint);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 0, 271, 171));
        verticalLayout_10 = new QVBoxLayout(groupBox_2);
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setContentsMargins(11, 11, 11, 11);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        gridLayout_4 = new QGridLayout();
        gridLayout_4->setSpacing(6);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        label_8 = new QLabel(groupBox_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_4->addWidget(label_8, 0, 0, 1, 1);

        c_target_groupBox_lineEdit_model = new QLineEdit(groupBox_2);
        c_target_groupBox_lineEdit_model->setObjectName(QString::fromUtf8("c_target_groupBox_lineEdit_model"));

        gridLayout_4->addWidget(c_target_groupBox_lineEdit_model, 0, 1, 1, 1);

        label_9 = new QLabel(groupBox_2);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_4->addWidget(label_9, 1, 0, 1, 1);

        c_target_groupBox_lineEdit_link = new QLineEdit(groupBox_2);
        c_target_groupBox_lineEdit_link->setObjectName(QString::fromUtf8("c_target_groupBox_lineEdit_link"));

        gridLayout_4->addWidget(c_target_groupBox_lineEdit_link, 1, 1, 1, 1);

        verticalLayout_10->addLayout(gridLayout_4);

        groupBox_8 = new QGroupBox(tab_constraint);
        groupBox_8->setObjectName(QString::fromUtf8("groupBox_8"));
        groupBox_8->setGeometry(QRect(10, 170, 341, 181));
        verticalLayout_11 = new QVBoxLayout(groupBox_8);
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setContentsMargins(11, 11, 11, 11);
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
        c_anchor_stacked = new QStackedWidget(groupBox_8);
        c_anchor_stacked->setObjectName(QString::fromUtf8("c_anchor_stacked"));
        page_3 = new QWidget();
        page_3->setObjectName(QString::fromUtf8("page_3"));
        layoutWidget_3 = new QWidget(page_3);
        layoutWidget_3->setObjectName(QString::fromUtf8("layoutWidget_3"));
        layoutWidget_3->setGeometry(QRect(0, 30, 241, 58));
        gridLayout_5 = new QGridLayout(layoutWidget_3);
        gridLayout_5->setSpacing(6);
        gridLayout_5->setContentsMargins(11, 11, 11, 11);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        label_10 = new QLabel(layoutWidget_3);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_5->addWidget(label_10, 0, 0, 1, 1);

        c_anchor_stacked_link_lineEdit_model = new QLineEdit(layoutWidget_3);
        c_anchor_stacked_link_lineEdit_model->setObjectName(QString::fromUtf8("c_anchor_stacked_link_lineEdit_model"));

        gridLayout_5->addWidget(c_anchor_stacked_link_lineEdit_model, 0, 1, 1, 1);

        label_11 = new QLabel(layoutWidget_3);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_5->addWidget(label_11, 1, 0, 1, 1);

        c_anchor_stacked_link_lineEdit_link = new QLineEdit(layoutWidget_3);
        c_anchor_stacked_link_lineEdit_link->setObjectName(QString::fromUtf8("c_anchor_stacked_link_lineEdit_link"));

        gridLayout_5->addWidget(c_anchor_stacked_link_lineEdit_link, 1, 1, 1, 1);

        c_anchor_stacked->addWidget(page_3);
        page_4 = new QWidget();
        page_4->setObjectName(QString::fromUtf8("page_4"));
        formLayoutWidget_2 = new QWidget(page_4);
        formLayoutWidget_2->setObjectName(QString::fromUtf8("formLayoutWidget_2"));
        formLayoutWidget_2->setGeometry(QRect(9, 9, 111, 92));
        formLayout_2 = new QFormLayout(formLayoutWidget_2);
        formLayout_2->setSpacing(6);
        formLayout_2->setContentsMargins(11, 11, 11, 11);
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        formLayout_2->setContentsMargins(0, 0, 0, 0);
        label_12 = new QLabel(formLayoutWidget_2);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_12);

        c_anchor_stacked_direction_doubleSpinBox_X = new QDoubleSpinBox(formLayoutWidget_2);
        c_anchor_stacked_direction_doubleSpinBox_X->setObjectName(QString::fromUtf8("c_anchor_stacked_direction_doubleSpinBox_X"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, c_anchor_stacked_direction_doubleSpinBox_X);

        label_13 = new QLabel(formLayoutWidget_2);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        formLayout_2->setWidget(1, QFormLayout::LabelRole, label_13);

        c_anchor_stacked_direction_doubleSpinBox_Y = new QDoubleSpinBox(formLayoutWidget_2);
        c_anchor_stacked_direction_doubleSpinBox_Y->setObjectName(QString::fromUtf8("c_anchor_stacked_direction_doubleSpinBox_Y"));

        formLayout_2->setWidget(1, QFormLayout::FieldRole, c_anchor_stacked_direction_doubleSpinBox_Y);

        label_14 = new QLabel(formLayoutWidget_2);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        formLayout_2->setWidget(2, QFormLayout::LabelRole, label_14);

        c_anchor_stacked_direction_doubleSpinBox_Z = new QDoubleSpinBox(formLayoutWidget_2);
        c_anchor_stacked_direction_doubleSpinBox_Z->setObjectName(QString::fromUtf8("c_anchor_stacked_direction_doubleSpinBox_Z"));

        formLayout_2->setWidget(2, QFormLayout::FieldRole, c_anchor_stacked_direction_doubleSpinBox_Z);

        c_anchor_stacked->addWidget(page_4);

        verticalLayout_11->addWidget(c_anchor_stacked);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setSpacing(10);
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        c_anchor_radioButton_link = new QRadioButton(groupBox_8);
        c_anchor_radioButton_link->setObjectName(QString::fromUtf8("c_anchor_radioButton_link"));
        c_anchor_radioButton_link->setChecked(true);

        horizontalLayout_13->addWidget(c_anchor_radioButton_link);

        c_anchor_radioButton_direction = new QRadioButton(groupBox_8);
        c_anchor_radioButton_direction->setObjectName(QString::fromUtf8("c_anchor_radioButton_direction"));

        horizontalLayout_13->addWidget(c_anchor_radioButton_direction);

        c_anchor_checkBox_worldframe = new QCheckBox(groupBox_8);
        c_anchor_checkBox_worldframe->setObjectName(QString::fromUtf8("c_anchor_checkBox_worldframe"));

        horizontalLayout_13->addWidget(c_anchor_checkBox_worldframe);

        verticalLayout_11->addLayout(horizontalLayout_13);

        tabs_main->addTab(tab_constraint, QString());

        verticalLayout->addWidget(tabs_main);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        btn_box_main = new QDialogButtonBox(ConfigureVirtualElementsDialog);
        btn_box_main->setObjectName(QString::fromUtf8("btn_box_main"));
        btn_box_main->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok | QDialogButtonBox::Reset);
        btn_box_main->setCenterButtons(false);

        horizontalLayout->addWidget(btn_box_main);

        verticalLayout->addLayout(horizontalLayout);

        gridLayout_2->addLayout(verticalLayout, 0, 0, 1, 1);

        retranslateUi(ConfigureVirtualElementsDialog);
        QObject::connect(btn_box_main, SIGNAL(accepted()), ConfigureVirtualElementsDialog, SLOT(accept()));
        QObject::connect(btn_box_main, SIGNAL(rejected()), ConfigureVirtualElementsDialog, SLOT(reject()));

        tabs_main->setCurrentIndex(0);
        sd_anchor_stacked->setCurrentIndex(0);
        c_anchor_stacked->setCurrentIndex(0);

        QMetaObject::connectSlotsByName(ConfigureVirtualElementsDialog);
    } // setupUi

    void retranslateUi(QDialog *ConfigureVirtualElementsDialog)
    {
        ConfigureVirtualElementsDialog->setWindowTitle(QApplication::translate("ConfigureVirtualElementsDialog", "CoSiMA Virtual Element Plugin", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_ACCESSIBILITY
        tab_spring_damper->setAccessibleName(QString());
#endif // QT_NO_ACCESSIBILITY
        sd_target_groupBox->setTitle(QApplication::translate("ConfigureVirtualElementsDialog", "Target", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Model:", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Link:", 0, QApplication::UnicodeUTF8));
        sd_stiffness_groupBox->setTitle(QApplication::translate("ConfigureVirtualElementsDialog", "Stiffness", 0, QApplication::UnicodeUTF8));
        label_25->setText(QApplication::translate("ConfigureVirtualElementsDialog", "X", 0, QApplication::UnicodeUTF8));
        label_26->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Y", 0, QApplication::UnicodeUTF8));
        label_27->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Z", 0, QApplication::UnicodeUTF8));
        label_28->setText(QApplication::translate("ConfigureVirtualElementsDialog", "R", 0, QApplication::UnicodeUTF8));
        label_29->setText(QApplication::translate("ConfigureVirtualElementsDialog", "P", 0, QApplication::UnicodeUTF8));
        label_30->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Y", 0, QApplication::UnicodeUTF8));
        sd_damping_groupBox->setTitle(QApplication::translate("ConfigureVirtualElementsDialog", "Damping", 0, QApplication::UnicodeUTF8));
        label_31->setText(QApplication::translate("ConfigureVirtualElementsDialog", "X", 0, QApplication::UnicodeUTF8));
        label_32->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Y", 0, QApplication::UnicodeUTF8));
        label_33->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Z", 0, QApplication::UnicodeUTF8));
        label_34->setText(QApplication::translate("ConfigureVirtualElementsDialog", "R", 0, QApplication::UnicodeUTF8));
        label_35->setText(QApplication::translate("ConfigureVirtualElementsDialog", "P", 0, QApplication::UnicodeUTF8));
        label_36->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Y", 0, QApplication::UnicodeUTF8));
        sd_anchor_groupBox->setTitle(QApplication::translate("ConfigureVirtualElementsDialog", "Anchor", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Model:", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Link:", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("ConfigureVirtualElementsDialog", "X", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Y", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Z", 0, QApplication::UnicodeUTF8));
        sd_anchor_radioButton_link->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Link", 0, QApplication::UnicodeUTF8));
        sd_anchor_radioButton_direction->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Direction", 0, QApplication::UnicodeUTF8));
        tabs_main->setTabText(tabs_main->indexOf(tab_spring_damper), QApplication::translate("ConfigureVirtualElementsDialog", "Spring-Damper", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("ConfigureVirtualElementsDialog", "Target", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Model:", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Link:", 0, QApplication::UnicodeUTF8));
        groupBox_8->setTitle(QApplication::translate("ConfigureVirtualElementsDialog", "Anchor", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Model:", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Link:", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("ConfigureVirtualElementsDialog", "X", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Y", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Z", 0, QApplication::UnicodeUTF8));
        c_anchor_radioButton_link->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Link", 0, QApplication::UnicodeUTF8));
        c_anchor_radioButton_direction->setText(QApplication::translate("ConfigureVirtualElementsDialog", "Direction", 0, QApplication::UnicodeUTF8));
        c_anchor_checkBox_worldframe->setText(QApplication::translate("ConfigureVirtualElementsDialog", "WorldFrame", 0, QApplication::UnicodeUTF8));
        tabs_main->setTabText(tabs_main->indexOf(tab_constraint), QApplication::translate("ConfigureVirtualElementsDialog", "Constraint", 0, QApplication::UnicodeUTF8));
    } // retranslateUi
};

namespace Ui
{
class ConfigureVirtualElementsDialog : public Ui_ConfigureVirtualElementsDialog
{
};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONFIGUREVIRTUALELEMENTSDIALOG_H
