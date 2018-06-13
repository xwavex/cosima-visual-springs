/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _GUI_COSIMA_VIRTUAL_ELEMTNS_WIDGET_HH_
#define _GUI_COSIMA_VIRTUAL_ELEMTNS_WIDGET_HH_

#include <QWidgetAction>
#include <QDialog>
#include <QLabel>
#include <QSplitter>
#include <QStackedWidget>
#include <QTreeWidgetItem>
#include <QGridLayout>
#include <QListWidgetItem>
// #include <QtWidgets/QWidgetAction>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
// #ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
// #include <gazebo/transport/transport.hh>

// #include <gazebo/gui/qt.h>
// #include <gazebo/gui/qt_test.h>
// #include <gazebo/gui/Conversions.hh>
// #include <gazebo/gui/EntityMaker.hh>
// #include <gazebo/gui/GuiIface.hh>
// #include <gazebo/gui/GuiEvents.hh>
// #include <gazebo/gui/GuiPlugin.hh>
// #include <gazebo/gui/GuiTypes.hh>
// #include <gazebo/gui/KeyEventHandler.hh>
// #include <gazebo/gui/LightMaker.hh>
// #include <gazebo/gui/ModelAlign.hh>
// #include <gazebo/gui/ModelManipulator.hh>
// #include <gazebo/gui/ModelSnap.hh>
// #include <gazebo/gui/MouseEventHandler.hh>
// #include <gazebo/gui/ModelMaker.hh>
// #include <gazebo/gui/SpaceNav.hh>
// #include <gazebo/gui/Actions.hh>
// #include <gazebo/gui/AlignWidget.hh>
// #include <gazebo/gui/ApplyWrenchDialog.hh>
// #include <gazebo/gui/CloneWindow.hh>
// #include <gazebo/gui/ConfigWidget.hh>
// #include <gazebo/gui/DataLogger.hh>
// #include <gazebo/gui/Editor.hh>
// #include <gazebo/gui/GLWidget.hh>
// #include <gazebo/gui/HotkeyDialog.hh>
// #include <gazebo/gui/InsertModelWidget.hh>
// #include <gazebo/gui/JointControlWidget.hh>
// #include <gazebo/gui/LayersWidget.hh>
// #include <gazebo/gui/MainWindow.hh>
// #include <gazebo/gui/ModelListWidget.hh>
// #include <gazebo/gui/ModelRightMenu.hh>
// #include <gazebo/gui/RenderWidget.hh>
// #include <gazebo/gui/SaveDialog.hh>
// #include <gazebo/gui/SplashScreen.hh>
// #include <gazebo/gui/TimePanel.hh>
// #include <gazebo/gui/TimeWidget.hh>
// #include <gazebo/gui/ToolsWidget.hh>
// #include <gazebo/gui/TopicSelector.hh>
// #include <gazebo/gui/TopToolbar.hh>
// #include <gazebo/gui/UserCmdHistory.hh>
// #include <gazebo/gui/ViewAngleWidget.hh>

#include <gazebo/transport/transport.hh>
#include "ConfigureVirtualElementsDialog.hh"

// #endif
using namespace gazebo;
namespace cosima
{
class ConfigureVirtualElementsDialogSpawner : public GUIPlugin
{
  Q_OBJECT

  /// \brief Constructor
  /// \param[in] _parent Parent widget
public:
  ConfigureVirtualElementsDialogSpawner();

  /// \brief Destructor
public:
  virtual ~ConfigureVirtualElementsDialogSpawner();

  /// \brief Callback trigged when the button is pressed.
protected slots:
  void OnButton();

public:
  gazebo::gui::ConfigureVirtualElementsDialog *ptr;
  void cb(ConstWorldStatisticsPtr &_msg);
  gazebo::transport::NodePtr node;
  gazebo::transport::SubscriberPtr sub;

  //     /// \brief Counter used to create unique model names
  //   private:
  //     unsigned int counter;

  //     /// \brief Node used to establish communication with gzserver.
  //   private:
  //     transport::NodePtr node;

  //     /// \brief Publisher of factory messages.
  //   private:
  //     transport::PublisherPtr factoryPub;
};
} // namespace cosima
#endif