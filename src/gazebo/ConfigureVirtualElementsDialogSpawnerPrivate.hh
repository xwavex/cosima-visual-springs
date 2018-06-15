#ifndef GAZEBO_GUI_ConfigureVirtualElementsDialogSpawnerPrivate_HH_
#define GAZEBO_GUI_ConfigureVirtualElementsDialogSpawnerPrivate_HH_
namespace cosima
{
class ConfigureVirtualElementsDialogSpawnerPrivate
{
public:
  ConfigureVirtualElementsDialogSpawnerPrivate() {}
  virtual void triggerSceneInfoRequest() = 0;
  virtual ~ConfigureVirtualElementsDialogSpawnerPrivate() {}
};
} // namespace cosima
#endif