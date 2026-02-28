#ifndef LRS_HALMSTAD_GUI_PLUGINS_UAV_SPAWNER_HH_
#define LRS_HALMSTAD_GUI_PLUGINS_UAV_SPAWNER_HH_

#include <memory>
#include <string>
#include <vector>

#include <QString>
#include <tinyxml2.h>

#include <gz/gui/Plugin.hh>
#include <gz/math/Pose3.hh>

class QProcess;

namespace lrs_halmstad_gui_plugins
{

class UavSpawner : public gz::gui::Plugin
{
  Q_OBJECT

  Q_PROPERTY(QString nextName READ NextName NOTIFY nextNameChanged)
  Q_PROPERTY(QString statusText READ StatusText NOTIFY statusTextChanged)
  Q_PROPERTY(bool busy READ Busy NOTIFY busyChanged)

  public: UavSpawner();
  public: ~UavSpawner() override;

  public: QString NextName() const;
  public: QString StatusText() const;
  public: bool Busy() const;

  public: Q_INVOKABLE void Spawn();

  signals: void nextNameChanged();
  signals: void statusTextChanged();
  signals: void busyChanged();

  protected: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  private: bool GenerateSdf(QString &_sdf, QString &_error) const;
  private: bool RequestSpawn(
      const QString &_sdf,
      const std::string &_name,
      QString &_error) const;
  private: bool EnsureSetPoseBridge(QString &_error);
  private: bool StartCameraBridge(const std::string &_name, QString &_error);
  private: std::string PackageExecutable(
      const std::string &_packageName,
      const std::string &_executableName,
      QString &_error) const;
  private: void StopBridgeProcess(QProcess *_process);
  private: std::string WorldName() const;
  private: void SetBusy(bool _busy);
  private: void SetStatus(const QString &_status);

  private: QString namePrefix_{"dji"};
  private: int nextIndex_{0};
  private: std::string robotType_{"m100"};
  private: bool withCamera_{true};
  private: bool modelStatic_{true};
  private: bool allowRenaming_{false};
  private: bool bridgeCamera_{true};
  private: bool bridgeSetPose_{true};
  private: std::string cameraName_{"camera0"};
  private: std::string relativeTo_{"world"};
  private: std::string configuredWorldName_;
  private: unsigned int requestTimeoutMs_{5000};
  private: gz::math::Pose3d spawnPose_{0.0, 0.0, 5.0, 0.0, 0.0, 0.0};
  private: QString statusText_{"Ready to spawn a UAV."};
  private: bool busy_{false};
  private: std::unique_ptr<QProcess> setPoseBridgeProcess_;
  private: std::vector<std::unique_ptr<QProcess>> cameraBridgeProcesses_;
};

}  // namespace lrs_halmstad_gui_plugins

#endif  // LRS_HALMSTAD_GUI_PLUGINS_UAV_SPAWNER_HH_
