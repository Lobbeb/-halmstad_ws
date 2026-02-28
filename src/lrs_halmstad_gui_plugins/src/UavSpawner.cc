#include "lrs_halmstad_gui_plugins/UavSpawner.hh"

#include <cstdlib>
#include <string>

#include <ament_index_cpp/get_package_prefix.hpp>

#include <QFileInfo>
#include <QProcess>
#include <QProcessEnvironment>
#include <QString>
#include <QStringList>

#include <gz/gui/qt.h>
#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

namespace lrs_halmstad_gui_plugins
{

namespace
{

std::string ReadText(const tinyxml2::XMLElement *_parent, const char *_name)
{
  if (_parent == nullptr)
  {
    return "";
  }

  const auto *elem = _parent->FirstChildElement(_name);
  if (elem == nullptr || elem->GetText() == nullptr)
  {
    return "";
  }

  return elem->GetText();
}

bool ReadBool(
    const tinyxml2::XMLElement *_parent,
    const char *_name,
    bool _fallback)
{
  if (_parent == nullptr)
  {
    return _fallback;
  }

  const auto *elem = _parent->FirstChildElement(_name);
  if (elem == nullptr)
  {
    return _fallback;
  }

  bool value = _fallback;
  if (elem->QueryBoolText(&value) == tinyxml2::XML_SUCCESS)
  {
    return value;
  }

  return _fallback;
}

double ReadDouble(
    const tinyxml2::XMLElement *_parent,
    const char *_name,
    double _fallback)
{
  if (_parent == nullptr)
  {
    return _fallback;
  }

  const auto *elem = _parent->FirstChildElement(_name);
  if (elem == nullptr)
  {
    return _fallback;
  }

  double value = _fallback;
  if (elem->QueryDoubleText(&value) == tinyxml2::XML_SUCCESS)
  {
    return value;
  }

  return _fallback;
}

int ReadInt(
    const tinyxml2::XMLElement *_parent,
    const char *_name,
    int _fallback)
{
  if (_parent == nullptr)
  {
    return _fallback;
  }

  const auto *elem = _parent->FirstChildElement(_name);
  if (elem == nullptr)
  {
    return _fallback;
  }

  int value = _fallback;
  if (elem->QueryIntText(&value) == tinyxml2::XML_SUCCESS)
  {
    return value;
  }

  return _fallback;
}

unsigned int ReadUnsignedInt(
    const tinyxml2::XMLElement *_parent,
    const char *_name,
    unsigned int _fallback)
{
  if (_parent == nullptr)
  {
    return _fallback;
  }

  const auto *elem = _parent->FirstChildElement(_name);
  if (elem == nullptr)
  {
    return _fallback;
  }

  unsigned int value = _fallback;
  if (elem->QueryUnsignedText(&value) == tinyxml2::XML_SUCCESS)
  {
    return value;
  }

  return _fallback;
}

std::string TrimmedSummary(const QString &_text)
{
  const auto simplified = _text.simplified();
  constexpr int kLimit = 180;

  if (simplified.size() <= kLimit)
  {
    return simplified.toStdString();
  }

  return simplified.left(kLimit).toStdString() + "...";
}

}  // namespace

UavSpawner::UavSpawner()
{
  this->title = "UAV spawner";
}

UavSpawner::~UavSpawner()
{
  this->StopBridgeProcess(this->setPoseBridgeProcess_.get());
  for (const auto &process : this->cameraBridgeProcesses_)
  {
    this->StopBridgeProcess(process.get());
  }
}

QString UavSpawner::NextName() const
{
  return QString("%1%2").arg(this->namePrefix_).arg(this->nextIndex_);
}

QString UavSpawner::StatusText() const
{
  return this->statusText_;
}

bool UavSpawner::Busy() const
{
  return this->busy_;
}

void UavSpawner::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (_pluginElem == nullptr)
  {
    return;
  }

  const auto namePrefix = ReadText(_pluginElem, "name_prefix");
  if (!namePrefix.empty())
  {
    this->namePrefix_ = QString::fromStdString(namePrefix);
  }

  this->nextIndex_ = ReadInt(_pluginElem, "start_index", this->nextIndex_);

  const auto robotType = ReadText(_pluginElem, "type");
  if (!robotType.empty())
  {
    this->robotType_ = robotType;
  }

  const auto cameraName = ReadText(_pluginElem, "camera_name");
  if (!cameraName.empty())
  {
    this->cameraName_ = cameraName;
  }

  const auto relativeTo = ReadText(_pluginElem, "relative_to");
  if (!relativeTo.empty())
  {
    this->relativeTo_ = relativeTo;
  }

  this->configuredWorldName_ = ReadText(_pluginElem, "world_name");
  this->withCamera_ = ReadBool(_pluginElem, "with_camera", this->withCamera_);
  this->modelStatic_ = ReadBool(_pluginElem, "model_static", this->modelStatic_);
  this->allowRenaming_ = ReadBool(_pluginElem, "allow_renaming", this->allowRenaming_);
  this->bridgeCamera_ = ReadBool(_pluginElem, "bridge_camera", this->bridgeCamera_);
  this->bridgeSetPose_ = ReadBool(_pluginElem, "bridge_set_pose", this->bridgeSetPose_);
  this->requestTimeoutMs_ =
      ReadUnsignedInt(_pluginElem, "request_timeout_ms", this->requestTimeoutMs_);

  this->spawnPose_.Set(
      ReadDouble(_pluginElem, "spawn_x", this->spawnPose_.Pos().X()),
      ReadDouble(_pluginElem, "spawn_y", this->spawnPose_.Pos().Y()),
      ReadDouble(_pluginElem, "spawn_z", this->spawnPose_.Pos().Z()),
      ReadDouble(_pluginElem, "spawn_roll", this->spawnPose_.Rot().Euler().X()),
      ReadDouble(_pluginElem, "spawn_pitch", this->spawnPose_.Rot().Euler().Y()),
      ReadDouble(_pluginElem, "spawn_yaw", this->spawnPose_.Rot().Euler().Z()));

  this->SetStatus(
      QString("Ready to spawn %1 at (%2, %3, %4).")
          .arg(this->NextName())
          .arg(this->spawnPose_.Pos().X(), 0, 'f', 1)
          .arg(this->spawnPose_.Pos().Y(), 0, 'f', 1)
          .arg(this->spawnPose_.Pos().Z(), 0, 'f', 1));
}

void UavSpawner::Spawn()
{
  if (this->busy_)
  {
    return;
  }

  const auto worldName = this->WorldName();
  if (worldName.empty())
  {
    this->SetStatus("World name is unset. Launch Gazebo through managed_clearpath_sim.");
    return;
  }

  const auto spawnName = this->NextName();
  QString sdf;
  QString error;

  this->SetBusy(true);
  this->SetStatus(QString("Generating %1...").arg(spawnName));

  if (!this->GenerateSdf(sdf, error))
  {
    this->SetStatus(error);
    this->SetBusy(false);
    return;
  }

  if (!this->RequestSpawn(sdf, spawnName.toStdString(), error))
  {
    this->SetStatus(error);
    this->SetBusy(false);
    return;
  }

  if (!this->EnsureSetPoseBridge(error))
  {
    this->SetStatus(error);
    this->SetBusy(false);
    return;
  }

  if (!this->StartCameraBridge(spawnName.toStdString(), error))
  {
    this->SetStatus(error);
    this->SetBusy(false);
    return;
  }

  ++this->nextIndex_;
  emit this->nextNameChanged();
  this->SetStatus(
      QString("Spawned %1 in %2 and bridged it to ROS 2.")
          .arg(spawnName)
          .arg(QString::fromStdString(worldName)));
  this->SetBusy(false);
}

bool UavSpawner::GenerateSdf(QString &_sdf, QString &_error) const
{
  std::string packagePrefix;
  try
  {
    packagePrefix = ament_index_cpp::get_package_prefix("lrs_halmstad");
  }
  catch (const std::exception &ex)
  {
    _error = QString("Failed to locate lrs_halmstad: %1").arg(ex.what());
    return false;
  }

  const QString program =
      QString::fromStdString(packagePrefix + "/lib/lrs_halmstad/generate_sdf");

  if (!QFileInfo::exists(program))
  {
    _error = QString("Missing generate_sdf executable at %1").arg(program);
    return false;
  }

  QProcess process;
  QStringList arguments{
      "--ros-args",
      "-p", QString("type:=%1").arg(QString::fromStdString(this->robotType_)),
      "-p", QString("name:=%1").arg(this->NextName()),
      "-p", "robot:=True",
      "-p", QString("with_camera:=%1").arg(this->withCamera_ ? "true" : "false"),
      "-p", QString("model_static:=%1").arg(this->modelStatic_ ? "true" : "false"),
      "-p", QString("camera_name:=%1").arg(QString::fromStdString(this->cameraName_)),
  };

  process.start(program, arguments);
  if (!process.waitForStarted(3000))
  {
    _error = QString("Failed to start %1").arg(program);
    return false;
  }

  if (!process.waitForFinished(10000))
  {
    process.kill();
    process.waitForFinished(1000);
    _error = "Timed out while generating UAV SDF.";
    return false;
  }

  const QString stdOut = QString::fromUtf8(process.readAllStandardOutput());
  const QString stdErr = QString::fromUtf8(process.readAllStandardError());

  if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
  {
    QString summary = stdErr.trimmed();
    if (summary.isEmpty())
    {
      summary = stdOut.trimmed();
    }
    _error = QString("generate_sdf failed: %1")
                 .arg(QString::fromStdString(TrimmedSummary(summary)));
    return false;
  }

  _sdf = stdOut.trimmed();
  if (_sdf.isEmpty())
  {
    _error = "generate_sdf returned an empty SDF string.";
    return false;
  }

  return true;
}

bool UavSpawner::RequestSpawn(
    const QString &_sdf,
    const std::string &_name,
    QString &_error) const
{
  const auto worldName = this->WorldName();
  if (worldName.empty())
  {
    _error = "World name is unset. Cannot call /world/<name>/create.";
    return false;
  }

  gz::msgs::EntityFactory request;
  request.set_name(_name);
  request.set_allow_renaming(this->allowRenaming_);
  request.set_sdf(_sdf.toStdString());
  request.set_relative_to(this->relativeTo_);
  gz::msgs::Set(request.mutable_pose(), this->spawnPose_);

  gz::msgs::Boolean reply;
  bool result = false;
  gz::transport::Node node;
  const std::string service = "/world/" + worldName + "/create";

  const bool executed =
      node.Request(service, request, this->requestTimeoutMs_, reply, result);

  if (!executed)
  {
    _error = QString("Timed out calling %1").arg(QString::fromStdString(service));
    return false;
  }

  if (!result)
  {
    _error = QString("%1 rejected the spawn request").arg(QString::fromStdString(service));
    return false;
  }

  if (!reply.data())
  {
    _error = QString("Gazebo did not create %1").arg(QString::fromStdString(_name));
    return false;
  }

  return true;
}

bool UavSpawner::EnsureSetPoseBridge(QString &_error)
{
  if (!this->bridgeSetPose_)
  {
    return true;
  }

  if (this->setPoseBridgeProcess_ != nullptr &&
      this->setPoseBridgeProcess_->state() != QProcess::NotRunning)
  {
    return true;
  }

  const auto worldName = this->WorldName();
  if (worldName.empty())
  {
    _error = "World name is unset. Cannot bridge /world/<name>/set_pose.";
    return false;
  }

  QString executableError;
  const auto executable =
      this->PackageExecutable("ros_gz_bridge", "parameter_bridge", executableError);
  if (executable.empty())
  {
    _error = executableError;
    return false;
  }

  auto process = std::make_unique<QProcess>();
  process->setProgram(QString::fromStdString(executable));
  process->setArguments(QStringList{
      QString("/world/%1/set_pose@ros_gz_interfaces/srv/SetEntityPose")
          .arg(QString::fromStdString(worldName)),
  });
  process->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  process->setProcessChannelMode(QProcess::MergedChannels);
  process->start();
  if (!process->waitForStarted(3000))
  {
    _error = "Failed to start ROS set_pose bridge.";
    return false;
  }

  this->setPoseBridgeProcess_ = std::move(process);
  return true;
}

bool UavSpawner::StartCameraBridge(const std::string &_name, QString &_error)
{
  if (!this->withCamera_ || !this->bridgeCamera_)
  {
    return true;
  }

  QString executableError;
  const auto executable =
      this->PackageExecutable("ros_gz_bridge", "parameter_bridge", executableError);
  if (executable.empty())
  {
    _error = executableError;
    return false;
  }

  const auto qName = QString::fromStdString(_name);
  const auto qCameraName = QString::fromStdString(this->cameraName_);

  auto process = std::make_unique<QProcess>();
  process->setProgram(QString::fromStdString(executable));
  process->setArguments(QStringList{
      QString("/%1/%2/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image")
          .arg(qName, qCameraName),
      QString("/%1/%2/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo")
          .arg(qName, qCameraName),
  });
  process->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  process->setProcessChannelMode(QProcess::MergedChannels);
  process->start();
  if (!process->waitForStarted(3000))
  {
    _error = QString("Failed to start ROS camera bridge for %1").arg(qName);
    return false;
  }

  this->cameraBridgeProcesses_.push_back(std::move(process));
  return true;
}

std::string UavSpawner::PackageExecutable(
    const std::string &_packageName,
    const std::string &_executableName,
    QString &_error) const
{
  std::string packagePrefix;
  try
  {
    packagePrefix = ament_index_cpp::get_package_prefix(_packageName);
  }
  catch (const std::exception &ex)
  {
    _error = QString("Failed to locate %1: %2")
                 .arg(QString::fromStdString(_packageName))
                 .arg(ex.what());
    return "";
  }

  const auto executablePath =
      packagePrefix + "/lib/" + _packageName + "/" + _executableName;

  if (!QFileInfo::exists(QString::fromStdString(executablePath)))
  {
    _error = QString("Missing executable %1")
                 .arg(QString::fromStdString(executablePath));
    return "";
  }

  return executablePath;
}

std::string UavSpawner::WorldName() const
{
  if (!this->configuredWorldName_.empty())
  {
    return this->configuredWorldName_;
  }

  const char *worldName = std::getenv("LRS_GAZEBO_WORLD");
  if (worldName == nullptr)
  {
    return "";
  }

  return worldName;
}

void UavSpawner::StopBridgeProcess(QProcess *_process)
{
  if (_process == nullptr || _process->state() == QProcess::NotRunning)
  {
    return;
  }

  _process->terminate();
  if (_process->waitForFinished(1000))
  {
    return;
  }

  _process->kill();
  _process->waitForFinished(1000);
}

void UavSpawner::SetBusy(bool _busy)
{
  if (this->busy_ == _busy)
  {
    return;
  }

  this->busy_ = _busy;
  emit this->busyChanged();
}

void UavSpawner::SetStatus(const QString &_status)
{
  if (this->statusText_ == _status)
  {
    return;
  }

  this->statusText_ = _status;
  emit this->statusTextChanged();
}

}  // namespace lrs_halmstad_gui_plugins

GZ_ADD_PLUGIN(
    lrs_halmstad_gui_plugins::UavSpawner,
    gz::gui::Plugin)

GZ_ADD_PLUGIN_ALIAS(lrs_halmstad_gui_plugins::UavSpawner, "UavSpawner")
