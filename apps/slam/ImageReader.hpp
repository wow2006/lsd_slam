#pragma once
#include <string>
#include <vector>
#include <atomic>
#include <thread>

#include <QObject>

namespace lsd_slam {
class Undistorter;
class SlamSystem;
}

namespace Application {

class ImageReader : public QObject {
  Q_OBJECT
public:
  explicit ImageReader();

  ~ImageReader();

  bool initialize(int argc, char **argv);

private:
  void run();


public slots:
  void cleanup();

protected:
  std::vector<std::string> m_vFiles;

  lsd_slam::Undistorter *m_pUndistorter = nullptr;

  std::unique_ptr<lsd_slam::SlamSystem> m_pSystem;

  std::thread mRunningThread;

  std::atomic<bool> m_bDone = false;
};

} // namespace Application
