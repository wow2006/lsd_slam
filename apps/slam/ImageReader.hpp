#pragma once
#include <string>
#include <vector>
#include <atomic>
#include <thread>

#include <QObject>

namespace lsd_slam {
class Undistorter;
class SlamSystem;
class Output3DWrapper;
}

namespace Application {

class ImageReader : public QObject {
  Q_OBJECT
public:
  explicit ImageReader();

  ~ImageReader() override;

  /** 
   * @brief Initialize SlamSystem
   * TODO:
   *  - remove argc, argv
   *  - remove program options with QApplication
   * @param argc
   * @param argv
   * 
   * @return initializtion successed
   */
  bool initialize(int argc, char **argv);

  void setViewer(lsd_slam::Output3DWrapper *pWrapper);

private:
  /** 
   * @brief Running Thread
   */
  void run();

public slots:
  /** 
   * @brief cleanup slot for close running thread
   */
  void cleanup();

protected:
  std::vector<std::string> m_vFiles;

  std::unique_ptr<lsd_slam::Undistorter> m_pUndistorter;

  std::unique_ptr<lsd_slam::SlamSystem> m_pSystem;

  std::thread mRunningThread;

  std::atomic<bool> m_bDone = false;
};

} // namespace Application
