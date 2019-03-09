#include <memory>

#include "qglViewer3DWrapper.hpp"
#include "ImageReader.hpp"

#include <QApplication>

#include <glog/logging.h>

int main(int argc, char **argv) {
  QApplication app(argc, argv);

  google::SetLogDestination(google::GLOG_INFO, "lsd_slam_info.log");
  google::InitGoogleLogging(argv[0]);

  auto pImageReader = std::make_unique<Application::ImageReader>();
  if(!pImageReader->initialize(argc, argv)) {
    return EXIT_FAILURE;
  }

  auto pViewer = std::make_unique<Wrapper::Viewer>();
  pViewer->show();

  pImageReader->setViewer(pViewer.get());

  QObject::connect(pViewer.get(), &Wrapper::Viewer::close,
                   pImageReader.get(), &Application::ImageReader::cleanup);

  return app.exec();
}
