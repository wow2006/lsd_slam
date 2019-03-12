#pragma once
#include <io_wrapper/output_3d_wrapper.h>
#include <QGLViewer/qglviewer.h>

namespace Wrapper {

class Viewer :  public QGLViewer ,public lsd_slam::Output3DWrapper {
Q_OBJECT
public:
  ~Viewer() override;

  void publishKeyframe(lsd_slam::Frame* pKeyFrame) override;

  void init() override;

  void draw() override;

signals:
  void close();

};

} // namespace Wrapper
