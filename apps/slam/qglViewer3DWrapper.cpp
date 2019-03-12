#include "qglViewer3DWrapper.hpp"

#include "model/frame.h"

namespace Wrapper {
constexpr std::size_t cDepthIndex = 0;

Viewer::~Viewer() {
  emit close();
}

void Viewer::init() {
  glDisable(GL_LIGHTING);
  glPointSize(3.0);
  setGridIsDrawn();
}

void Viewer::publishKeyframe(lsd_slam::Frame *pKeyFrame) {
    if(pKeyFrame == nullptr) {
        return;
    }

    const auto cam = pKeyFrame->getScaledCamToWorld();
}

void Viewer::draw() {
  drawAxis();
}

} // namespace Wrapper
