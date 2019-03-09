#include "qglViewer3DWrapper.hpp"

#include "model/frame.h"

namespace Wrapper {
constexpr std::size_t cDepthIndex = 0;

Viewer::~Viewer() {
  emit close();
}

void Viewer::publishKeyframe(lsd_slam::Frame *pKeyFrame) {
    if(pKeyFrame == nullptr) {
        return;
    }

    if(pKeyFrame->hasIDepthBeenSet()) {
        const auto pDepthBuffer = pKeyFrame->idepth(cDepthIndex);
    }

    std::cout << __PRETTY_FUNCTION__ << '\n';
}

void Viewer::draw() {
}

} // namespace Wrapper
