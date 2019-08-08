////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RenderDataImg.hpp"

namespace csp::lodbodies {

////////////////////////////////////////////////////////////////////////////////////////////////////

/* static */ RenderDataImg* RenderDataImg::create(TileNode* node) {
  return new RenderDataImg(node);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/* explicit */
RenderDataImg::RenderDataImg(TileNode* node)
    : RenderData(node) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/* virtual */
RenderDataImg::~RenderDataImg() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::lodbodies
