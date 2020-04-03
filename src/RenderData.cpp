////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RenderData.hpp"

#include "TileNode.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace csp::lodbodies {

////////////////////////////////////////////////////////////////////////////////////////////////////

/* virtual */
RenderData::~RenderData() = default;

////////////////////////////////////////////////////////////////////////////////////////////////////

TileId const& RenderData::getTileId() const {
  assert(mNode != nullptr);
  return mNode->getTileId();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/* explicit */
RenderData::RenderData(TileNode* node)
    : mNode(node)
    , mTexLayer(-1)
    , mLastFrame(-1) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

BoundingBox<double> const& RenderData::getBounds() const {
  return mTb;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderData::setBounds(BoundingBox<double> const& tb) {
  mTb        = tb;
  mHasBounds = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderData::removeBounds() {
  mTb        = BoundingBox<double>();
  mHasBounds = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool RenderData::hasBounds() const {
  return mHasBounds;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode* RenderData::getNode() const {
  return mNode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderData::setNode(TileNode* node) {
  mNode = node;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int RenderData::getLevel() const {
  return getTileId().level();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

glm::int64 RenderData::getPatchIdx() const {
  return getTileId().patchIdx();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int RenderData::getTexLayer() const {
  return mTexLayer;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderData::setTexLayer(int layer) {
  mTexLayer = layer;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int RenderData::getLastFrame() const {
  return mLastFrame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderData::setLastFrame(int frame) {
  mLastFrame = frame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int RenderData::getAge(int frame) const {
  return frame - mLastFrame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::lodbodies
