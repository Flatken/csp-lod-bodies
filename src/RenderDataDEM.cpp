////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RenderDataDEM.hpp"

#include "../../../src/cs-utils/utils.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace csp::lodplanets {

////////////////////////////////////////////////////////////////////////////////////////////////////

/* static */ RenderDataDEM* RenderDataDEM::create(TileNode* node) {
  return new RenderDataDEM(node);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/* explicit */
RenderDataDEM::RenderDataDEM(TileNode* node)
    : RenderData(node)
    , mLodDeltas()
    , mEdgeRData()
    , mFlags(0) {
  resetEdgeDeltas();
  resetEdgeRData();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/* virtual */
RenderDataDEM::~RenderDataDEM() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int RenderDataDEM::getEdgeDelta(int idx) const {
  return mLodDeltas[idx];
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderDataDEM::setEdgeDelta(int idx, int delta) {
  mLodDeltas[idx] = static_cast<glm::int8>(delta);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderDataDEM::resetEdgeDeltas() {
  for (std::size_t i = 0; i < mLodDeltas.size(); ++i)
    mLodDeltas[i] = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

RenderDataDEM* RenderDataDEM::getEdgeRData(int idx) const {
  return mEdgeRData[idx];
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderDataDEM::setEdgeRData(int idx, RenderDataDEM* rdata) {
  mEdgeRData[idx] = rdata;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderDataDEM::resetEdgeRData() {
  for (std::size_t i = 0; i < mEdgeRData.size(); ++i)
    mEdgeRData[i] = NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderDataDEM::addFlag(Flags flag) {
  mFlags |= static_cast<glm::uint8>(flag);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderDataDEM::subFlag(Flags flag) {
  mFlags &= ~static_cast<glm::uint8>(flag);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool RenderDataDEM::testFlag(Flags flag) {
  return (mFlags & static_cast<glm::uint8>(flag));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

glm::uint8 RenderDataDEM::getFlags() const {
  return mFlags;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RenderDataDEM::clearFlags() {
  mFlags = 0x00;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

RenderDataDEM::Flags operator&(RenderDataDEM::Flags lhs, RenderDataDEM::Flags rhs) {
  return RenderDataDEM::Flags(static_cast<int>(lhs) & static_cast<int>(rhs));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

RenderDataDEM::Flags operator|(RenderDataDEM::Flags lhs, RenderDataDEM::Flags rhs) {
  return RenderDataDEM::Flags(static_cast<int>(lhs) | static_cast<int>(rhs));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

RenderDataDEM::Flags& operator&=(RenderDataDEM::Flags& lhs, RenderDataDEM::Flags rhs) {
  return lhs = lhs & rhs;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

RenderDataDEM::Flags& operator|=(RenderDataDEM::Flags& lhs, RenderDataDEM::Flags rhs) {
  return lhs = lhs | rhs;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

RenderDataDEM::Flags operator~(RenderDataDEM::Flags lhs) {
  return RenderDataDEM::Flags(~static_cast<int>(lhs));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, RenderDataDEM::Flags flags) {
  os << "(";

  if (flags & cs::utils::enumCast(RenderDataDEM::Flags::eRender)) {
    os << "Render";
  }

  os << ")";

  return os;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::lodplanets
