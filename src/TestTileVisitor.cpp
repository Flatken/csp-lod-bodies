////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "TestTileVisitor.hpp"

namespace csp::lodplanets {

////////////////////////////////////////////////////////////////////////////////////////////////////

/* explicit */
TestTileVisitor::TestTileVisitor(TileQuadTree* treeDEM, TileQuadTree* treeIMG)
    : TileVisitor<TestTileVisitor>(treeDEM, treeIMG)
    , mLoadTilesDEM()
    , mLoadTilesIMG() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<TileId> const& TestTileVisitor::getLoadTilesDEM() const {
  return mLoadTilesDEM;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<TileId> const& TestTileVisitor::getLoadTilesIMG() const {
  return mLoadTilesIMG;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool TestTileVisitor::preTraverse() {
  mLoadTilesDEM.clear();
  mLoadTilesIMG.clear();

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool TestTileVisitor::preVisitRoot(TileId const& tileId) {
  bool      result  = true;
  TileNode* nodeDEM = getNodeDEM();
  TileNode* nodeIMG = getNodeIMG();

  if (!nodeDEM) {
    mLoadTilesDEM.push_back(tileId);
    result = false;
  }

  if (!nodeIMG) {
    mLoadTilesIMG.push_back(tileId);
    result = false;
  }

  if (result)
    result = visitLevel(tileId);

  return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool TestTileVisitor::preVisit(TileId const& tileId) {
  return visitLevel(tileId);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TestTileVisitor::postVisit(TileId const& tileId) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool TestTileVisitor::visitLevel(TileId const& tileId) {
  bool result = false;
  bool refine = refineTile();

  if (refine) {
    for (int i = 0; i < 4; ++i) {
      mLoadTilesDEM.push_back(HEALPix::getChildTileId(tileId, i));
      mLoadTilesIMG.push_back(HEALPix::getChildTileId(tileId, i));
    }

    result = refine;
  }

  return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool TestTileVisitor::refineTile() {
  return getLevel() < 2 ? true : false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::lodplanets