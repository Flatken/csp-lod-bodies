////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "TileNode.hpp"

namespace csp::lodbodies {

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode::TileNode()
    : mTile()
    , mParent(NULL)
    , mChildren()
    , mChildMaxLevel(0) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode::TileNode(TileBase* tile, int childMaxLevel, TileBase* secTile)
    : mTile(tile)
    , mSecTile(secTile)
    , mParent(NULL)
    , mChildren()
    , mChildMaxLevel(childMaxLevel) {
  if (mChildMaxLevel < 0 && mTile)
    mChildMaxLevel = mTile->getLevel();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode::TileNode(std::unique_ptr<TileBase>&& tile, int childMaxLevel, std::unique_ptr<TileBase>&& secTile)
    : mTile(std::move(tile))
    , mSecTile(std::move(secTile))
    , mParent(NULL)
    , mChildren()
    , mChildMaxLevel(childMaxLevel) {
  if (mChildMaxLevel < 0 && mTile)
    mChildMaxLevel = mTile->getLevel();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int TileNode::getLevel() const {
  return mTile->getLevel();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

glm::int64 TileNode::getPatchIdx() const {
  return mTile->getPatchIdx();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileId const& TileNode::getTileId() const {
  return mTile->getTileId();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::type_info const& TileNode::getTileTypeId() const {
  return mTile->getTypeId();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileDataType TileNode::getTileDataType() const {
  return mTile->getDataType();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileBase* TileNode::getTile() const {
  return mTile.get();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileBase*  TileNode::getSecTile() const {
  return mSecTile.get();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileBase* TileNode::releaseTile() {
  return mTile.release();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TileNode::setTile(TileBase* tile) {
  mTile.reset(tile);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TileNode::setSecTile(TileBase* tile) {
  mSecTile.reset(tile);
}
////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode* TileNode::getChild(int childIdx) const {
  return mChildren[childIdx].get();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode* TileNode::releaseChild(int childIdx) {
  if (mChildren[childIdx])
    mChildren[childIdx]->setParent(NULL);

  return mChildren[childIdx].release();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TileNode::setChild(int childIdx, TileNode* child) {
  // unset OLD parent
  if (mChildren[childIdx])
    mChildren[childIdx]->setParent(NULL);

  mChildren[childIdx].reset(child);

  // set NEW parent
  if (mChildren[childIdx])
    mChildren[childIdx]->setParent(this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int TileNode::getChildMaxLevel() const {
  return mChildMaxLevel;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TileNode::setChildMaxLevel(int maxLevel) {
  mChildMaxLevel = maxLevel;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode* TileNode::getParent() const {
  return mParent;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TileNode::setParent(TileNode* parent) {
  mParent = parent;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// NO CLASS methods
bool isLeaf(TileNode const& node) {
  return node.getLevel() == node.getChildMaxLevel();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool isInner(TileNode const& node) {
  return node.getLevel() < node.getChildMaxLevel();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool isRefined(TileNode const& node) {
  return node.getChild(0) && node.getChild(1) && node.getChild(2) && node.getChild(3);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string TileNode::getTime() const {
  return mTime;
}

std::string TileNode::getSecTime() const {
  return mSecTime;
}

void TileNode::setTime(std::string time) {
  mTime = time;
}

void TileNode::setSecTime(std::string time) {
  mSecTime = time;
}

} // namespace csp::lodbodies
