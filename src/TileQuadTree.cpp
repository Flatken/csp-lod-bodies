////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "TileQuadTree.hpp"

#include "HEALPix.hpp"
#include <iostream>

namespace csp::lodbodies {

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode* TileQuadTree::getRoot(int idx) const {
  return mRoots[idx].get();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TileQuadTree::setRoot(int idx, TileNode* root) {
  mRoots[idx].reset(root);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool insertNode(TileQuadTree* tree, TileNode* node) {
  bool          result = true;
  TileId const& tileId = node->getTileId();

  if (tileId.level() == 0) {
    if(tree->getRoot(HEALPix::getRootIdx(tileId)) != NULL) {
      removeNode(tree, tree->getRoot(HEALPix::getRootIdx(tileId)));
    }

    tree->setRoot(HEALPix::getRootIdx(tileId), node);
  } else {
    TileNode* parent = tree->getRoot(HEALPix::getRootIdx(tileId));

    for (int i = 1; i < tileId.level() && parent; ++i) {
      parent = parent->getChild(HEALPix::getChildIdxAtLevel(tileId, i));
    }

    if (parent) {
      if(parent->getChild(HEALPix::getChildIdxAtLevel(tileId, tileId.level())) != NULL) {
        removeNode(tree, parent->getChild(HEALPix::getChildIdxAtLevel(tileId, tileId.level())));
      }

      parent->setChild(HEALPix::getChildIdxAtLevel(tileId, tileId.level()), node);
    } else {
      result = false;
    }
  }

  return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool removeNode(TileQuadTree* tree, TileNode* node) {
  bool      result = false;
  TileNode* parent = node->getParent();

  // node must either have a parent or be a root of the tree (otherwise node
  // is not in tree or the data structure is corrupt).
  if (parent) {
    int childIdx = HEALPix::getChildIdx(node->getTileId());
    assert(parent->getChild(childIdx) == node);

    parent->setChild(childIdx, NULL);
    result = true;
  } else {
    int childIdx = HEALPix::getChildIdx(node->getTileId());
    assert(tree->getRoot(childIdx) == node);

    tree->setRoot(childIdx, NULL);
    result = true;
  }

  return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::lodbodies
