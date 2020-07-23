////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CSP_LOD_BODIES_RENDERDATA_HPP
#define CSP_LOD_BODIES_RENDERDATA_HPP

#include "TileBounds.hpp"
#include "TileId.hpp"

#include <boost/noncopyable.hpp>

namespace csp::lodbodies {

class TileNode;

/// The base class for all render data of a single TileNode.
class RenderData : private boost::noncopyable {
 public:
  virtual ~RenderData();

  TileNode*     getNode() const;
  void          setNode(TileNode* node, bool secTileActive = false);
  int           getLevel() const;
  glm::int64    getPatchIdx() const;
  TileId const& getTileId() const;

  int  getTexLayer() const;
  void setTexLayer(int layer);

  int  getLastFrame() const;
  void setLastFrame(int frame);
  int  getAge(int frame) const;

  bool getSecTileActive();

  BoundingBox<double> const& getBounds() const;
  void                       setBounds(BoundingBox<double> const& tb);
  void                       removeBounds();
  bool                       hasBounds() const;

 protected:
  explicit RenderData(TileNode* node = NULL, bool secTileActive = false);
  BoundingBox<double> mTb;
  bool                mHasBounds;

 private:
  TileNode* mNode;
  int       mTexLayer;
  int       mLastFrame;
  bool mSecTileActive;
};

} // namespace csp::lodbodies

#endif // CSP_LOD_BODIES_RENDERDATA_HPP
