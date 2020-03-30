////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CSP_LOD_BODIES_TERRAINSHADER_HPP
#define CSP_LOD_BODIES_TERRAINSHADER_HPP

#include <memory>
#include <string>

class VistaGLSLShader;

namespace csp::lodbodies {

/// The base class for the PlanetShader. It builds the shader from various sources and links it.
class TerrainShader {
 public:
  TerrainShader() {
  }
  TerrainShader(std::string vertexSource, std::string fragmentSource);
  virtual ~TerrainShader();

  virtual void bind();
  virtual void release();

  friend class TileRenderer;

 protected:
  virtual void compile();

  bool             mShaderDirty = true;
  std::string      mVertexSource;
  std::string      mFragmentSource;
  VistaGLSLShader* mShader = nullptr;
};

} // namespace csp::lodbodies

#endif // CSP_LOD_BODIES_TERRAINSHADER_HPP
