////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "TileSourceWebMapService.hpp"

#include "HEALPix.hpp"
#include "TileNode.hpp"

#include <VistaTools/tinyXML/tinyxml.h>

#include <boost/filesystem.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Info.hpp>
#include <curlpp/Infos.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/cURLpp.hpp>
#include <fstream>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image.h>

#include <tiffio.h>

namespace csp::lodplanets {

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {

////////////////////////////////////////////////////////////////////////////////////////////////////

enum class CopyPixels { eAll, eAboveDiagonal, eBelowDiagonal };

////////////////////////////////////////////////////////////////////////////////////////////////////

void createDirectoryRecursively(
    boost::filesystem::path const& path, boost::filesystem::perms permissions) {

  if (!boost::filesystem::exists(path.parent_path())) {
    createDirectoryRecursively(path.parent_path(), permissions);
  }

  boost::filesystem::create_directory(path);
  boost::filesystem::permissions(path, permissions);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
bool loadImpl(
    TileSourceWebMapService* source, TileNode* node, int level, int x, int y, CopyPixels which) {
  auto        tile = static_cast<Tile<T>*>(node->getTile());
  std::string cacheFile;

  try {
    cacheFile = source->loadData(level, x, y);
  } catch (std::exception const& e) {
    std::cerr << "Tile loading failed: " << e.what() << std::endl;
    return false;
  }

  if (tile->getDataType() == TileDataType::eFloat32) {
    TIFFSetWarningHandler(nullptr);
    auto data = TIFFOpen(cacheFile.c_str(), "r");
    if (!data) {
      std::cout << "Failed to load " << cacheFile << std::endl;
      return false;
    }

    unsigned imagelength;
    TIFFGetField(data, TIFFTAG_IMAGELENGTH, &imagelength);
    for (unsigned y = 0; y < imagelength; y++) {
      if (which == CopyPixels::eAll) {
        TIFFReadScanline(data, &tile->data()[257 * y], y);
      } else if (which == CopyPixels::eAboveDiagonal) {
        std::array<float, 257> tmp;
        TIFFReadScanline(data, tmp.data(), y);
        int offset = 257 * y;
        int count  = 257 - y - 1;
        std::memcpy(tile->data().data() + offset, tmp.data(), count * sizeof(float));
      } else if (which == CopyPixels::eBelowDiagonal) {
        std::array<float, 257> tmp;
        TIFFReadScanline(data, tmp.data(), y);
        int offset = 257 * y + (257 - y);
        int count  = y;
        std::memcpy(tile->data().data() + offset, tmp.data() + 257 - y, count * sizeof(float));
      }
    }
    TIFFClose(data);
  } else {
    int width, height, bpp;
    int channels = tile->getDataType() == TileDataType::eU8Vec3 ? 3 : 1;

    auto data = reinterpret_cast<T*>(stbi_load(cacheFile.c_str(), &width, &height, &bpp, channels));

    if (!data) {
      std::cout << "Failed to load " << cacheFile << std::endl;
      return false;
    }

    if (which == CopyPixels::eAll) {
      std::memcpy(tile->data().data(), data, channels * width * height);
    } else if (which == CopyPixels::eAboveDiagonal) {
      for (int y = 0; y < height; ++y) {
        int offset = width * y;
        int count  = channels * (width - y - 1);
        std::memcpy(tile->data().data() + offset, data + offset, count);
      }
    } else if (which == CopyPixels::eBelowDiagonal) {
      for (int y = 0; y < height; ++y) {
        int offset = width * y + (width - y);
        int count  = channels * y;
        std::memcpy(tile->data().data() + offset, data + offset, count);
      }
    }

    stbi_image_free(data);
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
void fillDiagonal(TileNode* node) {
  auto tile = static_cast<Tile<T>*>(node->getTile());
  for (unsigned y = 1; y <= 257; y++) {
    int pixelPos           = y * (257 - 1);
    tile->data()[pixelPos] = (y < 257) ? tile->data()[pixelPos - 1] : tile->data()[pixelPos + 1];
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
TileNode* loadImpl(TileSourceWebMapService* source, int level, glm::int64 patchIdx) {
  TileNode* node = new TileNode();

  node->setTile(new Tile<T>(level, patchIdx));
  node->setChildMaxLevel(std::min(level + 1, source->getMaxLevel()));

  int  x, y;
  bool onDiag = source->getXY(level, patchIdx, x, y);
  if (onDiag) {
    if (!loadImpl<T>(source, node, level, x, y, CopyPixels::eBelowDiagonal)) {
      delete node;
      return nullptr;
    }

    x += 4 * (1 << level);
    y -= 4 * (1 << level);

    if (!loadImpl<T>(source, node, level, x, y, CopyPixels::eAboveDiagonal)) {
      delete node;
      return nullptr;
    }

    fillDiagonal<T>(node);
  } else {
    if (!loadImpl<T>(source, node, level, x, y, CopyPixels::eAll)) {
      delete node;
      return nullptr;
    }
  }

  // TODO: the NE and NW edges of all tiles should contain the values of the
  // respective neighbours (for tile stiching). This is done by increasing the
  // bounding box of the request by one pixel - this works more or less in the
  // general case, but it doesn't when we are at a base patch border of the
  // northern hemisphere. In this case we will get empty pixels! Therefore we
  // fill the last column and row by copying.
  // The proper solution would load the real neighbouring tiles and copy the
  // pixel values!

  auto         tile   = static_cast<Tile<T>*>(node->getTile());
  glm::i64vec3 baseXY = HEALPix::getBaseXY(TileId(level, patchIdx));
  glm::int64   nSide  = HEALPix::getNSide(TileId(level, patchIdx));

  // northern hemisphere
  if (baseXY.x < 4) {
    // at north west boundary of base patch
    if (baseXY.z == nSide - 1) {
      // copy second pixel row to first
      for (int i = 0; i < 257; i++) {
        tile->data()[i + 257] = tile->data()[i + 257 * 2];
        tile->data()[i]       = tile->data()[i + 257 * 2];
      }
    }

    // at north east boundary of base patch
    if (baseXY.y == nSide - 1) {
      // copy last pixel column to last but one
      for (int i = 0; i < 257; i++) {
        tile->data()[i * 257 + 255] = tile->data()[i * 257 + 254];
        tile->data()[i * 257 + 256] = tile->data()[i * 257 + 254];
      }
    }
  }

  // flip y --- that shouldn't be requiered, but somehow is how it was
  // implemented in the original databases
  for (int i = 0; i < 257 / 2; i++) {
    std::swap_ranges(tile->data().data() + i * 257, tile->data().data() + (i + 1) * 257,
        tile->data().data() + (256 - i) * 257);
  }

  if (tile->getDataType() == TileDataType::eFloat32) {
    // Creating a MinMaxPyramid alongside the sampling beginning with a resolution of
    // 128x128
    // The MinMaxPyramid is later needed to deduce height information from this
    // coarser level DEM tile to deeper level IMG tiles
    auto demTile       = reinterpret_cast<Tile<float>*>(tile);
    auto minMaxPyramid = new MinMaxPyramid(demTile);
    demTile->setMinMaxPyramid(minMaxPyramid);
  }

  return node;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace

////////////////////////////////////////////////////////////////////////////////////////////////////

std::mutex TileSourceWebMapService::mTileSystemMutex;

////////////////////////////////////////////////////////////////////////////////////////////////////

TileSourceWebMapService::TileSourceWebMapService(
    std::string const& xmlConfigFile, std::string const& cacheDirectory)
    : TileSource()
    , mThreadPool(32)
    , mCache(cacheDirectory) {
  VistaXML::TiXmlDocument xDoc(xmlConfigFile);
  if (!xDoc.LoadFile()) {
    std::cout << "Failed to load IMG config file " << xmlConfigFile << ": "
              << "Cannont open file!" << std::endl;
    return;
  }

  const VistaXML::TiXmlElement* pRoot(xDoc.FirstChildElement());
  // Read Data
  if (std::string(pRoot->Value()) != "WMSConfig") {
    std::cout << "Failed to read WMS img config file " << xmlConfigFile << "!" << std::endl;
    return;
  }

  const VistaXML::TiXmlElement* pProperty(pRoot->FirstChildElement());
  while (pProperty != NULL) {
    if (std::string(pProperty->Value()) == "Property") {
      std::string       sName(pProperty->Attribute("Name"));
      std::stringstream ssValue(pProperty->Attribute("Value"));
      if (sName == "URL")
        ssValue >> mUrl;
      else if (sName == "Layers")
        ssValue >> mLayers;
      else if (sName == "Styles")
        ssValue >> mStyles;
      else if (sName == "Format")
        ssValue >> mFormat;
      else if (sName == "MaxLevel")
        ssValue >> mMaxLevel;
      else {
        std::cout << "Ignoring invalid entity " << sName << " while reading WMS IMG config file "
                  << xmlConfigFile << "!" << std::endl;
      }
    } else {
      std::cout << "Ignoring invalid entity " << pProperty->Value()
                << " while reading WMS IMG config file " << xmlConfigFile << "!" << std::endl;
    }
    pProperty = pProperty->NextSiblingElement();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/* virtual */ TileNode* TileSourceWebMapService::loadTile(int level, glm::int64 patchIdx) {
  if (mFormat == TileDataType::eFloat32)
    return loadImpl<float>(this, level, patchIdx);
  if (mFormat == TileDataType::eUInt8)
    return loadImpl<glm::uint8>(this, level, patchIdx);
  if (mFormat == TileDataType::eU8Vec3)
    return loadImpl<glm::u8vec3>(this, level, patchIdx);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool TileSourceWebMapService::getXY(int level, glm::int64 patchIdx, int& x, int& y) {
  std::array<glm::ivec2, 12> basePatchExtends = {glm::ivec2(1, 4), glm::ivec2(2, 3),
      glm::ivec2(3, 2), glm::ivec2(4, 1), glm::ivec2(0, 4), glm::ivec2(1, 3), glm::ivec2(2, 2),
      glm::ivec2(3, 1), glm::ivec2(0, 3), glm::ivec2(1, 2), glm::ivec2(2, 1), glm::ivec2(3, 0)};

  glm::i64vec3 baseXY = HEALPix::getBaseXY(TileId(level, patchIdx));

  x = basePatchExtends[baseXY[0]][0] * (1 << level) + baseXY[1];
  y = basePatchExtends[baseXY[0]][1] * (1 << level) + baseXY[2];

  if (basePatchExtends[baseXY[0]][0] == 0 && basePatchExtends[baseXY[0]][1] == 4) {
    // check if tile is located above the diagonal
    if (y > x + 4 * (1 << level)) {
      x += 4 * (1 << level);
      y -= 4 * (1 << level);
    }
    // check if tile is crossed by diagonal
    else if (y == x + 4 * (1 << level)) {
      return true;
    }
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string TileSourceWebMapService::loadData(int level, int x, int y) {

  std::string format;
  std::string type;

  if (mFormat == TileDataType::eFloat32) {
    format = "tiffGray";
    type   = "tiff";
  } else if (mFormat == TileDataType::eU8Vec3) {
    format = "pngRGB";
    type   = "png";
  } else {
    format = "pngGray";
    type   = "png";
  }

  std::stringstream cacheDir;
  cacheDir << mCache << "/" << mLayers << "/" << level << "/" << x;

  std::stringstream cacheFile(cacheDir.str());
  cacheFile << cacheDir.str() << "/" << y << "." << type;
  std::stringstream url;

  double size = 1.0 / (1 << level);

  url.precision(std::numeric_limits<double>::max_digits10);
  url << mUrl << "&version=1.1.0&request=GetMap&tiled=true&layers=" << mLayers
      << "&styles=" << mStyles << "&bbox=" << x * size << "," << y * size << "," << x * size + size
      << "," << y * size + size << "&width=257&height=257&srs=EPSG:900914&format=" << format;

  auto cacheFilePath(boost::filesystem::path(cacheFile.str()));

  // the file is already there, we can return it
  if (boost::filesystem::exists(cacheFilePath) &&
      boost::filesystem::file_size(cacheFile.str()) > 0) {
    return cacheFile.str();
  }

  // the file is corrupt not available
  {
    std::unique_lock<std::mutex> lock(mTileSystemMutex);

    if (boost::filesystem::exists(cacheFilePath) &&
        boost::filesystem::file_size(cacheFile.str()) == 0) {
      boost::filesystem::remove(cacheFilePath);
    }

    auto cacheDirPath(boost::filesystem::absolute(boost::filesystem::path(cacheDir.str())));
    if (!(boost::filesystem::exists(cacheDirPath))) {
      try {
        createDirectoryRecursively(cacheDirPath, boost::filesystem::perms::all_all);
      } catch (std::exception& e) {
        std::cerr << "Failed to create cache directory: " << e.what() << std::endl;
      }
    }
  }

  bool fail = false;
  {
    std::ofstream out;
    out.open(cacheFile.str(), std::ofstream::out | std::ofstream::binary);

    if (!out) {
      std::cerr << "Failed to open " << cacheFile.str() << " for writing!" << std::endl;
    }

    curlpp::Easy request;
    request.setOpt(new curlpp::options::Url(url.str()));
    request.setOpt(new curlpp::options::WriteStream(&out));
    request.setOpt(new curlpp::options::NoSignal(1));

    request.perform();

    fail = curlpp::Info<CURLINFO_CONTENT_TYPE, std::string>::get(request).substr(0, 11) ==
           "application";
  }

  if (fail) {
    std::ifstream     in(cacheFile.str());
    std::stringstream sstr;
    sstr << in.rdbuf();

    std::remove(cacheFile.str().c_str());
    throw std::runtime_error(sstr.str());
  } else {
    boost::filesystem::perms filePerms =
        boost::filesystem::perms::owner_read | boost::filesystem::perms::owner_write |
        boost::filesystem::perms::group_read | boost::filesystem::perms::group_write |
        boost::filesystem::perms::others_read | boost::filesystem::perms::others_write;
    boost::filesystem::permissions(cacheFilePath, filePerms);
  }

  return cacheFile.str();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/* virtual */ void TileSourceWebMapService::loadTileAsync(
    int level, glm::int64 patchIdx, OnLoadCallback cb) {
  mThreadPool.Enqueue([=]() {
    auto n = loadTile(level, patchIdx);
    cb(this, level, patchIdx, n);
  });
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int TileSourceWebMapService::getPendingRequests() {
  return mThreadPool.PendingTaskCount() + mThreadPool.RunningTaskCount();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::lodplanets