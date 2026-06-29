#include "geolib/io/export.h"
#include "geolib/datatypes.h"
#include "geolib/Mesh.h"
#include <assimp/material.h>
#include <assimp/types.h>
#include <assimp/vector3.h>
#include <cctype>
#include <ostream>
#include <sys/types.h>
#include <vector>

#if __has_include(<assimp/Exporter.hpp>)
#include <assimp/Exporter.hpp>
#include <assimp/mesh.h>
#include <assimp/scene.h>
#else
#include <assimp/aiMesh.h>
#include <assimp/aiScene.h>
#include <assimp/assimp.hpp>
#endif

#include <console_bridge/console.h>

#include <geolib/Shape.h>

#include <algorithm>
#include <sstream>
#include <string>

namespace geo::io
{

// ----------------------------------------------------------------------------------------------------

bool writeMeshFile(const std::string& filename, const Shape& shape, std::string format)
{
    aiScene a_scene;
    a_scene.mMeshes = new aiMesh*[1];
    a_scene.mMeshes[0] = new aiMesh();
    auto* a_mesh = a_scene.mMeshes[0];
    a_scene.mNumMeshes = 1;

    a_scene.mMaterials = new aiMaterial*[1];
    a_scene.mMaterials[0] = new aiMaterial;
    a_scene.mNumMaterials = 1;
    a_mesh->mMaterialIndex = 0;

    a_scene.mRootNode = new aiNode;
    auto* a_node = a_scene.mRootNode;
    a_node->mMeshes = new uint[1];
    a_node->mMeshes[0] = static_cast<uint>(0);
    a_node->mNumMeshes = 1;

    // Get mesh and its element from shape
    const geo::Mesh& mesh = shape.getMesh();
    const std::vector<geo::Vector3>& points = mesh.getPoints();
    const std::vector<geo::TriangleI>& triangle_is = mesh.getTriangleIs();

    // Transfer points to Assimp mesh
    a_mesh->mVertices = new aiVector3D[points.size()];
    a_mesh->mNumVertices = points.size();
    for (auto it = points.cbegin(); it != points.cend(); ++it)
    {
        const geo::Vector3& v = *it;
        a_mesh->mVertices[it - points.begin()] =
            aiVector3D(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
    }

    // Transfer faces to Assimp mesh
    a_mesh->mFaces = new aiFace[triangle_is.size()];
    a_mesh->mNumFaces = triangle_is.size();
    for (auto it = triangle_is.cbegin(); it != triangle_is.cend(); ++it)
    {
        aiFace& a_face = a_mesh->mFaces[it - triangle_is.begin()];
        a_face.mIndices = new uint[3];
        a_face.mNumIndices = 3;

        a_face.mIndices[0] = it->i1_;
        a_face.mIndices[1] = it->i2_;
        a_face.mIndices[2] = it->i3_;
    }

    // Check format
    std::transform(format.begin(), format.end(), format.begin(), ::tolower);
    if (format.empty())
    {
        if (filename.substr(filename.size() - 3) == "3ds")
            format = "3ds";
        else if (filename.substr(filename.size() - 3) == "3mf")
            format = "3mf";
        else if (filename.substr(filename.size() - 3) == "dae")
            format = "collada";
        else if (filename.substr(filename.size() - 3) == "ply")
            format = "ply";
        else if (filename.substr(filename.size() - 3) == "obj")
            format = "obj";
        else if (filename.substr(filename.size() - 3) == "stp")
            format = "stp";
        else if (filename.substr(filename.size() - 3) == "stl")
            format = "stl";
        else if (filename.substr(filename.size() - 1) == "x")
            format = "x";
        else if (filename.substr(filename.size() - 3) == "x3d")
            format = "x3d";
    }

    Assimp::Exporter a_exp;
    aiReturn const result = a_exp.Export(&a_scene, format, filename);
    if (result != AI_SUCCESS)
    {
        std::stringstream ss;
        ss << "Error" << '\n' << a_exp.GetErrorString() << '\n';
        const std::string& str = ss.str();
        CONSOLE_BRIDGE_logError(str.c_str());
        return false;
    }
    return true;
}

} // namespace geo::io
