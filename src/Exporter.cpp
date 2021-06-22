#include "geolib/Exporter.h"

#include <algorithm>
#include <sstream>
#include <string>

#ifdef ASSIMP_VERSION_3
    #include <assimp/Exporter.hpp>
    #include <assimp/scene.h>
    #include <assimp/mesh.h>
#else
    #include <assimp/assimp.hpp>
    #include <assimp/aiScene.h>
    #include <assimp/aiMesh.h>
#endif

#include <console_bridge/console.h>


namespace geo {

// ----------------------------------------------------------------------------------------------------

Exporter::Exporter()
{
}

// ----------------------------------------------------------------------------------------------------

Exporter::~Exporter()
{
}

// ----------------------------------------------------------------------------------------------------

bool Exporter::writeMeshFile(const std::string& filename, const Shape& shape, std::string format)
{
    aiScene aScene;
    aScene.mMeshes = new aiMesh*[1];
    aScene.mMeshes[0] = new aiMesh();
    auto aMesh = aScene.mMeshes[0];
    aScene.mNumMeshes = 1;

    aScene.mMaterials = new aiMaterial*[1];
    aScene.mMaterials[0] = new aiMaterial;
    aScene.mNumMaterials = 1;
    aMesh->mMaterialIndex = 0;

    aScene.mRootNode = new aiNode;
    auto aNode = aScene.mRootNode;
    aNode->mMeshes = new uint[1];
    aNode->mMeshes[0] = uint(0);
    aNode->mNumMeshes = 1;

    // Get mesh and its element from shape
    Mesh mesh = shape.getMesh();
    const std::vector<Vector3>& points = mesh.getPoints();
    const std::vector<TriangleI>& triangleIs = mesh.getTriangleIs();

    // Transfer points to Assimp mesh
    aMesh->mVertices = new aiVector3D[ points.size() ];
    aMesh->mNumVertices = points.size();
    for (std::vector<Vector3>::const_iterator it = points.cbegin(); it != points.cend(); ++it)
    {
        const Vector3& v = *it;
        aMesh->mVertices[it - points.begin()] = aiVector3D( v.x, v.y, v.z );
    }

    // Transfer faces to Assimp mesh
    aMesh->mFaces = new aiFace[triangleIs.size()];
    aMesh->mNumFaces = triangleIs.size();
    for (std::vector<TriangleI>::const_iterator it = triangleIs.cbegin(); it != triangleIs.cend(); ++it)
    {
                aiFace& aFace = aMesh->mFaces[it - triangleIs.begin()];
                aFace.mIndices = new uint[ 3 ];
                aFace.mNumIndices = 3;

                aFace.mIndices[0] = it->i1_;
                aFace.mIndices[1] = it->i2_;
                aFace.mIndices[2] = it->i3_;
    }


    //Check format
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

    Assimp::Exporter aExp;
    aiReturn result = aExp.Export(&aScene, format, filename);
    if (result != AI_SUCCESS)
    {
        std::stringstream ss;
        ss << "Error" << std::endl << aExp.GetErrorString() << std::endl;
        const std::string& str = ss.str();
        CONSOLE_BRIDGE_logError(str.c_str());
        return false;
    }
    return true;
}

}
