#include "geolib/Exporter.h"

#ifdef ASSIMP_VERSION_3
    #include <assimp/Exporter.hpp>
    #include <assimp/scene.h>
    #include <assimp/mesh.h>
    #include <assimp/postprocess.h>
#else
    #include <assimp/assimp.hpp>
    #include <assimp/aiScene.h>
    #include <assimp/aiMesh.h>
#endif

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

bool Exporter::writeMeshFile(const std::string& filename, const Shape& shape, double scale)
{
    aiScene aScene;
    aiMesh aMesh;
    aScene.mMeshes = new aiMesh*[ 1 ];
    aScene.mMeshes[ 0 ] = nullptr;
    aScene.mNumMeshes = 1;

    aScene.mMeshes[ 0 ] = new aiMesh();
    aScene.mMeshes[ 0 ]->mMaterialIndex = 0;
    aScene.mRootNode = new aiNode;

    Mesh mesh = shape.getMesh();
    const std::vector<Vector3>& points = mesh.getPoints();
    const std::vector<TriangleI>& triangleIs = mesh.getTriangleIs();

    aMesh.mVertices = new aiVector3D[points.size()];
    aMesh.mNumVertices = points.size();
    for (uint i = 0; i < points.size(); ++i)
    {
        const Vector3& p = points[i];
        aMesh.mVertices[i] = aiVector3D(p.x, p.y, p.z);
    }

    aMesh.mFaces = new aiFace[triangleIs.size()];
    aMesh.mNumFaces = triangleIs.size();
    for (uint i = 0; i < triangleIs.size(); ++i)
    {
        aiFace& aFace = aMesh.mFaces[i];
        const TriangleI& triangleI = triangleIs[i];
        aFace.mIndices = new uint[3];
        aFace.mNumIndices = 3;
        aFace.mIndices[0] = triangleI.i1_;
        aFace.mIndices[1] = triangleI.i2_;
        aFace.mIndices[2] = triangleI.i3_;
    }

    Assimp::Exporter aExp;
    aiReturn result = aExp.Export(&aScene, "stl", filename);
    if (result != AI_SUCCESS)
    {
        std::cout << "Error" << std::endl << aExp.GetErrorString() << std::endl;
        return false;
    }
    return true;
}

}
