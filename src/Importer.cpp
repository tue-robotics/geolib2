#include "geolib/Importer.h"

#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>

namespace geo {

Importer::Importer() {

}

Importer::~Importer() {
}

ShapePtr Importer::readMeshFile(const std::string& filename, double scale) {
    Assimp::Importer I;
    const aiScene* scene = I.ReadFile(filename, 0);

    ShapePtr shape(new Shape());
    if (scene) {
        for(unsigned int i = 0; i < scene->mNumMeshes; ++i) {
            aiMesh* m = scene->mMeshes[i];

            for(unsigned int j = 0; j < m->mNumVertices; ++j) {
                const aiVector3D& v = m->mVertices[j];
                shape->mesh_.addPoint(scale * v.x, scale * v.y, scale * v.z);
            }

            for(unsigned int j = 0; j < m->mNumFaces; ++j) {
                const aiFace& f = m->mFaces[j];
                if (f.mNumIndices == 3) {
                    shape->mesh_.addTriangle(f.mIndices[0], f.mIndices[1], f.mIndices[2]);
                }
            }
        }
    }

    return shape;
}

}
