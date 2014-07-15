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

    if (!scene) {
        return ShapePtr();
    }

    bool first = true;

    ShapePtr shape(new Shape());
    if (scene) {
        for(unsigned int i = 0; i < scene->mNumMeshes; ++i) {
            aiMesh* m = scene->mMeshes[i];

            std::map<int, std::map<int, std::map<int, int> > > xyz_map;
            std::vector<int> i_map(m->mNumVertices);

            for(unsigned int j = 0; j < m->mNumVertices; ++j) {
                const aiVector3D& v = m->mVertices[j];                

                int ix = 1000 * scale * v.x;
                int iy = 1000 * scale * v.y;
                int iz = 1000 * scale * v.z;

                bool match = false;
                std::map<int, std::map<int, std::map<int, int> > >::iterator it1 = xyz_map.find(ix);
                if (it1 != xyz_map.end()) {
                    std::map<int, std::map<int, int> >::iterator it2 = it1->second.find(iy);
                    if (it2 != it1->second.end()) {
                        std::map<int, int>::iterator it3 = it2->second.find(iz);
                        if (it3 != it2->second.end()) {
                            i_map[j] = it3->second;
                            match = true;
                        }
                    }
                }

                if (!match) {
                    int ip = shape->mesh_.addPoint(scale * v.x, scale * v.y, scale * v.z);
                    xyz_map[ix][iy][iz] = ip;
                    i_map[j] = ip;

                    if (first) {
                        // std::cout << scale * v.x << " " << scale * v.y << " " << scale * v.z << std::endl;
                        // std::cout << shape->mesh_.getPoints()[0] << std::endl;
                        first = false;
                    }

                }
            }

            std::map<int, std::map<int, std::set<int> > > triangle_map;

            int num_triangles = 0;
            for(unsigned int j = 0; j < m->mNumFaces; ++j) {
                const aiFace& f = m->mFaces[j];
                if (f.mNumIndices == 3) {
                    int ix = i_map[f.mIndices[0]];
                    int iy = i_map[f.mIndices[1]];
                    int iz = i_map[f.mIndices[2]];

                    bool match = false;
                    std::map<int, std::map<int, std::set<int> > >::iterator it1 = triangle_map.find(ix);
                    if (it1 != triangle_map.end()) {
                        std::map<int, std::set<int> >::iterator it2 = it1->second.find(iy);
                        if (it2 != it1->second.end()) {
                            if (it2->second.find(iz) != it2->second.end()) {
                                match = true;
                            }
                        }
                    }

                    if (!match) {
                        shape->mesh_.addTriangle(ix, iy, iz);
                        triangle_map[ix][iy].insert(iz);
                        ++num_triangles;
                    }
                }
            }
        }
    }

    // std::cout << shape->mesh_.getPoints()[0] << std::endl;


    return shape;
}

}
