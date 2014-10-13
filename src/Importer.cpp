#include "geolib/Importer.h"

#ifdef ASSIMP_VERSION_3
    #include <assimp/Importer.hpp>
    #include <assimp/scene.h>
#else
    #include <assimp/assimp.hpp>
    #include <assimp/aiScene.h>
#endif

namespace geo {

// ----------------------------------------------------------------------------------------------------

Importer::Importer()
{
}

// ----------------------------------------------------------------------------------------------------

Importer::~Importer()
{
}

// ----------------------------------------------------------------------------------------------------

void constructMesh(const aiScene* scene, aiNode* node, const geo::Pose3D& parent_pose, double scale, bool transform, geo::Mesh* mesh)
{
    const aiMatrix4x4& t = node->mTransformation;
    geo::Pose3D p;
    p.t = geo::Vector3(t.a4, t.b4, t.c4);
    p.R = geo::Matrix3(t.a1, t.a2, t.a3, t.b1, t.b2, t.b3, t.c1, t.c2, t.c3);

    geo::Pose3D pose = parent_pose * p;

    for(unsigned int i = 0; i < node->mNumChildren; ++i)
    {
        constructMesh(scene, node->mChildren[i], pose, scale, transform, mesh);
    }

    for(unsigned int i = 0; i < node->mNumMeshes; ++i)
    {
        aiMesh* m = scene->mMeshes[node->mMeshes[i]];

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
                geo::Vector3 p(v.x, v.y, v.z);
                if (transform)
                    p = pose * p;

                int ip = mesh->addPoint(scale * p.x, scale * p.y, scale * p.z);
                xyz_map[ix][iy][iz] = ip;
                i_map[j] = ip;
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
                    mesh->addTriangle(ix, iy, iz);
                    triangle_map[ix][iy].insert(iz);
                    ++num_triangles;
                }
            }
        }
    }

}

// ----------------------------------------------------------------------------------------------------

ShapePtr Importer::readMeshFile(const std::string& filename, double scale)
{
    Assimp::Importer I;
    const aiScene* scene = I.ReadFile(filename, 0);

    if (!scene)
        return ShapePtr();

    // TODO: get rid of this hack!!!
    bool transform = true;
    if (filename.substr(filename.size() - 3) == "3ds")
    {
        transform = false;
    }

    ShapePtr shape(new Shape());

    Mesh mesh;
    constructMesh(scene, scene->mRootNode, geo::Pose3D::identity(), scale, transform, &mesh);
    shape->setMesh(mesh);

    return shape;
}

}
