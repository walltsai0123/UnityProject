#include "Backend.h"
#include "Util.h"
#include <PluginAPI/IUnityInterface.h>
#include <igl/readMESH.h>
#include <igl/per_vertex_normals.h>
#include <map>
#include <vector>
#include <windows.h>

using V_RowMajor = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
using T_RowMajor = Eigen::Matrix<int, Eigen::Dynamic, 4, Eigen::RowMajor>;
using F_RowMajor = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RowVector3I = Eigen::Matrix<int, 1, 3>;
using RowVector4I = Eigen::Matrix<int, 1, 3>;
using std::map;
using std::vector;

namespace
{
    struct triangleCompare
    {
        bool operator()(const RowVector3I &a, const RowVector3I &b) const
        {
            if (a[0] < b[0])
                return true;
            if (a[0] > b[0])
                return false;

            if (a[1] < b[1])
                return true;
            if (a[1] > b[1])
                return false;

            if (a[2] < b[2])
                return true;
            if (a[2] > b[2])
                return false;
            return false;
        }
    };
    void computeSurfaceTriangles(T_RowMajor &T, F_RowMajor &F)
    {
        std::map<RowVector3I, int, triangleCompare> faceCounts;

        for (int i = 0; i < T.rows(); ++i)
        {
            auto t = T.row(i);
            RowVector3I faces[4];
            faces[0] << t[0], t[1], t[3];
            faces[1] << t[0], t[2], t[1];
            faces[2] << t[0], t[3], t[2];
            faces[3] << t[1], t[2], t[3];

            for (int y = 0; y < 4; y++)
                std::sort(faces[y].data(), faces[y].data() + faces[y].size());

            for (int y = 0; y < 4; y++)
                faceCounts[faces[y]]++;
        }
        // go back through the tets, if any of its faces have a count less than
        // 2, then it must be because it faces outside
        vector<RowVector3I> surfaceTriangles;
        for (int i = 0; i < T.rows(); ++i)
        {
            auto t = T.row(i);

            RowVector3I faces[4];

            // these are consistently ordered counter-clockwise
            faces[0] << t[0], t[1], t[3];
            faces[1] << t[0], t[2], t[1];
            faces[2] << t[0], t[3], t[2];
            faces[3] << t[1], t[2], t[3];

            RowVector3I facesSorted[4];

            // make a sorted copy, but keep the original around for rendering
            for (int y = 0; y < 4; y++)
            {
                facesSorted[y] = faces[y];
                std::sort(facesSorted[y].data(), facesSorted[y].data() + facesSorted[y].size());
            }

            // see which faces don't have a dual
            for (int y = 0; y < 4; y++)
            {
                if (faceCounts[facesSorted[y]] < 2)
                    surfaceTriangles.push_back(faces[y]);
            }

            // assign surfaceTriangles into F
            F.resize(surfaceTriangles.size(), 3);
            for (size_t i = 0; i < surfaceTriangles.size(); ++i)
            {
                F.row(i) = surfaceTriangles[i];
            }
        }
    }
    void centerToMean(V_RowMajor &V)
    {
        Eigen::RowVector3f mean = V.colwise().mean();
        V.rowwise() -= mean;
    }
}

void ApplyDirty(MeshState* state, const MeshDataNative data)
{
    MatrixToMap(state->V, data.VPtr);
    MatrixToMap(state->N, data.NPtr);
}
bool ReadMESH(const char *path,
              void *&VPtr, int &VSize,
              void *&NPtr, int &NSize,
              void *&FPtr, int &FSize,
              void *&TPtr, int &TSize)
{
    auto *V = new V_RowMajor();
    auto *T = new T_RowMajor();
    auto tempF = F_RowMajor();
    
    bool success = igl::readMESH(path, *V, *T, tempF);

    centerToMean(*V);

    auto *F = new F_RowMajor();
    computeSurfaceTriangles(*T, *F);

    auto *N = new V_RowMajor();
    igl::per_vertex_normals(*V, *F, *N);

    VSize = V->rows();
    NSize = N->rows();
    FSize = F->rows();
    TSize = T->rows();

    VPtr = V->data();
    NPtr = N->data();
    FPtr = F->data();
    TPtr = T->data();
    
    LOG("MESH Import " << (success ? "Successful: " : "Unsuccessful: ") << path)
    return success;
}