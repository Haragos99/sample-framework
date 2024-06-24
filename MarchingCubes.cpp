#include"MarchingCubes.h"



void MarchingCubes::compute(const std::vector<std::vector<std::vector<double>>>& scalarField) {
    for (int i = 0; i < scalarField.size() - 1; ++i) {
        for (int j = 0; j < scalarField[i].size() - 1; ++j) {
            for (int k = 0; k < scalarField[i][j].size() - 1; ++k) {
                double x = i/SIZE, y = j/ SIZE, z = k/ SIZE;

                x -= POSITION;
                y -= POSITION;
                z -= POSITION;
                // cell ordered according to convention in referenced website
                GridCell cell =
                {
                    {
                        {x, y, z}, {x + 1.0 / SIZE, y, z},
                        {x +1.0/ SIZE, y, z + 1.0 / SIZE}, {x, y, z + 1.0 / SIZE},
                        {x, y + 1.0 / SIZE, z}, {x + 1.0 / SIZE, y + 1.0 / SIZE, z},
                        {x + 1.0 / SIZE, y + 1.0 / SIZE, z + 1.0 / SIZE}, {x, y + 1.0 / SIZE, z + 1.0 / SIZE}
                    },
                    {
                        scalarField[i][j][k], scalarField[i + 1][j][k],
                        scalarField[i + 1][j][k + 1], scalarField[i][j][k + 1],
                        scalarField[i][j + 1][k], scalarField[i + 1][j + 1][k],
                        scalarField[i + 1][j + 1][k + 1], scalarField[i][j + 1][k + 1]
                    }
                };

                std::vector<std::vector<MyMesh::VertexHandle>> faceVertices = getVertices(cell);
                grid.push_back(cell);

                for (auto f : faceVertices)
                {
                    mesh_.add_face(f);
                }

            }
        }
    }
    mesh_.request_vertex_normals();
    mesh_.request_face_normals();
    mesh_.update_normals();
    for (auto v : mesh_.vertices())
    {
        // set size
        mesh_.set_point(v, mesh_.point(v)/10);
        MyMesh::Normal n(0, 0, 0);
        for (MyMesh::VertexFaceIter vf_it = mesh_.vf_iter(v); vf_it.is_valid(); ++vf_it)
        {
            MyMesh::FaceHandle fh = *vf_it;
            n += mesh_.normal(fh);
        }
        n.normalize();
        mesh_.set_normal(v, n);
    }

}




MyMesh::VertexHandle MarchingCubes::interpolate(MyMesh::Point& v1, double val1, MyMesh::Point& v2, double val2)
{
    MyMesh::Point interpolated;
    double mu = (threshold_ - val1) / (val2 - val1);

    interpolated[0] = mu * (v2[0] - v1[0]) + v1[0];
    interpolated[1] = mu * (v2[1] - v1[1]) + v1[1];
    interpolated[2] = mu * (v2[2] - v1[2]) + v1[2];

    return mesh_.add_vertex(interpolated);
}

int MarchingCubes::determineConfigurationIndex(const std::vector<double>& voxel) {
    int index = 0;
    for (int i = 0; i < 8; ++i) {
        if (voxel[i] > threshold_) {
            index |= (1 << i);
        }
    }
    return index;
}


std::vector<std::vector<MyMesh::VertexHandle>> MarchingCubes::getVertices(GridCell& cell) {
    std::vector<std::vector<MyMesh::VertexHandle>> vertices;

    int cubeIndex = calculate_cube_index(cell);
    auto intersections = get_intersection_coordinates(cell);
    std::vector<std::vector<MyMesh::VertexHandle>> triangles = get_triangles(intersections, cubeIndex);
    return triangles;
}


std::vector<std::vector<MyMesh::VertexHandle>> MarchingCubes::get_triangles(std::vector<MyMesh::VertexHandle>& intersections, int cubeIndex)
{
    std::vector<std::vector<MyMesh::VertexHandle>> triangles;
    for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3)
    {
        std::vector<MyMesh::VertexHandle> triangle(3);
        for (int j = 0; j < 3; j++)
            triangle[j] = intersections[triangleTable[cubeIndex][i + j]];
        triangles.push_back(triangle);
    }

    return triangles;
}




void MarchingCubes::draw() {
    for (auto f : mesh_.faces()) {
        glBegin(GL_POLYGON);
        for (auto v : mesh_.fv_range(f)) {

            Vec color = Vec(1, 1, 1);
            glColor3d(color.x, color.y, color.z);
            auto G = mesh_.normal(v);
            glNormal3dv(mesh_.normal(v).data());
            glVertex3dv(mesh_.point(v).data());
        }
        glEnd();
    }
    if (true) {
        for (auto g : grid)
        {
            for (const auto& p : g.vertex)
            {

                glDisable(GL_LIGHTING);

                glColor3d(0.0, 0.0, 1.0);
                glPointSize(50.0);
                glBegin(GL_POINTS);
                glVertex3dv(p.data());
                glEnd();
                glEnable(GL_LIGHTING);
            }
        }
    }
}


int MarchingCubes::calculate_cube_index(GridCell& cell)
{
    int cubeIndex = 0;
    for (int i = 0; i < 8; i++)
        if (cell.value[i] < threshold_) cubeIndex |= (1 << i);
    return cubeIndex;
}


std::vector<MyMesh::VertexHandle>MarchingCubes::get_intersection_coordinates(GridCell& cell)
{
    std::vector<MyMesh::VertexHandle> intersections(12);

    int cubeIndex = calculate_cube_index(cell);
    int intersectionsKey = edgeTable[cubeIndex];

    int idx = 0;
    while (intersectionsKey)
    {
        if (intersectionsKey & 1)
        {
            int v1 = edgeVertices[idx][0], v2 = edgeVertices[idx][1];
            MyMesh::VertexHandle intersectionPoint = interpolate(cell.vertex[v1], cell.value[v1],
                cell.vertex[v2], cell.value[v2]);
            intersections[idx] = intersectionPoint;
        }
        idx++;
        intersectionsKey >>= 1;
    }


    return intersections;
}
