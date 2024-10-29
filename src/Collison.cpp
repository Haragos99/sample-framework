#include "Collison.h"
void Collison::init(std::vector<Eigen::Vector4d> v)
{
    for (auto d : v)
    {
        deltas.push_back(Delta(d,1.0f,false));
    }
    err = Eigen::Vector3f(-1, -1, -1);  // Error bounds
    tmax = 1.0;
    tmaxiter = 1e7;
    tolerance = 1e-3;
    mc = 1e-6;
    smallestTio = 1.0f;
    alfa = 0;
}


void Collison::setRestToi(float newtoi)
{
    for (auto& d : deltas)
    {
        if (!d.isCollied)
        {
            d.toi = newtoi;
        }
    }
    
}

bool Collison::collisondetec(MyMesh& mesh, MyMesh& smooth)
{
    smallestTio = 1;
    tois.clear();
    Eigen::Vector3f v_t0, v_t1;
    Eigen::Vector3f f0_t0, f1_t0, f2_t0;
    Eigen::Vector3f f0_t1, f1_t1, f2_t1;
    float outtolerance;
    float toi = 0;
    bool isanycollied = false;
    MyMesh::VertexHandle vindex;
    MyMesh::FaceHandle findex;
    MyMesh::EdgeHandle eindex;

    for (auto v : colliedverteces)
    {
        if (deltas[v.idx()].isCollied) 
        {
            continue;
        }
        v_t1 = toEigenVec(mesh.point(v));
        v_t0 = toEigenVec(smooth.point(v));
        for (auto f : colliedfaces)
        {
            MyMesh::FaceVertexIter fv_it = mesh.fv_iter(f);
            MyMesh::VertexHandle v0 = *fv_it;
            MyMesh::VertexHandle v1 = *(++fv_it);
            MyMesh::VertexHandle v2 = *(++fv_it);

            bool isInTriangle = v0 == v || v1 == v || v2 == v;

            //bool isCollied = v0 == v || v1 == v || v2 == v;


            if (isInTriangle)
            {
                continue;
            }

            f0_t1 = toEigenVec(mesh.point(v0));
            f0_t0 = toEigenVec(smooth.point(v0));

            f1_t1 = toEigenVec(mesh.point(v1));
            f1_t0 = toEigenVec(smooth.point(v1));

            f2_t1 = toEigenVec(mesh.point(v2));
            f2_t0 = toEigenVec(smooth.point(v2));

            bool iscollied = ticcd::vertexFaceCCD(
                v_t0, f0_t0, f1_t0, f2_t0,
                v_t1, f0_t1, f1_t1, f2_t1,
                err, mc, toi, tolerance, tmax, tmaxiter, outtolerance
            );
            if (iscollied) {
                //mesh.data(v).color = Vec(1, 0, 0);
                //deltas[v.idx()].toi = toi;
                tois.push_back(toi);
                verteces.insert(v);
                if (toi < smallestTio )
                {
                    smallestTio = toi;
                    vindex = v;
                    findex = f;
                }
                isanycollied = true;
 
            }

        }
    }

    for (auto e : colliededges) {
        MyMesh::EdgeHandle eh1 = e;

        // Get the start and end vertices of the first edge
        MyMesh::HalfedgeHandle heh1 = mesh.halfedge_handle(eh1, 0);  // First halfedge
        MyMesh::VertexHandle v0_1 = mesh.from_vertex_handle(heh1);   // Start vertex of edge 1
        MyMesh::VertexHandle v1_1 = mesh.to_vertex_handle(heh1);     // End vertex of edge 1


        if (deltas[v0_1.idx()].isCollied || deltas[v1_1.idx()].isCollied)
        {
            continue;
        }

        // Convert OpenMesh vertices to Eigen vectors for t0
        Eigen::Vector3f ea0_t1 = toEigenVec(mesh.point(v0_1));
        Eigen::Vector3f ea1_t1 = toEigenVec(mesh.point(v1_1));

        // Assume edge 1 moves (displacement example for t1)
        Eigen::Vector3f ea0_t0 = toEigenVec(smooth.point(v0_1));
        Eigen::Vector3f ea1_t0 = toEigenVec(smooth.point(v1_1));


        // Loop over all edges (again) to check edge-edge collision
        for (auto e2 : colliededges) {
            MyMesh::EdgeHandle eh2 = e2;

            // Get the start and end vertices of the second edge
            MyMesh::HalfedgeHandle heh2 = mesh.halfedge_handle(eh2, 0);  // Second halfedge
            MyMesh::VertexHandle v0_2 = mesh.from_vertex_handle(heh2);   // Start vertex of edge 2
            MyMesh::VertexHandle v1_2 = mesh.to_vertex_handle(heh2);     // End vertex of edge 2



            if (v0_1 == v0_2 || v1_1 == v1_2 || v1_1 == v0_2 || v0_1 == v1_2)
            {
                continue;
            }

            // Convert OpenMesh vertices to Eigen vectors for t0
            Eigen::Vector3f eb0_t1 = toEigenVec(mesh.point(v0_2));
            Eigen::Vector3f eb1_t1 = toEigenVec(mesh.point(v1_2));

            // Assume edge 2 moves (displacement example for t1)
            Eigen::Vector3f eb0_t0 = toEigenVec(smooth.point(v0_2));
            Eigen::Vector3f eb1_t0 = toEigenVec(smooth.point(v1_2));

            // Perform edge-edge collision detection
            bool is_colliding = ticcd::edgeEdgeCCD(
                ea0_t0, ea1_t0, eb0_t0, eb1_t0,  // Edges at time t0
                ea0_t1, ea1_t1, eb0_t1, eb1_t1,  // Edges at time t1
                err,                             // Error bounds
                mc,                              // Minimum separation
                toi,                             // Time of impact (output)
                tolerance,                       // Solving precision
                tmax,                           // Time interval upper bound (0 <= t_max <= 1)
                tmaxiter,                         // Maximum iterations
                outtolerance,                // Output precision under max_itr
                true                             // Refine for zero toi
            );

            if (is_colliding) {

                //verteces.insert(v0_1);
                //verteces.insert(v1_1);
                if (toi < smallestTio)
                {
                    smallestTio = toi;
                    eindex = e;
                }
                isanycollied = true;
               

            }
        }
    }


    

    alfa = alfa + (1 - alfa) * smallestTio;
    prevTio = smallestTio;

    setSmalest(vindex, findex, eindex ,mesh);
    return isanycollied;
}

void Collison::setSmalest(MyMesh::VertexHandle& v, MyMesh::FaceHandle& f, MyMesh::EdgeHandle& e, MyMesh& mesh)
{
    if (!mesh.is_valid_handle(e))
    {
        if (!mesh.is_valid_handle(v) || !mesh.is_valid_handle(f))
        {
            return;
        }
        mesh.data(v).color = Vec(0, 0, 1);
        deltas[v.idx()].toi = alfa;
        setMeshTio(v, mesh);

        deltas[v.idx()].isCollied = true;
        for (auto vi : mesh.fv_range(f))
        {
            deltas[vi.idx()].toi = alfa;
            setMeshTio(vi, mesh);
            deltas[vi.idx()].isCollied = true;
            mesh.data(vi).color = Vec(0, 0, 0.5);
        }
    }
    else
    {
        MyMesh::HalfedgeHandle heh1 = mesh.halfedge_handle(e, 0);  // First halfedge
        MyMesh::VertexHandle v0 = mesh.from_vertex_handle(heh1);   // Start vertex of edge 
        MyMesh::VertexHandle v1 = mesh.to_vertex_handle(heh1);     // End vertex of edge 
        mesh.data(v0).color = Vec(0.5, 0, 0);
        deltas[v0.idx()].toi = alfa;
        setMeshTio(v0, mesh);
        deltas[v0.idx()].isCollied = true;

        mesh.data(v1).color = Vec(0.5, 0,0 );
        deltas[v1.idx()].toi = alfa;
        setMeshTio(v1, mesh);
        deltas[v1.idx()].isCollied = true;




    }
  
}




void Collison::setMeshTio(MyMesh::VertexHandle& v, MyMesh& mesh)
{
    mesh.point(v) = deltas[v.idx()].getDeltaPoint(mesh.data(v).C);
}


void Collison::test(MyMesh& mesh, MyMesh& smooth)
{
    int count = 0;
    //collisondetec(mesh, smooth);
    double counter = 0;
    prevTio = 0;
    
    for (int i = 0; i < 1; i++)
    {
        
        //restCollied();
        collisondetec(mesh, smooth);
        //while (collisondetec(mesh, smooth) || counter <= 1)
        {
            //alfa += 0.1;
            setRestToi(alfa);
            for (auto v : mesh.vertices())
            {
                setMeshTio(v, mesh);
            }
            if (alfa >= 1)
            {
                //break;
            }
            count++;
            counter += 0.1;
        }
        //setRestToi(1);
        for (auto v : mesh.vertices())
        {
            //setMeshTio(v, mesh);
        }
    }
    //smoothpoints(mesh);
}
void Collison::restCollied()
{
    for (auto& d : deltas)
    {
        d.isCollied = false;
    }
}



void Collison::smoothpoints(MyMesh& mesh)
{
    mesh.update_face_normals();
    mesh.update_vertex_normals();
    double smootingfactor = 0.03;
    for (int i = 0; i < 1; i++)
    {
        auto smooth = mesh;
        for (auto vis : verteces)
        {
            for (auto v : mesh.vv_range(vis)) {
                Vec Avg;
                int n = 0;
                for (auto vi : mesh.vv_range(v)) {

                    auto point = mesh.point(vi);

                    projectPointToPlane(mesh.point(v), mesh.normal(v), point);

                    Vec vertex = Vec(point);
                    Avg += vertex;
                    n++;
                }
                Avg /= n;
                MyMesh::Point pointavg = MyMesh::Point(Avg.x, Avg.y, Avg.z);

                mesh.point(v) += smootingfactor * (pointavg - mesh.point(v));

            }
        }

        smootingfactor = 0.005;
        for (auto v : verteces)
        {
            Vec Avg;
            int n = 0;
            for (auto vi : mesh.vv_range(v)) {

                auto point = mesh.point(vi);

                projectPointToPlane(mesh.point(v), mesh.normal(v), point);

                Vec vertex = Vec(point);
                Avg += vertex;
                n++;
            }
            Avg /= n;
            MyMesh::Point pointavg = MyMesh::Point(Avg.x, Avg.y, Avg.z);

            mesh.point(v) += smootingfactor * (pointavg - mesh.point(v));
        }


    }
   // verteces.clear();
}



void Collison::projectPointToPlane(const MyMesh::Point& P, const MyMesh::Normal& N, MyMesh::Point& Q) {
    // normalize the normal
    MyMesh::Normal norm = N;
    norm.normalize();

    // distance between the pointand the plane
    float d = dot(P - Q, norm);

    //  projected point
    Q = Q + (-d * norm);
}
