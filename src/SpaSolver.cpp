/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "slam_karto/SpaSolver.h"
#include <open_karto/Karto.h>

#include "ros/console.h"

SpaSolver::SpaSolver() {
}

SpaSolver::~SpaSolver() {
}

void SpaSolver::Clear() {
    corrections.clear();
}

const karto::ScanSolver::IdPoseVector &SpaSolver::GetCorrections() const {
    return corrections;
}

void SpaSolver::Compute() {
    corrections.clear();

    typedef std::vector <Node2d, Eigen::aligned_allocator<Node2d>> NodeVector;

    //char *name = (char *)"test";
    //m_Spa.writeSparseA((char *)"before_", false);
    //m_Spa.writeSparseA((char *) "before_t_", true);
    WriteGraphFile("before_");

    ROS_INFO("Calling doSPA for loop closure");
    m_Spa.doSPA(40);
    ROS_INFO("Finished doSPA for loop closure");
    //m_Spa.writeSparseA((char *)"after_", false);
    //m_Spa.writeSparseA((char *) "after_t_", true);


    NodeVector nodes = m_Spa.getNodes();
    forEach(NodeVector, &nodes)
    {
        karto::Pose2 pose(iter->trans(0), iter->trans(1), iter->arot);
        corrections.push_back(std::make_pair(iter->nodeId, pose));
        //std::cout << "[ INFO] [" << ros::Time::now() << "]: pose " << pose << std::endl;
    }
    wasOptimized = true;
}

void SpaSolver::AddNode(karto::Vertex <karto::LocalizedRangeScan> *pVertex) {
    karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
    Eigen::Vector3d vector(pose.GetX(), pose.GetY(), pose.GetHeading());
    m_Spa.addNode(vector, pVertex->GetObject()->GetUniqueId());

    // add new vertex to nodes
    // karto::Vertex <karto::LocalizedRangeScan> pVertexC = *pVertex;
    nodes.push_back(pVertex);
    if (wasOptimized) {
        WriteGraphFile("after_");
        wasOptimized = false;
    }
}

void SpaSolver::AddConstraint(karto::Edge <karto::LocalizedRangeScan> *pEdge) {
    karto::LocalizedRangeScan *pSource = pEdge->GetSource()->GetObject();
    karto::LocalizedRangeScan *pTarget = pEdge->GetTarget()->GetObject();
    karto::LinkInfo *pLinkInfo = (karto::LinkInfo * )(pEdge->GetLabel());

    karto::Pose2 diff = pLinkInfo->GetPoseDifference();
    Eigen::Vector3d mean(diff.GetX(), diff.GetY(), diff.GetHeading());

    karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
    Eigen::Matrix<double, 3, 3> m;
    m(0, 0) = precisionMatrix(0, 0);
    m(0, 1) = m(1, 0) = precisionMatrix(0, 1);
    m(0, 2) = m(2, 0) = precisionMatrix(0, 2);
    m(1, 1) = precisionMatrix(1, 1);
    m(1, 2) = m(2, 1) = precisionMatrix(1, 2);
    m(2, 2) = precisionMatrix(2, 2);

    m_Spa.addConstraint(pSource->GetUniqueId(), pTarget->GetUniqueId(), mean, m);

    // add new edge to edges
    // karto::Edge <karto::LocalizedRangeScan> pEdgeC = *pEdge;
    edges.push_back(pEdge);
}


///
/// Vertex/Node
/// std::vector< Vertex< T > * > 	    GetAdjacentVertices () const
/// const std::vector< Edge< T > * > & 	GetEdges () const
/// T * 	                            GetObject () const
///
/// Edge
/// EdgeLabel * 	    GetLabel ()
/// Vertex< T > * 	    GetSource () const
/// Vertex< T > * 	    GetTarget () const
///
/// LinkInfo inherited from EdgeLabel
/// const Matrix3 & 	GetCovariance ()
/// const Pose2 & 	    GetPose1 ()
/// const Pose2 & 	    GetPose2 ()
/// const Pose2 & 	    GetPoseDifference ()
///
/// LocalizedRangeScan
/// const Pose2 & 	            GetBarycenterPose () const
/// const BoundingBox2 & 	    GetBoundingBox () const
/// const Pose2 & 	            GetCorrectedPose () const
/// const Pose2 & 	            GetOdometricPose () const
/// const PointVectorDouble & 	GetPointReadings (kt_bool wantFiltered=false) const
/// Pose2 	                    GetReferencePose (kt_bool useBarycenter) const
/// Pose2 	                    GetSensorAt (const Pose2 &rPose) const
/// Pose2 	                    GetSensorPose () const
///
/// Pose
/// kt_double 	                    GetHeading () const
/// const Vector2< kt_double > & 	GetPosition () const
/// kt_double 	                    GetX () const
/// kt_double 	                    GetY () const
///
/// Matrix3
/// std::string 	    ToString () const
///
/// \param name
void SpaSolver::WriteGraphFile(std::string name) {
    // save result to File
    std::ofstream outputStream;
    outputStream.open(name + std::to_string(optimizationNumber) + ".karto", std::ofstream::binary);

    // write all vertices to the file
    for (auto const &it: nodes) {
        karto::Pose2 pose = it->GetObject()->GetCorrectedPose();
        outputStream << "VERTEX_SE2 " << it->GetObject()->GetUniqueId() << " " << pose.GetX() << " " << pose.GetY() << " "
                     << pose.GetHeading() << std::endl;
    }

    // write all edges to the file
    for (auto const &pEdge: edges) {
        karto::LocalizedRangeScan *pSource = pEdge->GetSource()->GetObject();
        karto::LocalizedRangeScan *pTarget = pEdge->GetTarget()->GetObject();
        //karto::Edge <karto::LocalizedRangeScan> pEdgeV = pEdge;
        karto::LinkInfo *pLinkInfo = (karto::LinkInfo * )(pEdge->GetLabel());

        karto::Pose2 diff = pLinkInfo->GetPoseDifference();
        outputStream << "EDGE_SE2 " << pSource->GetUniqueId() << " " << pTarget->GetUniqueId() << " " << diff.GetX() << " "
                     << diff.GetY()
                     << " " << diff.GetHeading() << " " << pLinkInfo->GetCovariance() << " " << std::endl;
    }

    outputStream.close();
    ++optimizationNumber;
}