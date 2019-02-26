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
    WriteGraphFile("after_");

    NodeVector nodes = m_Spa.getNodes();
    forEach(NodeVector, &nodes)
    {
        karto::Pose2 pose(iter->trans(0), iter->trans(1), iter->arot);
        corrections.push_back(std::make_pair(iter->nodeId, pose));
        //std::cout << "[ INFO] [" << ros::Time::now() << "]: pose " << pose << std::endl;
    }
}

void SpaSolver::AddNode(karto::Vertex <karto::LocalizedRangeScan> *pVertex) {
    karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
    Eigen::Vector3d vector(pose.GetX(), pose.GetY(), pose.GetHeading());
    m_Spa.addNode(vector, pVertex->GetObject()->GetUniqueId());

    // add new vertex to nodes
    // karto::Vertex <karto::LocalizedRangeScan> pVertexC = *pVertex;
    nodes.push_back(pVertex);
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

void SpaSolver::WriteGraphFile(std::string name) {
    // save result to File
    std::ofstream outputStream;
    outputStream.open(name + std::to_string(optimizationNumber) + ".karto", std::ofstream::binary);

    // write all vertices to the file
    for (auto const &it: nodes) {
        karto::Pose2 pose = it->GetObject()->GetCorrectedPose();
        outputStream << "NODE " << it->GetObject()->GetUniqueId() << " " << pose.GetX() << " " << pose.GetY() << " "
                     << pose.GetHeading() << std::endl;
    }

    // write all edges to the file
    for (auto const &pEdge: edges) {
        karto::LocalizedRangeScan *pSource = pEdge->GetSource()->GetObject();
        karto::LocalizedRangeScan *pTarget = pEdge->GetTarget()->GetObject();
        //karto::Edge <karto::LocalizedRangeScan> pEdgeV = pEdge;
        karto::LinkInfo *pLinkInfo = (karto::LinkInfo * )(pEdge->GetLabel());

        karto::Pose2 diff = pLinkInfo->GetPoseDifference();
        outputStream << "EDGE " << pSource->GetUniqueId() << " " << pTarget->GetUniqueId() << " " << diff.GetX() << " "
                     << diff.GetY()
                     << " " << diff.GetHeading() << std::endl;
    }

    outputStream.close();
    ++optimizationNumber;
}