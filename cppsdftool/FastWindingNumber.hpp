/*
* Copyright (C) 2020 Alec Jacobson <alecjacobson@gmail.com>
* 
* This Source Code Form is subject to the terms of the Mozilla Public License 
* v. 2.0. If a copy of the MPL was not distributed with this file, You can 
* obtain one at http://mozilla.org/MPL/2.0/
*
* Based on "Fast Winding Numbers for Soups and Clouds" SIGGRAPH 2018
* by Gavin Barill, Neil Dickson, Ryan Schmidt, David I.W. Levin, and Alec Jacobson
*/

#include <glm/glm.hpp>
#include <vector>
#include "PreparedTriangle.hpp"
#include "FastWindingNumber.h"
#include <glm/gtc/constants.hpp>


namespace HDK_Sample{ template <typename T1, typename T2> class UT_SolidAngle;}

struct FastWindingNumberBVH {
    HDK_Sample::UT_SolidAngle<float,float> ut_solid_angle;
    // Need copies of these so they stay alive between calls.
    std::vector<HDK_Sample::UT_Vector3T<float> > U;
    std::vector<int> F;
};

class FastWindingNumber {
public:
    FastWindingNumber(const std::vector<PreparedTriangle>& triangles) {
        initialize(triangles);
    }

    bool is_inside(const glm::dvec3& p, double beta = 2.0) const {
        return compute(p, beta) > 0.5;
    }

    double compute(const glm::dvec3& p, double beta = 2.0) const {
        HDK_Sample::UT_Vector3T<float>Qp;

        Qp[0] = p[0];
        Qp[1] = p[1];
        Qp[2] = p[2];

        return fwn_bvh.ut_solid_angle.computeSolidAngle(Qp, beta) / (4.0 * glm::pi<double>());
    }

    ~FastWindingNumber() = default;

private:
    FastWindingNumberBVH fwn_bvh;

    void initialize(const std::vector<PreparedTriangle>& input_triangles) {
        int order = 2;
        
        
        // Extra copies. Usuually this won't be the bottleneck.
        fwn_bvh.U.resize(input_triangles.size() * 3);
        fwn_bvh.F.resize(input_triangles.size() * 3);

        
        // Copy vertices and triangle indices
        for (size_t i = 0; i < input_triangles.size(); i++) {
            const auto& tri = input_triangles[i];
            int baseIdx = i * 3;

            // Copy vertices
            fwn_bvh.U[baseIdx][0] = tri.getVertexA().x;
            fwn_bvh.U[baseIdx][1] = tri.getVertexA().y;
            fwn_bvh.U[baseIdx][2] = tri.getVertexA().z;
            
            fwn_bvh.U[baseIdx + 1][0] = tri.getVertexB().x;
            fwn_bvh.U[baseIdx + 1][1] = tri.getVertexB().y;
            fwn_bvh.U[baseIdx + 1][2] = tri.getVertexB().z;
            
            fwn_bvh.U[baseIdx + 2][0] = tri.getVertexC().x;
            fwn_bvh.U[baseIdx + 2][1] = tri.getVertexC().y;
            fwn_bvh.U[baseIdx + 2][2] = tri.getVertexC().z;

            // Set indices
            fwn_bvh.F[baseIdx] = baseIdx;
            fwn_bvh.F[baseIdx + 1] = baseIdx + 1;
            fwn_bvh.F[baseIdx + 2] = baseIdx + 2;
        }

       /*

        for (int i = 0; i < input_triangles.size(); i += 3)
        {
            for (int j = 0; j < 3; j++)
                fwn_bvh.U[i * 3 + 0][j] = input_triangles[i].getVertexA()[j];

            fwn_bvh.F[i * 3 + 0] = i * 3 + 0;

            for (int j = 0; j < 3; j++)
                fwn_bvh.U[i * 3 + 1][j] = input_triangles[i].getVertexB()[j];

            fwn_bvh.F[i * 3 + 1] = i * 3 + 1;
            for (int j = 0; j < 3; j++)
                fwn_bvh.U[i * 3 + 2][j] = input_triangles[i].getVertexC()[j];

            fwn_bvh.F[i * 3 + 2] = i * 3 + 2;
        }*/

        fwn_bvh.ut_solid_angle.clear();
        fwn_bvh.ut_solid_angle.init(
            fwn_bvh.F.size()/3, 
            &fwn_bvh.F[0], 
            fwn_bvh.U.size(), 
            &fwn_bvh.U[0], 
            order);
    }
};