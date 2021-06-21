//
// Created by jsmar on 16/06/2021.
//

#ifndef QUEST_DEMO_SAT_COLLISION_TESTING_H
#define QUEST_DEMO_SAT_COLLISION_TESTING_H

#include <float.h>

#include "math.h"

#include <iostream>

inline void get_OBB_raw_vertex(const sVector3 obb_center,
                               const sVector3 obb_size,
                               const sQuaternion4 obb_rotation,
                               sVector3 *result) {
    result[0] = sVector3{};
    result[1] = {obb_size.x, 0.0f, 0.0f};
    result[2] = {0.0f, obb_size.y, 0.0f};
    result[3] = {obb_size.x, obb_size.y, 0.0f};
    result[4] = {0.0f, 0.0f, obb_size.z};
    result[5] = {0.0f, obb_size.y, obb_size.z};
    result[6] = {obb_size.x, 0.0f, obb_size.z};
    result[7] = {obb_size.x, obb_size.y, obb_size.z};

    for(int i = 0; i < 8; i++) {
        result[i] = rotate_vector3(result[i], obb_rotation);
        result[i] = { result[i].x + obb_center.x,
                      result[i].y + obb_center.y,
                      result[i].z + obb_center.z };
    }
}

inline float distance_to_plane(const sVector3 point, const sVector3 plane_normal) {
    dot_prod(point, plane_normal);
}

int OBB_faces_indexing[][] = {
    { 0, 1, 2, 3},
    {6, 7, 3, 1},
    {2, 3, 7, 5},
    {5, 4, 7, 6},
    {4, 6, 0, 1},
    {5, 4, 2, 0}
};

sVector3 OBB_normal_faces[] = {
    { 0.0f, 0.0f, -1.0f},
    { 1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    { 0.0f, 0.0f, 1.0f},
    { 0.0f, -1.0f, 0.0f},
    {-1.0f, 0.0f, 0.0f}
};

inline sVector3 get_point_in_face(const sVector3 *raw_vertex, const int face_id, const int vertex_num) {
    return raw_vertex[ OBB_faces_indexing[face_id][vertex_num] ];
}

inline int get_closest_point_to_face(int face_id, sVector3 *obj_1, sVector3 *obj_2, ) {


    for (int i = 0; i < 6; i++) {
        sVector3 norm = rotate_vector3(sVector3{1.0f, 0.0f, 0.0f}, obb1_rotation);
    }
}

inline bool sat_test(const sVector3 obb1_origin,
                          const sVector3 obb1_sizes,
                          const sQuaternion4 obb1_rotation,
                          const sVector3 obb2_origin,
                          const sVector3 obb2_sizes,
                          const sQuaternion4 obb2_rotation) {
    

}


inline bool intersect_vertex_group_on_axis(const sVector3 obb1[8],
                                           const sVector3 obb2[8],
                                           const sVector3 axis,
                                           float          *diff) {
    float min_1 = FLT_MAX;
    float max_1 = FLT_MIN;
    float min_2 = FLT_MAX;
    float max_2 = FLT_MIN;

    if (axis.x == 0 && axis.y == 0 && axis.z == 0) {
        return true;
    }

    // Get the min and max position for each vertex group
    // proyected on teh vertex
    for (int i = 0; i < 8; i++) {
        float dist_1 = dot_prod(obb1[i], axis);
        float dist_2 = dot_prod(obb2[i], axis);
        min_1 = (dist_1 < min_1) ? dist_1 : min_1;
        max_1 = (dist_1 > max_1) ? dist_1 : max_1;
        min_2 = (dist_2 < min_2) ? dist_2 : min_2;
        max_2 = (dist_2 > max_2) ? dist_2 : max_2;
    }

    float total_span = MAX(max_1, max_2) - MIN(min_1, min_2);
    float sum = max_1 - min_1 + max_2 - min_2;

    *diff = sum - total_span;

    std::cout << *diff << std::endl;

    return sum >= total_span;
}


inline bool SAT_OBB_v_OBB(const sVector3 obb1_origin,
                          const sVector3 obb1_sizes,
                          const sQuaternion4 obb1_rotation,
                          const sVector3 obb2_origin,
                          const sVector3 obb2_sizes,
                          const sQuaternion4 obb2_rotation) {
    sVector3 obb1_vertex[8] = {};
    sVector3 obb2_vertex[8] = {};

    sVector3 col_normal{};
    sVector3 col_normal1{};
    sVector3 col_normal2{};
    
    get_OBB_raw_vertex(obb1_origin,
                       obb1_sizes,
                       obb1_rotation,
                       &obb1_vertex[0]);
    get_OBB_raw_vertex(obb2_origin,
                       obb2_sizes,
                       obb2_rotation,
                       &obb2_vertex[0]);

    sVector3 norm1_x = rotate_vector3(sVector3{1.0f, 0.0f, 0.0f}, obb1_rotation);
    sVector3 norm1_y = rotate_vector3(sVector3{0.0f, 1.0f, 0.0f}, obb1_rotation);
    sVector3 norm1_z = rotate_vector3(sVector3{0.0f, 0.0f, 1.0f}, obb1_rotation);
    sVector3 norm2_x = rotate_vector3(sVector3{1.0f, 0.0f, 0.0f}, obb2_rotation);
    sVector3 norm2_y = rotate_vector3(sVector3{0.0f, 1.0f, 0.0f}, obb2_rotation);
    sVector3 norm2_z = rotate_vector3(sVector3{0.0f, 0.0f, 1.0f}, obb2_rotation);

    float min_diff = 200.0f;
    float diff = 202.0f;
    float min_diff1 = 200.0f;
    float diff1 = 2002.0f;
    float min_diff2 = 200.0f;
    float diff2 = 2002.0f;
    int col_case = -1;
    int col_case1 = -1;
    int col_case2 = -1;
    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       norm1_x,
                                       &diff)) {
        return false;
    }

    if (min_diff1 > diff) {
        col_normal1 = norm1_x;
        min_diff1 = diff;
        col_case1 = 0;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       norm1_y,
                                       &diff)) {
        return false;
    }

    if (min_diff1 > diff) {
        col_normal1 = norm1_y;
        min_diff1 = diff;
        col_case1 = 1;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       norm1_z,
                                       &diff)) {
        return false;
    }

    if (min_diff1 > diff) {
        col_normal1 = norm1_z;
        min_diff1 = diff;
        col_case1 = 2;
    }

    
     if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       norm2_x,
                                       &diff)) {
        return false;
    }

    if (min_diff2 > diff) {
        col_normal2 = norm2_x;
        min_diff2 = diff;
        col_case2 = 3;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       norm2_y,
                                       &diff)) {
        return false;
    }

    if (min_diff2 > diff) {
        col_normal2 = norm2_y;
        min_diff2 = diff;
        col_case = 4;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       norm2_z,
                                       &diff)) {
        return false;
    }

    if (min_diff2 > diff) {
        col_normal2 = norm2_z;
        min_diff2 = diff;
        col_case = 5;
    }



    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       cross_prod(norm1_x, norm2_x),
                                       &diff)) {
        std::cout << "6" << std::endl;
        return false;
    }

    if (min_diff > diff) {
        col_normal = cross_prod(norm1_x, norm2_x);
        min_diff = diff;
        col_case = 6;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       cross_prod(norm1_x, norm2_y),
                                       &diff)) {
        return false;
    }

    if (min_diff > diff) {
        col_normal = cross_prod(norm1_x, norm2_y);
        min_diff = diff;
        col_case = 7;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       cross_prod(norm1_x, norm2_z),
                                       &diff)) {
        return false;
    }

    if (min_diff > diff) {
        col_normal = cross_prod(norm1_x, norm2_z);
        min_diff = diff;
        col_case = 8;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       cross_prod(norm1_y, norm2_x),
                                       &diff)) {
        return false;
    }

    if (min_diff > diff) {
        col_normal = cross_prod(norm1_y, norm2_x);
        min_diff = diff;
        col_case = 9;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       cross_prod(norm1_y, norm2_y),
                                       &diff)) {
        return false;
    }

    if (min_diff > diff) {
        col_normal = cross_prod(norm1_y, norm2_y);
        min_diff = diff;
        col_case = 10;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       cross_prod(norm1_y, norm2_z),
                                       &diff)) {
        return false;
    }

    if (min_diff > diff) {
        col_normal = cross_prod(norm1_y, norm2_z);
        min_diff = diff;
        col_case = 11;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       cross_prod(norm1_z, norm2_x),
                                       &diff)) {
        return false;
    }

    if (min_diff > diff) {
        col_normal = cross_prod(norm1_z, norm2_x);
        min_diff = diff;
        col_case = 12;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       cross_prod(norm1_z, norm2_y),
                                       &diff)) {
        return false;
    }

    if (min_diff > diff) {
        col_normal = cross_prod(norm1_z, norm2_y);
        min_diff = diff;
        col_case = 13;
    }

    if(!intersect_vertex_group_on_axis(obb1_vertex,
                                       obb2_vertex,
                                       cross_prod(norm1_z, norm2_z),
                                       &diff)) {
        return false;
    }

    if (min_diff > diff) {
        col_normal = cross_prod(norm1_z, norm2_z);
        min_diff = diff;
        col_case = 14;
    }

    /// http://www.randygaul.net/2013/03/28/custom-physics-engine-part-2-manifold-generation/

    std::cout << "A diff " << min_diff1 << " B diff " << min_diff2 << std::endl;
    std::cout << " A Collision axis: " << col_normal1.x << " " << col_normal1.y << " " << col_normal1.z << std::endl;
    std::cout << " B Collision axis: " << col_normal2.x << " " << col_normal2.y << " " << col_normal2.z << std::endl;
    std::cout << "Collision axis: " << col_normal.x << " " << col_normal.y << " " << col_normal.z << std::endl;
    std::cout << "Depth: " << min_diff << std::endl;
    std::cout << col_case << std::endl;

    if (col_case <= 2) {
        // Face collision on object A
        switch(col_case) {
            case 0:
            std::cout << obb1_vertex[1].x << " " << obb1_vertex[1].y << " " << obb1_vertex[1].z  << " ++ " << obb1_vertex[3].x << " " << obb1_vertex[3].y << " " << obb1_vertex[3].z  << " ++ " << obb1_vertex[6].x << " " << obb1_vertex[6].y << " " << obb1_vertex[6].z  << " ++ " << obb1_vertex[7].x << " " << obb1_vertex[7].y << " " << obb1_vertex[7].z  << std::endl;
            break;
            case 1:
            std::cout << obb1_vertex[2].x << " " << obb1_vertex[2].y << " " << obb1_vertex[2].z  << " ++ " << obb1_vertex[3].x << " " << obb1_vertex[3].y << " " << obb1_vertex[3].z  << " ++ " << obb1_vertex[5].x << " " << obb1_vertex[5].y << " " << obb1_vertex[5].z  << " ++ " << obb1_vertex[7].x << " " << obb1_vertex[7].y << " " << obb1_vertex[7].z  << std::endl;
            break;
            case 2:
            std::cout << obb1_vertex[4].x << " " << obb1_vertex[4].y << " " << obb1_vertex[4].z  << " ++ " << obb1_vertex[6].x << " " << obb1_vertex[6].y << " " << obb1_vertex[6].z  << " ++ " << obb1_vertex[5].x << " " << obb1_vertex[5].y << " " << obb1_vertex[5].z  << " ++ " << obb1_vertex[7].x << " " << obb1_vertex[7].y << " " << obb1_vertex[7].z  << std::endl;
            break;
        }
    } else if (col_case >= 3 && col_case <= 5) {
        // Face collision on object B
        switch(col_case) {
            case 3:
            std::cout << obb2_vertex[1].x << " " << obb2_vertex[1].y << " " << obb2_vertex[1].z  << " ++ " << obb2_vertex[3].x << " " << obb2_vertex[3].y << " " << obb2_vertex[3].z  << " ++ " << obb2_vertex[6].x << " " << obb2_vertex[6].y << " " << obb2_vertex[6].z  << " ++ " << obb2_vertex[7].x << " " << obb2_vertex[7].y << " " << obb2_vertex[7].z  << std::endl;
            break;
            case 4:
            std::cout << obb2_vertex[2].x << " " << obb2_vertex[2].y << " " << obb2_vertex[2].z  << " ++ " << obb2_vertex[3].x << " " << obb2_vertex[3].y << " " << obb2_vertex[3].z  << " ++ " << obb2_vertex[5].x << " " << obb2_vertex[5].y << " " << obb2_vertex[5].z  << " ++ " << obb2_vertex[7].x << " " << obb2_vertex[7].y << " " << obb2_vertex[7].z  << std::endl;
            break;
            case 5:
            std::cout << obb2_vertex[4].x << " " << obb2_vertex[4].y << " " << obb2_vertex[4].z  << " ++ " << obb2_vertex[6].x << " " << obb2_vertex[6].y << " " << obb2_vertex[6].z  << " ++ " << obb2_vertex[5].x << " " << obb2_vertex[5].y << " " << obb2_vertex[5].z  << " ++ " << obb2_vertex[7].x << " " << obb2_vertex[7].y << " " << obb2_vertex[7].z  << std::endl;
            break;
        }
    } else if (col_case > 6) {

    }

    return true;
}

/*
inline float get_axis_overlap(const float size1,
                              const float size2,
                              const float min_distance){
    return (size1 / 2.0f) + (size2 / 2.0f) - ABS(min_distance);
}

inline bool SAT_AABB_AABB_collision(const sVector3        aabb1_center,
                                    const sVector3        aabb1_size,
                                    const sVector3        aabb2_center,
                                    const sVector3        aabb2_size,
                                    sCollisionManifold    *result) {
    if (aabb1_center.x <= (aabb2_center.x + aabb2_size.x) && (aabb1_center.x + aabb1_size.x) >= aabb2_center.x) {
        return false;
    }
    if (aabb1_center.y <= (aabb2_center.y + aabb2_size.y) && (aabb1_center.y + aabb1_size.y) >= aabb2_center.y) {
        return false;
    }
    if (aabb1_center.z <= (aabb2_center.z + aabb2_size.z) && (aabb1_center.z + aabb1_size.z) >= aabb2_center.z) {
        return false;
    }

    // Fill collision manifold
    float overlap_x = get_axis_overlap(aabb1_size.x,
                                       aabb2_size.x,
                                       aabb1_center.x - aabb1_center.x);

    float overlap_y = get_axis_overlap(aabb1_size.y,
                                       aabb2_size.y,
                                       aabb1_center.y - aabb1_center.y);

    float overlap_z = get_axis_overlap(aabb1_size.z,
                                       aabb2_size.z,
                                       aabb1_center.z - aabb1_center.z);

    return true;
}*/

#endif //QUEST_DEMO_SAT_COLLISION_TESTING_H
