#include <iostream>

#include "math.h"
#include "SAT_collision_testing.h"

void get_transform(sMat44 *result, sVector3 pos, sVector3 scale, sQuaternion4 rot) {
    sMat44 scale_mat;
    result->rotate(&rot);
    scale_mat.set_scale(scale);

    result->multiply(&scale_mat);
    result->set_position(pos);
}

int main() {
    sMat44 transf1, transf2;

    sQuaternion4 rot1{1.0f, 0.0f, 0.0f, 0.0f};
    sQuaternion4 rot2{1.0f, 0.0f, 0.0f, 0.0f};

    get_transform(&transf1, {0.f, 0.f, 0.f}, {1.0f, 1.0f, 1.0f}, rot1);
    get_transform(&transf2, {0.560f, 0.330f, 1.04f}, {1.0f, 1.0f, 1.0f}, rot2);

    sCollisionManifold man;

    bool result = SAT_OBB_v_OBB(transf1, rot1, transf2, rot2, &man);
    std::cout << result << std::endl;

    sVector3 vert[8];
    get_OBB_raw_vertex(transf1, vert);

    for(int i = 0; i < 8; i++) {
        std::cout << vert[i].x << " " << vert[i].y << " " << vert[i].z << std::endl;
    }
    

    return 0;
}


/**

COMO HACERLO

Dot prod de todos los vertices de un cuerpo vs la normal invertida de la cara
Pero si lo hacemos asi, sera 8 ptos * 6 caras = 48 dot prods y cross prods
esto no es escabale... Pero conseguiria perfectamente los ptos que estan
dentro de OBB
MMmmmm
*/