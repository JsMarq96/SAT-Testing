#include <iostream>

#include "math.h"
#include "SAT_collision_testing.h"

int main() {
    sVector3 scale {1.0f, 1.0f, 1.0f};

    sVector3 pos1{0.050f, 0.f, 0.0f};
    sQuaternion4 rot1{1.0f, 0.00f, 0.0f, 0.0f};

    sVector3 pos2{0.00f, 0.f, 0.0f};
    sQuaternion4 rot2{1.0f, 0.0f, 0.0f, 0.0f};

    bool result = SAT_OBB_v_OBB(pos1, scale, rot1, pos2, scale, rot2);
    std::cout << result  << std::endl;

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