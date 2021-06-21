
#ifndef QUEST_DEMO_SAT_COLLISION_TEST
#define QUEST_DEMO_SAT_COLLISION_TEST

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




#endif