#include "AStar.h"
#include <stdio.h>
#include <time.h>

#define MAX_NODES   1024
#define MAX_POS     32
#define HOP_LEN     2
#define X_STEP      1
#define Y_STEP      1
#define Y_PER_ROW   32

typedef struct node {
    float x;
    float y;
    struct node* neighbors[MAX_NODES];
} node;

static float hopCost(void *srcNode, void *dstNode, void *prevsrcNode, void *context) {
    node* src = (node*)srcNode;
    node* dst = (node*)dstNode;
    node* prev_src = (node*)prevsrcNode;

    if (prevsrcNode != NULL) {
        // this actual for first node of path
    }

    /* using the Manhattan distance because the sqrt() and pow() functions
       needed for Euclidean distance are part of math.h, which is an external
       library that I would have to rebuild as a RISC-V library
    */
    float x_delta, y_delta;
    // to avoid needing abs(), subtract the smaller from the greater
    x_delta = (src->x > dst->x) ? (src->x - dst->x) : (dst->x - src->x);
    y_delta = (src->y > dst->y) ? (src->y - dst->y) : (dst->y - src->y);
    return x_delta + y_delta;
}

static void nodeNeighbors(ASNeighborList neighbors, void* srcNode, void* fromsrcNode, void* context) {
    node* src = (node*)srcNode;
    int i;
    for (i = 0; i < MAX_NODES; i++) {
        if (src->neighbors[i]) {
            // cost and pose for node i equal cost and pose for 
            ASNeighborListAdd(neighbors, (void*)src->neighbors[i], 
                    hopCost(srcNode, (void*)(src->neighbors[i]), fromsrcNode,  context));
        }
    }
}

static const ASPathNodeSource pathSource = {
    sizeof(node), 
    &nodeNeighbors,
    &hopCost,
    NULL,
    NULL
};

int main(int argc, char** argv) {
    node* graph[MAX_NODES];
    int i, j;
    for (i = 0; i < MAX_NODES; i++) {
        graph[i] = calloc(1, sizeof(node));
        graph[i]->x = i*X_STEP % MAX_POS;
        graph[i]->y = (int)(i/Y_PER_ROW)*Y_STEP % MAX_POS;
    }
    for (i = 0; i < MAX_NODES; i++) {
        //printf("node = %d. x = %.1f, y = %.1f\n", i, graph[i]->x, graph[i]->y);
    }
    int hopCount;
    float cost;
    ASPath path;
    for (i = 0; i < MAX_NODES; i++) {
        //printf("node %d :", i);
        for (j = 0; j < MAX_NODES; j++) {
            if (i != j && hopCost(graph[i], graph[j], NULL, NULL) < HOP_LEN) {
                // set by map allowed direction type
                graph[i]->neighbors[j] = graph[j];
                graph[j]->neighbors[i] = graph[i];

                //printf(" %d,", j);
            }
        }

        //printf("\n");
    }

    clock_t begin = clock();

    i = 0;
    j = 1023;
    //for (i = 0; i < MAX_NODES; i++) {
    //    for (j = 0; j < MAX_NODES; j++) {
            path = ASPathCreate(&pathSource, (void*)NULL, graph[i], graph[j]);
            cost = ASPathGetCost(path);
            hopCount = ASPathGetCount(path);
            printf("path from %d to %d: cost=%f, hopCount=%d\n", i, j, cost, hopCount);
            ASPathDestroy(path);
    //    }
    //}
    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    printf("Processing time: %f\n", time_spent);
    return 0;
}
