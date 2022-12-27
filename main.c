#include "AStar.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>

#define MAX_NODES   1024
#define MAX_POS     32000
#define HOP_LEN     2000
#define X_STEP      1000
#define Y_STEP      1000
#define Y_PER_ROW   32

typedef struct node {
    uint64_t x; // mm
    uint64_t y; // mm
    struct node* neighbors[MAX_NODES];
} node;


typedef struct Context {
    uint64_t v;
    uint64_t w;
    //uint64_t timestamp;
} Context;

static uint64_t manhetten_dist(uint64_t x1, uint64_t y1, uint64_t x2, uint64_t y2) {
    uint64_t x_delta, y_delta;
    // to avoid needing abs(), subtract the smaller from the greater
    x_delta = (x1 > x2) ? (x1 - x2) : (x2 - x1);
    y_delta = (y1 > y2) ? (y1 - y2) : (y2 - y1);
    return x_delta + y_delta;
}

// Use for heuristic only (node to goal dist)
static uint64_t hopCost(void *srcNode, void *dstNode, void *context) {
    node* src = (node*)srcNode;
    node* dst = (node*)dstNode;
    //Context *c = (Context*)context;

    return manhetten_dist(src->x, src->y, dst->x, dst->y);
}
// result in degrees
static uint64_t angleBetweenVectors(uint64_t x1, uint64_t y1, uint64_t x2, uint64_t y2)
{   
    return 360*(1-((x1*x2+y1*y2)/(sqrt(x1*x1+y1*y1)*sqrt(x2*x2+y2*y2))))/2; // improve perfomance
}

static uint64_t neighborCost(void *srcNode, void *dstNode, void *fromsrcNode, void *context) {
    node* src = (node*)srcNode;
    node* dst = (node*)dstNode;
    node* fsrc = (node*)fromsrcNode;
    Context *c = (Context*)context;

    if (src->x == dst->x && src->y == dst->y) {
        return 0.0;
    }

    // Increase cost koefficient for change direction considering (1.0 for 180 deg. and 0.0 for 0 deg.)
    uint64_t k = 0;
    // Get current direction of AGV
    uint64_t dir_x, dir_y;
    dir_x = src->x - fsrc->x;
    dir_y = src->y - fsrc->y;

    // Get target direction of AGV
    uint64_t goal_x, goal_y;
    goal_x = dst->x - src->x;
    goal_y = dst->y - src->y;
    
    if ((dir_x+dir_y) != 0.0 && (goal_x+goal_y) != 0.0) {
        k = angleBetweenVectors(dir_x, dir_y, goal_x, goal_y);
        k /= c->w;
    }

    return manhetten_dist(src->x, src->y, dst->x, dst->y)/c->v + k;
}

static void nodeNeighbors(ASNeighborList neighbors, void* srcNode, void* fromsrcNode, void* context) {
    node* src = (node*)srcNode;
    Context *ctx = (Context *)context;

    // todo: get cost for srcNode;

    int i;
    for (i = 0; i < MAX_NODES; i++) {
        // check if node collision by time (cost) with existing paths (get from context)
        if (src->neighbors[i]) {
            uint64_t neighbor_cost = neighborCost(srcNode, (void*)(src->neighbors[i]), fromsrcNode,  context);
            //if ((ctx->timestamp + srcNode_cost + neighbor_cost) is not equal for x/y and cost for all robots
            ASNeighborListAdd(neighbors, (void*)src->neighbors[i], neighbor_cost);
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
    ASPath path;
    for (i = 0; i < MAX_NODES; i++) {
        //printf("node %d :", i);
        for (j = 0; j < MAX_NODES; j++) {
            if (i != j && hopCost(graph[i], graph[j], NULL) < HOP_LEN) { // todo: this must be change
                // set by map allowed direction type
                graph[i]->neighbors[j] = graph[j];
                graph[j]->neighbors[i] = graph[i];

                //printf(" %d,", j);
            }
        }

        //printf("\n");
    }

    Context context;
    context.v = 2.0;
    context.w = 1.0;

    clock_t begin = clock();

    i = 0;
    j = 1023;
    //for (i = 0; i < MAX_NODES; i++) {
    //    for (j = 0; j < MAX_NODES; j++) {
            path = ASPathCreate(&pathSource, (void*)(&context), graph[i], graph[j]);

            hopCount = ASPathGetCount(path);
            uint64_t cost;
            cost = ASPathGetCost(path, hopCount);
            for (int ind=0; ind<hopCount; ind++) {
                cost = ASPathGetCost(path, ind);
                printf("step %d: cost=%f\n", ind, cost/1000.0);
            }
            printf("path from %d to %d: cost=%f, hopCount=%d\n", i, j, cost/1000.0, hopCount);
            ASPathDestroy(path);
    //    }
    //}
    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    printf("Processing time: %f\n", time_spent);
    return 0;
}
