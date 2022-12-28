#include "AStar.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>

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


typedef struct Context {
    float v;
    float w;
    //uint64_t timestamp;
} Context;

static float manhetten_dist(float x1, float y1, float x2, float y2) {
    float x_delta, y_delta;
    // to avoid needing abs(), subtract the smaller from the greater
    x_delta = (x1 > x2) ? (x1 - x2) : (x2 - x1);
    y_delta = (y1 > y2) ? (y1 - y2) : (y2 - y1);
    return x_delta + y_delta;
}

// Use for heuristic only (node to goal dist)
static float hopCost(void *srcNode, void *dstNode, void *context) {
    node* src = (node*)srcNode;
    node* dst = (node*)dstNode;
    //Context *c = (Context*)context;

    return manhetten_dist(src->x, src->y, dst->x, dst->y);
}

static float angleBetweenVectors(float x1, float y1, float x2, float y2)
{   
    // float dot = x1*x2+y1*y2;
    // if (dot > 0) { // в одну сторону
    //     return 0.0;
    // } else if (dot < 0) { // в противоположные стороны
    //     return 3.14;
    // }// перпендикулярно
    // return 3.14/2.0;

    return 3.14*(1.0-((x1*x2+y1*y2)/(sqrt(x1*x1+y1*y1)*sqrt(x2*x2+y2*y2))))/2.0; // improve perfomance
}

static float neighborCost(void *srcNode, void *dstNode, void *fromsrcNode, void *context) {
    node* src = (node*)srcNode;
    node* dst = (node*)dstNode;
    node* fsrc = (node*)fromsrcNode;
    Context *c = (Context*)context;

    if (src->x == dst->x && src->y == dst->y) {
        return 0.0;
    }

    // Increase cost koefficient for change direction considering (1.0 for 180 deg. and 0.0 for 0 deg.)
    float k = 0.0;
    // Get current direction of AGV
    float dir_x, dir_y;
    dir_x = src->x - fsrc->x;
    dir_y = src->y - fsrc->y;

    // Get target direction of AGV
    float goal_x, goal_y;
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
            float neighbor_cost = neighborCost(srcNode, (void*)(src->neighbors[i]), fromsrcNode,  context);
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

    //i = 0;
    //j = 1023;
    float cost;
    for (i = 0; i < MAX_NODES; i++) {
        printf("%d\n", 100*i/MAX_NODES);
        for (j = 0; j < MAX_NODES; j++) {
            //if (i != j) {
                ASPath path = ASPathCreate(&pathSource, (void*)(&context), graph[i], graph[j]);

                hopCount = ASPathGetCount(path);
                cost = ASPathGetCost(path, hopCount);
                for (int ind=0; ind<hopCount; ind++) {
                    cost = ASPathGetCost(path, ind);
                    //printf("step %d: cost=%f\n", ind, cost);
                }
                ASPathDestroy(path);
            //}
        }
        system("clear");
    }
    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    time_spent /= MAX_NODES*MAX_NODES;
    printf("path from %d to %d: cost=%f, hopCount=%d\n", i, j, cost, hopCount);
    printf("Processing time: %f\n", time_spent);
    return 0;
}
