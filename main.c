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
#define MAX_NEIGHBORS 6

typedef struct node {
    size_t index;
    float x;
    float y;
    size_t neighbors_count;
    struct node* neighbors[MAX_NEIGHBORS];
} node;


typedef struct Context {
    float v;
    float w;
    float timestamp;
} Context;

static float manhetten_dist(float x1, float y1, float x2, float y2) {
    float x_delta, y_delta;
    // to avoid needing abs(), subtract the smaller from the greater
    x_delta = (x1 > x2) ? (x1 - x2) : (x2 - x1);
    y_delta = (y1 > y2) ? (y1 - y2) : (y2 - y1);
    return x_delta + y_delta;
}

static float euclidian_dist(float x1, float y1, float x2, float y2) {
    float x_delta, y_delta;
    x_delta = x1 - x2;
    y_delta = y1 - y2;
    return sqrt(x_delta*x_delta + y_delta*y_delta);
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

    return euclidian_dist(src->x, src->y, dst->x, dst->y)/c->v + k;
}

static void nodeNeighbors(ASNeighborList neighbors, void* srcNode, float srcNode_cost, void* fromsrcNode, void* context) {
    node* src = (node*)srcNode;
    Context *ctx = (Context *)context;

    int i;
    for (i = 0; i < src->neighbors_count; i++) {
        // check if node collision by time (cost) with existing paths (get from context)
        if (src->neighbors[i]) {
            float neighbor_cost = neighborCost(srcNode, (void*)(src->neighbors[i]), fromsrcNode,  context);
            
            // get cost for all robots path in src->neighbors[i] node
            
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

    // set graph poses
    for (i = 0; i < MAX_NODES; i++) {
        graph[i] = calloc(1, sizeof(node));
        graph[i]->index = i;
        graph[i]->neighbors_count = 0; // paranoja mod. Calloc init as 0 by default.
        graph[i]->x = i*X_STEP % MAX_POS;
        graph[i]->y = (int)(i/Y_PER_ROW)*Y_STEP % MAX_POS;
        for (j = 0; j < MAX_NEIGHBORS;  j++) graph[i]->neighbors[j] = NULL; // paranoja mod. Calloc init as 0 by default.
    }

    // fill graph edges
    int hopCount;
    for (i = 0; i < MAX_NODES; i++) {
        for (j = 0; j < MAX_NODES; j++) {
            if (i != j && hopCost(graph[i], graph[j], NULL) < HOP_LEN) { // todo: this must be change
                // set by map allowed direction type
                if (graph[i]->neighbors_count < MAX_NEIGHBORS)
                    graph[i]->neighbors[graph[i]->neighbors_count++] = graph[j];
                if (graph[j]->neighbors_count < MAX_NEIGHBORS)
                    graph[j]->neighbors[graph[j]->neighbors_count++] = graph[i];
            }
        }
    }
    Context context;
    context.v = 2.0;
    context.w = 1.0;
    context.timestamp = clock() / CLOCKS_PER_SEC;

    clock_t begin = clock();

    i = 0;
    j = 1023;
    float cost;
    for (i = 0; i < MAX_NODES; i++) {
        printf("Processing... %d%\n", 100*i/MAX_NODES);
        for (j = 0; j < MAX_NODES; j++) {

                ASPath path = ASPathCreate(&pathSource, (void*)(&context), graph[i], graph[j]);
                ASPath path_ = ASPathCopy(path);
                hopCount = ASPathGetCount(path_);
                cost = ASPathGetCost(path_, hopCount);
                for (int ind=0; ind<hopCount; ind++) {
                    cost = ASPathGetCost(path_, ind);
                    node *n = (node*)ASPathGetNode(path_, ind);
                    //printf("index %ld: x=%f y=%f cost=%f neighbors=%ld\n", n->index, n->x, n->y, cost, n->neighbors_count);
                }
                ASPathDestroy(path);
                ASPathDestroy(path_);
        }
        system("clear");
    }
    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    //time_spent /= MAX_NODES*MAX_NODES;
    //printf("path from %d to %d: cost=%f, hopCount=%d\n", i, j, cost, hopCount);
    printf("Processing time: %f (avg: %f)\n", time_spent, time_spent/(MAX_NODES*MAX_NODES));
    return 0;
}
