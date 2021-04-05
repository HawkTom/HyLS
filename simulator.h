#ifndef LOSP_SIMULATOR_H
#define LOSP_SIMULATOR_H


#include "utils.h"

void dfs(int u, int *visited, int *parent, int *low, int *disc, int *time, int (* AdMatrix)[MAX_NODE_TAG_LENGTH]);
void findBridge(int (* AdMatrix)[MAX_NODE_TAG_LENGTH]);

typedef struct edge
{
    int head_node;
    int tail_node;
    int trav_cost;
    int demand;
    int link;
    unsigned int change;
    int unserved;
} Edge;

// typedef struct remain_state
// {
//     int point;
//     int rcap;
//     int rseq[MAX_TASK_SEQ_LENGTH];
// } Rmstate;

void saveGraph(Edge *graph, Vehicles info, int vnum, int new_req_edge_num, int new_nonreq_edge_num, int edge_num);

#endif //DCARP_SIMULATOR_H