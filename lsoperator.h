#include "utils.h"


#define LAMBDA 100000

typedef struct lns_route{
    int Route[101][MAX_TASK_SEQ_LENGTH];
    int total_cost;
    int loads[101];
    int total_vio_loads;
    double fitness;
} lns_route;


int lma_single_insertion(lns_route *curr_solution, lns_route *next_solution, int u, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks);
int lma_double_insertion(lns_route *curr_solution, lns_route *next_solution, int u, int x, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks);
int lma_swap(lns_route *curr_solution, lns_route *next_solution, int u, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks);
int lma_reverse(lns_route *curr_solution, lns_route *next_solution, int u, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks);
int lma_cross(lns_route *curr_solution, lns_route *next_solution, int u, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks);

void update_global_best_solution(lns_route *route_solution, Individual *global_solution, const Task *inst_tasks, const char *type);

int check_cost1(lns_route curr_solution, const Task *inst_tasks);
// void client_single_insertion(Individual *indi, const Task *inst_tasks);
// void client_double_insertion(Individual *indi, const Task *inst_tasks);
// void client_swap(Individual *indi, const Task *inst_tasks);
// void client_reverse(Individual *indi, const Task *inst_tasks);
// void client_cross(Individual *indi, const Task *inst_tasks);

int client_single_insertion(Individual *indi, const Task *inst_tasks);
int client_double_insertion(Individual *indi, const Task *inst_tasks);
int client_swap(Individual *indi, const Task *inst_tasks);
int client_reverse(Individual *indi, const Task *inst_tasks);
int client_cross(Individual *indi, const Task *inst_tasks);


int client1_single_insertion(Individual *indi, const Task *inst_tasks);
int client1_double_insertion(Individual *indi, const Task *inst_tasks);
int client1_swap(Individual *indi, const Task *inst_tasks);
int client1_reverse(Individual *indi, const Task *inst_tasks);
int client1_cross(Individual *indi, const Task *inst_tasks);

void lma_moves(Individual *indi, const Task *inst_tasks);
void move_vt_to_first(Individual *dst, Individual *src, const Task *inst_tasks);
int ILS(Individual *InitSolution, const Task *inst_tasks_vt);
int ILS1(Individual *InitSolution, const Task *inst_tasks_vt);