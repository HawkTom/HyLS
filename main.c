#include <stdlib.h>
#include <stdio.h>
#include <omp.h>

#include "utils.h"
#include "heuristic.h"
#include "lsoperator.h"



#define AXVSIZE 15

int req_arc_num = 0; //NRA
int req_edge_num; //NRE
int nonreq_arc_num = 0;
int nonreq_edge_num;
int vertex_num;
int vehicle_num;
int capacity;
int task_num;
int total_arc_num;
int DEPOT;
int trav_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
int serve_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
int min_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
int shortest_path[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];

int cost_backup[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
char map[20];

double remain_cap_ratio_lb = 0;
double remain_cap_ratio_ub = 0.5;

int terminal_condition = 20; //if the best has nevere been changed over 20 iterations, stop the algorithm
int terminal_duration = 5;
int debug = 0;
int logflag = 0;
char map[20];



Individual best_si_solution;
Individual best_di_solution;
Individual best_swap_solution;
Individual best_rev_solution;
Individual best_cross_solution;

Individual best1_si_solution;
Individual best1_di_solution;
Individual best1_swap_solution;
Individual best1_rev_solution;
Individual best1_cross_solution;

Individual best_lma_solution;

Individual IndiList[20];
int IndiListLength = 0;

double total_time = 0;
int best= INF;



void get_first_solution(Individual *solution, const Task *inst_tasks);
void process_rmstate(const Task *inst_tasks_vt, Rmstate *rms, Individual *solution);
int get_additional_cost(Rmstate *rms);
int update_global_best();
int update1_global_best();
void copy_individual(Individual *dest, Individual *src);


void method(Individual InitSolution, const Task *inst_tasks_vt);
void method1(Individual *InitSolution, const Task *inst_tasks_vt);
void HyLS(Individual *InitSolution, const Task *inst_tasks_vt);
void sequential_moves(Individual *InitSolution, const Task *inst_tasks_vt);
void clear_IndiList();

void cal_tasks_num(int *seq);

int main(int argc, char *argv[])
{   
    printf("hello world xxx\n");

    char path[30];


    // strcpy(map, "egl/egl-e4-A.dat");
    // strcpy(path, "3/e4-A.dat");
    // int instance = 1;

    sprintf(map, "egl/egl-%s.dat", argv[1]);
    sprintf(path, "3/%s.dat", argv[1]);
    int instance = atoi(argv[2]);

    printf("%s %s %d\n", map, path, instance);
    // exit(0);

    printf("path: %s\n", map);
    FILE *fp;
    if (instance == 1)
    {
        fp = fopen(path, "w");
        if (fp == NULL)
        {
            printf("can't open %s\n", path);
        }
        fprintf(fp, "%s\n", map);
    } else
    {
        fp = fopen(path, "a");
        if (fp == NULL)
        {
            printf("can't open %s\n", path);
        }        
    }
    
    fclose(fp);

    // initial a seed
    int seed = 1602922520;
    srand(seed);

    // load map
    Task inst_tasks[MAX_TASKS_TAG_LENGTH];
    Task inst_tasks_vt[MAX_TASKS_TAG_LENGTH];
    Arc inst_arcs[MAX_ARCS_TAG_LENGTH];
    Vehicles state;

    readMap(inst_tasks, inst_arcs, map);
    readMap(inst_tasks_vt, inst_arcs, map);

    update_cost(inst_tasks, inst_arcs);
    mod_dijkstra();

    // use a optimizer to obtain an initial solution
    Individual firstSolution;
    get_first_solution(&firstSolution, inst_tasks);
    // MAENS(inst_tasks, &firstSolution);

    printf("First solution's cost: %d\n", firstSolution.TotalCost);

    // execute the solution and simulate the changes
    seed = time(NULL) + instance * 7654321;
    // seed = 1618871908;
    printf("seed: %d \n", seed);
    

    Rmstate rms;
    nextScenario(&firstSolution, inst_tasks, inst_arcs, &rms, seed);
    // update graph
    update_cost(inst_tasks, inst_arcs);
    mod_dijkstra();
    // update the solution


    construct_virtual_task(inst_tasks, inst_tasks_vt, &rms);
    Individual InitSolution, GlobalBest;
    process_rmstate(inst_tasks_vt, &rms, &InitSolution);
    int additional_cost = get_additional_cost(&rms);

    printf("instance: %d remain_tasks %d vehicles %d \n", instance, req_edge_num-rms.point[0], rms.point[0]); 


    // the begining of optimizatin for DCARP instance
    // local search portfolio

    

    fp = fopen(path, "a");
    fprintf(fp, "instance %d seed %d addcost %d remain_tasks %d vehicles %d \n", instance, seed, additional_cost, req_edge_num-rms.point[0], rms.point[0]);
    fclose(fp);



    
    
    Individual InitSolution0;
    copy_individual(&InitSolution0, &InitSolution);

    best = INF;
    IndiListLength = 0;
    clear_IndiList();
    
    
    Individual ILSIndi, ILSIndi1;
    for (int run = 1; run <= 25; run ++)
    {
        int seed2 = time(NULL) + instance * 7654321;
        srand(seed2);
        copy_individual(&ILSIndi, &InitSolution0);
        copy_individual(&ILSIndi1, &InitSolution0);
        int ILSbest, ILSbest1;
        #pragma omp parallel sections
        {
            #pragma omp section
            ILSbest = ILS(&ILSIndi, inst_tasks_vt);
            #pragma omp section
            ILSbest1 = ILS1(&ILSIndi1, inst_tasks_vt);
        }
        // cal_tasks_num(InitSolution.Sequence);
        printf("run: %d ILS: %d ILS1: %d \n", run, ILSbest - additional_cost, ILSbest1 - additional_cost);
        fp = fopen(path, "a");
        fprintf(fp, "run %d seed %d ILS %d ILS1 %d \n", run, seed, ILSbest - additional_cost, ILSbest1 - additional_cost);
        fclose(fp);
    }

    copy_individual(&InitSolution, &InitSolution0);
    HyLS(&InitSolution, inst_tasks_vt);
    // cal_tasks_num(InitSolution.Sequence);
    printf("HyLS: %d\n", best-additional_cost);
    fp = fopen(path, "a");
    fprintf(fp, "HyLS %d ", best-additional_cost);
    fclose(fp);

    copy_individual(&InitSolution, &InitSolution0);
    lma_moves(&InitSolution, inst_tasks_vt);
    printf("LMA Move: %d\n", InitSolution.TotalCost-additional_cost);
    fp = fopen(path, "a");
    fprintf(fp, "LMA %d ", InitSolution.TotalCost-additional_cost);
    fclose(fp);


    copy_individual(&InitSolution, &InitSolution0);
    sequential_moves(&InitSolution, inst_tasks_vt);
    printf("Seq Move: %d\n", InitSolution.TotalCost-additional_cost);
    fp = fopen(path, "a");
    fprintf(fp, "Seq %d\n", InitSolution.TotalCost-additional_cost);
    fclose(fp);
    


    return 0;
}


void cal_tasks_num(int *seq)
{
    int k = 0;
    for (int i = 1; i <= seq[0]; i++)
    {
        // if (seq[i] == 0) continue;
        printf("%d  ", seq[i]);
        k ++;
    }
    printf("total: %d\n", k);
}


void clear_IndiList()
{
    for (int i = 1; i < 20; i++)
    {
        memset(IndiList[i].Sequence, 0, sizeof(IndiList[i].Sequence));
        memset(IndiList[i].Loads, 0,  sizeof(IndiList[i].Loads));
        IndiList[i].TotalCost = 0;
        IndiList[i].TotalVioLoad = 0;
        IndiList[i].Fitness = 0;
    }
}


int ILS(Individual *InitSolution, const Task *inst_tasks_vt)
{
    clock_t start_t, finish_t;
    start_t = clock();
    double duration = 0.0;

    int ILS_best = INF;
    while(1)
    {
        method1(InitSolution, inst_tasks_vt);
        if (InitSolution->TotalCost < ILS_best)
        {
            ILS_best = InitSolution->TotalCost;
        }

        memset(InitSolution->Assignment, 0, sizeof(InitSolution->Assignment));
        for (int i =1; i <= InitSolution->Sequence[0]; i++)
        {
            if (InitSolution->Sequence[i] == 0) continue;
            InitSolution->Assignment[0] ++;
            InitSolution->Assignment[InitSolution->Assignment[0]] = InitSolution->Sequence[i];        
        }
        memset(InitSolution->Sequence, 0, sizeof(InitSolution->Sequence));
        memset(InitSolution->Loads, 0,  sizeof(InitSolution->Loads));
        InitSolution->TotalCost = 0;
        InitSolution->TotalVioLoad = 0;
        InitSolution->Fitness = 0;  

        int perm_index[250];
        rand_perm(perm_index, InitSolution->Assignment[0]);
        
        int one_seq[250];
        memcpy(one_seq, InitSolution->Assignment, sizeof(InitSolution->Assignment));

        for (int i = 1; i <= one_seq[0]; i++)
        {
            InitSolution->Assignment[i] = one_seq[perm_index[i]];
        }

        InitSolution->TotalCost = split(InitSolution->Sequence, InitSolution->Assignment, inst_tasks_vt);

        Individual tmpSolution;
        move_vt_to_first(&tmpSolution, InitSolution, inst_tasks_vt);
        copy_individual(InitSolution, &tmpSolution);
        finish_t = clock();
        duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
        // printf("ILS: %d %f\n", ILS_best, duration);
        if (duration > terminal_duration) break;
    }
    // printf("ILS: %d %f\n", ILS_best, duration);
    return ILS_best;
}


int ILS1(Individual *InitSolution, const Task *inst_tasks_vt)
{
    clock_t start_t, finish_t;
    start_t = clock();
    double duration = 0.0;

    int ILS_best = INF;
    while(1)
    {
        sequential_moves(InitSolution, inst_tasks_vt);
        if (InitSolution->TotalCost < ILS_best)
        {
            ILS_best = InitSolution->TotalCost;
        }

        memset(InitSolution->Assignment, 0, sizeof(InitSolution->Assignment));
        for (int i =1; i <= InitSolution->Sequence[0]; i++)
        {
            if (InitSolution->Sequence[i] == 0) continue;
            InitSolution->Assignment[0] ++;
            InitSolution->Assignment[InitSolution->Assignment[0]] = InitSolution->Sequence[i];        
        }
        memset(InitSolution->Sequence, 0, sizeof(InitSolution->Sequence));
        memset(InitSolution->Loads, 0,  sizeof(InitSolution->Loads));
        InitSolution->TotalCost = 0;
        InitSolution->TotalVioLoad = 0;
        InitSolution->Fitness = 0;  

        int perm_index[250];
        rand_perm(perm_index, InitSolution->Assignment[0]);
        
        int one_seq[250];
        memcpy(one_seq, InitSolution->Assignment, sizeof(InitSolution->Assignment));

        for (int i = 1; i <= one_seq[0]; i++)
        {
            InitSolution->Assignment[i] = one_seq[perm_index[i]];
        }

        InitSolution->TotalCost = split(InitSolution->Sequence, InitSolution->Assignment, inst_tasks_vt);

        Individual tmpSolution;
        move_vt_to_first(&tmpSolution, InitSolution, inst_tasks_vt);
        copy_individual(InitSolution, &tmpSolution);
        finish_t = clock();
        duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
        // printf("ILS: %d %f\n", ILS_best, duration);
        if (duration > terminal_duration) break;
    }
    // printf("ILS: %d %f\n", ILS_best, duration);
    return ILS_best;
}


void move_vt_to_first(Individual *dst, Individual *src, const Task *inst_tasks)
{
    int i, load;
    load = 0;
    memset(dst->Sequence, 0, sizeof(int) * MAX_TASK_SEQ_LENGTH);
    memset(dst->Loads, 0, sizeof(int) * 50);
    dst->Sequence[0] = 1;
    dst->Sequence[1] = 0;
    for (i = 2; i <= src->Sequence[0]; i++)
    {
        if (src->Sequence[i] == 0)
        {
            dst->Sequence[0] ++;
            dst->Sequence[dst->Sequence[0]] = 0;
            dst->Loads[0] ++;
            dst->Loads[dst->Loads[0]] = load;
            load = 0;
            continue;
        }
        if (inst_tasks[src->Sequence[i]].vt > 0 && src->Sequence[i-1] != 0)
        {
            dst->Sequence[0] ++;
            dst->Sequence[dst->Sequence[0]] = 0;
            dst->Loads[0] ++;
            dst->Loads[dst->Loads[0]] = load;
            load = 0;
        }

        load += inst_tasks[src->Sequence[i]].demand;
        dst->Sequence[0] ++;
        dst->Sequence[dst->Sequence[0]] = src->Sequence[i];
    }
    dst->TotalCost = src->TotalCost;
}


void HyLS(Individual *InitSolution, const Task *inst_tasks_vt)
{
    for (int i = 1; i < AXVSIZE; i++)
    {
        method1(InitSolution, inst_tasks_vt);
        if (InitSolution->TotalCost < best)
        {
            best = InitSolution->TotalCost;
        }
        copy_individual(InitSolution, &IndiList[i]);
        if (i >= IndiListLength)
        {
            break;
        }
    }
}

void method1(Individual *InitSolution, const Task *inst_tasks_vt)
{

    copy_individual(&best1_si_solution, InitSolution);
    copy_individual(&best1_di_solution, InitSolution);
    copy_individual(&best1_swap_solution, InitSolution);
    copy_individual(&best1_rev_solution, InitSolution);
    copy_individual(&best1_cross_solution, InitSolution);

    int improves[6];
    int terminate_flag = 0;

    clock_t start_t, finish_t;
    start_t = clock();
    double duration = 0.0, duration1=0.0;
    
    while (1)
    {
        memset(improves, 0, sizeof(improves));
        // printf("Log: %d %d\n", InitSolution.TotalCost, additional_cost);

        // improves[1] = client_single_insertion(&InitSolution, inst_tasks_vt);
        // improves[2] = client_double_insertion(&InitSolution, inst_tasks_vt);
        // improves[3] = client_swap(&InitSolution, inst_tasks_vt);
        // improves[4] = client_reverse(&InitSolution, inst_tasks_vt);
        // improves[5] = client_cross(&InitSolution, inst_tasks_vt);

        #pragma omp parallel sections
        {
            #pragma omp section
            improves[1] = client1_single_insertion(InitSolution, inst_tasks_vt);
            if (improves[1] && IndiListLength <= AXVSIZE - 1)
            {
                IndiListLength ++;
                copy_individual(&IndiList[IndiListLength], &best1_si_solution);
            }
            
                
            #pragma omp section
            improves[2] = client1_double_insertion(InitSolution, inst_tasks_vt);
            if (improves[2]  && IndiListLength <= AXVSIZE - 1)
            {
                IndiListLength ++;
                copy_individual(&IndiList[IndiListLength], &best1_di_solution);
            }

            #pragma omp section
            improves[3] = client1_swap(InitSolution, inst_tasks_vt);
            if (improves[3]  && IndiListLength <= AXVSIZE - 1)
            {
                IndiListLength ++;
                copy_individual(&IndiList[IndiListLength], &best1_swap_solution);
            }

            #pragma omp section
            improves[4] = client1_reverse(InitSolution, inst_tasks_vt);
            if (improves[4]  && IndiListLength <= AXVSIZE - 1)
            {
                IndiListLength ++;
                copy_individual(&IndiList[IndiListLength], &best1_rev_solution);
            }

            #pragma omp section
            improves[5] = client1_cross(InitSolution, inst_tasks_vt);
            if (improves[5]  && IndiListLength <= AXVSIZE - 1)
            {
                IndiListLength ++;
                copy_individual(&IndiList[IndiListLength], &best1_cross_solution);
            }
        }

        finish_t = clock();
        duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
        terminate_flag = 0;
        for (int i = 1; i <= 5; i++)
        {
            // printf("%d\t", improves[i]);
            terminate_flag += improves[i];
        }
        // printf("\n");

        int flag = update1_global_best();

        switch (flag)
        {
        case 1:
            copy_individual(InitSolution, &best1_si_solution);
            break;
        case 2:
            copy_individual(InitSolution, &best1_di_solution);
            break;
        case 3:
            copy_individual(InitSolution, &best1_swap_solution);
            break;
        case 4:
            copy_individual(InitSolution, &best1_rev_solution);
            break;
        case 5:
            copy_individual(InitSolution, &best1_cross_solution);
            break;
        default:
            break;
        }

        if (terminate_flag == 0 || duration > terminal_duration)
        {
            break;
        }
        // printf("1 -> Log: %d %d\n", InitSolution->TotalCost, terminate_flag);
    }
    total_time += duration;
    // printf("1 -> Log: %d\n", InitSolution->TotalCost);
    
}

void sequential_moves(Individual *InitSolution, const Task *inst_tasks_vt)
{
    copy_individual(&best_si_solution, InitSolution);
    copy_individual(&best_di_solution, InitSolution);
    copy_individual(&best_swap_solution, InitSolution);

    int improves[4];
    int terminate_flag = 0;

    clock_t start_t, finish_t;
    start_t = clock();
    double duration = 0.0, duration1=0.0;
    
    while (1)
    {
        memset(improves, 0, sizeof(improves));
        // printf("Log: %d %d\n", InitSolution.TotalCost, additional_cost);

        #pragma omp parallel sections
        {
            #pragma omp section
            improves[1] = client_single_insertion(InitSolution, inst_tasks_vt);
                
            #pragma omp section
            improves[2] = client_double_insertion(InitSolution, inst_tasks_vt);

            #pragma omp section
            improves[3] = client_swap(InitSolution, inst_tasks_vt);
        }

        finish_t = clock();
        duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
        terminate_flag = 0;
        for (int i = 1; i <= 3; i++)
        {
            terminate_flag += improves[i];
        }
        // printf("\n");

        // int flag = update_global_best();
        if (best_si_solution.TotalCost < InitSolution->TotalCost)
        {
            copy_individual(InitSolution, &best_si_solution);
        }
        if (best_di_solution.TotalCost < InitSolution->TotalCost)
        {
            copy_individual(InitSolution, &best_di_solution);
        }
        if (best_swap_solution.TotalCost < InitSolution->TotalCost)
        {
            copy_individual(InitSolution, &best_swap_solution);
        }

        if (terminate_flag == 0 || duration > terminal_duration)
        {
            break;
        }
        // printf("Seq -> Log: %d %d\n", InitSolution->TotalCost, terminate_flag);
    }
    // printf("Sequential -> Log: %d time: %f \n", InitSolution->TotalCost, duration);
}





int update_global_best()
{
    int cost[6];
    cost[1] = best_si_solution.TotalCost;
    cost[2] = best_di_solution.TotalCost;
    cost[3] = best_swap_solution.TotalCost;
    cost[4] = best_rev_solution.TotalCost;
    cost[5] = best_cross_solution.TotalCost;


    int i, k = 0;
    int mincost = INF;

    for (i = 1; i <= 5; i++)
    {
        // printf("%d \t", cost[i]);
        if (cost[i] < mincost)
        {
            mincost = cost[i];
            k = i;
        }
    }
    // printf("minvalue:%d, minindex:%d \n", mincost, k);
    return k;
}

int update1_global_best()
{
    int cost[6];
    cost[1] = best1_si_solution.TotalCost;
    cost[2] = best1_di_solution.TotalCost;
    cost[3] = best1_swap_solution.TotalCost;
    cost[4] = best1_rev_solution.TotalCost;
    cost[5] = best1_cross_solution.TotalCost;


    int i, k = 0;
    int mincost = INF;

    for (i = 1; i <= 5; i++)
    {
        // printf("%d \t", cost[i]);
        if (cost[i] < mincost)
        {
            mincost = cost[i];
            k = i;
        }
    }
    // printf("minvalue:%d, minindex:%d \n", mincost, k);
    return k;
}


void copy_individual(Individual *dest, Individual *src)
{
    memcpy(dest->Sequence, src->Sequence, sizeof(src->Sequence));
    memcpy(dest->Loads, src->Loads, sizeof(src->Loads));
    dest->TotalCost = src->TotalCost;
}

void get_first_solution(Individual *solution, const Task *inst_tasks)
{
    int ServMark[2*req_edge_num+1]; // the mark for served tasks
    memset(ServMark, 1, sizeof(ServMark));
    path_scanning(solution, inst_tasks, ServMark);
}

void process_rmstate(const Task *inst_tasks_vt, Rmstate *rms, Individual *solution)
{
    int i,j;
    int task_seq[MAX_TASK_SEQ_LENGTH];
    int actual_req_edge_num = req_edge_num - rms->point[0];


    for (i = 1; i <= rms->rroutes[0][0]; i++)
    {
        for (j = 1; j <= rms->rroutes[i][0]; j++)
        {
            if (rms->rroutes[i][j] > actual_req_edge_num)
            {
                rms->rroutes[i][j] += rms->point[0];
            }
        }
    }

    memset(task_seq, 0, sizeof(task_seq));
    task_seq[0] = 1;
    int load = 0;
    for (i = 1; i <= rms->point[0]; i++)
    {
        task_seq[0] ++;
        task_seq[task_seq[0]] =  actual_req_edge_num + i;
        // rms->rroutes[i][1] = actual_req_edge_num + i;
        JoinArray(task_seq, rms->rroutes[i]); // one more zero ***
        
        // rms->rroutes
    }
    memcpy(solution->Sequence, task_seq, sizeof(task_seq));
    solution->TotalCost = get_task_seq_total_cost(task_seq, inst_tasks_vt);
}

int get_additional_cost(Rmstate *rms)
{
    int total_cost = 0;
    for (int i = 1; i <= rms->point[0]; i++)
    {
        total_cost += min_cost[rms->point[i]][DEPOT];
        // rms->rroutes
    }
    return total_cost;
}