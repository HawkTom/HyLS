//
// Created by hao on 09/06/2020.
//

#include "utils.h"

int costlb = INF;
int costub = 0;
int demandlb = INF;
int demandub = 0;

int edge_index[2];

void readMap(Task *inst_tasks, Arc *inst_arcs, const char *map1)
{
    FILE *fp;

    char dummy[101];

//    fp = fopen("../instances/example.dat", "r");
    char path[101];
    // strcpy(path, "../map/");
    strcpy(path, "map/");
    strcat(path, map1);
    fp = fopen(path, "r");
    if (fp == NULL)
    {
        printf("The file <%s> can't be open\n", path);
        exit(0);
    }

    while (fscanf(fp, "%s", dummy) != EOF)
    {

        if (strcmp(dummy, "VERTICES")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &vertex_num);
        }
        else if (strcmp(dummy, "ARISTAS_REQ") == 0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &req_edge_num);
        }
        else if (strcmp(dummy, "ARISTAS_NOREQ")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &nonreq_edge_num);
        }
        else if (strcmp(dummy, "VEHICULOS")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &vehicle_num);
        }
        else if (strcmp(dummy, "CAPACIDAD")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &capacity);
        }
        else if (strcmp(dummy, "LISTA_ARISTAS_REQ")==0) {

            fscanf(fp, "%s", dummy);
            task_num = 2 * req_edge_num + req_arc_num;
            total_arc_num = task_num + 2 * nonreq_edge_num + nonreq_arc_num;
            for (int i = 1; i <= req_edge_num; i++) {
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d,", &inst_tasks[i].head_node);
                fscanf(fp, "%d)", &inst_tasks[i].tail_node);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_tasks[i].serv_cost);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_tasks[i].demand);

                inst_tasks[i].dead_cost = inst_tasks[i].serv_cost;
                inst_tasks[i].inverse = i + req_edge_num;
                inst_tasks[i].vt = 0;

                inst_tasks[i + req_edge_num].head_node = inst_tasks[i].tail_node;
                inst_tasks[i + req_edge_num].tail_node = inst_tasks[i].head_node;
                inst_tasks[i + req_edge_num].dead_cost = inst_tasks[i].dead_cost;
                inst_tasks[i + req_edge_num].serv_cost = inst_tasks[i].serv_cost;
                inst_tasks[i + req_edge_num].demand = inst_tasks[i].demand;
                inst_tasks[i + req_edge_num].inverse = i;
                inst_tasks[i + req_edge_num].vt = 0;


                inst_arcs[i].head_node = inst_tasks[i].head_node;
                inst_arcs[i].tail_node = inst_tasks[i].tail_node;
                inst_arcs[i].trav_cost = inst_tasks[i].dead_cost;
                inst_arcs[i].change = 0;
                inst_arcs[i].link = 0;
                inst_arcs[i + req_edge_num].head_node = inst_arcs[i].tail_node;
                inst_arcs[i + req_edge_num].tail_node = inst_arcs[i].head_node;
                inst_arcs[i + req_edge_num].trav_cost = inst_arcs[i].trav_cost;
                inst_arcs[i + req_edge_num].change = 0;
                inst_arcs[i+ req_edge_num].link = 0;

                if (costlb > inst_tasks[i].dead_cost)
                    costlb = inst_tasks[i].dead_cost;

                if (costub < inst_tasks[i].dead_cost)
                    costub = inst_tasks[i].dead_cost;

                if (demandlb > inst_tasks[i].demand)
                    demandlb = inst_tasks[i].demand;

                if (demandub < inst_tasks[i].demand)
                    demandub = inst_tasks[i].demand;

            }
        }
        else if (strcmp(dummy, "LISTA_ARISTAS_NOREQ")==0)
        {
            fscanf(fp, "%s", dummy);
            for (int i=task_num+1; i<=task_num+nonreq_edge_num;i++)
            {
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d,", &inst_arcs[i].head_node);
                fscanf(fp, "%d)", &inst_arcs[i].tail_node);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_arcs[i].trav_cost);
                inst_arcs[i].change = 0;
                inst_arcs[i].link = 0;

                inst_arcs[i + nonreq_edge_num].head_node = inst_arcs[i].tail_node;
                inst_arcs[i + nonreq_edge_num].tail_node = inst_arcs[i].head_node;
                inst_arcs[i + nonreq_edge_num].trav_cost = inst_arcs[i].trav_cost;
                inst_arcs[i + nonreq_edge_num].change = 0;
                inst_arcs[i + nonreq_edge_num].link = 0;

                if (costlb > inst_arcs[i].trav_cost)
                    costlb = inst_arcs[i].trav_cost;

                if (costub < inst_arcs[i].trav_cost)
                    costub = inst_arcs[i].trav_cost;

            }
        }
        else if (strcmp(dummy, "DEPOSITO")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &DEPOT);
        }

    }

    fclose(fp);

    inst_tasks[0].head_node = DEPOT;
    inst_tasks[0].tail_node = DEPOT;
    inst_tasks[0].dead_cost = 0;
    inst_tasks[0].serv_cost = 0;
    inst_tasks[0].demand = 0;
    inst_tasks[0].inverse = 0;
    inst_arcs[0].head_node = DEPOT;
    inst_arcs[0].tail_node = DEPOT;
    inst_arcs[0].trav_cost = 0;

    for (int i=1; i<=total_arc_num; i++)
    {
        cost_backup[inst_arcs[i].head_node][inst_arcs[i].tail_node] = inst_arcs[i].trav_cost;
    }
    memset(edge_index, 0, sizeof(edge_index));
    edge_index[0] = req_edge_num;
    edge_index[1] = nonreq_edge_num;
//    edge_index[2] = task_num + nonreq_edge_num;
}

void mod_dijkstra()
{
    memset(min_cost, 0, sizeof(min_cost));
    memset(shortest_path, 0, sizeof(shortest_path));
    int i, j, k, m, minimum;
//    printf("Dijastra\n");
    for (i = 1; i <= vertex_num; i++)
    {
        for (j = 1; j <= vertex_num; j++)
        {
            if (j == i)
                continue;

            shortest_path[i][j][0] = 1;
            shortest_path[i][j][1] = i;
            min_cost[i][j] = INF;
        }
    }

    int mark[MAX_NODE_TAG_LENGTH], dist[MAX_NODE_TAG_LENGTH], dist1[MAX_NODE_TAG_LENGTH], nearest_neighbor[MAX_NODE_TAG_LENGTH];

    for (i = 1; i <= vertex_num; i++)
    {
        mark[i] = 1;

        for (j = 1; j <= vertex_num; j++)
        {
            if (j == i)
                continue;

            mark[j] = 0;
            dist[j] = trav_cost[i][j];
            dist1[j] = dist[j];
        }

        for (k = 1; k < vertex_num; k++)
        {
            minimum = INF;
            nearest_neighbor[0] = 0;

            for (j = 1; j <= vertex_num; j++)
            {
                if (mark[j])
                    continue;

                if (dist1[j] == INF)
                    continue;

                if (dist1[j] < minimum)
                    minimum = dist1[j];
            }

            if (minimum == INF)
                continue;

            for (j = 1; j <= vertex_num; j++)
            {
                if (mark[j])
                    continue;

                if (dist1[j] == minimum)
                {
                    nearest_neighbor[0] ++;
                    nearest_neighbor[nearest_neighbor[0]] = j;
                }
            }

            int v = nearest_neighbor[1];
            dist1[v] = INF;
            mark[v] = 1;

            if (shortest_path[i][v][0] == 0 || (shortest_path[i][v][0] > 0 && shortest_path[i][v][shortest_path[i][v][0]] != v))
            {
                shortest_path[i][v][0] ++;
                shortest_path[i][v][shortest_path[i][v][0]] = v;
            }

            for (j = 1; j <= vertex_num; j++)
            {
                if (mark[j])
                    continue;

                if (minimum+trav_cost[v][j] < dist[j])
                {
                    dist[j] = minimum+trav_cost[v][j];
                    dist1[j] = minimum+trav_cost[v][j];
                    for (m = 0; m <= shortest_path[i][v][0]; m++)
                    {
                        shortest_path[i][j][m] = shortest_path[i][v][m];
                    }
                }
            }

            for (j = 1; j <= vertex_num; j++)
            {
                if (j == i)
                    continue;

                min_cost[i][j] = dist[j];
            }
        }
    }

    for (i = 1; i <= vertex_num; i++)
    {
        for (j = 1; j <= vertex_num; j++)
        {
            if (shortest_path[i][j][0] == 1)
                shortest_path[i][j][0] = 0;
        }
    }

    for (i = 1; i <= vertex_num; i++)
    {
        shortest_path[i][i][0] = 1;
        shortest_path[i][i][1] = i;
        min_cost[i][i] = 0;
    }
}


void update_cost(const Task *inst_tasks, const Arc *inst_arcs)
{

    memset(trav_cost, 0, sizeof(trav_cost));
    // update trav_cost, serve_cost, min_cost after each dynamic change
    for (int i=1; i<=vertex_num; i++)
    {
        for (int j=1; j<=vertex_num; j++)
        {
            trav_cost[i][j] = INF;
            serve_cost[i][j] = 0;
        }
    }

    trav_cost[1][0] = 0;
    trav_cost[0][1] = 0;

    for (int i=1; i<=task_num; i++)
    {
        serve_cost[inst_tasks[i].head_node][inst_tasks[i].tail_node] = inst_tasks[i].serv_cost;
    }

    for (int i=1; i<=total_arc_num; i++)
    {
        // printf("%d, %d: %d\n", inst_arcs[i].head_node, inst_arcs[i].tail_node, inst_arcs[i].trav_cost);
        trav_cost[inst_arcs[i].head_node][inst_arcs[i].tail_node] = inst_arcs[i].trav_cost;
    }
//    printf("\n");
}

void construct_virtual_task(const Task *inst_tasks, Task *tasks_vt, const Rmstate *rms)
{

    int i;

    int actual_req_edge_num = req_edge_num;
    req_edge_num += rms->point[0];
    task_num = 2 * req_edge_num;

    
    for (i = 1; i <= actual_req_edge_num; i++)
    {
        tasks_vt[i].head_node = inst_tasks[i].head_node;
        tasks_vt[i].tail_node = inst_tasks[i].tail_node;
        tasks_vt[i].dead_cost = inst_tasks[i].dead_cost;
        tasks_vt[i].serv_cost = inst_tasks[i].serv_cost;
        tasks_vt[i].demand = inst_tasks[i].demand;
        tasks_vt[i].inverse = i + req_edge_num;
        tasks_vt[i].vt = 0;

        tasks_vt[i + req_edge_num].head_node = tasks_vt[i].tail_node;
        tasks_vt[i + req_edge_num].tail_node = tasks_vt[i].head_node;
        tasks_vt[i + req_edge_num].dead_cost = tasks_vt[i].dead_cost;
        tasks_vt[i + req_edge_num].serv_cost = tasks_vt[i].serv_cost;
        tasks_vt[i + req_edge_num].demand = tasks_vt[i].demand;
        tasks_vt[i + req_edge_num].inverse = i;
        tasks_vt[i + req_edge_num].vt = 0;
    }
    if (rms->point[0] == 0)
    {
        return;
    }

    for (i = actual_req_edge_num+1; i <= req_edge_num; i++)
    {
        tasks_vt[i].head_node = DEPOT;
        tasks_vt[i].tail_node = rms->point[i-actual_req_edge_num];
        tasks_vt[i].dead_cost = 0;
        tasks_vt[i].serv_cost = min_cost[tasks_vt[i].head_node][tasks_vt[i].tail_node];
        tasks_vt[i].demand = capacity - rms->rcap[i-actual_req_edge_num];
        if (tasks_vt[i].demand == 0)
        {
            tasks_vt[i].demand = 1;
        }
        tasks_vt[i].inverse = i + req_edge_num;
        tasks_vt[i].vt = 1;

        tasks_vt[i + req_edge_num].head_node = tasks_vt[i].head_node;
        tasks_vt[i + req_edge_num].tail_node = tasks_vt[i].tail_node;
        tasks_vt[i + req_edge_num].dead_cost = tasks_vt[i].dead_cost;
        tasks_vt[i + req_edge_num].serv_cost = tasks_vt[i].serv_cost;
        tasks_vt[i + req_edge_num].demand = tasks_vt[i].demand;
        tasks_vt[i + req_edge_num].inverse = i;
        tasks_vt[i + req_edge_num].vt = 1;
    }
    tasks_vt[0].head_node = DEPOT;
    tasks_vt[0].tail_node = DEPOT;
    tasks_vt[0].dead_cost = 0;
    tasks_vt[0].serv_cost = 0;
    tasks_vt[0].demand = 0;
    tasks_vt[0].inverse = 0;

    


}

void check_solution_valid(Individual solution, const Task *inst_task)
{
    int i, j;
    int used[MAX_TASK_SEQ_LENGTH];
    memset(used, 0, sizeof(used));
    for(i = 1; i <= solution.Sequence[0]; i++)
    {
        if (solution.Sequence[i] == 0)
        {
            continue;
        }
        if (solution.Sequence[i] <= req_edge_num)
        {
            used[solution.Sequence[i]] = 1;
        } else
        {
            used[inst_task[solution.Sequence[i]].inverse] = 1;
        }
    }
    int flag = 0;
    // printf("lack of: ");
    for (i = 1; i <= req_edge_num; i++)
    {
        if(used[i] == 0)
        {
            printf("%d \t", i);
            flag = 1;
        }
    }
    // printf("\n");
    // get_each_load(solution.Sequence, inst_task);
    if ( solution.TotalCost != get_task_seq_total_cost(solution.Sequence, inst_task))
    {
        flag = 1;
        printf("solution's cost error\n");
    }
    if (flag)
    {
        printf("\n");
        exit(0);
    }
}