//
// Created by hao on 08/06/2020.
//

#include "simulator.h"

int choose_stop_time(int *cost);
void random_stop_for_solution(Individual *Solution, const Task *inst_tasks, Rmstate *rmstate);
void small_dynamic_change(Task *inst_tasks, Arc *inst_arcs, Rmstate *rmstate, unsigned int seed);


void nextScenario(Individual *Solution, Task *inst_tasks, Arc *inst_arcs, Rmstate *rmstate, unsigned int seed)
{
    int tau = 400;
    int serve_tasks[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];;
    memset(serve_tasks, 0, sizeof(serve_tasks));

    srand(seed);
    random_stop_for_solution(Solution, inst_tasks, rmstate);
    small_dynamic_change(inst_tasks, inst_arcs, rmstate, seed);
}

void random_stop_for_solution(Individual *Solution, const Task *inst_tasks, Rmstate *rmstate)
{
    int i, j;
    int Route[101][MAX_TASK_SEQ_LENGTH];
    memset(Route, 0, sizeof(Route));

    int Positions[101], route_index[101];
    find_ele_positions(Positions, Solution->Sequence, 0);
    Route[0][0] = Positions[0]-1;
    for (i=1; i < Positions[0]; i++)
    {
        AssignSubArray(Solution->Sequence, Positions[i], Positions[i+1], Route[i]);
        route_index[0]++;
        route_index[route_index[0]] = i;
    }

    int schedule[101][MAX_TASK_SEQ_LENGTH];
    memset(schedule, 0, sizeof(schedule));

    if (route_index[0] <= vehicle_num)
    {
        memcpy(schedule, Route, sizeof(Route));
    } else
    {
        // get cost of each route
        int cost[101];
        memset(cost, 0, sizeof(cost));
        cost[0] = 0;
        for (i = 1; i <= Route[0][0]; i++)
        {
            cost[0] ++;
            cost[cost[0]] = get_task_seq_total_cost(Route[i], inst_tasks);
        }

        int tmp;
        for (i = 1; i < route_index[0]; i++)
        {
            for (j = i+1; j <= route_index[0]; j++)
            {
                if (cost[route_index[i]] < cost[route_index[j]])
                {
                    tmp = route_index[i];
                    route_index[i] = route_index[j];
                    route_index[j] = tmp;
                }
            }
        }

        schedule[0][0] = vehicle_num;
        int k = 0, flag = 0;
        for (i = 1; i <= route_index[0]; i++)
        {
            if (k == 0)
            {
                flag = 1 - flag;
            }
            
            if(flag == 1)
            {
                k ++;
                JoinArray(schedule[k], Route[route_index[i]]);
                schedule[k][0] --;
            } else
            {
                JoinArray(schedule[k], Route[route_index[i]]);
                schedule[k][0] --;
                k --;
            }

            if (k == vehicle_num)
            {
                flag = 1 - flag;
            }
        }
        for (i = 1; i <= vehicle_num; i++)
        {
            schedule[i][0] ++;
        }

    }
    

    int RemainRoute[101][MAX_TASK_SEQ_LENGTH];
    memset(RemainRoute, 0, sizeof(RemainRoute));
    int stop_point[101];
    memset(stop_point, 0, sizeof(stop_point));
    int remain_capacity[101];
    memset(remain_capacity, 0, sizeof(remain_capacity));
    int tmp_tour[500], start, end, load;
    int availEdge[MAX_ARCS_TAG_LENGTH][4];
    int stop_task_idx;

    for (i = 1; i <= schedule[0][0]; i ++)
    {
        
        // choose a suitable stop task  index
        memset(availEdge, 0, sizeof(availEdge));
        load = 0;
        for (j = 2; j < schedule[i][0]; j++)
        {
            if (schedule[i][j+1] == 0 || schedule[i][j] == 0)
            {
                load = 0;
                continue;
            }
            
            load += inst_tasks[schedule[i][j]].demand;
            availEdge[0][0]++;
            availEdge[availEdge[0][0]][0] = inst_tasks[schedule[i][j]].tail_node;
            availEdge[availEdge[0][0]][1] = inst_tasks[schedule[i][j+1]].head_node;
            availEdge[availEdge[0][0]][2] = load;
            availEdge[availEdge[0][0]][3] = j+1;
        }

        int total_avail = availEdge[0][0];
        if (total_avail > 3)
        {
            total_avail = 3;
        }
        if (total_avail == 0)
        {
            continue;
        }
        int stop_edge = rand_choose(total_avail);
        start = availEdge[stop_edge][0];
        end = availEdge[stop_edge][1];

        memset(tmp_tour, 0, sizeof(tmp_tour));
        AssignArray(shortest_path[start][end], tmp_tour);

        RemainRoute[0][0] ++;
        AssignSubArray(schedule[i], availEdge[stop_edge][3], schedule[i][0], RemainRoute[RemainRoute[0][0]]);
        stop_point[0]++;
        stop_point[stop_point[0]] = tmp_tour[rand_choose(tmp_tour[0])];

        remain_capacity[0]++;
        remain_capacity[remain_capacity[0]] = capacity - availEdge[stop_edge][2];
    }

    memcpy(rmstate->point, stop_point, sizeof(stop_point));
    memcpy(rmstate->rcap, remain_capacity, sizeof(remain_capacity));
    memcpy(rmstate->rroutes, RemainRoute, sizeof(RemainRoute));

}

void small_dynamic_change(Task *inst_tasks, Arc *inst_arcs, Rmstate *rmstate, unsigned int seed)
{
    int i, j;

    int tour[101][5000];
    memset(tour, 0, sizeof(tour));
    tour[0][0] = rmstate->rroutes[0][0];

    for (i = 1; i <= rmstate->rroutes[0][0]; i++)
    {
        int start = rmstate->point[i];
        for (j = 1; j <= rmstate->rroutes[i][0]; j++)
        {
            int task;
            task = rmstate->rroutes[i][j];
            JoinArray(tour[i], shortest_path[start][inst_tasks[task].head_node]);
            start = inst_tasks[task].tail_node;
        }
    }
    
    int passMatrix[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    memset(passMatrix, 0, sizeof(passMatrix));

    for (i = 1; i <= tour[0][0]; i++)
    {
        for (j = 1; j < tour[i][0]; j++)
        {
            passMatrix[tour[i][j]][tour[i][j+1]] ++;
            passMatrix[tour[i][j+1]][tour[i][j]] ++;
        }
    }

    int AdMatrix[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    memset(AdMatrix, 0, sizeof(AdMatrix));

    for (i = 1; i <= req_edge_num; i++)
    {
        AdMatrix[inst_arcs[i].head_node][inst_arcs[i].tail_node] = inst_arcs[i].trav_cost;
        AdMatrix[inst_arcs[i].tail_node][inst_arcs[i].head_node] = inst_arcs[i].trav_cost;
    }

    for (i = req_edge_num * 2 + 1; i <= req_edge_num * 2 + nonreq_edge_num; i++)
    {
        AdMatrix[inst_arcs[i].head_node][inst_arcs[i].tail_node] = inst_arcs[i].trav_cost;
        AdMatrix[inst_arcs[i].tail_node][inst_arcs[i].head_node] = inst_arcs[i].trav_cost;
    }

    findBridge(AdMatrix);

    int usedTimesValue[MAX_ARCS_TAG_LENGTH];
    for (i = 1; i <= MAX_NODE_TAG_LENGTH; i++)
    {
        for (j = i + 1; j < MAX_NODE_TAG_LENGTH; j++)
        {
            if (passMatrix[i][j] > 0)
            {
                usedTimesValue[0]++;
                usedTimesValue[usedTimesValue[0]] = passMatrix[i][j];
                // printf("%d %d, %d\n", i, j, passMatrix[i][j]);
            }
        }
    }
    for (i = 1; i <= usedTimesValue[0]; i++)
    {
        for (j = i+1; j <= usedTimesValue[0]; j++)
        {
            if (usedTimesValue[i] < usedTimesValue[j])
            {
                int tmp = usedTimesValue[i];
                usedTimesValue[i] = usedTimesValue[j];
                usedTimesValue[j] = tmp;
            }
        }
    }

    int mediaValue = usedTimesValue[(int)(usedTimesValue[0]/2)];


    int availEdgeMatrix[MAX_NODE_TAG_LENGTH][2];
    memset(availEdgeMatrix, 0, sizeof(availEdgeMatrix));
    availEdgeMatrix[0][0] = 0;
    for (i = 1; i <= MAX_NODE_TAG_LENGTH; i++)
    {
        for (j = i + 1; j < MAX_NODE_TAG_LENGTH; j++)
        {
            if (passMatrix[i][j] == mediaValue && AdMatrix[i][j] > 0)
            {
                availEdgeMatrix[0][0] += 1;
                availEdgeMatrix[availEdgeMatrix[0][0]][0] = i;
                availEdgeMatrix[availEdgeMatrix[0][0]][1] = j;
            }
        }
    }

    int idx = 1, idx1 = availEdgeMatrix[0][0];

    float rnum = (float) rand() / RAND_MAX;

    int head = availEdgeMatrix[idx][0];
    int tail = availEdgeMatrix[idx][1];
    
    if (rnum < 0.1)
    {
        AdMatrix[head][tail] = INF;
    } else
    {
        AdMatrix[head][tail] = (int) ( AdMatrix[head][tail] * 2);
    }
    AdMatrix[tail][head] += AdMatrix[head][tail];

    if (idx1 > 1)
    {
        head = availEdgeMatrix[idx1][0];
        tail = availEdgeMatrix[idx1][1];
        AdMatrix[head][tail] += 1.5;
        AdMatrix[tail][head] += AdMatrix[head][tail];
    }

    // update cost
    for (i = 1; i <= req_edge_num; i++)
    {
        if (AdMatrix[inst_arcs[i].head_node][inst_arcs[i].tail_node] < 0)
        {
            AdMatrix[inst_arcs[i].head_node][inst_arcs[i].tail_node] *= -1;
            AdMatrix[inst_arcs[i].tail_node][inst_arcs[i].head_node] *= -1;
        }
        inst_arcs[i].trav_cost = AdMatrix[inst_arcs[i].head_node][inst_arcs[i].tail_node];
        inst_arcs[i+req_edge_num].trav_cost = inst_arcs[i].trav_cost;
    }

    for (i = req_edge_num * 2 + 1; i <= req_edge_num * 2 + nonreq_edge_num; i++)
    {
        if (AdMatrix[inst_arcs[i].head_node][inst_arcs[i].tail_node] < 0)
        {
            AdMatrix[inst_arcs[i].head_node][inst_arcs[i].tail_node] *= -1;
            AdMatrix[inst_arcs[i].tail_node][inst_arcs[i].head_node] *= -1;
        }
        inst_arcs[i].trav_cost = AdMatrix[inst_arcs[i].head_node][inst_arcs[i].tail_node];
        inst_arcs[i+nonreq_edge_num].trav_cost = inst_arcs[i].trav_cost;
    }

    for (i = 1; i <= req_edge_num; i++)
    {
        if (AdMatrix[inst_arcs[i].head_node][inst_arcs[i].tail_node] < 0)
        {
            AdMatrix[inst_arcs[i].head_node][inst_arcs[i].tail_node] *= -1;
            AdMatrix[inst_arcs[i].tail_node][inst_arcs[i].head_node] *= -1;
        }
        inst_tasks[i].dead_cost = AdMatrix[inst_tasks[i].head_node][inst_tasks[i].tail_node];
        inst_tasks[i+req_edge_num].dead_cost = inst_tasks[i].dead_cost;
    }

}


/*
 input: inst_tasks, inst_arcs
 output: new_inst_tasks, new_inst_tasks, new_req_edge_num, new_nonreq_edge_num, new_vertex_num;
 */
void dynamicChange(Task *inst_tasks, Arc *inst_arcs, const int (*serve_tasks)[MAX_NODE_TAG_LENGTH], Vehicles *info, int *unserved_seq, unsigned int seed)
{



    double p2, p3, p4, p5, p_bdrr, p_crr, p_crbb;
    p2 = 0.5; p3 = 0.9;  // related to cost
    p4 = 0.35; p5 = 0.35; // related to demand
    p_bdrr = 0.5; // breakdown road recover
    p_crbb = 0.6; // congestion road become better
    p_crr = 0.3; // congestion road recover

    int i, j;
    Edge graph[MAX_TASKS_TAG_LENGTH];
    int AdMatrix[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    memset(AdMatrix, 0, sizeof(AdMatrix));

//    int unserved_seq[MAX_TASKS_TAG_LENGTH];
//    memset(unserved_seq, 0, sizeof(unserved_seq));

    int unServMatrix[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    for (i = 1; i <= unserved_seq[0]; i++)
    {
        if (unserved_seq[i] != 0)
        {
            unServMatrix[inst_tasks[unserved_seq[i]].head_node][inst_tasks[unserved_seq[i]].tail_node] = i;
            unServMatrix[inst_tasks[unserved_seq[i]].tail_node][inst_tasks[unserved_seq[i]].head_node] = i + 10000;
        }
    }


    // req_edge_num : edge_index[0];
    // nonreq_edge_num: edge_index[1];
    req_edge_num = edge_index[0];
    nonreq_edge_num = edge_index[1];

    // printf("req_edge_num: %d non: %d \n", req_edge_num, nonreq_edge_num);
    // for (i = 1; i <= 2*(req_edge_num+nonreq_edge_num); i++)
    // {
    //     if(inst_arcs[i].link > 0)
    //     {
    //         printf("%d: (%d, %d), link: %d\n", i, inst_arcs[i].head_node, inst_arcs[i].tail_node, inst_arcs[i].link);
    //     }
    // }

    // printf("<<< ---- >> \n");

    for (i = 1; i <= req_edge_num; i++)
    {
        graph[i].head_node = inst_tasks[i].head_node;
        graph[i].tail_node = inst_tasks[i].tail_node;
        graph[i].trav_cost = inst_tasks[i].dead_cost;
        if (serve_tasks[graph[i].head_node][graph[i].tail_node] || serve_tasks[graph[i].tail_node][graph[i].head_node])
        {
            graph[i].demand = 0;
        } else{
            graph[i].demand = inst_tasks[i].demand;
        }

        if (unServMatrix[graph[i].head_node][graph[i].tail_node])
        {
            graph[i].unserved = unServMatrix[graph[i].head_node][graph[i].tail_node];
        } else {
            graph[i].unserved = 0;
        }

        graph[i].change = inst_arcs[i].change;
        graph[i].link = inst_arcs[i].link;
        if (graph[i].link > 2*req_edge_num)
        {
            graph[i].link -= req_edge_num;
        }
        AdMatrix[graph[i].head_node][graph[i].tail_node] = i;
        AdMatrix[graph[i].tail_node][graph[i].head_node] = i;
        // if(graph[i].link > 0)
        // {
        //     printf("%d: (%d, %d), link: %d  demand:%d\n", i, graph[i].head_node, graph[i].tail_node, graph[i].link, graph[i].demand);
        // }
    }


    for (j=req_edge_num+1; j<=req_edge_num+nonreq_edge_num; j++)
    {
        i = j + req_edge_num;
        graph[j].head_node = inst_arcs[i].head_node;
        graph[j].tail_node = inst_arcs[i].tail_node;
        graph[j].trav_cost = inst_arcs[i].trav_cost;
        graph[j].demand = 0;
        graph[j].change = inst_arcs[i].change;
        graph[j].link = inst_arcs[i].link - req_edge_num;
        if (inst_arcs[i].link < 2*req_edge_num)
        {
            graph[j].link += req_edge_num;
        }
        AdMatrix[graph[j].head_node][graph[j].tail_node] = j;
        AdMatrix[graph[j].tail_node][graph[j].head_node] = j;
        // if(graph[j].link > 0)
        // {
        //     printf("%d: (%d, %d), link: %d\n", j, graph[j].head_node, graph[j].tail_node, graph[j].link);
        // }
    }


    // find all bridges
    // if AdMatrix[i][j] = -1, (i, j) is a bridge;
    findBridge(AdMatrix);

    float rnum[10];
    // srand(seed);

    int edge_num = req_edge_num + nonreq_edge_num;
    int newnode = 0;
    int mergenode = 0;

    int nodeList[MAX_NODE_TAG_LENGTH];
    memset(nodeList, 0, sizeof(nodeList));
    for (i = 1; i <= vertex_num; i++)
    {
        nodeList[i] = 1;
    }

    // Demand change
    int demand_change;
    for (i=1; i<= req_edge_num + nonreq_edge_num; i++)
    {
        if ( (( rand() / (float) RAND_MAX) < p4) && (graph[i].change != 2))
        {
            // add some demands
            // demand_change = (int)(( rand() / (double) RAND_MAX ) * (demandub - demandlb));
            demand_change = (int)(( rand() / (double) RAND_MAX ) * graph[i].trav_cost + 1);
            if (graph[i].demand > 0) // no increase
                continue;

            if (graph[i].demand + demand_change < capacity)
            {
                graph[i].demand += demand_change; // It is might different from different solution;
            }
            else
            {
                graph[i].demand = capacity;
            }
        }
        //  printf("<%d, %d>: demand: %d\n", graph[i].head_node, graph[i].tail_node, graph[i].demand);
    }


    // Cost change
    int tmpNode, tmpDmd, tmpCost;
    for (i=1; i<= req_edge_num + nonreq_edge_num; i++)
    {
        for (j=0; j < 10; j++)
        {
            rnum[j] = ( rand() / (double) RAND_MAX);
            //2, 3, 7, 6, 4, 5
        }
        if(graph[i].change == 0)
        {
            // Event 1 happen
            if (rnum[2] < p2 && rnum[3] < p3 )
            {
                // road become congestion
                graph[i].trav_cost += (int)(rnum[7] * (costub - costlb));
                graph[i].change = 1;
            }
            // Event 2 happen
            if (rnum[2] < p2 && rnum[3] >= p3 )
            {
                // road break down
                if (AdMatrix[graph[i].head_node][graph[i].tail_node] != -1)
                {

                    AdMatrix[graph[i].head_node][graph[i].tail_node] = 0;
                    AdMatrix[graph[i].tail_node][graph[i].head_node] = 0;
                    findBridge(AdMatrix);
                    // printf("Divide: \n <%d, %d>: demand: %d, cost: %d\n", graph[i].head_node, graph[i].tail_node, graph[i].demand, graph[i].trav_cost);

                    tmpDmd = graph[i].demand;
                    tmpCost = graph[i].trav_cost;
                    tmpNode = graph[i].tail_node;

                    newnode ++;
                    double rnum_scale = rnum[6] * 0.4 + 0.3;
                    graph[i].tail_node = vertex_num + newnode;
                    graph[i].demand = (int)(rnum_scale * tmpDmd);
                    graph[i].trav_cost = (int)(rnum_scale * tmpCost)+1;
                    graph[i].link = ++edge_num;
                    graph[i].unserved = graph[i].unserved * -1;
                    graph[i].change = 2;
                    nodeList[graph[i].tail_node] = 1;
                    // printf("To \n>> %d: <%d, %d>: demand: %d, cost: %d, link: %d \n", i, graph[i].head_node, graph[i].tail_node, graph[i].demand, graph[i].trav_cost, graph[i].link);


                    newnode ++;
                    graph[edge_num].head_node = vertex_num + newnode;
                    graph[edge_num].tail_node = tmpNode;
                    graph[edge_num].demand = tmpDmd - graph[i].demand;
                    graph[edge_num].trav_cost = tmpCost - graph[i].trav_cost;
                    graph[edge_num].link = i;
                    graph[edge_num].change = 2;
                    nodeList[graph[edge_num].head_node] = 1;
                    // printf(">> %d: <%d, %d>: demand: %d, cost: %d link: %d\n", edge_num, graph[edge_num].head_node, graph[edge_num].tail_node, graph[edge_num].demand, graph[edge_num].trav_cost, graph[edge_num].link);

                    


                }
            }
            continue;
        }

        if(graph[i].change == 1)
        {
            if (rnum[4] < p_crr) // congestion road recover
            {
                graph[i].trav_cost = cost_backup[graph[i].head_node][graph[i].tail_node];
                graph[i].change = 0;
            }
            else if (rnum[4] > p_crbb) // congestion road become worse
            {
                graph[i].trav_cost = (int) ((1.0 - rnum[8]) * graph[i].trav_cost + 1.0 * rnum[8] * cost_backup[graph[i].head_node][graph[i].tail_node]) + 1;
            }
            else // congestion road become better
            {
                graph[i].trav_cost += (int)(rnum[8] * 0.9 * (costub - costlb)) + 1;
            }
            continue;
        }

        if(graph[i].change == 2)
        {
            // breakdown road recover
            if (rnum[5] < p_bdrr)
            {
                int node1, node2, node3;
                if (graph[i].head_node < graph[i].tail_node)
                {
                    nodeList[graph[i].tail_node] = 0;
                    nodeList[graph[graph[i].link].head_node] = 0;
                    // printf("merge: (%d, %d), (%d, %d) \n", graph[i].head_node, graph[i].tail_node, graph[graph[i].link].head_node, graph[graph[i].link].tail_node);
                    node1 = graph[i].tail_node;
                    node2 = graph[graph[i].link].head_node;
                    graph[i].tail_node = graph[graph[i].link].tail_node;
                    node3 = graph[i].tail_node;
                } else {
                    nodeList[graph[i].head_node] = 0;
                    nodeList[graph[graph[i].link].tail_node] = 0;
                    // printf("merge: (%d, %d), (%d, %d) \n", graph[i].head_node, graph[i].tail_node, graph[graph[i].link].head_node, graph[graph[i].link].tail_node);
                    node1 = graph[i].head_node;
                    node2 = graph[graph[i].link].tail_node;
                    graph[i].head_node = graph[graph[i].link].head_node;
                    node3 = graph[i].head_node;
                }
                graph[i].demand += graph[graph[i].link].demand;
                graph[i].trav_cost += graph[graph[i].link].trav_cost;
                graph[i].change = 0;
                graph[graph[i].link].change = 7;
                graph[i].unserved = graph[i].unserved * -1;
                graph[graph[i].link].unserved = graph[graph[i].link].unserved * -1;
                graph[i].link = 0;
                mergenode += 2;
                // printf("To: <%d, %d>: demand: %d, cost: %d\n", graph[i].head_node, graph[i].tail_node, graph[i].demand, graph[i].trav_cost);
                for (int k=1; k<=info->stop[0]; k++)
                {
                    if (info->stop[k] == node1 || info->stop[k] == node2)
                    {
                        info->stop[k] = node3;
                    }
                }
            }
            continue;
        }
    }


    // update graph => inst_tasks;
    Edge new_edges[MAX_TASKS_TAG_LENGTH];
    int new_vertex_num = vertex_num + newnode - mergenode;
    int new_req_edge_num = 0;
    int new_nonreq_edge_num = 0;

    int start_index = vertex_num+newnode+1;
    for (i = 1; i <= vertex_num+newnode; i++)
    {
        if (nodeList[i] == 0)
        {
            start_index = i - 1;
            break;
        }
    }
    int vnum = start_index;

    j = 0;
    int head, tail;
    for (i = 1; i <= edge_num; i++)
    {
        if (graph[i].change != 7)
        {
            j++;
            if (graph[i].tail_node > start_index)
            {
                head = graph[i].head_node;
                tail = ++vnum;
                // graph[graph[i].link].link = j;

            } else if (graph[i].head_node > start_index)
            {
                head = ++vnum;
                tail = graph[i].tail_node;
                new_edges[j].link = graph[i].link;
                // new_edges[graph[i].link].link = j;
            } else {
                head = graph[i].head_node;
                tail = graph[i].tail_node;
                new_edges[j].link = graph[i].link;
            }

            // if(head == 15 && tail == 17)
            // {
            //     printf("stop.\n");
            // }
            // if (new_edges[graph[i].link].head_node == 15 && new_edges[graph[i].link].tail_node == 17)
            // {
            //     printf("stop.\n");
            // }
            if (graph[i].link > i)
            {
                graph[graph[i].link].link = j;
            }
            if (graph[i].link < i)
            {
                new_edges[graph[i].link].link = j;
                new_edges[j].link = graph[i].link;
            }

            new_edges[j].head_node = head;
            new_edges[j].tail_node = tail;
            new_edges[j].trav_cost = graph[i].trav_cost;
            new_edges[j].demand = graph[i].demand;
            new_edges[j].change = graph[i].change;
            new_edges[j].unserved = graph[i].unserved;
            if (new_edges[j].demand > 0)
            {
                new_req_edge_num ++;
            } else{
                new_nonreq_edge_num ++;
            }
        }
    }

//    saveGraph(new_edges, *info, vnum, new_req_edge_num, new_nonreq_edge_num, j);

    int new_task_num = 2 * new_req_edge_num;
    Task new_inst_tasks[MAX_TASKS_TAG_LENGTH];
    Arc new_inst_arcs[MAX_ARCS_TAG_LENGTH];

    int mapping1[2*new_req_edge_num+new_nonreq_edge_num+1];
    int mapping2[2*new_req_edge_num+new_nonreq_edge_num+1];
    memset(mapping1, 0, sizeof(mapping1)); // mapping1[u] = i -> the uth edge in new_inst_tasks -> the ith edge in new_edges 
    memset(mapping2, 0, sizeof(mapping2)); // mapping2[i] = u -> the ith edge in new_edges -> the uth edge in new_inst_tasks
    int u = 0, v = new_task_num;
    for (i=1; i <= j; i++)
    {
        // if (new_edges[i].link > 0)
        // {
        //     printf("%d: (%d, %d), link: %d\n", i, new_edges[i].head_node, new_edges[i].tail_node, new_edges[i].link);
        // }
        if (new_edges[i].demand > 0)
        {
            u ++;
            new_inst_tasks[u].head_node = new_edges[i].head_node;
            new_inst_tasks[u].tail_node = new_edges[i].tail_node;
            new_inst_tasks[u].serv_cost = new_edges[i].trav_cost;
            if (new_inst_tasks[u].serv_cost == 0)
            {
                new_inst_tasks[u].serv_cost = 10;
            }
            new_inst_tasks[u].dead_cost = new_edges[i].trav_cost;
            new_inst_tasks[u].demand = new_edges[i].demand;
            new_inst_tasks[u].inverse = u + new_req_edge_num;

            new_inst_tasks[u+new_req_edge_num].head_node =  new_inst_tasks[u].tail_node;
            new_inst_tasks[u+new_req_edge_num].tail_node = new_inst_tasks[u].head_node;
            new_inst_tasks[u+new_req_edge_num].dead_cost = new_inst_tasks[u].dead_cost;
            new_inst_tasks[u+new_req_edge_num].serv_cost = new_inst_tasks[u].serv_cost;
            new_inst_tasks[u+new_req_edge_num].demand = new_inst_tasks[u].demand;
            new_inst_tasks[u+new_req_edge_num].inverse = u;

            new_inst_arcs[u].head_node = new_inst_tasks[u].head_node;
            new_inst_arcs[u].tail_node = new_inst_tasks[u].tail_node;
            new_inst_arcs[u].trav_cost = new_inst_tasks[u].dead_cost;
            new_inst_arcs[u].change = new_edges[i].change;
            new_inst_arcs[u].link = new_edges[i].link;
            mapping1[u] = i;
            mapping2[i] = u;

            new_inst_arcs[u+new_req_edge_num].head_node = new_inst_tasks[u].tail_node;
            new_inst_arcs[u+new_req_edge_num].tail_node = new_inst_tasks[u].head_node;
            new_inst_arcs[u+new_req_edge_num].trav_cost = new_inst_tasks[u].dead_cost;
            new_inst_arcs[u+new_req_edge_num].change = new_inst_arcs[u].change;

        } else{
            v++;
            new_inst_arcs[v].head_node = new_edges[i].head_node;
            new_inst_arcs[v].tail_node = new_edges[i].tail_node;
            new_inst_arcs[v].trav_cost = new_edges[i].trav_cost;
            new_inst_arcs[v].change = new_edges[i].change;
            new_inst_arcs[v].link = new_edges[i].link;
            mapping1[v] = i;
            mapping2[i] = v;
            // mapping1[v - new_req_edge_num] = i;
            // mapping2[i] = v - new_req_edge_num;

            new_inst_arcs[v+new_nonreq_edge_num].head_node = new_edges[i].tail_node;
            new_inst_arcs[v+new_nonreq_edge_num].tail_node = new_edges[i].head_node;
            new_inst_arcs[v+new_nonreq_edge_num].trav_cost = new_edges[i].trav_cost;
            new_inst_arcs[v+new_nonreq_edge_num].change = new_edges[i].change;
        }
    }
    // printf("xxxx\n");

    // mapping unserved tasks
    int tmp_new_seq[MAX_TASK_SEQ_LENGTH];
    memcpy(tmp_new_seq, unserved_seq, sizeof(tmp_new_seq));
    for (i = 1; i <= j; i++)
    {
        int idx;
        if (new_edges[i].unserved != 0)
        {
            if (new_edges[i].unserved > 10000)
            {
                idx = new_edges[i].unserved - 10000;
                if (mapping2[i] > new_req_edge_num)
                {
                    tmp_new_seq[idx] = mapping2[i]-new_req_edge_num;
                } else
                {
                    tmp_new_seq[idx] = mapping2[i]+new_req_edge_num;
                }
                continue;
            }

            if (new_edges[i].unserved < 0)
            {
                if (new_edges[i].unserved < -10000)
                {
                    idx = new_edges[i].unserved * -1 - 10000;
                } else {
                    idx = new_edges[i].unserved * -1;
                }
                tmp_new_seq[idx] = -1;
                continue;
            }

            idx = new_edges[i].unserved;
            tmp_new_seq[idx] = mapping2[i];
        }
    }
    for (i = 1; i <= tmp_new_seq[0]; i++)
    {
        if (tmp_new_seq[i] > 2*new_req_edge_num)
        {
            delete_element(tmp_new_seq, i);
        }
    }
    memcpy(info->remain_seqs, tmp_new_seq, sizeof(tmp_new_seq));
//    for (i = 1; i <= info->remain_seqs[0]; i++)
//    {
//        printf("%d ", info->remain_seqs[i]);
//    }
//    printf("\n");



    int temp;
    for(i = 1; i <= new_req_edge_num; i++)
    {
        if(new_inst_arcs[i].change == 2)
        {
            new_inst_arcs[i].link = mapping2[new_edges[mapping1[i]].link];
            // printf("%d:(%d, %d, %d) \n", i, new_inst_arcs[i].head_node, new_inst_arcs[i].tail_node, new_inst_arcs[i].link);
        }
    }
    for(i = new_task_num + 1; i <= new_task_num + new_nonreq_edge_num; i++)
    {
        if(new_inst_arcs[i].change == 2)
        {
            // new_inst_arcs[i].link = mapping2[new_edges[mapping1[i-new_req_edge_num]].link];
            new_inst_arcs[i].link = mapping2[new_edges[mapping1[i]].link];
    //         // printf("%d:(%d, %d, %d)\n", i, new_inst_arcs[i].head_node, new_inst_arcs[i].tail_node, new_inst_arcs[i].link);
        }
    }
    // printf("\n");
    // printf("dynamic Change !!!\n");

    req_edge_num = new_req_edge_num;
    nonreq_edge_num = new_nonreq_edge_num;
    task_num = new_task_num;
    vertex_num = new_vertex_num;
    total_arc_num = task_num + 2 * nonreq_edge_num + nonreq_arc_num;
    memcpy(inst_tasks, new_inst_tasks, sizeof(new_inst_tasks));
    memcpy(inst_arcs, new_inst_arcs, sizeof(new_inst_arcs));


    inst_tasks[0].head_node = DEPOT;
    inst_tasks[0].tail_node = DEPOT;
    inst_tasks[0].dead_cost = 0;
    inst_tasks[0].serv_cost = 0;
    inst_tasks[0].demand = 0;
    inst_tasks[0].inverse = 0;
    inst_arcs[0].head_node = DEPOT;
    inst_arcs[0].tail_node = DEPOT;
    inst_arcs[0].trav_cost = 0;

    edge_index[0] = req_edge_num;
    edge_index[1] = nonreq_edge_num;

}


void saveGraph(Edge *graph, Vehicles info, int vnum, int new_req_edge_num, int new_nonreq_edge_num, int edge_num)
{
    int i;
    char path[30];
    static int num = 0;

    num ++;
    sprintf(path, "../dmap/egl/%d", num);
    if (num == 3)
    {
        num = 0;
    }
//    printf("%s\n", path);
    FILE *fp;
    fp = fopen(path, "w+");
    // fprintf(fp, "name: %s\n", map);
    // fprintf(fp, "vertices: %d\n", vnum);
    // fprintf(fp, "req_edge_num: %d\n", new_req_edge_num);
    // fprintf(fp, "non_req_edge_num: %d\n", new_nonreq_edge_num);
    // fprintf(fp, "vehicles: %d\n", vehicle_num);
    // fprintf(fp, "outside_vehicles: %d\n", info.stop[0]);
    // fprintf(fp, "stop_point: ");
    // for (i = 1; i <= info.stop[0]; i++)
    // {
    //     fprintf(fp, "%d ", info.stop[i]);
    // }
    // fprintf(fp, "\n");

    // fprintf(fp, "capacity: %d\n", capacity);
    // fprintf(fp, "remaining_capacity: ");
    // for (i = 1; i <= info.remain_capacity[0]; i++)
    // {
    //     fprintf(fp, "%d ", info.remain_capacity[i]);
    // }
    // fprintf(fp, "\n");
    // fprintf(fp, "remaining_tasks: ");
    // for (i = 0; i <= info.remain_seqs[0]; i++)
    // {
    //     fprintf(fp, "%d ", info.remain_seqs[i]);
    // }
    // fprintf(fp, "\nEdges: \n");
    // for (i = 1; i <= edge_num; i++)
    // {
    //     fprintf(fp, "(%d, %d), cost: %d, demand: %d, change: %d\n", graph[i].head_node, graph[i].tail_node, graph[i].trav_cost, graph[i].demand, graph[i].change);
    // }

    for (i = 1; i <= edge_num; i++)
    {
        fprintf(fp, "%d %d %d\n", graph[i].head_node, graph[i].tail_node, graph[i].trav_cost);
    }
    fclose(fp);
}

void findBridge(int (*AdMatrix)[MAX_NODE_TAG_LENGTH])
{
    int visited[vertex_num+1];
    int disc[vertex_num+1];
    int low[vertex_num+1];
    int parent[vertex_num+1];

    memset(visited, 0, sizeof(visited));
    memset(disc, INF, sizeof(disc));
    memset(low, INF, sizeof(low));
    memset(parent, -1, sizeof(parent));

    int time = 0;
    for (int i=1; i <= vertex_num; i++)
    {
        if ( !visited[i])
            dfs(i, visited, parent, low, disc, &time, AdMatrix);
    }

}

void dfs(int u, int *visited, int *parent, int *low, int *disc, int *time, int (*AdMatrix)[MAX_NODE_TAG_LENGTH])
{
    visited[u] = 1;
    disc[u] = *time;
    low[u] = *time;
    *time  += 1;

    int v;
    for (int i=1; i <= vertex_num; i++)
    {
        if (AdMatrix[u][i] > 0)
        {
            v = i;
            if ( !visited[v] )
            {
                parent[v] = u;
                dfs(v, visited, parent, low, disc, time, AdMatrix);

                low[u] = (low[u] < low[v] )? low[u] : low[v];
                if (low[v] > disc[u])
                {
                    AdMatrix[u][v] = -1 * AdMatrix[u][v];
                    AdMatrix[v][u] = -1 * AdMatrix[v][u];
                //    printf("bridge: %d, %d\n", u, v); //get bridges
                }
            }
            else if ( v != parent[u])
            {
                low[u] = (low[u] < disc[v] )? low[u] : disc[v];
            }
        }

    }


}