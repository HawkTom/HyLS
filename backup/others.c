
#include "lsoperator.h"

void lma_moves(Individual *indi, const Task *inst_tasks)
{

    int i, j, k;
    lns_route curr_solution, next_solution;

    char type[30];

    strcpy(type, "LMA Move");

    int Positions[101];
    find_ele_positions(Positions, indi->Sequence, 0);
    curr_solution.Route[0][0] = Positions[0] - 1;
    curr_solution.loads[0] = Positions[0] - 1;
    for (i = 1; i < Positions[0]; i++)
    {
        AssignSubArray(indi->Sequence, Positions[i], Positions[i+1], curr_solution.Route[i]); // Route[i]: 0 x x x x 0
        curr_solution.loads[i] = 0;
        for (j = Positions[i]+1; j < Positions[i+1]; j++)
        {
            curr_solution.loads[i] += inst_tasks[indi->Sequence[j]].demand;
        }
    }
    curr_solution.fitness = indi->TotalCost + LAMBDA * indi->TotalVioLoad;;
    curr_solution.total_vio_loads = indi->TotalVioLoad;
    curr_solution.total_cost = indi->TotalCost;

    int u, pos_u, trip_u, v, pos_v, trip_v, x, y, inv_u;
    int improve = 0;
    int global_improve = 0;

    // next_solution = curr_solution;
    // check_cost1(curr_solution, inst_tasks);

    while (1)
    {
        next_solution = curr_solution;
        for (trip_u = 1; trip_u <= curr_solution.Route[0][0]; trip_u ++)
        {
            for (pos_u = 2; pos_u < curr_solution.Route[trip_u][0]; pos_u++)
            {
                // u = curr_solution.Route[trip_u][pos_u];
                // inv_u = inst_tasks[u].inverse;

                // next_solution.total_cost = curr_solution.total_cost
                //                            - min_cost[inst_tasks[curr_solution.Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                //                            - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution.Route[trip_u][pos_u+1]].head_node]
                //                            + min_cost[inst_tasks[curr_solution.Route[trip_u][pos_u-1]].tail_node][inst_tasks[inv_u].head_node]
                //                            + min_cost[inst_tasks[inv_u].tail_node][inst_tasks[curr_solution.Route[trip_u][pos_u+1]].head_node];


                // next_solution.fitness = next_solution.total_cost + LAMBDA * next_solution.total_vio_loads;
                // if (next_solution.fitness < curr_solution.fitness)
                // {
                //     // printf("++++");
                //     curr_solution.total_cost = next_solution.total_cost;
                //     curr_solution.fitness = next_solution.fitness;
                //     curr_solution.Route[trip_u][pos_u] = inst_tasks[u].inverse;
                // }
                u = curr_solution.Route[trip_u][pos_u];
                

                for (trip_v = trip_u; trip_v <= curr_solution.Route[0][0]; trip_v ++)
                {
                    for (pos_v = pos_u + 1; pos_v < curr_solution.Route[trip_v][0]; pos_v ++)
                    {

                        v = curr_solution.Route[trip_v][pos_v];

                        improve = lma_single_insertion(&curr_solution, &next_solution, u, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
                        // check_cost1(curr_solution, inst_tasks);

                        if (improve){
                            goto new_step;
                        }

                        if (trip_u == trip_v && pos_v - pos_u > 1 || trip_v != trip_u)
                        {

                            x = curr_solution.Route[trip_u][pos_u+1];
                            if (trip_v != trip_u && x != 0)
                            {
                                improve = lma_double_insertion(&curr_solution, &next_solution, u, x, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
                                // check_cost1(curr_solution, inst_tasks);
                                if (improve){
//                                    printf("double insertion \n");
                                    goto new_step;
                                }
                            }
                        }

                        if (trip_u == trip_v && pos_v - pos_u > 1 || trip_v != trip_u)
                        {
                            improve = lma_swap(&curr_solution, &next_solution, u, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
                            // check_cost1(curr_solution, inst_tasks);
                            if (improve){
//                                printf("swap \n");
                                goto new_step;
                            }
                        }

                        x = curr_solution.Route[trip_u][pos_u+1];
                        y = curr_solution.Route[trip_v][pos_v+1];             
                        if (trip_u == trip_v && y != 0 && pos_v > pos_u)
                        {                            
                            improve = lma_reverse(&curr_solution, &next_solution, u, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
                            // check_cost1(curr_solution, inst_tasks);
                            if (improve){
                                goto new_step;
                            }
                        }


                        if (trip_v != trip_u && x != 0 && y != 0)
                        {
                            improve = lma_cross(&curr_solution, &next_solution, u, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
                            // check_cost1(curr_solution, inst_tasks);
                            if (improve){
                                goto new_step;
                            }
                        }


                        
                    }
                }
            }
        }


    new_step:
        if (improve)
        {
            // check_cost1(curr_solution, inst_tasks);
            update_global_best_solution(&curr_solution, &best_lma_solution, inst_tasks, type);
            improve = 0;
            continue;
        } else {
            break;
        }
    }

    Individual mted_child;
    // route -> sequence
    mted_child.Sequence[0] = 1;
    k = 0;
    for (i = 1; i <= curr_solution.Route[0][0]; i++)
    {
        if (curr_solution.Route[i][0] > 2)
        {
            mted_child.Sequence[0] --;
            JoinArray(mted_child.Sequence, curr_solution.Route[i]);
            k ++;
            mted_child.Loads[k] = curr_solution.loads[i];
        }
    }
    mted_child.Loads[0] = k;
    mted_child.TotalCost = curr_solution.total_cost;
    mted_child.TotalVioLoad = curr_solution.total_vio_loads;

    if ( mted_child.TotalCost != get_task_seq_total_cost(mted_child.Sequence, inst_tasks))
    {
        printf("101: lmsals total cost error \n");
        // longjmp(buf, 2);
        exit(0);
    }
}