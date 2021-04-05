


#include "lsoperator.h"


int client1_single_insertion(Individual *indi, const Task *inst_tasks)
{

    int i, j, k;
    lns_route curr_solution, next_solution;

    char type[30];

    strcpy(type, "Single Insertion");

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

    
    while (1)
    {
        next_solution = curr_solution;
        for (trip_u = 1; trip_u <= curr_solution.Route[0][0]; trip_u ++)
        {
            for (pos_u = 2; pos_u < curr_solution.Route[trip_u][0]; pos_u++)
            {
                u = curr_solution.Route[trip_u][pos_u];

                for (trip_v = trip_u; trip_v <= curr_solution.Route[0][0]; trip_v ++)
                {
                    for (pos_v = pos_u + 1; pos_v < curr_solution.Route[trip_v][0]; pos_v ++)
                    {

                        v = curr_solution.Route[trip_v][pos_v];

                        improve = lma_single_insertion(&curr_solution, &next_solution, u, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);

                        if (improve){
//                            printf("single insertion \n");
                            goto new_step;
                        }
                    }
                }
            }
        }


    new_step:
        if (improve)
        {
            // update to the server
            update_global_best_solution(&curr_solution, &best1_si_solution, inst_tasks, type);
            global_improve = 1;
            improve = 0;
            continue;
        } else {
            break;
        }
    }



    update_global_best_solution(&curr_solution, &best1_si_solution, inst_tasks, type);
    
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
    return global_improve;

}


int client1_double_insertion(Individual *indi, const Task *inst_tasks)
{

    int i, j, k;
    lns_route curr_solution, next_solution;

    char type[30];

    strcpy(type, "Double Insertion");

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

    
    while (1)
    {
        next_solution = curr_solution;
        for (trip_u = 1; trip_u <= curr_solution.Route[0][0]; trip_u ++)
        {
            for (pos_u = 2; pos_u < curr_solution.Route[trip_u][0]; pos_u++)
            {
                u = curr_solution.Route[trip_u][pos_u];

                for (trip_v = trip_u; trip_v <= curr_solution.Route[0][0]; trip_v ++)
                {
                    for (pos_v = pos_u + 1; pos_v < curr_solution.Route[trip_v][0]; pos_v ++)
                    {

                        v = curr_solution.Route[trip_v][pos_v];

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



                    }
                }
            }
        }


    new_step:
        if (improve)
        {
            // update to the server
            update_global_best_solution(&curr_solution, &best1_di_solution, inst_tasks, type);
            global_improve = 1;
            improve = 0;
            continue;
            // break;
        } else {
            break;
        }
    }


    update_global_best_solution(&curr_solution, &best1_di_solution, inst_tasks, type);
    
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
        printf("214: lmsals total cost error \n");
        // longjmp(buf, 2);
        exit(0);
    }
    return global_improve;

}


int client1_swap(Individual *indi, const Task *inst_tasks)
{

    int i, j, k;
    lns_route curr_solution, next_solution;

    char type[30];

    strcpy(type, "SWAP");

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

    
    while (1)
    {
        next_solution = curr_solution;
        for (trip_u = 1; trip_u <= curr_solution.Route[0][0]; trip_u ++)
        {
            for (pos_u = 2; pos_u < curr_solution.Route[trip_u][0]; pos_u++)
            {
                u = curr_solution.Route[trip_u][pos_u];

                for (trip_v = trip_u; trip_v <= curr_solution.Route[0][0]; trip_v ++)
                {
                    for (pos_v = pos_u + 1; pos_v < curr_solution.Route[trip_v][0]; pos_v ++)
                    {

                        v = curr_solution.Route[trip_v][pos_v];

                        if (trip_u == trip_v && pos_v - pos_u > 1 || trip_v != trip_u)
                        {
                            improve = lma_swap(&curr_solution, &next_solution, u, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
                            // check_cost1(curr_solution, inst_tasks);
                            if (improve){
//                                printf("swap \n");
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
            // update to the server
            update_global_best_solution(&curr_solution, &best1_swap_solution, inst_tasks, type);
            global_improve = 1;
            improve = 0;
            continue;
            // break;
        } else {
            break;
        }
    }


    update_global_best_solution(&curr_solution, &best1_swap_solution, inst_tasks, type);
    
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
        printf("320: lmsals total cost error \n");
        // longjmp(buf, 2);
        exit(0);
    }

    return global_improve;
}

int client1_reverse(Individual *indi, const Task *inst_tasks)
{

    int i, j, k;
    lns_route curr_solution, next_solution;

    char type[30];

    strcpy(type, "Reverse");

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

    
    while (1)
    {
        next_solution = curr_solution;
        for (trip_u = 1; trip_u <= curr_solution.Route[0][0]; trip_u ++)
        {
            for (pos_u = 2; pos_u < curr_solution.Route[trip_u][0]; pos_u++)
            {
                u = curr_solution.Route[trip_u][pos_u];

                for (trip_v = trip_u; trip_v <= curr_solution.Route[0][0]; trip_v ++)
                {
                    // printf("%d, %d ||", curr_solution.Route[trip_u][0],  curr_solution.Route[trip_v][0]);
                    for (pos_v = 2; pos_v < curr_solution.Route[trip_v][0]; pos_v ++)
                    {

                        // printf("pos: %d %d\n", pos_u, pos_v);
                        v = curr_solution.Route[trip_v][pos_v];

                        x = curr_solution.Route[trip_u][pos_u+1];

                        y = curr_solution.Route[trip_v][pos_v+1];
                        // printf("u: %d x:%d v:%d y:%d\n", u,x,v,y);

                        // printf("trip: %d %d\n", trip_u, trip_v);
                        // if (pos_u == 2 && pos_v == 2 && trip_u == 1 && trip_v == 4)
                        // {
                        //     printf("stop\n");
                        // }

                        if (trip_u == trip_v && y != 0 && pos_v > pos_u)
                        {
                            improve = lma_reverse(&curr_solution, &next_solution, u, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
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
            // update to the server
            update_global_best_solution(&curr_solution, &best1_rev_solution, inst_tasks, type);
            global_improve = 1;
            improve = 0;
            continue;
        } else {
            break;
        }
    }


    update_global_best_solution(&curr_solution, &best1_rev_solution, inst_tasks, type);
    
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
        printf("436: lmsals total cost error \n");
        // longjmp(buf, 2);
        exit(0);
    }
    return global_improve;
}

int client1_cross(Individual *indi, const Task *inst_tasks)
{
    int i, j, k;
    lns_route curr_solution, next_solution;

    char type[30];

    strcpy(type, "Cross");

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

    
    while (1)
    {
        next_solution = curr_solution;
        for (trip_u = 1; trip_u <= curr_solution.Route[0][0]; trip_u ++)
        {
            for (pos_u = 2; pos_u < curr_solution.Route[trip_u][0]; pos_u++)
            {
                u = curr_solution.Route[trip_u][pos_u];

                for (trip_v = trip_u; trip_v <= curr_solution.Route[0][0]; trip_v ++)
                {
                    // printf("%d, %d ||", curr_solution.Route[trip_u][0],  curr_solution.Route[trip_v][0]);
                    for (pos_v = 2; pos_v < curr_solution.Route[trip_v][0]; pos_v ++)
                    {

                        // printf("pos: %d %d\n", pos_u, pos_v);
                        v = curr_solution.Route[trip_v][pos_v];

                        x = curr_solution.Route[trip_u][pos_u+1];

                        y = curr_solution.Route[trip_v][pos_v+1];
                        // printf("u: %d x:%d v:%d y:%d\n", u,x,v,y);

                        // printf("trip: %d %d\n", trip_u, trip_v);
                        // if (pos_u == 2 && pos_v == 2 && trip_u == 1 && trip_v == 4)
                        // {
                        //     printf("stop\n");
                        // }

                        if (trip_v != trip_u && x != 0 && y != 0)
                        {
                            improve = lma_cross(&curr_solution, &next_solution, u, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
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
            // update to the server
            update_global_best_solution(&curr_solution, &best1_cross_solution, inst_tasks, type);
            global_improve = 1;
            improve = 0;
            continue;
        } else {
            break;
        }
    }

    // printf("xxxx\n");
    update_global_best_solution(&curr_solution, &best1_cross_solution, inst_tasks, type);
    
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
        printf("553: lmsals total cost error \n");
        // longjmp(buf, 2);
        exit(0);
    }
    if ( best1_cross_solution.Sequence[0] == 0)
    {
        printf("564: lmsals total cost error \n");
        // longjmp(buf, 2);
        exit(0);
    }
    return global_improve;

}




