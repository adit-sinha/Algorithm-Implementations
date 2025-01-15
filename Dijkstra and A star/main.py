import time
from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq

def dijkstra(gmap, s, g):
    
    open = []
    closed = {} #implemented in python through hash table, so better
    node_no = 0

    heapq.heappush(open, (0, s))
    closed[s.state_hash()] = (s.get_g(), s)
    
    
    while open:
        n = heapq.heappop(open)[1]
        node_no += 1 #goal node is also expanded
        if (n == g): return n.get_g(), node_no
        

        children = gmap.successors(n)
        for child in children:
           
            gnew = n.get_g() + gmap.cost(child.get_x() - n.get_x(), child.get_y() - n.get_y())
            if child.state_hash() not in closed:
                
                heapq.heappush(open, (gnew, child))
                closed[child.state_hash()] = (gnew, child)

            else:
                if  (gnew < closed[child.state_hash()][0]):
                    
                    child.set_g(gnew)
                    closed[child.state_hash()] = (child.get_g(), child)
                    heapq.heappush(open,(child.get_g(), child))
                    
        
    return -1, node_no #No path found

def a_(gmap, s, g):
    open = []
    closed = {} #implemented in python through hash table, so better
    node_no = 0

    heapq.heappush(open, (0, s))
    closed[s.state_hash()] = (s.get_g(), s)

    dx = abs(s.get_x() - g.get_x())
    dy = abs(s.get_y() - g.get_y())

    hval = 1.5*min(dx,dy) + abs(dx-dy)

    s.set_cost(s.get_g() + hval) #storing f-values
    heapq.heappush(open, (0,s))
    
    while open:
        n = heapq.heappop(open)[1]
        node_no += 1 #goal node is also expanded
        if (n == g): return n.get_cost(), node_no
        
        closed[n.state_hash()] = (n.get_cost(), n)

        children = gmap.successors(n)

        for child in children:
            dx = abs(child.get_x() - g.get_x())
            dy = abs(child.get_y() - g.get_y())

            hval = 1.5*min(dx,dy) + abs(dx-dy)
            newf = child.get_g() + hval

            if child.state_hash() in closed:
                if  (newf < closed[child.state_hash()][0]):
                    child.set_cost(newf)
                    closed[child.state_hash()] = (child.get_cost(), child)
                    heapq.heappush(open,(child.get_cost(), child))
                    
            else:             
                child.set_cost(newf)
                closed[child.state_hash()] = (child.get_cost(), child)
                heapq.heappush(open,(child.get_cost(), child))
            

    return -1, node_no #No path found

def main():
    """
    Function for testing your A* and Dijkstra's implementation. 
    Run it with a -help option to see the options available. 
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = False
    for o, a in optlist:
        if o in ("-help"):
            print("Examples of Usage:")
            print("Solve set of test instances and generate plots: main.py --plots")
            exit()
        elif o in ("--plots"):
            plots = True

    test_instances = "test-instances/testinstances.txt"
    
    # Dijkstra's algorithm and A* should receive the following map object as input
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_dijkstra = []  
    nodes_expanded_astar = []

    time_dijkstra = []  
    time_astar = []

    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
        
    for i in range(0, len(start_states)):    
        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        cost, expanded_diskstra = dijkstra(gridded_map, start, goal)# replace None, None with the call to your Dijkstra's implementation
        time_end = time.time()
        nodes_expanded_dijkstra.append(expanded_diskstra)
        time_dijkstra.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()    

        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        cost, expanded_astar = a_(gridded_map, start, goal) # replace None, None with the call to your A* implementation
        time_end = time.time()

        nodes_expanded_astar.append(expanded_astar)
        time_astar.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_astar, nodes_expanded_dijkstra, "Nodes Expanded (A*)", "Nodes Expanded (Dijkstra)", "nodes_expanded")
        plotter.plot_results(time_astar, time_dijkstra, "Running Time (A*)", "Running Time (Dijkstra)", "running_time")

if __name__ == "__main__":
    main()
