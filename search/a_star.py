import heapq

def a_star(initial_state):
    """
    A* Search algorithm to find the minimum cost path from initial_state to a goal state.
    """
    def heuristic(state):
        targets = state.get_targets_positions()
        if not targets:
            return 0
        agent_pos = state.get_agent_position()
        # Manhattan distance to the nearest target is an admissible heuristic.
        # Since the goal is to collect all targets, the distance to the nearest one 
        # is always less than or equal to the actual cost.
        return min(abs(agent_pos[0] - t[0]) + abs(agent_pos[1] - t[1]) for t in targets)

    # Priority queue stores tuples of (priority, cost, count, current_state, path_to_current_state).
    # priority = current path cost + heuristic
    # count is used to break ties and avoid comparing GameState objects.
    start_h = heuristic(initial_state)
    queue = [(start_h, 0, 0, initial_state, [])]
    count = 1
    
    # visited stores the minimum path cost (g-score) to reach each state.
    visited = {initial_state: 0}

    while queue:
        priority, cost, _, current_state, path = heapq.heappop(queue)
        
        # Check if it's a goal state when expanding
        if current_state.is_goal_state():
            return path
            
        # If we already found a strictly cheaper way to this state, skip it.
        if cost > visited.get(current_state, float('inf')):
            continue

        for action, step_cost, next_state in current_state.get_successors(toward_walls=True):
            # Skip successor states that result in a collision.
            if next_state.is_collision_state():
                continue
            
            new_cost = cost + step_cost
            if next_state not in visited or new_cost < visited[next_state]:
                visited[next_state] = new_cost
                new_priority = new_cost + heuristic(next_state)
                heapq.heappush(queue, (new_priority, new_cost, count, next_state, path + [action]))
                count += 1
                
    return []
