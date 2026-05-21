import heapq

def ucs(initial_state):
    """
    Uniform Cost Search (UCS) algorithm to find the minimum cost path from initial_state to a goal state.
    """
    # Priority queue stores tuples of (cost, count, current_state, path_to_current_state).
    # count is used to break ties and avoid comparing GameState objects.
    queue = [(0, 0, initial_state, [])]
    count = 1
    
    # visited stores the minimum cost to reach each state.
    visited = {initial_state: 0}

    while queue:
        cost, _, current_state, path = heapq.heappop(queue)
        
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
                heapq.heappush(queue, (new_cost, count, next_state, path + [action]))
                count += 1
                
    return []
