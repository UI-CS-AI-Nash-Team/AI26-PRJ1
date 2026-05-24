import heapq
from functools import lru_cache
from math import inf


MOVES = (
    (-1, 0),
    (1, 0),
    (0, -1),
    (0, 1),
)


def a_star(initial_state):
    """
    A* search with a strong admissible heuristic.

    The heuristic solves a relaxed version of the problem: collect all remaining
    targets on the static map while keeping walls and terrain costs, but ignoring
    the moving enemy. This keeps A* optimal and greatly reduces node expansion.
    """

    rows, cols = initial_state.get_grid_size()
    walls = frozenset(initial_state.get_crates_positions())
    distance_cache = {}

    def in_bounds(position):
        row, col = position
        return 0 <= row < rows and 0 <= col < cols

    def static_distances_from(source):
        """Return Dijkstra distances from source on the static terrain grid."""

        cached = distance_cache.get(source)
        if cached is not None:
            return cached

        distances = {source: 0}
        frontier = [(0, source)]

        while frontier:
            current_cost, position = heapq.heappop(frontier)

            if current_cost != distances.get(position):
                continue

            row, col = position
            for dr, dc in MOVES:
                next_position = (row + dr, col + dc)

                if not in_bounds(next_position) or next_position in walls:
                    continue

                new_cost = current_cost + initial_state.get_terrain_cost(next_position)

                if new_cost < distances.get(next_position, inf):
                    distances[next_position] = new_cost
                    heapq.heappush(frontier, (new_cost, next_position))

        distance_cache[source] = distances
        return distances

    @lru_cache(maxsize=None)
    def relaxed_collection_cost(position, remaining_targets):
        """Exact target-collection cost in the static relaxed problem."""

        if not remaining_targets:
            return 0

        distances = static_distances_from(position)
        best_cost = inf

        for index, target in enumerate(remaining_targets):
            target_cost = distances.get(target, inf)
            if target_cost == inf:
                continue

            next_targets = (
                remaining_targets[:index]
                + remaining_targets[index + 1:]
            )
            total_cost = target_cost + relaxed_collection_cost(target, next_targets)
            best_cost = min(best_cost, total_cost)

        return best_cost

    def heuristic(state):
        targets = tuple(sorted(state.get_targets_positions()))
        if not targets:
            return 0

        return relaxed_collection_cost(state.get_agent_position(), targets)

    def rebuild_path(goal_state):
        path = []
        state = goal_state

        while parents[state][0] is not None:
            previous_state, action = parents[state]
            path.append(action)
            state = previous_state

        path.reverse()
        return path

    # Queue entries are ordered by f, then h, then larger g. The tie-breaker
    # keeps equal-f searches focused toward completing the remaining targets.
    start_h = heuristic(initial_state)
    queue = [(start_h, start_h, 0, 0, initial_state)]
    count = 1

    g_score = {initial_state: 0}
    parents = {initial_state: (None, None)}
    closed = set()

    while queue:
        _, _, negative_cost, _, current_state = heapq.heappop(queue)
        cost = -negative_cost

        # Ignore older heap entries after a better path to the same state exists.
        if cost != g_score.get(current_state, inf):
            continue

        if current_state in closed:
            continue

        if current_state.is_goal_state():
            return rebuild_path(current_state)

        closed.add(current_state)

        for action, step_cost, next_state in current_state.get_successors(
            toward_walls=False
        ):

            if next_state.is_collision_state():
                continue

            new_cost = cost + step_cost

            if next_state in closed and new_cost >= g_score.get(next_state, inf):
                continue

            if new_cost < g_score.get(next_state, inf):
                next_h = heuristic(next_state)

                if next_h == inf:
                    continue

                g_score[next_state] = new_cost
                parents[next_state] = (current_state, action)
                heapq.heappush(
                    queue,
                    (new_cost + next_h, next_h, -new_cost, count, next_state)
                )
                count += 1

    return []
