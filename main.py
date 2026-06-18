import heapq

from config_parser import ConfigParser, ConfigSyntaxError
from graph import Graph


def dijkstra(graph: Graph, start: str, goal: str) -> list[str]:
    priority_queue: list[tuple[float, str]] = [(0.0, start)]
    parent: dict[str, str | None] = {start: None}
    best_cost: dict[str, float] = {start: 0.0}

    while priority_queue:
        curr_cost, curr = heapq.heappop(priority_queue)

        if curr_cost > best_cost[curr]:
            continue

        if curr == goal:
            break

        for neighbor, _ in graph.neighbors(curr):
            if graph.is_blocked(neighbor):
                continue

            new_cost = curr_cost + graph.move_cost(neighbor)

            if new_cost < best_cost.get(neighbor, float("inf")):
                best_cost[neighbor] = new_cost
                parent[neighbor] = curr
                heapq.heappush(priority_queue, (new_cost, neighbor))

    if goal not in parent:
        return []

    path: list[str] = []
    hub: str | None = goal

    while hub is not None:
        path.append(hub)
        hub = parent[hub]

    return path[::-1]


if __name__ == "__main__":
    try:
        config = ConfigParser().parse("./maps/medium/01_dead_end_trap.txt")
    except FileNotFoundError as e:
        print(f"[ERROR]: File not found — {e.filename}")
        exit(1)
    except PermissionError as e:
        print(f"[ERROR]: Permission denied reading '{e.filename}'")
        exit(1)
    except ConfigSyntaxError as e:
        print(f"[ERROR]: {e}")
        exit(1)

    graph = Graph(config)

    start_hub, end_hub = graph.get_route_endpoints()

    print(dijkstra(graph, start_hub.name, end_hub.name))
