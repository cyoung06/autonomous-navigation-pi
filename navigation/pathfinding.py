import array

import heapq

from navigation.world import World


def find_path(origin: array, target: array, world: World):
    originCell = world.get_cell(origin)
    targetCell = world.get_cell(target)

    distances = {node: float('inf') for node in world.nodes}  # start로 부터의 거리 값을 저장하기 위함
    distances[originCell] = 0  # 시작 값은 0이어야 함
    connections = {originCell: originCell}

    queue = []
    heapq.heappush(queue, [distances[originCell], originCell])  # 시작 노드부터 탐색 시작 하기 위함.

    while queue:  # queue에 남아 있는 노드가 없으면 끝
        current_distance, current_destination = heapq.heappop(queue)  # 탐색 할 노드, 거리를 가져옴.

        if distances[current_destination] < current_distance:  # 기존에 있는 거리보다 길다면, 볼 필요도 없음
            continue

        for neighbor in current_destination.neighbors:
            distance = current_distance + neighbor.rel_pos.length()  # 해당 노드를 거쳐 갈 때 거리
            if distance < distances[neighbor.target]:  # 알고 있는 거리 보다 작으면 갱신
                distances[neighbor.target] = distance
                connections[neighbor.target] = neighbor
                heapq.heappush(queue, [distance, neighbor.target])

    st = targetCell
    route = []
    while st != originCell:
        route.append(connections[st])
        st = connections[st].origin

    return route