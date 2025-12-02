import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython.display import HTML
import networkx as nx
import random
import heapq

def animate(frames: list, start=None, p=None, interval=200):
    '''
    Turns a list of frames (made up of graphs) into a small animation and saves to optional
    specified path
    Parameters
    ----------
    frames: list
        List of networkx graphs
    start - optional:
        starting node for the graph. If specified, graph will use a bfs structure, otherwise it'll
        use a spring_layout
    path - optional: 
        path to save the resulting animation, will save using specified extension 
        if supplied and suported by your system, or will default to gif if no
        extension supplied
    interval - optional:
        interval in ms for the time between frames, default is 200ms
    Returns
        matplotlib FuncAnimation
    '''
    fig, ax = plt.subplots()
    G = frames[0]
    if start == None:
        pos = nx.spring_layout(G)
    else:
        if start not in list(G.nodes):
            print("Could not find starting node in graph, defaulting to spring")
            pos = nx.spring_layout(G)
        else: 
            pos = nx.bfs_layout(G, start)
    def update(frame):
        ax.clear()
        H = frames[frame]
        nx.draw(H, pos, with_labels=True, ax=ax, node_color=[H.nodes[n].get("color", "white") for n in H.nodes()], edge_color=[H.edges[i, j].get('color', 'black') for i, j in H.edges()], edgecolors='black')
        nx.draw_networkx_edge_labels(H, pos, edge_labels=nx.get_edge_attributes(H, 'weight', default=1))

    anim = FuncAnimation(fig, update, frames=len(frames), interval=interval)
    if p != None:
        ind = p.find('.')
        if ind == -1:
            anim.save(p+".gif")
        else:
            try:
                anim.save(p)
            except ValueError as e:
                print(e, "Make sure your environment supports writing this file format. Defaulting to gif")
                p = p[:ind]
                anim.save(p+".gif")
    return anim


def dijkstras(G: nx.Graph, start, end=None, frames=None):
    '''
    Parameters
    ----------
    G: 
        The Graph to search
    start:
        The node to begin the search
    end - optional: 
        The target node. If None
    frames - optional: 
        An array to be appended. Will append the state of the graph each step of the algorithm
    Returns
        list of nodes for the shortest path if end was set, otherwise a dict of 
        end_node: path pairs where end_node is every node in the graph and path is their respective
        path from start to that node
    '''
    dist = {node: float('inf') for node in G.nodes}
    dist[start] = 0
    parent = {node: None for node in G.nodes}

    # Initialize all node colors to white
    nx.set_node_attributes(G, 'white', 'color')

    pq = [(0, start)]
    visited = set()

    def save_frame():
        """Save a copy of G into frames"""
        if frames is not None:
            frames.append(G.copy())
    G.nodes[start]['color'] = 'green'
    if end:
        G.nodes[end]['color'] = 'green'
    save_frame()
    while pq:
        for _, node in pq:
            G.nodes[node]['color'] = 'red'
        save_frame()
        # for _, node in pq: # I used to set all red "candidate" nodes to white again, but the flashing made it difficult to look at
        #     G.nodes[node]['color'] = 'white'
        current_dist, u = heapq.heappop(pq)
        if u in visited:
            G.nodes[u]['color'] = 'gray' 
            continue
        
        G.nodes[u]['color'] = 'blue'
        save_frame()

        visited.add(u)

        # Mark as fully processed
        G.nodes[u]['color'] = 'gray'
        save_frame()

        # If we reached the end node early
        if end is not None and u == end:
            break

        for v in G.neighbors(u):
            w = G[u][v].get('weight', 1)

            if dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                parent[v] = u
                heapq.heappush(pq, (dist[v], v))

    def reconstruct_path(target):
        path = []
        while target is not None:
            path.append(target)
            target = parent[target]
        return path[::-1]

    if end is not None:
        path = reconstruct_path(end)

        if frames is not None:
            for i in range(len(path)):
                G.nodes[path[i]]['color'] = 'green'
                if i + 1 != len(path):
                    G.edges[path[i], path[i+1]]['color'] = "green"
            for _ in range(len(frames) // 5): # Repeats frame for pause on solution
                save_frame()

        return path

    else:
        all_paths = {node: reconstruct_path(node) for node in G.nodes}
        return all_paths


if __name__ == '__main__':
    G = nx.Graph()
    # Graph with nodes labeled with all upper case letters
    # Should mention my implementation of the animation is not well optimized so increasing our number
    # of nodes to something like 50, or all lower and upper case letters, takes the animation 2 mins to complete
    # If we plan on larger visual demos or this becomes a problem, let me know
    # There is likely room for significant runtime improvements (especially since I redraw every frame)
    for i in range(ord('A'), ord('Z') + 1):
        G.add_node(chr(i))
    items = list(G.nodes)
    for node in G.nodes:
        chance = 1.0
        while (random.random() < chance):
            if len(G[node]) < 3:
                to_connect = random.choice(items)
                while to_connect == node:
                    to_connect = random.choice(items)
                G.add_edge(node, to_connect, weight=random.randint(1, 10))
                if len(G[to_connect]) > 2:
                    items.remove(to_connect)
            chance /= 2
    # By this point we should have a relatively alright undirected graph with no loops
    items = list(G.nodes)
    start = random.choice(items)
    end = random.choice(items)
    print(f'start: {start}, end: {end}')
    frames = []
    path = dijkstras(G, start, end, frames)
    print(f'Path taken: {path}')
    print(f'Number of frames: {len(frames)}')
    # Gif is usually supported so it's my default, if you have something like ffmpeg installed and it can
    # be recognized you can use other file formats like mp4
    anim = animate(frames, start=start, p="out.gif", interval=10000 // len(frames))
    plt.show()