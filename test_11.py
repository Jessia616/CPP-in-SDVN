import networkx as nx

g = nx.DiGraph()

g.add_edge(1, 2, weight=1)
g.add_edge(3, 4, weight=1)
g.add_edge(2, 3, weight=1)
g.add_edge(4, 5, weight=1)
g.add_edge(5, 6, weight=1)

if(g.has_edge(1,5)):
    print(1)
else:
    print(0)