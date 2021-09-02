This library contains a set of algorithms for working with the routing graph.  
The main features are:

* build a graph by specifying the vertices in Cartesian coordinates
* check if the graph is strongly connected (`Kosaraju's` algorithm)
* find the shortest route from the starting point to one or more destination points (`Dijkstra's` algorithm)

The key features of this library in comparison with similar ones are:  

* the ability to set start and destination points outside the graph  
 (in other words, it is possible to project these points onto the original graph and add corresponding edges to it)
* multilevel graph supported
* the resulting route contains events - hints in which direction to move to reach the destination 

## Build

To build a library and unit tests that check the validity of a given graph and find the optimal route, you need to run a number of commands:

```
1. cmake -S . -B build -DJSON_CI=On
2. cmake --build build
```

## Example

Create and fill in the graph:

```cpp
    mRouteGrapgh = RouteGraph();
    mRouteGrapgh.addLevel(RouteGraph::Level{.id=22129, .width=0, .height=0, .smoothRadius=0});
    const int vid1 = mRouteGrapgh.addVertex(RouteGraph::Vertex{LocationPoint{.level=22129, .x=18.32, .y=21.33}, .id=125441});
    const int vid2 = mRouteGrapgh.addVertex(RouteGraph::Vertex{LocationPoint{.level=22129, .x=18.24, .y=8.85}, .id=125442});
    const int vid3 = mRouteGrapgh.addVertex(RouteGraph::Vertex{LocationPoint{.level=22129, .x=19.02, .y=8.82}, .id=125449});
    const int vid4 = mRouteGrapgh.addVertex(RouteGraph::Vertex{LocationPoint{.level=22129, .x=18.87, .y=4.15}, .id=125450});
    const int vid5 = mRouteGrapgh.addVertex(RouteGraph::Vertex{LocationPoint{.level=22129, .x=19.73, .y=8.79}, .id=125451});


    const int eid1 = mRouteGrapgh.addEdge(RouteGraph::Edge{.level=22129, .id=1, .src=vid1, .dst=vid2, .weight=1.0});
    const int eid2 = mRouteGrapgh.addEdge(RouteGraph::Edge{.level=22129, .id=2, .src=vid2, .dst=vid1, .weight=1.0});
    const int eid3 = mRouteGrapgh.addEdge(RouteGraph::Edge{.level=22129, .id=3, .src=vid3, .dst=vid4, .weight=1.0});
    const int eid4 = mRouteGrapgh.addEdge(RouteGraph::Edge{.level=22129, .id=4, .src=vid4, .dst=vid3, .weight=1.0});
    const int eid5 = mRouteGrapgh.addEdge(RouteGraph::Edge{.level=22129, .id=5, .src=vid3, .dst=vid5, .weight=1.0});
    const int eid6 = mRouteGrapgh.addEdge(RouteGraph::Edge{.level=22129, .id=6, .src=vid5, .dst=vid3, .weight=1.0});
    const int eid7 = mRouteGrapgh.addEdge(RouteGraph::Edge{.level=22129, .id=7, .src=vid2, .dst=vid3, .weight=1.0});
    const int eid8 = mRouteGrapgh.addEdge(RouteGraph::Edge{.level=22129, .id=8, .src=vid3, .dst=vid2, .weight=1.0});
```

Check if the constructed graph is valid:  
This operation includes:

1. snapping close edges of the graph
2. splitting graph edges
3. checking the intersecting edges of the graph
4. level connectivity check
5. checking the connectivity of transitions between levels 

```cpp
int errNum = mRouteGrapgh.checkFull(&errors);
if (!(errNum > 0))
    std::cout << "Graph is valid." << std::endl;
```

Set start and destination points:

```cpp
  LocationPoint P{.level=22129, .x=17.14, .y=18.27};
  LocationPoint Q{.level=22129, .x=19.71, .y=6.78};
```

Build a route:  

This method returns a structure ``RoutePath`` that contains:

1. the length of the route 
2. a set of bypass vertices
3. a set of events

```cpp
RoutePath path = mRouteGrapgh.getPath(P, Q);
```
