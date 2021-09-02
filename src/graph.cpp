#include <algorithm>
#include <navigine/indoor_routing/graph.h>
#include <navigine/indoor_routing/geometry.h>

namespace navigine {
namespace indoor_routing {

bool Graph::isEmpty()const
{
  return mVertice.empty();
}

void Graph::clear()
{
  mVertice.clear();
  mEdges.clear();
}

void Graph::addVertex(int v)
{
  // Verifying vertex
  if (v < 0)
    return;

  mVertice.insert(v);
}

void Graph::removeVertex(int v)
{
  mVertice.erase(v);
  for(auto iter = mEdges.begin(); iter != mEdges.end(); )
    if (iter->begin == v || iter->end == v)
      mEdges.erase(iter++);
    else
      ++iter;
}

void Graph::addEdge(int v1, int v2, double weight)
{
  // Verifying edge parameters
  if (v1 < 0 || v2 < 0 || v1 == v2 ||
      mVertice.find(v1) == mVertice.end() ||
      mVertice.find(v2) == mVertice.end())
    return;

  weight = (std::max)(weight, EPSILON);

  // Searching for duplicates. If duplicate edge exist,
  // replace its weight with the specified value.
  auto iter = mEdges.lower_bound(Edge{.begin=v1, .end=v2, .weight=0});
  if (iter != mEdges.end() && iter->begin == v1 && iter->end == v2)
  {
    Edge& e = const_cast<Edge&>(*iter);
    e.weight = weight;
    return;
  }

  mEdges.insert(iter, Edge{.begin=v1, .end=v2, .weight=weight});
}

void Graph::removeEdge(int v1, int v2)
{
  // Verifying edge parameters
  if (v1 < 0 || v2 < 0 || v1 == v2 ||
      mVertice.find(v1) == mVertice.end() ||
      mVertice.find(v2) == mVertice.end())
    return;

  // Searching for (v1 -> v2) edge. If such edge exist, remove it.
  auto iter = mEdges.lower_bound(Edge{.begin=v1, .end=v2, .weight=0.0});
  if (iter != mEdges.end() && iter->begin == v1 && iter->end == v2)
    mEdges.erase(iter);
}

double Graph::getPath(int v1, int v2, std::vector<int>* path, std::map<int, EdgeProperties>* stmap)const
{
  if (mVertice.find(v1) == mVertice.end() || mVertice.find(v2) == mVertice.end())
    return -1.0;

  std::set<int> current; // stores the working set: unvisited reachable vertices
  std::map<int, EdgeProperties> stateMap;
  stateMap[v1] = EdgeProperties();
  current.insert(v1);

  while (true)
  {
    // Searching for the closest vertex...
    std::pair<int, EdgeProperties> p0;
    p0.first = -1;

    for(auto it = current.begin(); it != current.end(); ++it)
    {
      auto iter = stateMap.find(*it);
      if (p0.first == -1 || iter->second.edgeWeight < p0.second.edgeWeight)
        p0 = *iter;
    }

    if (p0.first < 0)
      break; // All reachable vertice are visited => finished

    current.erase(p0.first);

    for(auto eIter = mEdges.lower_bound(Edge{.begin=p0.first, .end=0, .weight=0.0});
        eIter != mEdges.end() && eIter->begin == p0.first; ++eIter)
    {
      const Edge& e = *eIter;
      const double dist0 = p0.second.edgeWeight + e.weight;

      auto iter = stateMap.find(e.end);
      if (iter != stateMap.end())
      {
        if (iter->second.edgeWeight > dist0)
        {
          iter->second.parentVertex  = p0.first;
          iter->second.edgeWeight = dist0;
        }
      }
      else
      {
        stateMap[e.end] = EdgeProperties{.parentVertex=p0.first, .edgeWeight=dist0};
        current.insert(e.end);
      }
    }
  }

  if (path)
  {
    path->clear();
    for(int v = v2; v >= 0; )
    {
      auto iter = stateMap.find(v);
      if (iter == stateMap.end())
        return -1.0;

      path->push_back(v);
      v = iter->second.parentVertex;
    }
    std::reverse(path->begin(), path->end());
  }

  if (stmap)
    *stmap = stateMap;

  return stateMap[v2].edgeWeight;
}

void Graph::dfsVisit(const std::set<Edge>& edges, int u, std::map<int, VertexTimes>& stateMap, int& time)
{
  stateMap[u].entryTime = ++time;
  for(auto eIter = edges.lower_bound(Edge{.begin=u, .end=0, .weight=0.0});
      eIter != edges.end() && eIter->begin == u; ++eIter)
  {
    int v = eIter->end;
    if (stateMap[v].entryTime < 0)
      dfsVisit(edges, v, stateMap, time);
  }
  stateMap[u].exitTime = ++time;
}

std::vector<std::set<int>> Graph::getSCC()const
{
  // Doing DFS
  std::map<int, VertexTimes> stateMap;
  for(int u : mVertice)
    stateMap[u] = VertexTimes();

  int time = 0;
  for(int u : mVertice)
    if (stateMap[u].entryTime < 0)
      dfsVisit(mEdges, u, stateMap, time);

  // Building reverse edges
  std::set<Edge> rEdges;
  for(auto e : mEdges)
    rEdges.insert(Edge{.begin=e.end, .end=e.begin, .weight=e.weight});

  // Building ordered vertex set
  std::map<int,int> vmap;
  for(auto p : stateMap)
    vmap[-p.second.exitTime] = p.first;

  // Reinitializing stateMap
  for(int u : mVertice)
    stateMap[u] = VertexTimes();

  // Reinitializing time
  time = 0;

  // Doing DFS for reverse graph
  for(auto u : vmap)
    if (stateMap[u.second].entryTime < 0)
      dfsVisit(rEdges, u.second, stateMap, time);

  std::vector<int> tree(time, 0);
  for(auto p : stateMap)
    tree[p.second.entryTime  - 1] = tree[p.second.exitTime - 1] = p.first;

  std::vector<std::set<int>> sccList;
  std::set<int> scc;

  for(size_t i = 0, marker = -1; i < tree.size(); ++i)
  {
    if (tree[i] == marker)
    {
      sccList.push_back(scc);
      scc.clear();
      marker = -1;
      continue;
    }

    if (marker < 0)
      marker = tree[i];

    scc.insert(tree[i]);
  }

  return sccList;
}

void Graph::print(FILE* fp)const
{
  fprintf(fp, "Route graph:\n");
  for(std::set<Edge>::const_iterator iter = mEdges.begin(); iter != mEdges.end(); ++iter)
    fprintf(fp, "Edge %d -> %d [weight=%.2f]\n", iter->begin, iter->end, iter->weight);
}

} } // namespace navigine::indoor_routing
