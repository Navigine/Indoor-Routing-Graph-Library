#pragma once

#include <vector>
#include <set>
#include <map>

namespace navigine {
namespace indoor_routing {

class Graph
{
  public:
    struct EdgeProperties
    {
      int parentVertex = -1;
      double edgeWeight = 0.0;
    };

  private:
    struct VertexTimes
    {
      int entryTime = -1;
      int exitTime = -1;
    };

    struct Edge
    {
      int begin = -1;
      int end = -1;
      double weight = 0.0;

      bool operator== ( const Edge& e ) const { return begin == e.begin && end == e.end; }
      bool operator!= ( const Edge& e ) const { return begin != e.begin || end != e.end; }
      bool operator<  ( const Edge& e ) const { return begin < e.begin || (begin == e.begin && end < e.end); }
      bool operator>  ( const Edge& e ) const { return begin > e.begin || (begin == e.begin && end > e.end); }
    };

  public:
    /// Check if graph is empty
    bool isEmpty()const;

    /// Clear graph
    void clear();

    /// Add vertex to the graph.
    void addVertex(int v);
    void removeVertex(int v);

    /// Add edge to the graph.
    void addEdge(int v1, int v2, double weight = 1.0);
    void removeEdge(int v1, int v2);

    /// Determine the lightest path from v1 to v2.
    /// On success, function returns the overall weight of the found path,
    /// which is stored in the corresponding output variable.
    /// Otherwise, function returns the negative value.
    double getPath(int v1, int v2, std::vector<int>* path = 0,
                   std::map<int, EdgeProperties>* stateMap = 0)const;

    std::vector<std::set<int>> getSCC()const;

    void print(FILE* fp = stdout)const;

  private:
    static void dfsVisit(const std::set<Edge>& edges, int u, std::map<int, VertexTimes>& m, int& time);

  private:
    std::set<int> mVertice;
    std::set<Edge> mEdges;
};

} } // namespace navigine::indoor_routing
