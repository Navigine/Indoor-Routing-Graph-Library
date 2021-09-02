#pragma once

#include <map>
#include <set>
#include <string>

#include <navigine/indoor_routing/geometry.h>
#include <navigine/indoor_routing/graph.h>

namespace navigine {
namespace indoor_routing {

struct RouteEvent
{
  enum class Event {
    TurnLeft,
    TurnRight,
    Transition
  };

  Event type;
  int value = 0;
  double distance  = 0;

  RouteEvent() = default;
  RouteEvent(Event _type, int _value, double _distance)
    : type     ( _type  )
    , value    ( _value )
    , distance ( _distance )
  { }
};

struct RoutePath
{
  double length = 0;
  std::vector<LocationPoint> points = {};
  std::vector<RouteEvent> events = {};

  bool isValid() const { return length > 0 && !points.empty(); }
};

struct RouteOptions
{
  bool isSourceOuter;
  bool isDestinationOuter;
};

class RouteGraph
{
  public:
    struct Level
    {
      int id = 0;              // Identifier
      double width = 0;        // Width in meters
      double height = 0;       // Height in meters
      double smoothRadius = 0; // Smooth radius

      bool operator==(const Level& level)const
      {
        return id == level.id;
      }

      bool operator<(const Level& level)const
      {
        return id < level.id;
      }

      bool isValid()const
      {
        return id > 0;
      }
    };

  public:
    struct Vertex: LocationPoint
    {
      int id = 0;                // Vertex identifier
      bool isElevation = false;  // Vertex is external?
      std::string name = "";     // Vertex name

      bool operator== ( const Vertex& v ) const { return level == v.level && id == v.id; }
      bool operator!= ( const Vertex& v ) const { return level != v.level || id != v.id; }
      bool operator<  ( const Vertex& v ) const { return level < v.level || (level == v.level && id < v.id); }
      bool operator>  ( const Vertex& v ) const { return level > v.level || (level == v.level && id > v.id); }

      bool isValid() const { return id > 0; }
    };

  public:
    struct Edge
    {
      int level = 0;
      int id = 0;
      int src = 0;
      int dst = 0;
      double weight = 1.0;

      bool operator== ( const Edge& e ) const { return level == e.level && id == e.id; }
      bool operator!= ( const Edge& e ) const { return level != e.level || id != e.id; }
      bool operator<  ( const Edge& e ) const { return level < e.level || (level == e.level && id < e.id); }
      bool operator>  ( const Edge& e ) const { return level > e.level || (level == e.level && id > e.id); }

      bool isValid() const { return id > 0; }
    };

  public:
    using LevelIterator = std::set<Level>::const_iterator;
    using VertexIterator = std::set<RouteGraph::Vertex>::const_iterator;
    using EdgeIterator = std::set<RouteGraph::Edge>::const_iterator;

  public:
    RouteGraph();
    RouteGraph(const RouteGraph& G);
    RouteGraph& operator=(const RouteGraph& G);

    // Clear/empty graph
    void clear();
    bool isEmpty()const;

    // Add/get Level
    void addLevel(const Level& level);
    Level getLevel(int id)const;

    // Manipulating vertex
    int  addVertex(Vertex v);
    void removeVertex(int id);
    void moveVertex(int id, double x, double y);
    bool hasVertex(int id)const;
    void setVertexElevation(int id, bool elevation);
    Vertex getVertex(int id)const;
    VertexIterator vertexBegin()const;
    VertexIterator vertexEnd()const;

    std::vector<Vertex> getElevationPoints()const;
    std::vector<Vertex> getElevationPoints(int level)const;

    // Manipulating edge
    int addEdge(Edge e);
    void removeEdge(int id);
    bool hasEdge(int id)const;
    Edge getEdge(int id)const;
    Edge getEdge(int level, int src, int dst)const;
    void setEdgeWeight(int id, double weight);
    EdgeIterator edgeBegin()const;
    EdgeIterator edgeEnd()const;

    void setSmoothRadius(double radius);
    void setSmoothRadius(int level, double radius);
    double getSmoothRadius(int level)const;

  public:
    double getDistance (LocationPoint P) const;
    LocationPoint getProjection (LocationPoint P) const;
    RoutePath getPath (LocationPoint P, LocationPoint Q) const;
    std::vector<RoutePath> getPaths (LocationPoint P, std::vector<LocationPoint> targets) const;

  public:
    int checkCloseVertice (std::vector<std::string>* errors = 0) const;
    int checkSplitEdges (std::vector<std::string>* errors = 0) const;
    int checkCrossEdges (std::vector<std::string>* errors = 0) const;
    int checkLevelConnectivity (std::vector<std::string>* errors = 0) const;
    int checkElevationConnectivity (std::vector<std::string>* errors = 0) const;
    int checkFullConnectivity (std::vector<std::string>* errors = 0) const;
    int checkFull (std::vector<std::string>* errors = 0) const;

  private:
    Graph buildGraph (void) const;

    RoutePath getSmoothPath(const PolyLine& points)const;
    PolyLine getSmoothPoly(const PolyLine& points, std::vector<RouteEvent>* events)const;

  private:
    std::set<Level> mLevels = {};         // Set of Levels
    std::set<Vertex> mVertice = {};       // Set of vertice
    std::set<Edge> mEdges = {};           // Set of edges
    std::map<int, VertexIterator> mVertexMap = {};
    std::map<int, EdgeIterator> mEdgeMap = {};
};

std::vector<PolyLine> SplitPolyLine(const PolyLine& points, size_t nparts);

} } // namespace navigine::indoor_routing
