#include <algorithm>
#include <limits.h>
#include <cmath>

#include <navigine/indoor_routing/router.h>

namespace navigine {
namespace indoor_routing {

const double STICK_DISTANCE     = 0.1;  // meters
const double MIN_ROTATION_ANGLE = 10;   // degrees

RouteGraph::RouteGraph()
{ }

RouteGraph::RouteGraph(const RouteGraph& G)
  : mLevels ( G.mLevels )
  , mVertice ( G.mVertice )
  , mEdges ( G.mEdges )
{
  for(VertexIterator vIter = mVertice.begin(); vIter != mVertice.end(); ++vIter)
    mVertexMap[vIter->id] = vIter;
  for(EdgeIterator eIter = mEdges.begin(); eIter != mEdges.end(); ++eIter)
    mEdgeMap[eIter->id] = eIter;
}

RouteGraph& RouteGraph::operator=(const RouteGraph& G)
{
  mLevels = G.mLevels;
  mVertice = G.mVertice;
  mEdges = G.mEdges;
  mVertexMap.clear();
  mEdgeMap.clear();
  for(VertexIterator vIter = mVertice.begin(); vIter != mVertice.end(); ++vIter)
    mVertexMap[vIter->id] = vIter;
  for(EdgeIterator eIter = mEdges.begin(); eIter != mEdges.end(); ++eIter)
    mEdgeMap[eIter->id] = eIter;

  return *this;
}

void RouteGraph::clear()
{
  *this = RouteGraph();
}

bool RouteGraph::isEmpty()const
{
  return mLevels.empty() &&
         mVertice.empty() &&
         mEdges.empty();
}

void RouteGraph::addLevel(const Level& level)
{
  mLevels.insert(level);
}

RouteGraph::Level RouteGraph::getLevel(int id)const
{
  LevelIterator iter = mLevels.lower_bound(Level{.id=id, .width=0, .height=0, .smoothRadius=0});
  if (iter != mLevels.end() && iter->id == id)
    return *iter;
  return Level();
}

int RouteGraph::addVertex(Vertex v)
{
  // Determine vertex id (if necessary)
  if (!v.id)
    v.id = mVertexMap.empty() ? 1 : mVertexMap.rbegin()->first + 1;

  // Check if vertex already exists
  if (hasVertex(v.id))
    return 0;

  // Check if level exists
  Level level = getLevel(v.level);
  if (!level.isValid())
    return 0;

  // Add vertex to RouteGraph
  std::pair<VertexIterator, bool> p = mVertice.insert(v);
  mVertexMap[v.id] = p.first;
  return v.id;
}

void RouteGraph::removeVertex(int id)
{
  // Check if vertex with the specified id exist
  std::map<int, VertexIterator>::iterator iter = mVertexMap.find(id);
  if (iter == mVertexMap.end())
    return;

  // Remove all edges incident to the specified vertex
  for(EdgeIterator eIter = edgeBegin(); eIter != edgeEnd(); )
    if (eIter->src == id || eIter->dst == id)
      removeEdge((eIter++)->id);
    else
      ++eIter;

  // Remove vertex from RouteGraph
  mVertice.erase(iter->second);
  mVertexMap.erase(iter);
}

void RouteGraph::moveVertex(int id, double x, double y)
{
  // Check if vertex with the specified id exist
  std::map<int, VertexIterator>::iterator iter = mVertexMap.find(id);
  if (iter == mVertexMap.end())
    return;

  // Modify vertex coordinates
  Vertex& P = const_cast<Vertex&>(*iter->second);
  P.x = x;
  P.y = y;
}

bool RouteGraph::hasVertex(int id)const
{
  return mVertexMap.find(id) != mVertexMap.end();
}

void RouteGraph::setVertexElevation(int id, bool elevation)
{
  std::map<int, VertexIterator>::const_iterator iter = mVertexMap.find(id);
  if (iter != mVertexMap.end())
  {
    Vertex& P = const_cast<Vertex&>(*(iter->second));
    P.isElevation = elevation;
  }
}

RouteGraph::Vertex RouteGraph::getVertex(int id)const
{
  std::map<int, VertexIterator>::const_iterator iter = mVertexMap.find(id);
  if (iter != mVertexMap.end())
    return *(iter->second);

  return Vertex();
}

RouteGraph::VertexIterator RouteGraph::vertexBegin (void) const { return mVertice.begin(); }
RouteGraph::VertexIterator RouteGraph::vertexEnd   (void) const { return mVertice.end();   }

std::vector<RouteGraph::Vertex> RouteGraph::getElevationPoints()const
{
  std::vector<Vertex> elevationPoints;
  for(VertexIterator vIter = mVertice.begin(); vIter != mVertice.end(); ++vIter)
    if (vIter->isElevation)
      elevationPoints.push_back(*vIter);
  return elevationPoints;
}

std::vector<RouteGraph::Vertex> RouteGraph::getElevationPoints(int level)const
{
  std::vector<Vertex> elevationPoints;
  for(VertexIterator vIter = mVertice.lower_bound(Vertex{LocationPoint{.level=level, .x=0, .y=0}, .id=0});
      vIter != mVertice.end() && vIter->level == level; ++vIter)
  {
    if (vIter->isElevation)
      elevationPoints.push_back(*vIter);
  }
  return elevationPoints;
}

int RouteGraph::addEdge(Edge e)
{
  // Determine edge id (if necessary)
  if (!e.id)
    e.id = mEdgeMap.empty() ? 1 : mEdgeMap.rbegin()->first + 1;

  // Check if edge already exists
  if (hasEdge(e.id))
    return 0;

  // Validate edge
  if (e.src <= 0 || e.dst <= 0 || e.src == e.dst)
    return 0;

  // Check if edge's source and destination vertice exist
  if (!hasVertex(e.src) || !hasVertex(e.dst))
    return 0;

  // Add edge to RouteGraph
  std::pair<EdgeIterator, bool> p = mEdges.insert(e);
  mEdgeMap[e.id] = p.first;
  return e.id;
}

void RouteGraph::removeEdge(int id)
{
  // Check if edge with the specified id exist
  std::map<int, EdgeIterator>::iterator iter = mEdgeMap.find(id);
  if (iter == mEdgeMap.end())
    return;

  // Remove edge from RouteGraph
  mEdges.erase(iter->second);
  mEdgeMap.erase(iter);
}

bool RouteGraph::hasEdge(int id)const
{
  return mEdgeMap.find(id) != mEdgeMap.end();
}

RouteGraph::Edge RouteGraph::getEdge(int id)const
{
  std::map<int, EdgeIterator>::const_iterator iter = mEdgeMap.find(id);
  return iter == mEdgeMap.end() ? Edge() : *(iter->second);
}

RouteGraph::Edge RouteGraph::getEdge(int level, int src, int dst)const
{

  for(EdgeIterator eIter = mEdges.lower_bound(Edge{.level=level, .id=0, .src=0, .dst=0, .weight=0.0});
      eIter != mEdges.end() && eIter->level == level; ++eIter)
  {
    if (eIter->src == src && eIter->dst == dst)
      return *eIter;
  }
  return Edge();
}

void RouteGraph::setEdgeWeight(int id, double weight)
{
  std::map<int, EdgeIterator>::const_iterator iter = mEdgeMap.find(id);
  if (iter == mEdgeMap.end())
    return;

  Edge& e = const_cast<Edge&>(*(iter->second));
  e.weight = (std::max)(weight, EPSILON);
}

RouteGraph::EdgeIterator RouteGraph::edgeBegin (void) const { return mEdges.begin(); }
RouteGraph::EdgeIterator RouteGraph::edgeEnd   (void) const { return mEdges.end();   }

void RouteGraph::setSmoothRadius(double radius)
{
  for(std::set<Level>::iterator iter = mLevels.begin(); iter != mLevels.end(); ++iter)
  {
    Level& level = const_cast<Level&>(*iter);
    level.smoothRadius = radius;
  }
}

void RouteGraph::setSmoothRadius(int level, double radius)
{
  for(std::set<Level>::iterator iter = mLevels.begin(); iter != mLevels.end(); ++iter)
    if (iter->id == level)
    {
      Level& level = const_cast<Level&>(*iter);
      level.smoothRadius = radius;
    }
}

double RouteGraph::getSmoothRadius(int level)const
{
  for(std::set<Level>::iterator iter = mLevels.begin(); iter != mLevels.end(); ++iter)
    if (iter->id == level)
      return iter->smoothRadius;
  return 0.0;
}

Graph RouteGraph::buildGraph()const
{
  Graph G;

  // Adding vertice
  for(auto v : mVertice)
    G.addVertex(v.id);

  // Adding edges
  for(auto e : mEdges)
  {
    double weight = e.weight;
    if (e.level < INT_MAX)
    {
      Vertex u = getVertex(e.src);
      Vertex v = getVertex(e.dst);
      weight *= GetDist(u.x, u.y, v.x, v.y);
    }
    G.addEdge(e.src, e.dst, weight);
  }

  // Connect close vertice
  for(auto v : mVertice)
  {
    for(VertexIterator vIter = mVertice.upper_bound(Vertex{LocationPoint{.level=v.level, .x=0, .y=0}, .id=v.id});
        vIter != mVertice.end() && vIter->level == v.level; ++vIter)
    {
      double dist = GetDist(v.x, v.y, vIter->x, vIter->y);
      if (dist < STICK_DISTANCE)
      {
        G.addEdge(v.id, vIter->id, dist);
        G.addEdge(vIter->id, v.id, dist);
      }
    }
  }

  // Splitting edges
  for(auto e : mEdges)
  {
    if (e.level == INT_MAX)
      continue;

    Vertex A = this->getVertex(e.src);
    Vertex B = this->getVertex(e.dst);

    std::vector<std::pair<double,Vertex> > splitPoints;
    splitPoints.push_back(std::make_pair(0.0, A));
    splitPoints.push_back(std::make_pair(1.0, B));

    for(VertexIterator vIter = mVertice.lower_bound(Vertex{LocationPoint{.level=e.level, .x=0, .y=0}, .id=0});
        vIter != mVertice.end() && vIter->level == e.level; ++vIter)
    {
      double px = 0.0;
      double py = 0.0;
      double k  = GetProjection(A.x, A.y, B.x, B.y, vIter->x, vIter->y, &px, &py);
      if (GetDist(vIter->x, vIter->y, px, py) < STICK_DISTANCE && 0 < k && k < 1)
        splitPoints.push_back(std::make_pair(k, *vIter));
    }

    // Splitting edge
    if (splitPoints.size() >= 3)
    {
      std::sort(splitPoints.begin(), splitPoints.end());
      for(int i = 1; i < (int)splitPoints.size(); ++i)
      {
        double dist = GetDist(splitPoints[i-1].second.x,
                              splitPoints[i-1].second.y,
                              splitPoints[i].second.x,
                              splitPoints[i].second.y);
        G.addEdge(splitPoints[i-1].second.id,
                  splitPoints[i].second.id,
                  dist * e.weight);
      }
    }
  }

  return G;
}

RoutePath RouteGraph::getPath(LocationPoint P, LocationPoint Q)const
{
  RouteGraph G ( *this );

  LocationPoint P1 = this->getProjection(P);

  LocationPoint Q1 = this->getProjection(Q);

  if (!P1.isValid() || !Q1.isValid())
    return RoutePath();

  const int pid  = G.addVertex(Vertex{LocationPoint{.level=P.level, .x=P.x, .y=P.y}, .id=0});
  const int qid  = G.addVertex(Vertex{LocationPoint{.level=Q.level, .x=Q.x, .y=Q.y}, .id=0});

  const int pid1 = G.addVertex(Vertex{LocationPoint{.level=P.level, .x=P1.x, .y=P1.y}, .id=0});
  const int qid1 = G.addVertex(Vertex{LocationPoint{.level=Q.level, .x=Q1.x, .y=Q1.y}, .id=0});

  Graph graph = G.buildGraph();
  graph.addEdge(pid, pid1, 1.0);
  graph.addEdge(qid1, qid, 1.0);

  std::vector<int> ipath;
  double w = graph.getPath(pid, qid, &ipath);

  if (w > 0)
  {
    PolyLine polyLine;
    for(int i = 0; i < (int)ipath.size(); ++i)
      polyLine.push_back(G.getVertex(ipath[i]));
    return getSmoothPath(polyLine);
  }

  return RoutePath();
}

std::vector<RoutePath> RouteGraph::getPaths(LocationPoint P, std::vector<LocationPoint> targets)const
{
  std::vector<RoutePath> paths ( targets.size() );
  std::vector<int> qids ( targets.size() );
  std::vector<int> qids1 ( targets.size() );

  LocationPoint P1 = this->getProjection(P);
  if (!P1.isValid())
    return paths;

  RouteGraph G ( *this );

  const int pid  = G.addVertex(Vertex{LocationPoint{.level=P.level, .x=P.x, .y=P.y}, .id=0});
  const int pid1 = G.addVertex(Vertex{LocationPoint{.level=P.level, .x=P1.x, .y=P1.y}, .id=0});

  for(int i = 0; i < (int)targets.size(); ++i)
  {
    LocationPoint Q  = targets[i];
    LocationPoint Q1 = this->getProjection(Q);

    qids[i]  = G.addVertex(Vertex{LocationPoint{.level=Q.level, .x=Q.x, .y=Q.y}, .id=0});
    qids1[i] = G.addVertex(Vertex{LocationPoint{.level=Q.level, .x=Q1.x, .y=Q1.y}, .id=0});
  }

  Graph graph = G.buildGraph();
  graph.addEdge(pid, pid1, 1.0);
  for(int i = 0; i < (int)qids.size(); ++i)
    graph.addEdge(qids1[i], qids[i], 1.0);

  std::map<int, Graph::EdgeProperties> stateMap;
  graph.getPath(pid, pid, 0, &stateMap);

  for(int t = 0; t < (int)targets.size(); ++t)
  {
    std::vector<int> ipath;
    bool ok = true;

    for(int v = qids[t]; v >= 0; )
    {
      auto iter = stateMap.find(v);
      if (iter == stateMap.end())
      {
        ok = false;
        break;
      }
      ipath.push_back(v);
      v = iter->second.parentVertex;
    }
    if (!ok)
      continue; // Route is not found

    // Reversing path
    std::reverse(ipath.begin(), ipath.end());

    // Building the RoutePath
    PolyLine polyLine;
    for(int i = 0; i < (int)ipath.size(); ++i)
      polyLine.push_back(G.getVertex(ipath[i]));
    paths[t] = getSmoothPath(polyLine);
  }
  return paths;
}

RoutePath RouteGraph::getSmoothPath(const PolyLine& points)const
{
  if (points.empty())
    return RoutePath();

  RoutePath path;
  for(int i = 0, i0 = 0; i0 < (int)points.size(); i0 = i)
  {
    while (i < (int)points.size() && points[i].level == points[i0].level)
      ++i;

    std::vector<RouteEvent> levelEvents;
    std::vector<LocationPoint> levelPoints = getSmoothPoly(PolyLine(points.begin() + i0, points.begin() + i), &levelEvents);

    if (!path.events.empty())
      path.events.push_back(RouteEvent(RouteEvent::Event::Transition, points[i0].level, path.length));

    for(int j = 0; j < (int)levelEvents.size(); ++j)
      path.events.push_back(RouteEvent(levelEvents[j].type,
                                       levelEvents[j].value,
                                       levelEvents[j].distance + path.length));

    path.points.push_back(levelPoints[0]);
    for(int j = 1; j < (int)levelPoints.size(); ++j)
    {
      path.length += GetDist(levelPoints[j], levelPoints[j-1]);
      path.points.push_back(levelPoints[j]);
    }
  }
  return path;
}

PolyLine RouteGraph::getSmoothPoly(const PolyLine& points, std::vector<RouteEvent>* events)const
{
  if (points.size() < 3)
    return points;

  const double radius = (std::max)(getSmoothRadius(points[0].level), EPSILON);

  /// Step 1: simplify the polyline using the Douglas-Peuker algorithm
  std::vector<bool> keepPoint(points.size(), true);
  std::vector<std::pair<int,int> > stack;
  stack.push_back(std::make_pair(0, points.size() - 1));

  while (!stack.empty())
  {
    int startPos = stack.back().first;
    int stopPos  = stack.back().second;
    stack.pop_back();

    if (stopPos - startPos <= 1)
      continue;

    double dmax = 0.0;
    int    pos  = startPos;

    for(int i = startPos + 1; i <= stopPos - 1; ++i)
    {
      if (keepPoint[i])
      {
        double d = GetDist(points[startPos], points[stopPos], points[i]);
        if (d > dmax)
        {
          dmax = d;
          pos = i;
        }
      }
    }

    if (dmax >= radius)
    {
      stack.push_back(std::make_pair(startPos, pos));
      stack.push_back(std::make_pair(pos, stopPos));
    }
    else
    {
      for(int i = startPos + 1; i <= stopPos - 1; ++i)
        keepPoint[i] = false;
    }
  }

  PolyLine points1;
  for(int i = 0; i < (int)points.size(); ++i)
    if (keepPoint[i])
      points1.push_back(points[i]);

  /// Step 2: round polyline and calculate route events
  PolyLine points2;
  points2.push_back(points1[0]);

  double dist = 0.0;
  for(int i = 1; i < (int)points1.size() - 1; ++i)
  {
    // Rounding angle ABC
    LocationPoint A = points1[i-1];
    LocationPoint B = points1[i];
    LocationPoint C = points1[i+1];

    double AB = GetDist(A, B);
    double BC = GetDist(B, C);
    double angle = acos(std::min(std::max(((B.x - A.x) * (B.x - C.x) + (B.y - A.y) * (B.y - C.y)) / AB / BC, -1.0), 1.0));
    double len = radius * cos(angle/2) / (1 - sin(angle/2));
    len = std::min(len, AB/2);
    len = std::min(len, BC/2);

    int rot_angle = static_cast<int>(round(180 - angle * 180 / M_PI));

    double k1 = len / AB;
    double k2 = len / BC;

    LocationPoint B1{.level=B.level, .x=A.x * k1 + B.x * (1-k1), .y=A.y * k1 + B.y * (1-k1)};
    LocationPoint B2{.level=B.level, .x=C.x * k2 + B.x * (1-k2), .y=C.y * k2 + B.y * (1-k2)};

    // Solving system of equations:
    // ox * (B.x - A.x) + oy * (B.y - A.y) = B1.x * (B.x - A.x) + B1.y * (B.y - A.y)
    // ox * (B.x - C.x) + oy * (B.y - C.y) = B2.x * (B.x - C.x) + B2.y * (B.y - C.y)
    double det  = (B.x - A.x) * (B.y - C.y) - (B.y - A.y) * (B.x - C.x);
    double detX = (B1.x * (B.x - A.x) + B1.y * (B.y - A.y)) * (B.y - C.y) - (B.y - A.y) * (B2.x * (B.x - C.x) + B2.y * (B.y - C.y));
    double detY = (B.x - A.x) * (B2.x * (B.x - C.x) + B2.y * (B.y - C.y)) - (B1.x * (B.x - A.x) + B1.y * (B.y - A.y)) * (B.x - C.x);

    LocationPoint O{.level=B.level, .x=detX / det, .y=detY / det};
    double R = GetDist(O, B1); // Real smooth radius

    if (R < 0.01)
    {
      points2.push_back(points1[i]);
      dist += GetDist(points2[points2.size()-2],
                      points2[points2.size()-1]);
      if (rot_angle >= MIN_ROTATION_ANGLE && events)
        events->push_back(RouteEvent(det > 0 ? RouteEvent::Event::TurnRight : RouteEvent::Event::TurnLeft, rot_angle, dist));
      continue;
    }

    double alpha1 = atan2(B1.y - O.y, B1.x - O.x);
    double alpha2 = atan2(B2.y - O.y, B2.x - O.x);
    double dalpha = alpha2 - alpha1;
    if (dalpha > +M_PI) dalpha -= 2 * M_PI;
    if (dalpha < -M_PI) dalpha += 2 * M_PI;

    const int nsteps = std::max(2, (int)std::round(std::fabs(angle * 180.0 / M_PI / MIN_ROTATION_ANGLE)));
    for(int k = 0; k <= nsteps; ++k)
    {
      double a = alpha1 + k * dalpha / nsteps;

      LocationPoint X{.level=B.level, .x=O.x + R * cos(a), .y=O.y + R * sin(a)};

      points2.push_back(X);
      dist += GetDist(points2[points2.size()-2],
                      points2[points2.size()-1]);
      if (rot_angle >= MIN_ROTATION_ANGLE && events && k == nsteps/2)
        events->push_back(RouteEvent(det > 0 ? RouteEvent::Event::TurnRight : RouteEvent::Event::TurnLeft, rot_angle, dist));
    }
  }
  points2.push_back(points1.back());
  return points2;
}

double RouteGraph::getDistance(LocationPoint P)const
{
  LocationPoint Q = this->getProjection(P);
  return Q.isValid() ? GetDist(P, Q) : NAN;
}

LocationPoint RouteGraph::getProjection(LocationPoint P)const
{
  LocationPoint P0;
  double d0 = NAN;
  Edge e0;

  // Searching for the best edge point
  for(EdgeIterator eIter = mEdges.lower_bound(Edge{.level=P.level, .id=0, .src=0, .dst=0, .weight=0.0});
      eIter != mEdges.end() && eIter->level == P.level; ++eIter)
  {
    Edge e = *eIter;
    Vertex u = getVertex(e.src);
    Vertex v = getVertex(e.dst);

    LocationPoint P1 = P;
    double k = GetProjection(u.x, u.y, v.x, v.y, P.x, P.y, &P1.x, &P1.y);

    if (k < 0)
      P1 = u;
    else if (k > 1)
      P1 = v;

    double d = GetDist(P, P1);

    if (std::isnan(d0) || d < d0)
    {
      P0 = P1;
      d0 = d;
      e0 = e;
    }
  }

  return P0;
}

int RouteGraph::checkCloseVertice(std::vector<std::string>* errors)const
{
  int errNum = 0;
  for(auto v : mVertice)
  {
    for(VertexIterator vIter = mVertice.upper_bound(Vertex{LocationPoint{.level=v.level, .x=0, .y=0}, .id=v.id});
        vIter != mVertice.end() && vIter->level == v.level; ++vIter)
    {
      if (v.id == vIter->id)
        continue;

      if (GetDist(v.x, v.y, vIter->x, vIter->y) < STICK_DISTANCE)
      {
        if (++errNum && errors)
        {
          char cs[1024] = { 0 };
          sprintf(cs, "<type=\"close_vertice\" level=\"%d\" "
                      "id1=\"%d\" x1=\"%.2f\" y1=\"%.2f\" "
                      "id2=\"%d\" x2=\"%.2f\" y2=\"%.2f\"/>",
                  v.level, v.id, v.x, v.y,
                  vIter->id, vIter->x, vIter->y);
          errors->push_back(std::string(cs));
        }
      }
    }
  }
  return errNum;
}

int RouteGraph::checkSplitEdges(std::vector<std::string>* errors)const
{
  int errNum = 0;
  for(auto v : mVertice)
  {
    for(EdgeIterator eIter = mEdges.upper_bound(Edge{.level=v.level, .id=0, .src=0, .dst=0, .weight=0.0});
        eIter != mEdges.end() && eIter->level == v.level; ++eIter)
    {
      Vertex A = this->getVertex(eIter->src);
      Vertex B = this->getVertex(eIter->dst);

      // Check if v is close to A or B
      if (v.id == A.id || v.id == B.id ||
          GetDist(v.x, v.y, A.x, A.y) < STICK_DISTANCE ||
          GetDist(v.x, v.y, B.x, B.y) < STICK_DISTANCE)
        continue;

      double px = 0.0;
      double py = 0.0;
      double k  = GetProjection(A.x, A.y, B.x, B.y, v.x, v.y, &px, &py);

      if (GetDist(v.x, v.y, px, py) < STICK_DISTANCE && 0 < k && k < 1)
      {
        if (++errNum && errors)
        {
          char cs[1024] = { 0 };
          sprintf(cs, "<type=\"split_edge\" level=\"%d\" "
                      "id1=\"%d\" x1=\"%.2f\" y1=\"%.2f\" "
                      "id2=\"%d\" x2=\"%.2f\" y2=\"%.2f\" "
                      "id3=\"%d\" x3=\"%.2f\" y3=\"%.2f\"/>",
                  v.level, v.id, v.x, v.y,
                  A.id, A.x, A.y, B.id, B.x, B.y);
          errors->push_back(std::string(cs));
        }
      }
    }
  }
  return errNum;
}

int RouteGraph::checkCrossEdges(std::vector<std::string>* errors)const
{
  int errNum = 0;
  for(auto e : mEdges)
  {
    Vertex A = this->getVertex(e.src);
    Vertex B = this->getVertex(e.dst);
    if (e.level == INT_MAX)
      continue;

    for(EdgeIterator eIter = mEdges.upper_bound(Edge{.level=e.level, .id=e.id, .src=0, .dst=0, .weight=0.0});
        eIter != mEdges.end() && eIter->level == e.level; ++eIter)
    {
      Vertex C = this->getVertex(eIter->src);
      Vertex D = this->getVertex(eIter->dst);
      if (CheckIntersection(A.x, A.y, B.x, B.y, C.x, C.y, D.x, D.y))
      {
        if (++errNum && errors)
        {
          char cs[1024] = { 0 };
          sprintf(cs, "<type=\"cross_edges\" level=\"%d\" "
                      "id1=\"%d\" x1=\"%.2f\" y1=\"%.2f\" "
                      "id2=\"%d\" x2=\"%.2f\" y2=\"%.2f\" "
                      "id3=\"%d\" x3=\"%.2f\" y3=\"%.2f\" "
                      "id4=\"%d\" x4=\"%.2f\" y4=\"%.2f\"/>",
                  A.level,
                  A.id, A.x, A.y, B.id, B.x, B.y,
                  C.id, C.x, C.y, D.id, D.x, D.y);
          errors->push_back(std::string(cs));
        }
      }
    }
  }
  return errNum;
}

// Check connectivity inside the levels
int RouteGraph::checkLevelConnectivity(std::vector<std::string>* errors)const
{
  int errNum = 0;
  for(auto level : mLevels)
  {
    Graph G;

    int vCount = 0;

    for(VertexIterator vIter = mVertice.upper_bound(Vertex{LocationPoint{.level=level.id, .x=0, .y=0}, .id=0});
        vIter != mVertice.end() && vIter->level == level.id; ++vIter)
    {
      G.addVertex(vIter->id);
      ++vCount;
    }

    int eCount = 0;
    for(EdgeIterator eIter = mEdges.upper_bound(Edge{.level=level.id, .id=0, .src=0, .dst=0, .weight=0.0});
        eIter != mEdges.end() && eIter->level == level.id; ++eIter)
    {
      G.addEdge(eIter->src, eIter->dst, 1);
      ++eCount;
    }

    if (!vCount)
    {
      if (++errNum && errors)
      {
        char cs[1024] = { 0 };
        sprintf(cs, "<type=\"level_no_vertice\" level=\"%d\"/>", level.id);
        errors->push_back(std::string(cs));
      }
      continue;
    }

    if (!eCount)
    {
      if (++errNum && errors)
      {
        char cs[1024] = { 0 };
        sprintf(cs, "<type=\"level_no_edges\" level=\"%d\"/>", level.id);
        errors->push_back(std::string(cs));
      }
      continue;
    }

    // Check level connectivity
    std::vector<std::set<int> > sccList = G.getSCC();
    if (sccList.size() > 1)
    {
      for(int i = 1; i < (int)sccList.size(); ++i)
        for(int j = i - 1; j < i; ++j)
        {
          if (sccList[i].empty() || sccList[j].empty())
            continue;

          // Searching the closest unreachable vertex std::pair
          Vertex v1, v2;
          double d0 = -1.0;

          for(int i1 : sccList[i])
            for(int i2 : sccList[j])
            {
              Vertex u1 = getVertex(i1);
              Vertex u2 = getVertex(i2);
              double d = GetDist(u1.x, u1.y, u2.x, u2.y);
              if (d0 < 0 || d < d0)
              {
                d0 = d;
                v1 = u1;
                v2 = u2;
              }
            }

          if (!v1.isValid() || !v2.isValid())
            continue;

          if (++errNum && errors)
          {
            char cs[1024] = { 0 };
            sprintf(cs, "<type=\"level_not_connected\" level=\"%d\" "
                        "id1=\"%d\" x1=\"%.2f\" y1=\"%.2f\" id2=\"%d\" x2=\"%.2f\" y2=\"%.2f\"/>",
                    level.id, v1.id, v1.x, v1.y, v2.id, v2.x, v2.y);
            errors->push_back(std::string(cs));
          }
        }
    }
  }
  return errNum;
}

int RouteGraph::checkElevationConnectivity(std::vector<std::string>* errors)const
{
  if (mLevels.size() < 2)
    return 0;

  int errNum = 0;

  Graph G;
  for(auto level : mLevels)
    G.addVertex(level.id);

  int eCount = 0;
  for(EdgeIterator eIter = mEdges.upper_bound(Edge{.level=INT_MAX, .id=0, .src=0, .dst=0, .weight=0.0});
      eIter != mEdges.end() && eIter->level == INT_MAX; ++eIter)
  {
    Vertex u = getVertex(eIter->src);
    Vertex v = getVertex(eIter->dst);
    G.addEdge(u.level, v.level, 1);
    ++eCount;
  }

  std::vector<std::set<int> > sccList = G.getSCC();
  if (sccList.size() > 1)
  {
    for(int i = 1; i < (int)sccList.size(); ++i)
      for(int j = i - 1; j < i; ++j)
      {
        if (sccList[i].empty() || sccList[j].empty())
          continue;

        for(int id1 : sccList[i])
          for(int id2 : sccList[j])
          {
            if (++errNum && errors)
            {
              char cs[1024] = { 0 };
              sprintf(cs, "<type=\"levels_not_connected\" level1=\"%d\" level2=\"%d\"/>", id1, id2);
              errors->push_back(std::string(cs));
            }
          }
      }
  }

  return errNum;
}

int RouteGraph::checkFullConnectivity(std::vector<std::string>* errors)const
{
  int errNum = 0;

  if (mLevels.size() > 1)
  {
    for(auto level : mLevels)
    {
      Graph G;

      int vCount = 0, extCount = 0;
      for(VertexIterator vIter = mVertice.upper_bound(Vertex{LocationPoint{.level=level.id, .x=0, .y=0}, .id=0});
          vIter != mVertice.end() && vIter->level == level.id; ++vIter)
      {
        ++vCount;
        if (vIter->isElevation)
          ++extCount;
      }

      if (vCount && !extCount)
      {
        if (++errNum && errors)
        {
          char cs[1024] = { 0 };
          sprintf(cs, "<type=\"level_no_elevations\" level=\"%d\"/>", level.id);
          errors->push_back(std::string(cs));
        }
      }
    }
  }

  if (errNum > 0)
    return errNum;

  // Check the graph connectivity
  Graph G;
  for(auto v : mVertice)
    G.addVertex(v.id);

  for(auto e : mEdges)
    G.addEdge(e.src, e.dst, 1);

  std::vector<std::set<int> > sccList = G.getSCC();
  if (sccList.size() > 1)
  {
    for(int i = 1; i < (int)sccList.size() - 1; ++i)
      for(int j = i - 1; j < i; ++j)
      {
        if (sccList[i].empty() || sccList[j].empty())
          continue;

        Vertex v1 = getVertex(*sccList[i].begin());
        Vertex v2 = getVertex(*sccList[j].begin());
        for(int i1 : sccList[i])
        {
          Vertex u1 = getVertex(i1);
          if (u1.isElevation)
            v1 = u1;
        }

        for(int i2 : sccList[j])
        {
          Vertex u2 = getVertex(i2);
          if (u2.isElevation)
            v2 = u2;
        }

        if (++errNum && errors)
        {
          char cs[1024] = { 0 };
          sprintf(cs, "<type=\"graph_not_connected\" "
                      "level1=\"%d\" id1=\"%d\" x1=\"%.2f\" y1=\"%.2f\" "
                      "level2=\"%d\" id2=\"%d\" x2=\"%.2f\" y2=\"%.2f\"/>",
                  v1.level, v1.id, v1.x, v1.y,
                  v2.level, v2.id, v2.x, v2.y);
          errors->push_back(std::string(cs));
        }
      }
  }

  return errNum;
}

int RouteGraph::checkFull(std::vector<std::string>* errors)const
{
  // Check if graph is empty (has no vertice)
  if (mVertice.empty())
  {
    if (errors)
    {
      char cs[1024] = { 0 };
      sprintf(cs, "<type=\"graph_empty\"/>");
      errors->push_back(std::string(cs));
    }
    return 1;
  }

  int n1 = this->checkCloseVertice(errors);
  int n2 = this->checkSplitEdges(errors);
  int n3 = this->checkCrossEdges(errors);
  int n4 = this->checkLevelConnectivity(errors);
  int n5 = this->checkElevationConnectivity(errors);
  if (n5 == 0)
    n5 = this->checkFullConnectivity(errors);
  return n1 + n2 + n3 + n4 + n5;
}

std::vector<PolyLine> SplitPolyLine(const PolyLine& points, size_t nparts)
{
  if (points.size() < 2)
    return {points};

  std::vector<PolyLine> parts;

  double totalLen = 0.0;
  for(size_t i = 0; i + 1 < points.size(); ++i)
    totalLen += GetDist(points[i], points[i+1]);

  const double splitLen = totalLen / nparts;

  double curLen = 0.0;
  PolyLine curPoly;

  curPoly.push_back(points[0]);

  for(size_t i = 1; i < points.size(); )
  {
    const double len = GetDist(curPoly.back(), points[i]);

    if (curLen + len < splitLen)
    {
      if (fabs(len) > EPSILON)
      {
        // Avoid duplicate points
        curLen += len;
        curPoly.push_back(points[i]);
      }
      ++i;
      continue;
    }

    const double k = (splitLen - curLen) / len;

    const auto splitPoint = LocationPoint{.level=points[i].level,
                                          .x=curPoly.back().x * (1 - k) + points[i].x * k,
                                          .y=curPoly.back().y * (1 - k) + points[i].y * k};
    curPoly.push_back(splitPoint);

    parts.push_back(curPoly);
    if (parts.size() == nparts)
      break;

    curLen = 0.0;
    curPoly = PolyLine();
    curPoly.push_back(splitPoint);
  }

  if (parts.size() < nparts)
    parts.push_back(curPoly);

  return parts;
}

} } // namespace navigine::indoor_routing
