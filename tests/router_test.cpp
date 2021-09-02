#include <string>
#include <vector>
#include <boost/test/included/unit_test.hpp>
#include <navigine/indoor_routing/router.h>

namespace navigine {
namespace indoor_routing {

namespace tests {

struct BoostRouter
{
  BoostRouter()
  {
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
  }
  RouteGraph mRouteGrapgh;
};

BOOST_AUTO_TEST_SUITE(TestRouterAlgos)

BOOST_FIXTURE_TEST_CASE(TestGetPath, BoostRouter)
{
  double eps = 1;

  LocationPoint P{.level=22129, .x=17.14, .y=18.27};
  LocationPoint Q{.level=22129, .x=19.71, .y=6.78};

  RoutePath path = mRouteGrapgh.getPath(P, Q);

  BOOST_REQUIRE_EQUAL((int)path.points.size(), 6);
  BOOST_CHECK_CLOSE(path.length, 14.12, eps);

  BOOST_REQUIRE_EQUAL((int)path.events.size(), 4);
  BOOST_REQUIRE_EQUAL(path.events[0].value, 90);
  BOOST_CHECK_CLOSE(path.events[0].distance, 1.17, eps);
  BOOST_REQUIRE_EQUAL(path.events[1].value, 88);
  BOOST_CHECK_CLOSE(path.events[1].distance, 10.57, eps);
  BOOST_REQUIRE_EQUAL(path.events[2].value, 90);
  BOOST_CHECK_CLOSE(path.events[2].distance, 11.35, eps);
  BOOST_REQUIRE_EQUAL(path.events[3].value, 90);
  BOOST_CHECK_CLOSE(path.events[3].distance, 13.37, eps);
}

BOOST_FIXTURE_TEST_CASE(TestCheckFull, BoostRouter)
{
  std::vector<std::string> errors;
  int errNum = mRouteGrapgh.checkFull(&errors);
  BOOST_REQUIRE_EQUAL(errNum > 0, false);
}

BOOST_AUTO_TEST_SUITE_END()

} } } // navigine::indoor_routing::tests
