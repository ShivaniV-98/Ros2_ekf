#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/circle_detection.hpp"
using Catch::Matchers::WithinRel;
using turtlelib::Vector2D;
using turtlelib::Circle2D;
using turtlelib::fit_circle;

constexpr double FLOAT_TOL = 1e-4;

TEST_CASE("circle fitting", "[circle_detection]") { //Nick Morales

    SECTION("test 1") {
        std::vector<Vector2D> points = {{1,7},{2,6},{5,8},{7,7},{9,5},{3,7}};
        Circle2D correct = {{4.615482, 2.807354}, 4.8275};

        auto [result, error] = fit_circle(points);

        REQUIRE_THAT(result.center.x, WithinRel(correct.center.x, FLOAT_TOL));
        REQUIRE_THAT(result.center.y, WithinRel(correct.center.y, FLOAT_TOL));
        REQUIRE_THAT(result.radius, WithinRel(correct.radius, FLOAT_TOL));
    }

    SECTION("test 2") {
        std::vector<Vector2D> points = {{-1,0},{-0.3,-0.06},{0.3,0.1},{1,0}};
        Circle2D correct = {{0.4908357, -22.15212}, 22.17979};

        auto [result, error] = fit_circle(points);

        REQUIRE_THAT(result.center.x, WithinRel(correct.center.x, FLOAT_TOL));
        REQUIRE_THAT(result.center.y, WithinRel(correct.center.y, FLOAT_TOL));
        REQUIRE_THAT(result.radius, WithinRel(correct.radius, FLOAT_TOL));
    }
}
