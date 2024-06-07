#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <string>
#include <sstream>
using std::stringstream;
using std::string;
using Catch::Matchers::WithinRel;
using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::Line2D;
using turtlelib::deg2rad;
using turtlelib::PI;
using turtlelib::INF;
using turtlelib::normalize_angle;
using turtlelib::get_mean;
using turtlelib::dot;
using turtlelib::angle_between;
using turtlelib::integrate_twist;
using turtlelib::DiffDrive;
using turtlelib::DiffDriveConfig;
using turtlelib::Wheel;

constexpr double FLOAT_TOL = 1e-8;

TEST_CASE("angle normalization", "[normalize_angle]") { //Nick Morales
    REQUIRE_THAT(normalize_angle(PI), WithinRel(PI, FLOAT_TOL));
    REQUIRE_THAT(normalize_angle(-PI), WithinRel(PI, FLOAT_TOL));
    REQUIRE_THAT(normalize_angle(0.0), WithinRel(0.0, FLOAT_TOL));
    REQUIRE_THAT(normalize_angle(-PI/4.0), WithinRel(-PI/4.0, FLOAT_TOL));
    REQUIRE_THAT(normalize_angle(3.0*PI/2.0), WithinRel(-PI/2.0, FLOAT_TOL));
    REQUIRE_THAT(normalize_angle(-5.0*PI/2.0), WithinRel(-PI/2.0, FLOAT_TOL));
}

TEST_CASE("mean calculation", "[mean]") { //Nick Morales
    
    SECTION("test 1") {
        std::vector<double> v {5, 3, 8, 7, 9};
        double correct = 6.4;

        auto result = get_mean(v);

        REQUIRE_THAT(result, WithinRel(correct, FLOAT_TOL));
    }

    SECTION("test 1") {
        std::vector<double> v {5.3, 6.4, 7.2, 1.3};
        double correct = 5.05;

        auto result = get_mean(v);

        REQUIRE_THAT(result, WithinRel(correct, FLOAT_TOL));
    }
}

TEST_CASE("vector magnitude", "[vector]") { //Nick Morales
    Vector2D V1 {3.0, 4.0};
    Vector2D V2 {15.0, 112.0};

    REQUIRE_THAT(V1.magnitude(), WithinRel(5.0, FLOAT_TOL));
    REQUIRE_THAT(V2.magnitude(), WithinRel(113.0, FLOAT_TOL));
}

TEST_CASE("vector addition", "[vector]") { //Nick Morales
    double x1 = 0.44, y1 = 2.14;
    Vector2D V1 {x1,y1};
    double x2 = -1.37, y2 = 0.53;
    Vector2D V2 {x2,y2};


    SECTION("in-place modification") {
        V1 += V2;

        //V1 modified
        REQUIRE_THAT(V1.x, WithinRel(x1 + x2, FLOAT_TOL));
        REQUIRE_THAT(V1.y, WithinRel(y1 + y2, FLOAT_TOL));

        //V2 not modified
        REQUIRE_THAT(V2.x, WithinRel(x2, FLOAT_TOL));
        REQUIRE_THAT(V2.y, WithinRel(y2, FLOAT_TOL));

    }

    SECTION("return to separate variable") {
        Vector2D Vres = V1 + V2;

        //V1 not modified
        REQUIRE_THAT(V1.x, WithinRel(x1, FLOAT_TOL));
        REQUIRE_THAT(V1.y, WithinRel(y1, FLOAT_TOL));

        //V2 not modified
        REQUIRE_THAT(V2.x, WithinRel(x2, FLOAT_TOL));
        REQUIRE_THAT(V2.y, WithinRel(y2, FLOAT_TOL));

        //Result
        REQUIRE_THAT(Vres.x, WithinRel(x1 + x2, FLOAT_TOL));
        REQUIRE_THAT(Vres.y, WithinRel(y1 + y2, FLOAT_TOL));
    }
}

TEST_CASE("vector subtraction", "[vector]") { //Nick Morales
    double x1 = 3.94, y1 = -0.70;
    Vector2D V1 {x1,y1};
    double x2 = -4.90, y2 = 2.34;
    Vector2D V2 {x2,y2};


    SECTION("in-place modification") {
        V1 -= V2;

        //V1 modified
        REQUIRE_THAT(V1.x, WithinRel(x1 - x2, FLOAT_TOL));
        REQUIRE_THAT(V1.y, WithinRel(y1 - y2, FLOAT_TOL));

        //V2 not modified
        REQUIRE_THAT(V2.x, WithinRel(x2, FLOAT_TOL));
        REQUIRE_THAT(V2.y, WithinRel(y2, FLOAT_TOL));

    }

    SECTION("return to separate variable") {
        Vector2D Vres = V1 - V2;

        //V1 not modified
        REQUIRE_THAT(V1.x, WithinRel(x1, FLOAT_TOL));
        REQUIRE_THAT(V1.y, WithinRel(y1, FLOAT_TOL));

        //V2 not modified
        REQUIRE_THAT(V2.x, WithinRel(x2, FLOAT_TOL));
        REQUIRE_THAT(V2.y, WithinRel(y2, FLOAT_TOL));

        //Result
        REQUIRE_THAT(Vres.x, WithinRel(x1 - x2, FLOAT_TOL));
        REQUIRE_THAT(Vres.y, WithinRel(y1 - y2, FLOAT_TOL));
    }
}

TEST_CASE("scaling vectors", "[vector]") { //Nick Morales
    double x1 = -0.03, y1 = 4.19;
    Vector2D V1 {x1,y1};
    double scalar = -3.29;


    SECTION("in-place modification") {
        V1 *= scalar;

        //V1 modified
        REQUIRE_THAT(V1.x, WithinRel(x1*scalar, FLOAT_TOL));
        REQUIRE_THAT(V1.y, WithinRel(y1*scalar, FLOAT_TOL));
    }

    SECTION("return to separate variable") {
        Vector2D VresR = V1*scalar;
        Vector2D VresL = scalar*V1;

        //V1 not modified
        REQUIRE_THAT(V1.x, WithinRel(x1, FLOAT_TOL));
        REQUIRE_THAT(V1.y, WithinRel(y1, FLOAT_TOL));

        //Result from right multiplication
        REQUIRE_THAT(VresR.x, WithinRel(x1*scalar, FLOAT_TOL));
        REQUIRE_THAT(VresR.y, WithinRel(y1*scalar, FLOAT_TOL));

        //Result from left multiplication
        REQUIRE_THAT(VresL.x, WithinRel(x1*scalar, FLOAT_TOL));
        REQUIRE_THAT(VresL.y, WithinRel(y1*scalar, FLOAT_TOL));
    }
}

TEST_CASE("vector equivalence", "[vector]") { //Nick Morales
    Vector2D V1 {-3.30, -4.65};
    Vector2D V2 {-3.30, -4.65};
    Vector2D V3 {-1.26, -4.65};
    Vector2D V4 {-3.30, -4.45};
    Vector2D V5 {-1.26, 2.70};

    REQUIRE(almost_equal(V1, V2));
    REQUIRE(!almost_equal(V1, V3));
    REQUIRE(!almost_equal(V1, V4));
    REQUIRE(!almost_equal(V1, V5));
}

TEST_CASE("vector dot product", "[vector]") { //Nick Morales
    double x1 = 4.04, y1 = 4.68;
    Vector2D V1 {x1,y1};
    double x2 = 1.42, y2 = -2.41;
    Vector2D V2 {x2,y2};


    double dot_product = dot(V1,V2);

    //V1 not modified
    REQUIRE_THAT(V1.x, WithinRel(x1, FLOAT_TOL));
    REQUIRE_THAT(V1.y, WithinRel(y1, FLOAT_TOL));

    //V2 not modified
    REQUIRE_THAT(V2.x, WithinRel(x2, FLOAT_TOL));
    REQUIRE_THAT(V2.y, WithinRel(y2, FLOAT_TOL));

    //Result
    REQUIRE_THAT(dot_product, WithinRel(x1*x2 + y1*y2, FLOAT_TOL));
}

TEST_CASE("vector angle", "[vector]") { //Nick Morales
    Vector2D Vright {1,0};
    Vector2D Vleft {-1,0};
    Vector2D Vup {0,1};
    Vector2D Vdown {0,-1};
    Vector2D Vupdiag {1,1};
    Vector2D Vdowndiag {1,-1};

    REQUIRE_THAT(angle_between(Vright, Vup), WithinRel(PI/2, FLOAT_TOL));
    REQUIRE_THAT(angle_between(Vright, Vdown), WithinRel(-PI/2, FLOAT_TOL));
    REQUIRE_THAT(angle_between(Vup, Vright), WithinRel(-PI/2, FLOAT_TOL));
    REQUIRE_THAT(angle_between(Vleft, Vdown), WithinRel(PI/2, FLOAT_TOL));
    REQUIRE_THAT(angle_between(Vdown, Vleft), WithinRel(-PI/2, FLOAT_TOL));
    REQUIRE_THAT(angle_between(Vupdiag, Vright), WithinRel(-PI/4, FLOAT_TOL));
    REQUIRE_THAT(angle_between(Vdowndiag, Vleft), WithinRel(-3*PI/4, FLOAT_TOL));
}

TEST_CASE("constructors and getters", "[transform]") { //Nick Morales
    SECTION("default constructor") {
        Transform2D T;

        REQUIRE_THAT(T.translation().x, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(0.0, FLOAT_TOL));
    }
    SECTION("pure translation constructor") {
        double x = 7.2, y = -5.6;
        Transform2D T {Vector2D{x,y}};

        REQUIRE_THAT(T.translation().x, WithinRel(x, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(y, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(0.0, FLOAT_TOL));
    }
    SECTION("pure rotation constructor") {
        double rot = 1.23;
        Transform2D T {rot};

        REQUIRE_THAT(T.translation().x, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(rot, FLOAT_TOL));
    }
    SECTION("rotation and translation constructor") {
        double x = -5.7, y = 7.2, rot = -0.58;
        Transform2D T{Vector2D{x,y}, rot};

        REQUIRE_THAT(T.translation().x, WithinRel(x, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(y, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(rot, FLOAT_TOL));
    }
}

TEST_CASE("transform a vector", "[transform]") { //Nick Morales
    double x = 3.4, y = 2.2, rot = -0.86;
    Transform2D Tab{Vector2D{x,y}, rot};
    Vector2D vb {-1.1, 2.9};

    Vector2D va = Tab(vb);

    //Vector is transformed
    REQUIRE_THAT(va.x, WithinRel(4.88006221741585, FLOAT_TOL));
    REQUIRE_THAT(va.y, WithinRel(4.92569547686056, FLOAT_TOL));

    //Transform is not modified
    REQUIRE_THAT(Tab.translation().x, WithinRel(x, FLOAT_TOL));
    REQUIRE_THAT(Tab.translation().y, WithinRel(y, FLOAT_TOL));
    REQUIRE_THAT(Tab.rotation(), WithinRel(rot, FLOAT_TOL));
}

TEST_CASE("transform a twist", "[transform]") { //Nick Morales
    double x = -9.1, y = 6.4, rot = 1.57;
    Transform2D Tab{Vector2D{x,y}, rot};
    Twist2D Vb {-1.2,3.3, 2.0};

    Twist2D Va = Tab(Vb);

    //Vector is transformed
    REQUIRE_THAT(Va.w, WithinRel(-1.2, FLOAT_TOL));
    REQUIRE_THAT(Va.x, WithinRel(-9.67737148771825, FLOAT_TOL));
    REQUIRE_THAT(Va.y, WithinRel(-7.61840839290348, FLOAT_TOL));

    //Transform is not modified
    REQUIRE_THAT(Tab.translation().x, WithinRel(x, FLOAT_TOL));
    REQUIRE_THAT(Tab.translation().y, WithinRel(y, FLOAT_TOL));
    REQUIRE_THAT(Tab.rotation(), WithinRel(rot, FLOAT_TOL));
}

TEST_CASE("inverse", "[transform]") { //Nick Morales
    double x = 0.22, y = -0.22, rot = 2.44;
    Transform2D T{Vector2D{x,y}, rot};

    Transform2D Tinv = T.inv();

    //Vector is transformed
    REQUIRE_THAT(Tinv.translation().x, WithinRel(0.310035044090672, FLOAT_TOL));
    REQUIRE_THAT(Tinv.translation().y, WithinRel(-0.0260436448235487, FLOAT_TOL));
    REQUIRE_THAT(Tinv.rotation(), WithinRel(-2.44, FLOAT_TOL));

    //Transform is not modified
    REQUIRE_THAT(T.translation().x, WithinRel(x, FLOAT_TOL));
    REQUIRE_THAT(T.translation().y, WithinRel(y, FLOAT_TOL));
    REQUIRE_THAT(T.rotation(), WithinRel(rot, FLOAT_TOL));
}

TEST_CASE("transform composition", "[transform]") { //Nick Morales
    double x1 = -3.72, y1 = -4.67, rot1 = 2.85;
    Transform2D T1{Vector2D{x1,y1}, rot1};
    double x2 = 4.93, y2 = 1.28, rot2 = 2.97;
    Transform2D T2{Vector2D{x2,y2}, rot2};
    double xres = -8.80986293693519, yres = -4.47870106321921, rotres = -0.4631853071795862;


    SECTION("in-place modification") {
        T1*=T2;

        //T1 modified
        REQUIRE_THAT(T1.translation().x, WithinRel(xres, FLOAT_TOL));
        REQUIRE_THAT(T1.translation().y, WithinRel(yres, FLOAT_TOL));
        REQUIRE_THAT(T1.rotation(), WithinRel(rotres, FLOAT_TOL));

        //T2 not modified
        REQUIRE_THAT(T2.translation().x, WithinRel(x2, FLOAT_TOL));
        REQUIRE_THAT(T2.translation().y, WithinRel(y2, FLOAT_TOL));
        REQUIRE_THAT(T2.rotation(), WithinRel(rot2, FLOAT_TOL));

    }

    SECTION("return to separate variable") {
        Transform2D Tres = T1*T2;

        //T1 not modified
        REQUIRE_THAT(T1.translation().x, WithinRel(x1, FLOAT_TOL));
        REQUIRE_THAT(T1.translation().y, WithinRel(y1, FLOAT_TOL));
        REQUIRE_THAT(T1.rotation(), WithinRel(rot1, FLOAT_TOL));

        //T2 not modified
        REQUIRE_THAT(T2.translation().x, WithinRel(x2, FLOAT_TOL));
        REQUIRE_THAT(T2.translation().y, WithinRel(y2, FLOAT_TOL));
        REQUIRE_THAT(T2.rotation(), WithinRel(rot2, FLOAT_TOL));

        //Result
        REQUIRE_THAT(Tres.translation().x, WithinRel(xres, FLOAT_TOL));
        REQUIRE_THAT(Tres.translation().y, WithinRel(yres, FLOAT_TOL));
        REQUIRE_THAT(Tres.rotation(), WithinRel(rotres, FLOAT_TOL));
    }
}

TEST_CASE("output stream", "[transform]") { //Nick Morales
    stringstream ss;
    
    SECTION("input 1") {
        Transform2D T{Vector2D{-3.38, -2.22}, deg2rad(117)};

        ss << T;

        REQUIRE (ss.str() == "deg: 117 x: -3.38 y: -2.22");
    }

    SECTION("input 2") {
        Transform2D T{Vector2D{2.84, -2.97}, deg2rad(117)};

        ss << T;

        REQUIRE (ss.str() == "deg: 117 x: 2.84 y: -2.97");
    }
}

TEST_CASE("input stream", "[transform]") { //Nick Morales
    stringstream ss;
    string line;
    Transform2D T;

    SECTION("with labels") {
        line = "deg: -24 x: -3.24 y: 3.83";
        ss << line;
        ss >> T;

        REQUIRE_THAT(T.translation().x, WithinRel(-3.24, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(3.83, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(deg2rad(-24), FLOAT_TOL));
    }

    SECTION("without labels") {
        line = "142 1.73 -2.23";
        ss << line;
        ss >> T;

        REQUIRE_THAT(T.translation().x, WithinRel(1.73, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(-2.23, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(deg2rad(142), FLOAT_TOL));
    }
}

TEST_CASE("transform equivalence", "[transform]") { //Nick Morales
    Transform2D T1 {{-4.81, 2.36}, -2.83};
    Transform2D T2 {{-4.81, 2.36}, -2.83};
    Transform2D T3 {{2.47, 2.36}, -2.83};
    Transform2D T4 {{-4.81, -1.03}, -2.83};
    Transform2D T5 {{-4.81, 2.36}, 1.09};
    Transform2D T6 {{0.00, -2.43}, 4.05};

    REQUIRE(almost_equal(T1, T2));
    REQUIRE(!almost_equal(T1, T3));
    REQUIRE(!almost_equal(T1, T4));
    REQUIRE(!almost_equal(T1, T5));
    REQUIRE(!almost_equal(T1, T6));
}

TEST_CASE("integrate a twist", "[transform]") { //Nick Morales
    Twist2D twist;
    Transform2D Tbbp;

    SECTION("pure translation") {
        twist.x = -0.52;
        twist.y = -0.05;

        Tbbp = integrate_twist(twist);

        REQUIRE_THAT(Tbbp.translation().x, WithinRel(twist.x, FLOAT_TOL));
        REQUIRE_THAT(Tbbp.translation().y, WithinRel(twist.y, FLOAT_TOL));
        REQUIRE_THAT(Tbbp.rotation(), WithinRel(0.0, FLOAT_TOL));
    }

    SECTION("pure rotation") {
        //abs(twist.w) < PI
        twist.w = 2.57;

        Tbbp = integrate_twist(twist);

        REQUIRE_THAT(Tbbp.translation().x, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(Tbbp.translation().y, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(Tbbp.rotation(), WithinRel(twist.w, FLOAT_TOL));

        //abs(twist.w) > PI
        twist.w = 3.66;

        Tbbp = integrate_twist(twist);

        REQUIRE_THAT(Tbbp.translation().x, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(Tbbp.translation().y, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(Tbbp.rotation(), WithinRel(twist.w - 2*PI, FLOAT_TOL));
    }

    SECTION("translation and rotation") {
        //abs(twist.w) < PI
        twist.w = -1.24;
        twist.x = -2.15;
        twist.y = -2.92;

        Tbbp = integrate_twist(twist);

        REQUIRE_THAT(Tbbp.translation().x, WithinRel(-3.229863264722, FLOAT_TOL));
        REQUIRE_THAT(Tbbp.translation().y, WithinRel(-1.05645265317421, FLOAT_TOL));
        REQUIRE_THAT(Tbbp.rotation(), WithinRel(twist.w, FLOAT_TOL));
        
        //abs(twist.w) > PI
        twist.w = -4.84;
        twist.x = 2.54;
        twist.y = 1.08;

        Tbbp = integrate_twist(twist);

        REQUIRE_THAT(Tbbp.translation().x, WithinRel(-0.325783634885417, FLOAT_TOL));
        REQUIRE_THAT(Tbbp.translation().y, WithinRel(-0.679331668841286, FLOAT_TOL));
        REQUIRE_THAT(Tbbp.rotation(), WithinRel(twist.w + 2*PI, FLOAT_TOL));
    }
}

TEST_CASE("driving forward", "[diffdrive]") { //Nick Morales

    DiffDrive robot {0.16, 0.033};
    Twist2D twist {0,0.165,0};
    DiffDriveConfig config {
        Transform2D {Vector2D{0.165,0},0},
        Wheel {5.0, 5.0}
    };
    

    SECTION("forward kinematics") {
        
        //Update configuration using new wheel position
        robot.update_config(config.wheel_pos);

        //Check that configuration matches expected configuration
        REQUIRE_THAT(robot.config().location.translation().x, 
            WithinRel(config.location.translation().x, FLOAT_TOL));
        REQUIRE_THAT(robot.config().location.translation().y, 
            WithinRel(config.location.translation().y, FLOAT_TOL));
        REQUIRE_THAT(robot.config().location.rotation(), 
            WithinRel(config.location.rotation(), FLOAT_TOL));
        REQUIRE_THAT(robot.config().wheel_pos.left, 
            WithinRel(config.wheel_pos.left, FLOAT_TOL));
        REQUIRE_THAT(robot.config().wheel_pos.right, 
            WithinRel(config.wheel_pos.right, FLOAT_TOL));
    }

    SECTION("inverse kinematics") {
        
        //Calculate wheel velocities to get to location in one unit of time
        Wheel wheel_vel = robot.get_required_wheel_vel(twist);

        //Check that wheel velocities match the new wheel posiitons
        //(since we're looking at one time unit)
        REQUIRE_THAT(config.wheel_pos.left, 
            WithinRel(wheel_vel.left, FLOAT_TOL));
        REQUIRE_THAT(config.wheel_pos.right, 
            WithinRel(wheel_vel.right, FLOAT_TOL));
    }

}

TEST_CASE("pure rotation", "[diffdrive]") { //Nick Morales

    DiffDrive robot {0.16, 0.033};
    Twist2D twist {0.515625,0,0};
    DiffDriveConfig config {
        Transform2D {Vector2D{0,0},0.515625},
        Wheel {-1.25, 1.25}
    };
    

    SECTION("forward kinematics") {
        
        //Update configuration using new wheel position
        robot.update_config(config.wheel_pos);

        //Check that configuration matches expected configuration
        REQUIRE_THAT(robot.config().location.translation().x, 
            WithinRel(config.location.translation().x, FLOAT_TOL));
        REQUIRE_THAT(robot.config().location.translation().y, 
            WithinRel(config.location.translation().y, FLOAT_TOL));
        REQUIRE_THAT(robot.config().location.rotation(), 
            WithinRel(config.location.rotation(), FLOAT_TOL));
        REQUIRE_THAT(robot.config().wheel_pos.left, 
            WithinRel(config.wheel_pos.left, FLOAT_TOL));
        REQUIRE_THAT(robot.config().wheel_pos.right, 
            WithinRel(config.wheel_pos.right, FLOAT_TOL));
    }

    SECTION("inverse kinematics") {
        
        //Calculate wheel velocities to get to location in one unit of time
        Wheel wheel_vel = robot.get_required_wheel_vel(twist);

        //Check that wheel velocities match the new wheel posiitons
        //(since we're looking at one time unit)
        REQUIRE_THAT(config.wheel_pos.left, 
            WithinRel(wheel_vel.left, FLOAT_TOL));
        REQUIRE_THAT(config.wheel_pos.right, 
            WithinRel(wheel_vel.right, FLOAT_TOL));
    }

}

TEST_CASE("arc of a circle", "[diffdrive]") { //Nick Morales

    DiffDrive robot {0.16, 0.033};
    Twist2D twist {0.309375,0.14025,0};
    DiffDriveConfig config {
        Transform2D {Vector2D{0.138023393683395,0.0215224326983102},0.309375},
        Wheel {3.5, 5.0}
    };
    

    SECTION("forward kinematics") {
        
        //Update configuration using new wheel position
        robot.update_config(config.wheel_pos);

        //Check that configuration matches expected configuration
        REQUIRE_THAT(robot.config().location.translation().x, 
            WithinRel(config.location.translation().x, FLOAT_TOL));
        REQUIRE_THAT(robot.config().location.translation().y, 
            WithinRel(config.location.translation().y, FLOAT_TOL));
        REQUIRE_THAT(robot.config().location.rotation(), 
            WithinRel(config.location.rotation(), FLOAT_TOL));
        REQUIRE_THAT(robot.config().wheel_pos.left, 
            WithinRel(config.wheel_pos.left, FLOAT_TOL));
        REQUIRE_THAT(robot.config().wheel_pos.right, 
            WithinRel(config.wheel_pos.right, FLOAT_TOL));
    }

    SECTION("inverse kinematics") {
        
        //Calculate wheel velocities to get to location in one unit of time
        Wheel wheel_vel = robot.get_required_wheel_vel(twist);

        //Check that wheel velocities match the new wheel posiitons
        //(since we're looking at one time unit)
        REQUIRE_THAT(config.wheel_pos.left, 
            WithinRel(wheel_vel.left, FLOAT_TOL));
        REQUIRE_THAT(config.wheel_pos.right, 
            WithinRel(wheel_vel.right, FLOAT_TOL));
    }

}

TEST_CASE("exceptions", "[diffdrive]") { //Nick Morales

    SECTION("invalid robot dimensions") {
        REQUIRE_THROWS(DiffDrive {0, 0});
        REQUIRE_THROWS(DiffDrive {0, 5});
        REQUIRE_THROWS(DiffDrive {7.2, 0});
        REQUIRE_THROWS(DiffDrive {-3.4, 2});
        REQUIRE_THROWS(DiffDrive {5.3, -1.2});
        REQUIRE_THROWS(DiffDrive {-5.6, -1.7});
        REQUIRE_NOTHROW(DiffDrive {8.2, 2.4});
    }

    SECTION("invalid twists") {
        DiffDrive robot {0.16, 0.033};
        
        REQUIRE_NOTHROW(robot.get_required_wheel_vel(Twist2D{0, 0, 0}));
        REQUIRE_NOTHROW(robot.get_required_wheel_vel(Twist2D{0.5, 0, 0}));
        REQUIRE_NOTHROW(robot.get_required_wheel_vel(Twist2D{0, 0.5, 0}));
        REQUIRE_THROWS(robot.get_required_wheel_vel(Twist2D{0, 0, 0.5}));
        REQUIRE_NOTHROW(robot.get_required_wheel_vel(Twist2D{-0.5, 0, 0}));
        REQUIRE_NOTHROW(robot.get_required_wheel_vel(Twist2D{0,-0.5, 0}));
        REQUIRE_THROWS(robot.get_required_wheel_vel(Twist2D{0, 0, -0.5}));
        REQUIRE_NOTHROW(robot.get_required_wheel_vel(Twist2D{0.5, 0.5, 0}));
        REQUIRE_NOTHROW(robot.get_required_wheel_vel(Twist2D{-0.5, -0.5, 0}));
        REQUIRE_THROWS(robot.get_required_wheel_vel(Twist2D{0.5, 0.5, 0.5}));
        REQUIRE_THROWS(robot.get_required_wheel_vel(Twist2D{-0.5, -0.5, -0.5}));
    }
}

TEST_CASE("constructors and getters", "[line]") { //Nick Morales
    SECTION("regular slope") {
        Vector2D start {3.0, 1.0};
        Vector2D end {4.0, 3.5};
        Line2D line {start, end};

        REQUIRE_THAT(line.start().x, WithinRel(start.x, FLOAT_TOL));
        REQUIRE_THAT(line.start().y, WithinRel(start.y, FLOAT_TOL));
        REQUIRE_THAT(line.end().x, WithinRel(end.x, FLOAT_TOL));
        REQUIRE_THAT(line.end().y, WithinRel(end.y, FLOAT_TOL));
        REQUIRE_THAT(line.slope(), WithinRel(2.5, FLOAT_TOL));
        REQUIRE_THAT(line.y_intercept(), WithinRel(-6.5, FLOAT_TOL));
    }

    SECTION("infinite slope") {
        Vector2D start {3.0, 1.0};
        Vector2D end {3.0, 3.5};
        Line2D line {start, end};

        REQUIRE_THAT(line.start().x, WithinRel(start.x, FLOAT_TOL));
        REQUIRE_THAT(line.start().y, WithinRel(start.y, FLOAT_TOL));
        REQUIRE_THAT(line.end().x, WithinRel(end.x, FLOAT_TOL));
        REQUIRE_THAT(line.end().y, WithinRel(end.y, FLOAT_TOL));
        REQUIRE_THAT(line.slope(), WithinRel(INF, FLOAT_TOL));
        REQUIRE_THAT(line.y_intercept(), WithinRel(INF, FLOAT_TOL));
    }

    SECTION("zero slope") {
        Vector2D start {3.0, 1.0};
        Vector2D end {5.0, 1.0};
        Line2D line {start, end};

        REQUIRE_THAT(line.start().x, WithinRel(start.x, FLOAT_TOL));
        REQUIRE_THAT(line.start().y, WithinRel(start.y, FLOAT_TOL));
        REQUIRE_THAT(line.end().x, WithinRel(end.x, FLOAT_TOL));
        REQUIRE_THAT(line.end().y, WithinRel(end.y, FLOAT_TOL));
        REQUIRE_THAT(line.slope(), WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(line.y_intercept(), WithinRel(start.y, FLOAT_TOL));
    }
}

TEST_CASE("intersections", "[line]") { //Nick Morales
    SECTION("intersecting non-parallel line segments") {
        Line2D line1 {{0.0, 1.0}, {4.0, 3.0}};
        Line2D line2 {{0.0, 3.0}, {2.0, 0.0}};

        auto intersection_points = find_intersection(line1, line2);
        
        REQUIRE(intersection_points.size() == 1);
        REQUIRE_THAT(intersection_points.at(0).x, WithinRel(1.0, FLOAT_TOL));
        REQUIRE_THAT(intersection_points.at(0).y, WithinRel(1.5, FLOAT_TOL));
    }

    SECTION("non-intersecting non-parallel line segments") {
        Line2D line1 {{0.0, 1.0}, {4.0, 3.0}};
        Line2D line2 {{4.0, 0.0}, {5.0, -3.0}};

        auto intersection_points = find_intersection(line1, line2);
        
        REQUIRE(intersection_points.size() == 0);
    }

    SECTION("intersecting parallel line segments") {
        Line2D line1 {{0.0, 1.0}, {4.0, 3.0}};
        Line2D line2 {{2.0, 2.0}, {6.0, 4.0}};

        auto intersection_points = find_intersection(line1, line2);
        
        REQUIRE(intersection_points.size() == 1);
        REQUIRE_THAT(intersection_points.at(0).x, WithinRel(2.0, FLOAT_TOL));
        REQUIRE_THAT(intersection_points.at(0).y, WithinRel(2.0, FLOAT_TOL));
    }

    SECTION("non-intersecting parallel line segments 1") {
        Line2D line1 {{0.0, 1.0}, {4.0, 3.0}};
        Line2D line2 {{6.0, 4.0}, {8.0, 5.0}};

        auto intersection_points = find_intersection(line1, line2);
        
        REQUIRE(intersection_points.size() == 0);
    }

    SECTION("non-intersecting parallel line segments 2") {
        Line2D line1 {{0.0, 1.0}, {4.0, 3.0}};
        Line2D line2 {{2.0, 0.0}, {6.0, 2.0}};

        auto intersection_points = find_intersection(line1, line2);
        
        REQUIRE(intersection_points.size() == 0);
    }
}