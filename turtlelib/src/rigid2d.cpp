#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <numeric>

namespace turtlelib
{

    double normalize_angle(double rad) {
        //Bound rotation between -pi and pi
        while (rad > PI) {
            rad -= 2.0*PI;
        }
        while (rad <= -PI) {
            rad += 2.0*PI;
        }

        return rad;
    }

    //https://stackoverflow.com/questions/28574346/find-average-of-input-to-vector-c
    double get_mean(const std::vector<double> & values) {
        if(values.empty()) {
            return 0.0;
        }

        return std::reduce(values.begin(), values.end()) / static_cast<double>(values.size());
    }

    std::tuple<double, double> get_mean_and_std_dev(const std::vector<double> & values) {
        const auto mean = get_mean(values);

        if (values.size() < 2) {
            return {mean, 0.0};
        }

        double std_dev = 0.0;

        for (const auto value : values) {
            std_dev += std::pow((value - mean), 2);
        }
        
        std_dev = std::sqrt(std_dev / (values.size() - 1));

        return {mean, std_dev};

    }


    ////////// VECTOR2D START //////////

    double Vector2D::magnitude() const {
        return std::sqrt(std::pow(x,2) + std::pow(y,2));
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs) {
        lhs += rhs;
        return lhs;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs) {
        lhs -= rhs;
        return lhs;
    }

    Vector2D & Vector2D::operator*=(const double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vector2D operator*(Vector2D vector, const double scalar) {
        vector*=scalar;
        return vector;
    }

    Vector2D operator*(const double scalar, Vector2D vector) {
        vector*=scalar;
        return vector;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        os << '[' << v.x << ' ' << v.y << ']';
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v) {
        //remove leading whitespace
        is >> std::ws;

        auto c = is.peek();

        if (c == '[') {
            //Remove leading bracket
            c = is.get();
        }

        //Get actual data
        is >> v.x >> v.y;

        c = is.peek();

        if (c == ']') {
            //Remove trailing bracket
            c = is.get();
        }

        return is;
    }

    bool almost_equal(const Vector2D & v1, const Vector2D & v2, double epsilon) {
        return almost_equal(v1.x, v2.x, epsilon) && almost_equal(v1.y, v2.y, epsilon);
    }

    Vector2D normalize(const Vector2D & v) {
        const auto mag = v.magnitude();

        return {
            v.x / mag,
            v.y / mag
        };
    }

    double dot(const Vector2D & lhs, const Vector2D & rhs) {
        return lhs.x*rhs.x + lhs.y*rhs.y;
    }

    double angle_between(const Vector2D & start, const Vector2D & end) {
        //https://www.mathworks.com/matlabcentral/answers/180131-how-can-i-find-the-angle-between-two-vectors-including-directional-information
        return std::atan2(start.x*end.y - start.y*end.x, dot(start, end));
    }

    ////////// VECTOR2D END //////////



    ////////// TWIST2D START //////////

    std::ostream & operator<<(std::ostream & os, const Twist2D & V) {
        os << '[' << V.w << ' ' << V.x << ' ' << V.y << ']';
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & V) {
        //remove leading whitespace
        is >> std::ws;

        auto c = is.peek();

        if (c == '[') {
            //Remove leading bracket
            c = is.get();
        }

        //Get actual data
        is >> V.w >> V.x >> V.y;

        c = is.peek();

        if (c == ']') {
            //Remove trailing bracket
            c = is.get();
        }

        return is;
    }

    ////////// TWIST2D END //////////
    
    
    
    ////////// TRANSFORM2D START //////////

    /// Private variables are already initialized as an identity transformation
    Transform2D::Transform2D() {}

    Transform2D::Transform2D(Vector2D trans): Transform2D(trans, 0.0) {}

    Transform2D::Transform2D(double rot): Transform2D(Vector2D {0.0, 0.0}, rot) {}

    Transform2D::Transform2D(Vector2D trans, double rot)
    : trans_{trans}, rot_{normalize_angle(rot)}
    {
        cache_trig();
    }

    void Transform2D::cache_trig() {
        rot_sin_ = std::sin(rot_);
        rot_cos_ = std::cos(rot_);
    }

    Vector2D Transform2D::operator()(Vector2D v) const {
        return {
            v.x*rot_cos_ - v.y*rot_sin_ + trans_.x,
            v.x*rot_sin_ + v.y*rot_cos_ + trans_.y
        };
    }

    Twist2D Transform2D::operator()(Twist2D V) const {
        return {
            V.w,
            V.w*trans_.y + V.x*rot_cos_ - V.y*rot_sin_,
            -V.w*trans_.x + V.x*rot_sin_ + V.y*rot_cos_
        };
    }

    Transform2D Transform2D::inv() const {
        return {
            //translation
            {
                -trans_.x*rot_cos_ - trans_.y*rot_sin_,
                -trans_.y*rot_cos_ + trans_.x*rot_sin_
            },
            //rotation
            -rot_
        };
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
        
        //Output translation adds current translation to incoming translation
        //modified by current rotation
        trans_.x += rhs.translation().x*rot_cos_
                  - rhs.translation().y*rot_sin_;
        trans_.y += rhs.translation().x*rot_sin_
                  + rhs.translation().y*rot_cos_;
        
        //Output rotation just adds the angles together
        rot_ += rhs.rotation();

        //Bound rotation
        rot_ = normalize_angle(rot_);

        //Cache trig values
        cache_trig();

        return *this;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs) {
        lhs*=rhs;
        return lhs;
    }

    Vector2D Transform2D::translation() const { return trans_; }

    double Transform2D::rotation() const { return rot_; }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf) {
        os << "deg: " << rad2deg(tf.rotation())
           << " x: " << tf.translation().x
           << " y: " << tf.translation().y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf) {
        double deg, x, y;
        
        //remove leading whitespace
        is >> std::ws;

        auto c = is.peek();

        //remove any characters that aren't digits
        while (!(std::isdigit(c) || (c == '-') || (c == '.'))) {
            c = is.get();
            c = is.peek();
        }

        //Get rotation
        is >> deg;


        c = is.peek();

        //remove any characters that aren't digits
        while (!(std::isdigit(c) || (c == '-') || (c == '.'))) {
            c = is.get();
            c = is.peek();
        }

        //Get x translation
        is >> x;


        c = is.peek();

        //remove any characters that aren't digits
        while (!(std::isdigit(c) || (c == '-') || (c == '.'))) {
            c = is.get();
            c = is.peek();
        }

        //Get y translation
        is >> y;

        //Init transform
        tf = Transform2D{
            {x,y},
            deg2rad(deg)
        };

        return is;
    }

    bool almost_equal(const Transform2D & T1, const Transform2D & T2, double epsilon) {
        return almost_equal(T1.translation(), T2.translation(), epsilon) &&
               almost_equal(T1.rotation(), T2.rotation(), epsilon);
    }

    Transform2D integrate_twist(Twist2D twist) {
        if (almost_equal(twist.w, 0.0)) {
            return {
                {twist.x, twist.y},
                0.0
            };
        } else {
            //Frames:
            //b = body frame before motion
            //bp = body frame after motion
            //s = frame at center of rotation (COR) aligned with b frame
            //sp = frame at center of rotation (COR) aligned with bp frame
            
            //Purely rotational motion at COR to get from s to sp
            Transform2D Tssp {twist.w};

            //Use adjoint between known twists in s and b frames to solve for Tsb
            Transform2D Tsb {
                {
                    twist.y / twist.w,
                    -twist.x / twist.w
                },
                0.0 //no rotation between s and b frames, they are aligned
            };

            //Tbbp = TbsTsspTspbp
            //Tsb = Tspbp
            //so
            //Tbbp = Tsb^-1 * Tssp * Tsb
            return Tsb.inv()*Tssp*Tsb;
        }
    }

    ////////// TRANSFORM2D END //////////

    ////////// LINE2D START //////////

    Line2D::Line2D(Vector2D start, Vector2D end)
    : start_{start}, end_{end}, 
    min_{std::min(start_.x, end_.x), std::min(start_.y, end_.y)},
    max_{std::max(start_.x, end_.x), std::max(start_.y, end_.y)},
    vec_{end_ - start_}
    {
        if (almost_equal(start_.x, end_.x)) { //infinite slope
            slope_ = INF;
            if (almost_equal(start_.x, 0.0)) {
                y_intercept_ = 0.0;
            } else {
                y_intercept_ = INF;
            }
        } else {
            slope_ = (end_.y - start_.y) / (end_.x - start_.x);

            y_intercept_ = start_.y - slope_*start_.x;
        }
    }

    Vector2D Line2D::start() const {return start_;}

    Vector2D Line2D::end() const {return end_;}

    double Line2D::slope() const {return slope_;}

    double Line2D::y_intercept() const {return y_intercept_;}

    Vector2D Line2D::min() const {return min_;}

    Vector2D Line2D::max() const {return max_;}

    Vector2D Line2D::vec() const {return vec_;}

    double Line2D::calc_y(double x) const {
        if (slope_ == INF) {
            return 0.0; //undefined
        } else {
            return slope_*x + y_intercept_;
        }
    }

    std::vector<Vector2D> find_intersection(const Line2D & line1, const Line2D & line2) {
        std::vector<Vector2D> intersection_points;

        if (almost_equal(line1.slope(), line2.slope())) { //parallel lines
            if (line1.slope() == INF) {
                //parallel vertical lines, must have same x coordinate
                if (almost_equal(line1.start().x, line2.start().x)) {
                    if (line2.min().y >= line1.min().y && line2.min().y <= line1.max().y) {
                        //Min of line 2 is in line1
                        intersection_points.push_back(line2.min());
                    } else if (line2.max().y >= line1.min().y && line2.max().y <= line1.max().y) {
                        //Max of line 2 is in line 1
                        intersection_points.push_back(line1.min());
                    }
                }
            } else {
                //parallel nonvertical lines, must have same y intercept
                if (almost_equal(line1.y_intercept(), line2.y_intercept())) {
                    if (line2.min().x >= line1.min().x && line2.min().x <= line1.max().x) {
                        //Min of line 2 is in line1
                        intersection_points.push_back({line2.min().x, line2.calc_y(line2.min().x)});
                    } else if (line2.max().x >= line1.min().x && line2.max().x <= line1.max().x) {
                        //Max of line 2 is in line 1
                        intersection_points.push_back({line1.min().x, line1.calc_y(line1.min().x)});
                    }
                }
            }
        } else { //not parallel lines
            //Find x point of intersection
            double x;
            if (line1.slope() == INF) {
                x = line1.start().x;
            } else if (line2.slope() == INF) {
                x = line2.start().x;
            } else {
                x = (line1.y_intercept() - line2.y_intercept()) /
                    (line2.slope() - line1.slope());
            }

            //Intersection occurs if the x coordinate is within both segments
            if (x >= line1.min().x && x <= line1.max().x &&
                x >= line2.min().x && x <= line2.max().x)
            {
                //Lines intersect, calculate intersection point
                intersection_points.push_back({x, line1.calc_y(x)});
            }
        }

        return intersection_points;
    }

    std::vector<Vector2D> find_intersection(const Line2D & line, const Circle2D & circle) {
        //Derived from here
        //https://mathworld.wolfram.com/Circle-LineIntersection.html
        std::vector<Vector2D> intersection_points;

        //Normalize line to circle being at origin
        Line2D norm_line {line.start() - circle.center, line.end() - circle.center};

        //Calculate magnitude and determinant
        auto norm_line_mag_squared = std::pow(norm_line.vec().magnitude(), 2);
        auto det = norm_line.start().x*norm_line.end().y - norm_line.end().x*norm_line.start().y;

        //Calculate discriminant
        auto disc = std::pow(circle.radius,2)*norm_line_mag_squared - std::pow(det,2);
        
        //Discriminant greater than 0 indicates possible intersection
        if (disc >= 0.0) {
            std::vector<Vector2D> possible_points;
            //calculate possible intersection points
            possible_points.push_back({
                (det*norm_line.vec().y + sgn<double>(norm_line.vec().y)*norm_line.vec().x*std::sqrt(disc)) / norm_line_mag_squared,
                (-det*norm_line.vec().x + std::abs(norm_line.vec().y)*std::sqrt(disc)) / norm_line_mag_squared
            });

            possible_points.push_back({
                (det*norm_line.vec().y - sgn<double>(norm_line.vec().y)*norm_line.vec().x*std::sqrt(disc)) / norm_line_mag_squared,
                (-det*norm_line.vec().x - std::abs(norm_line.vec().y)*std::sqrt(disc)) / norm_line_mag_squared
            });

            //Check if these points are actually within the normalized line segment
            //If so, add them to the output vector
            for (const auto & point : possible_points) {
                if (point.x >= norm_line.min().x && point.x <= norm_line.max().x) {
                    intersection_points.push_back(point);
                }
            }
        }

        //Return intersection points to original coordinate system
        for (auto & point : intersection_points) {
            point += circle.center;
        }

        return intersection_points;
    }

    ////////// LINE2D END //////////
}