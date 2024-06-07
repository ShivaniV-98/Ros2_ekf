#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include<limits>
#include<vector>
#include<tuple>

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief infinity
    constexpr double INF=std::numeric_limits<double>::infinity();

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        return std::abs(d1 - d2) < epsilon;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return deg * PI / 180.0;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad * 180.0 / PI;
    }
    
    /// @brief determine the sign of an input value
    /// @tparam T - type of input variable
    /// @param val - value to check type of
    /// @return -1 for negative, 0 for 0, 1 for positive
    /// https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(1.23456, 1.23456), "nonzero float almost equal failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad 0 failed");

    static_assert(almost_equal(deg2rad(135), 2.356194490192345), "deg2rad 135 failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg 0 failed");

    static_assert(almost_equal(rad2deg(PI / 6.0), 30.0), "rad2deg PI/6 failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "rad2deg and deg2rad back failed");

    static_assert(almost_equal(rad2deg(deg2rad(352.0)), 352.0), "deg2rad and rad2deg back failed");

    /// \brief bound an angle to equivalent angle between (-PI, PI]
    /// \param rad angle to normalize
    /// \return normalized angle
    double normalize_angle(double rad);

    /// \brief calculate the mean of a vector of values
    /// \param values - values to average
    /// \return the mean
    double get_mean(const std::vector<double> & values);

    /// \brief calculate the mean and standard deviation of a vector of values
    /// \param values - values to find mean and standard deviation
    /// \return a tuple with the mean and the average
    std::tuple<double, double> get_mean_and_std_dev(const std::vector<double> & values);

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief compute the magnitude of the vector
        /// \return the magnitude of the vector
        double magnitude() const;

        /// \brief add this vector with another and store the result 
        /// in this object
        /// \param rhs - the vector to add
        /// \return a reference to the newly added vector
        Vector2D & operator+=(const Vector2D & rhs);

        /// \brief subtract another vector from this vector and store
        /// the result in this object
        /// \param rhs - the vector to subtract
        /// \return a reference to the newly subtracted vector
        Vector2D & operator-=(const Vector2D & rhs);

        /// \brief multiply a vector by a scalar and store
        /// the result in this object
        /// \param scalar - the scalar to multiply by
        /// \return a reference to the newly multiplied vector
        Vector2D & operator*=(const double scalar);
    };

    /// \brief add two vectors together, returning their sum
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the sum of the two vectors
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief subtract a vector from another, returning the difference
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the difference of the two vectors
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief multiply a vector by a scalar, returning the scaled vector
    /// \param vector - the vector to be multiplied
    /// \param scalar - the scalar to multiply by
    /// \return the scaled vector
    Vector2D operator*(Vector2D vector, const double scalar);

    /// \brief multiply a vector by a scalar, returning the scaled vector
    /// \param scalar - the scalar to multiply by
    /// \param vector - the vector to be multiplied
    /// \return the scaled vector
    Vector2D operator*(const double scalar, Vector2D vector);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user types
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin) and processing stops.
    ///
    /// We have lower level control however.
    /// std::peek() looks at the next unprocessed character in the buffer without removing it
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// std::get() removes the next unprocessed character from the buffer.
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/get
    /// When you call std::peek() it will wait for there to be at least one character in the buffer (e.g., the user types a character)
    /// HINT: this function can be written in under 20 lines and uses only std::peek(), std::get(), istream::operator>>() and a little logic
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// @brief approximately compare two Vector2Ds for equivalence
    /// @param v1 - first vector
    /// @param v2 - second vector
    /// \param epsilon - absolute threshold required for equality
    /// @return true if both x and y components are are within the epsilon threshold of eachother
    bool almost_equal(const Vector2D & v1, const Vector2D & v2, double epsilon=1.0e-12);

    /// \brief normalize a 2 dimensional vector
    /// \param v - the vector to normalize
    Vector2D normalize(const Vector2D & v);

    /// \brief compute the dot product of two vectors
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the dot product
    double dot(const Vector2D & lhs, const Vector2D & rhs);

    /// \brief - compute the angle between two vectors, with directional information
    /// (sign) based on the order in which the vectors are passed into the function
    /// \param start - the vector from which the measurement is started
    /// \param end - the vector to which the measurement goes
    /// \return the angle between the vectors, signed positive for CCW motion and
    /// negative for CW motion
    double angle_between(const Vector2D & start, const Vector2D & end);

    /// \brief A 2-Dimensional Twist
    struct Twist2D {
        /// \brief the angular velocity
        double w = 0.0;

        /// \brief the linear x velocity
        double x = 0.0;

        /// \brief the lineary y velocity
        double y = 0.0;
    };

    /// \brief output a 2 dimensional twist as [wcomponent xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param V - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & V);

    // \brief input a 2 dimensional twist
    ///   You should be able to read vectors entered as follows:
    ///   [w x y] or w x y
    /// \param is - stream from which to read
    /// \param V [out] - output twist
    std::istream & operator>>(std::istream & is, Twist2D & V);

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    private:
        /// \brief translational component
        Vector2D trans_ {0.,0.};
        /// \brief rotational component
        double rot_ = 0.;

        /// \brief cached sine of rotational angle
        double rot_sin_ = 0.;

        /// \brief cached cosine of rotational angle
        double rot_cos_ = 1.;

        /// \brief calculate and store trigonmetric values from rotational component
        /// to increase computational efficiency
        void cache_trig();

    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param rot - angle of the rotation, in radians
        explicit Transform2D(double rot);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(Vector2D trans, double rot);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply a transformation to a Twist2D
        /// \param V - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D V) const;


        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    /// For example:
    /// 90 2 3
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// @brief approximately compare two Transform2Ds for equivalence
    /// @param T1 - first transform
    /// @param T2 - second transform
    /// \param epsilon - absolute threshold required for equality
    /// @return true if both translation and rotational components are within the epsilon threshold
    /// of eachother
    bool almost_equal(const Transform2D & T1, const Transform2D & T2, double epsilon=1.0e-12);

    /// \brief compute the transformation corresponding to a rigid body following
    /// a constant twist (in its original body frame) for one time unit
    /// \param twist twist to follow
    /// \return transformation from original body frame to frame after following twist
    Transform2D integrate_twist(Twist2D twist);

    
    /// \brief a line segment between two points
    class Line2D
    {
    private:
        /// \brief the line start point, represented as a vector from the origin
        Vector2D start_;
        
        /// \brief the line end point, represented as a vector from the origin
        Vector2D end_;
        
        /// \brief the slope of the line segment
        double slope_;

        /// \brief the y intercept of the line segment
        double y_intercept_;

        /// \brief the minimum x and y values of the line segment
        Vector2D min_;

        /// \brief the maximum x and y values of the line segment
        Vector2D max_;

        /// \brief the vector from the start point to the end point
        Vector2D vec_;

    public:
        /// \brief init line and calculate slope and y intercept
        /// \param start - start point, represented as a vector from the origin
        /// \param end - end point, represented as a vector from the origin
        Line2D(Vector2D start, Vector2D end);
        
        /// \brief getter for start point
        /// \return start point
        Vector2D start() const;

        /// \brief getter for end point
        /// \return end point
        Vector2D end() const;

        /// \brief getter for slope
        /// \return slope
        double slope() const;

        /// \brief getter for the y intercept
        /// \return y intercept
        double y_intercept() const;

        /// @brief getter for the minimum x and y values
        /// @return the minimum x and y values
        Vector2D min() const;

        /// @brief getter for the maximum x and y values
        /// @return the maximum x and y values
        Vector2D max() const;

        /// \brief getter for the vector from the start point to the end point
        /// \return vector from the start point to the end point
        Vector2D vec() const;

        /// \brief find y coordinate of a point on the line. may not be within line segment
        /// \param x - x coordinate of point
        /// \return y coordinate of point
        double calc_y(double x) const;
    };

    /// \brief determine if two line segments intersect
    /// \param line1 - first line segment
    /// \param line2 - second line segment
    /// \return a vector of intersection points as Vector2Ds, if any exist
    std::vector<Vector2D> find_intersection(const Line2D & line1, const Line2D & line2);

    /// \brief a circle
    struct Circle2D
    {
        /// \brief the center of the circle, represented as a vector from the origin
        Vector2D center {0.0, 0.0};

        /// \brief the radius of the circle
        double radius = 0.0;
    };

    /// \brief determine if a line segment intersects with a circle
    /// \param line - line segment
    /// \param circle - circle
    /// \return a vector of intersection points as Vector2Ds, if any exist
    std::vector<Vector2D> find_intersection(const Line2D & line, const Circle2D & circle);

}

#endif
