#include <armadillo>
#include <exception>
#include "turtlelib/circle_detection.hpp"



namespace turtlelib
{
    std::tuple<Circle2D, double> fit_circle(const std::vector<Vector2D> & points) {
        //based on:
        //https://projecteuclid.org/journals/electronic-journal-of-statistics/volume-3/issue-none/Error-analysis-for-circle-fitting-algorithms/10.1214/09-EJS419.full

        //If less than minimum number of points, return default circle object
        if (points.size() < 4) {
            return {};
        }

        const auto num_points = points.size();

        //Collect points into x and y vectors
        std::vector<double> x_coords;
        std::vector<double> y_coords;
        x_coords.reserve(num_points);
        y_coords.reserve(num_points);

        for (const auto & point : points) {
            x_coords.push_back(point.x);
            y_coords.push_back(point.y);
        }

        //Calculate centroid of points
        const Vector2D centroid {get_mean(x_coords), get_mean(y_coords)};

        std::vector<double> z_vals (num_points, 0.0);
        arma::mat Z {num_points, 4, arma::fill::zeros};

        for (size_t i = 0; i < num_points; i++) {
            //Init references
            auto & x = x_coords[i];
            auto & y = y_coords[i];
            auto & z = z_vals[i];

            //Shift coordinates so that the centroid is at the origin
            x -= centroid.x;
            y -= centroid.y;

            //Compute z values
            z = x*x + y*y;

            //Form the data matrix Z from the data points
            Z(i, 0) = z;
            Z(i, 1) = x;
            Z(i, 2) = y;
            Z(i, 3) = 1.0;
        }

        //Compute the mean of z
        const auto z_mean = get_mean(z_vals);

        //Form the moment matrix M
        arma::mat M = (1.0 / num_points) * Z.t() * Z;

        //Form the constraint matrix inverse H_inv for the "hyperaccurate algebraic fit"
        arma::mat H_inv {4, 4, arma::fill::zeros};

        H_inv(0,3) = 0.5;
        H_inv(1,1) = 1.0;
        H_inv(2,2) = 1.0;
        H_inv(3,0) = 0.5;
        H_inv(3,3) = -2*z_mean;

        //Compute singular value decomposition of Z
        //https://arma.sourceforge.net/docs.html#svd
        arma::mat U, V;
        arma::vec s;

        arma::svd(U, s, V, Z);

        //This is a full sigma matrix if Z is square.
        //if not, this contains the portion of the full sigma matrix
        //that we need for calculations
        arma::mat sigma {4, 4, arma::fill::zeros};

        for (size_t i = 0; i < s.size(); i++) {
            sigma(i,i) = s(i);
        }

        arma::cx_vec A;

        //The smallest singular value is the last value in s.
        //Perform different operations based on the magnitude of this value
        if (s.back() > 1.0e-12) {
            
            arma::mat Y = V*sigma*V.t();
            arma::mat Q = Y*H_inv*Y;
            
            //Get eigenvalues and eigenvectors of Q
            arma::cx_vec eigvals;
            arma::cx_mat eigvecs;

            arma::eig_gen(eigvals, eigvecs, Q);

            //Find smallest positive eigenvalue
            size_t index = -1;
            double min;

            for (size_t i = 0; i < eigvals.size(); i++) {
                const double real = eigvals(i).real();

                if (real > 0) {
                    if ((index == static_cast<size_t>(-1)) || (real < min)) {
                        index = i;
                        min = real;
                    }
                }
            }

            if (index == static_cast<size_t>(-1)) {
                throw (std::logic_error("Positive eigenvalue not found"));
            }

            //Solve for A
            A = Y.i()*eigvecs.col(index);

        } else {
            arma::vec A_temp = V.col(3);

            A = arma::cx_vec(A_temp, arma::vec(A_temp.size(), arma::fill::zeros));
        }

        //Determine parameters for the circle equation
        const auto A1 = A(0).real();
        const auto A2 = A(1).real();
        const auto A3 = A(2).real();
        const auto A4 = A(3).real();

        const auto a = -A2 / (2.0*A1);
        const auto b = -A3 / (2.0*A1);
        const auto R2 = (A2*A2 + A3*A3 - 4.0*A1*A4)/(4.0*A1*A1);

        //Calculate root-mean-square-error
        double error = 0.0;

        for (size_t i = 0; i < num_points; i++) {
            error += std::pow(std::pow(x_coords[i] - a, 2) + std::pow(y_coords[i] - b, 2) - R2, 2);
        }

        error = std::sqrt(error / num_points);

        return {
            {
                {a + centroid.x, b + centroid.y}, //center
                std::sqrt(R2) //radius
            },
            error
        };
    }
}