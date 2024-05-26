#include <PCA.hpp>
#include <Eigen/Dense>
#include <iostream>

int main() {
    constexpr int ndim = 2;
    Eigen::MatrixXd obs_points(7, 2);
    /*
    obs_points << 1, 2, 3, 4, 1, 2.3, 3.5,
                  1, 2, 3, 4, 1.2, 2, 3; */

    obs_points << 1, 1, 2, 2, 3, 3, 4, 4, 1, 1.2, 2.3,2, 3.5,3;

    Eigen::MatrixXd free_points(2,2);
    free_points <<  2, 2.1,
                    2, 2.2;

    Eigen::VectorXd q_near(2);
    q_near << 0, 0.1;

    Eigen::MatrixXd q_rands(2, 1);
    q_rands << 2,
               2;

    auto [obslens, obsaxes, obs_mean] = PCAEllipsoid(obs_points);
    print_matrix(obslens);
    print_matrix(obsaxes);
    print_matrix(obs_mean);

    return 0;
}