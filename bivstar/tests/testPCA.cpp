#include <PCA.hpp>
#include <Eigen/Dense>
#include <boost/math/distributions/chi_squared.hpp>
#include <iostream>
using std::cout;
constexpr int ndim = 2;

namespace
{
    double chi_ppf(double p, int df)
    {
        boost::math::chi_squared_distribution<double> chi_dist(df);
        return boost::math::quantile(chi_dist, p);
    }
}  // namespace

const double chi_ppf_ = chi_ppf(1 - 2 * (1 - std::pow(0.5, 0.5)), ndim);

Eigen::MatrixXd VineLikeExpansion(const Eigen::MatrixXd &obs_states, const Eigen::MatrixXd &free_states,
                                  Eigen::VectorXd &q_near, Eigen::MatrixXd &q_rands)
{

    auto eps = PCAEllipsoid(obs_states);

    auto&& [obs_eigenvalues, obsaxes, obscenter] = eps;

    auto obslen = obs_eigenvalues.array().sqrt();

    std::cout << "epl\n";
    print_matrix(obslen);
    print_matrix(obsaxes);
    print_matrix(obscenter);

    auto is_point_in_ellipsoid = [](const Eigen::VectorXd &point, const Eigen::VectorXd &center,
                                    const Eigen::MatrixXd &axes, const Eigen::VectorXd &radii)
    {
        assert(axes.rows() == radii.rows());

        double distance_squared = 0;
        for (Eigen::Index i = 0; i < axes.cols(); i++)
        {
            double r = radii(i);
            if (r == 0)
            {
                return false;
            }
            auto projection = (point - center).dot(axes.col(i));
            distance_squared += (projection / r) * (projection / r);
        }
        // std::cout << "In\n";
        // print_matrix(point);
        // std::cout << distance_squared << "\n";
        return distance_squared <= 1;
    };

    std::vector<Eigen::VectorXd> tendril_set;

    for (auto &&free_sample : free_states.rowwise())
    {
        if (is_point_in_ellipsoid(free_sample, obscenter, obsaxes, obslen))
        {
            tendril_set.emplace_back(free_sample);
        }
    }

    auto project =
        [=](const Eigen::MatrixXd &eigenvectors, const Eigen::VectorXd &sample, const Eigen::VectorXd &center, int k) -> Eigen::VectorXd
    {
        Eigen::VectorXd dir = sample - center;
        cout << "dir\n";
        print_matrix(dir);
        Eigen::VectorXd projection = Eigen::VectorXd::Zero(center.size());
        // OMPL_INFORM("vectors.cols()=%llu, dir.rows()=%llu, k=%d", eigenvectors.colStride(), dir.rows(), k);
        for (int i = 0; i < k; i++)
        {
            const Eigen::VectorXd &v = eigenvectors.col(i);
            projection += v.dot(dir) * v;
            std::cout <<"v\n";
        }

        return projection + center;
    };

    // OMPL_INFORM("%llu", tendril_set.size());

    Eigen::MatrixXd q_projections(ndim, q_rands.rows());
    int i = 0;
    for (auto &&q_rand : q_rands.rowwise())
    {
        q_projections.col(i) = project(obsaxes, q_rand, q_near, ndim - 1);
        i++;
    }
    std::cout << "projection" << q_projections.rows() << " " << q_projections.cols() << "\n";
    print_matrix(q_projections);

    if (tendril_set.size() < 2)
    {
        return q_projections;
    }

    std::cout << "tendril\n";
    for(auto&& tendril: tendril_set)
        print_matrix(tendril);

    auto contains =
        [](const Eigen::VectorXd &eigenvalues, const Eigen::MatrixXd &eigenvectors, const Eigen::VectorXd& mean, const Eigen::VectorXd &sample)
    {
        Eigen::VectorXd transformed_sample = eigenvectors.transpose() * (sample - mean);
        // std::cout << "transformed_sample\n";
        // print_matrix(transformed_sample);
        auto d = std::sqrt((transformed_sample.transpose() * eigenvalues * transformed_sample)(0, 0));
        //std::cout << "d=" << d  << " ppf=" << chi_ppf_ << std::endl;
        return d < chi_ppf_;
    };

    if (contains(obs_eigenvalues, obsaxes, obscenter, q_near))
    {
        auto [free_eigenvalues, free_eigenvectors, freecenter] = PCAEllipsoid(free_states);
        Eigen::MatrixXd extend_q(ndim, q_rands.rows());
        int i = 0;
        for (auto &&q_rand : q_rands.rowwise())
        {
            extend_q.col(i++) = (project(free_eigenvectors, q_rand, q_near, ndim - 1));
        }
        return extend_q;
    }

    Eigen::VectorXd q_2 = Eigen::VectorXd::Zero(ndim);

    for (auto &&tendril_p : tendril_set)
    {
        q_2 += tendril_p;
    }

    q_2 /= tendril_set.size();

    print_matrix(q_2);
    print_matrix(q_projections);

    return q_projections;
}

int main()
{
    Eigen::MatrixXd obs_points(7, 2);
    /*
    obs_points << 1, 2, 3, 4, 1, 2.3, 3.5,
                  1, 2, 3, 4, 1.2, 2, 3; */

    obs_points << 1, 1,
                  2, 2,
                  3, 3,
                  4, 4,
                  1, 1.2,
                  2.3, 2,
                  3.5, 3;

    Eigen::MatrixXd free_points(2, 2);
    free_points << 2, 2, 2.1, 2.2;

    Eigen::VectorXd q_near(2);
    q_near << 0, 0.1;

    Eigen::MatrixXd q_rands(1, 2);
    q_rands << 2, 2;

    print_matrix(VineLikeExpansion(obs_points, free_points, q_near, q_rands));

    return 0;
}