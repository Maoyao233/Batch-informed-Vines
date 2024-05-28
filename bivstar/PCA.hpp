#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <numeric>

void print_matrix(const Eigen::MatrixXd &matrix)
{
    size_t cols = matrix.cols();
    for (auto &&row : matrix.rowwise())
    {
        for (size_t i = 0; i < cols; i++)
        {
            std::cout << row(i) << " ";
        }
        std::cout << std::endl;
    }
}

struct PCAEllipsoid
{
    PCAEllipsoid(const Eigen::MatrixXd &data)
      : sorted_eigenvalues_(data.cols())
      , sorted_egienvectors_(data.cols(), data.cols())
      , colwiseMean_(data.colwise().mean())
    {
        size_t ndim = data.cols();

        auto centeredData = data.rowwise() - colwiseMean_.transpose();
        Eigen::MatrixXd cov = (centeredData.adjoint() * centeredData) / (centeredData.rows() - 1);

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver{cov};
        if (solver.info() != Eigen::Success)
        {
            throw std::logic_error("Failed to finish PCA!");
        }

        std::vector<int> indices(ndim);
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(),
                  [&values = solver.eigenvalues()](int i, int j) { return values[i] > values[j]; });

        // sorted_eigenvalues_ = solver.eigenvalues();
        sorted_eigenvalues_.resize(ndim);
        for (size_t i = 0; i < ndim; i++)
        {
            sorted_eigenvalues_(i) = solver.eigenvalues()(indices[i]);

            sorted_egienvectors_.col(i) = solver.eigenvectors().col(indices[i]);
        }
    }

    Eigen::VectorXd sorted_eigenvalues_;
    Eigen::MatrixXd sorted_egienvectors_;
    Eigen::VectorXd colwiseMean_;
};