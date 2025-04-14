#ifndef MPCDPS_MEAN_COV_FILTER_H
#define MPCDPS_MEAN_COV_FILTER_H

#include <Eigen/Core>

namespace mpcdps {

    template <int N>
    class MeanCovFilter
    {
    public:
        using X = Eigen::Matrix<double, N, 1>;
        using M = Eigen::Matrix<double, N, N>;

        MeanCovFilter()
        {
            _m.setZero();
            _C.setZero();
        }

        void push(const X& x)
        {
            ++_n;
            _m += (x - _m)/double(_n);
            _C += x * x.transpose();
        }

        const X& get_mean() const
        {
            return _m;
        }

        M get_covariance() const 
        {
            if (_n == 0)
                return _C;
            return _C / double(_n) - _m * _m.transpose();
        }

        int size() const
        {
            return _n;
        }

        void reset()
        {
            _n = 0;
            _m.setZero();
            _C.setZero();
        }

    protected:
        X _m;
        M _C;
        int _n = 0;
    };

}

#endif