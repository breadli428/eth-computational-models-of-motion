#pragma once

#include "Minimizer.h"

#include <random>

class RandomMinimizer : public MinimizationMethod
{
public:
    RandomMinimizer(const VectorXd &upperLimit = VectorXd(), const VectorXd &lowerLimit = VectorXd(), double fBest = HUGE_VAL, const VectorXd &xBest = VectorXd())
        : searchDomainMax(upperLimit), searchDomainMin(lowerLimit), fBest(fBest), xBest(xBest){
        fBest = HUGE_VAL;

        // initialize random number generator
        rng.seed(std::random_device()());
        dist = std::uniform_real_distribution<>(0.0,1.0);
    }

    virtual ~RandomMinimizer() {}

    virtual bool minimize(const ObjectiveFunction *function, VectorXd &x) {
        for (int i = 0; i < iterations; ++i) {

            // generate a random number in [0, 1]
            // double a = dist(rng);

            ////////////////////////////////////// 1

            VectorXd xr(x.size());
            for (int i = 0; i < x.size(); ++i) {
                xr[i] = dist(rng) * (searchDomainMax[i] - searchDomainMin[i]) + searchDomainMin[i];
            }

            double f = function->evaluate(xr);
            if(f < fBest){
                x = xr;
                fBest = f;
            }

            ////////////////////////////////////// 1
        }
        return false;
    }

public:
    int iterations = 1;
    VectorXd searchDomainMax, searchDomainMin;
    VectorXd xBest;
    double fBest;

    std::uniform_real_distribution<double> dist;
    std::mt19937 rng;
};
