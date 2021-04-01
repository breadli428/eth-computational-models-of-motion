#ifndef BOIDS_H
#define BOIDS_H
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Sparse>
template <typename T, int dim>
using Vector = Eigen::Matrix<T, dim, 1, 0, dim, 1>;

template <typename T, int n, int m>
using Matrix = Eigen::Matrix<T, n, m, 0, n, m>;

// add more for yours
enum MethodTypes {
        FREEFALL=0, SEPARATION=1, ALIGNMENT=2, COHESION=3, LEADER=4
    };

template <class T, int dim>
class Boids
{
    typedef Matrix<T, Eigen::Dynamic, 1> VectorXT;
    typedef Matrix<T, dim,Eigen::Dynamic> TVStack;
    typedef Vector<T, dim> TV;
    typedef Matrix<T, dim, dim> TM;
    
private:
    TVStack positions;
    TVStack velocities;
    int n;
    bool update = false;
public:
    Boids() :n(1) {}
    Boids(int n) :n(n) {
        initializePositions();
    }
    ~Boids() {}

    void setParticleNumber(int n) {n = n;}
    int getParticleNumber() { return n; }
    void initializePositions()
    {
        positions = TVStack::Zero(dim, n).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}); 
        velocities = TVStack::Zero(dim, n);
    }

    void updateBehavior(MethodTypes type)
    {
        if(!update)
            return;
        switch (type)
        {  
        default:
            break;
        }
    }
    void pause()
    {
        update = !update;
    }
    TVStack getPositions()
    {
        return positions;
    }
};
#endif
