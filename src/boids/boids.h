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
enum InitTypes {
        ZERO=0, ORTHOGONAL=1, RAMDOM=2
    };

enum MethodTypes {
        FREEFALL=0, SEPARATION=1, ALIGNMENT=2, COHESION=3, LEADING=4, CIRCULAR=5, COLLISION_AVOIDANCE=6
    };

enum UpdateTypes {
        EXPLICIT_EULER=0, SYMPLECTIC_EULER=1, EXPLICIT_MIDPOINT=2
    };

template <class T, int dim>
class Boids
{
    typedef Matrix<T, Eigen::Dynamic, 1> VectorXT;
    typedef Matrix<T, dim, Eigen::Dynamic> TVStack;
    typedef Vector<T, dim> TV;
    typedef Matrix<T, dim, dim> TM;
    
private:
    TVStack positions;
    TVStack velocities;
    int n;
    bool update = false;
    float h, range, f_scaler;
    TV target_pos;
    bool obstacle_enabled = false;

public:
    Boids() :n(1) {}
    Boids(int n) :n(n) {
        initializePositions(ZERO);
    }
    ~Boids() {}
    
    struct Obstacle
    {
        TV pos;
        float radius;
    } obstacle;

    void setParticleNumber(int n) {n = n;}
    int getParticleNumber() { return n; }
    void initializePositions(InitTypes init_vel)
    {
        positions = TVStack::Zero(dim, n).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}); 
        switch (init_vel)
        {
            case 0: velocities = TVStack::Zero(dim, n); break;
            case 1: circular_vel_init(); break;
            case 2: velocities = TVStack::Zero(dim, n).unaryExpr([&](T dummy){return static_cast <T> (rand() - 0.5 * RAND_MAX) / static_cast <T> (RAND_MAX);}); break;
            case 3: velocities = TVStack::Zero(dim, n).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}); break;
            default : velocities = TVStack::Zero(dim, n); break;
        }
        
    }

    void updateBehavior(MethodTypes type, UpdateTypes rule)
    {
        std::function<TVStack(TVStack)> method;
        if(!update)
            return;
        switch (type)
        {  
            case 0: method = std::bind(&Boids::freefall, this, std::placeholders::_1); break;
            case 1: method = std::bind(&Boids::separation, this, std::placeholders::_1); break;
            case 2: method = std::bind(&Boids::alignment, this, std::placeholders::_1); break;
            case 3: method = std::bind(&Boids::cohesion, this, std::placeholders::_1); break;
            case 4: method = std::bind(&Boids::leading, this, std::placeholders::_1); break;
            case 5: method = std::bind(&Boids::circular, this, std::placeholders::_1); break;
            case 6: method = std::bind(&Boids::collision_avoidance, this, std::placeholders::_1); break;
            default: method = std::bind(&Boids::freefall, this, std::placeholders::_1); break;
        }
        switch (rule)
        {
            case 0: explicit_euler(h, method); break;
            case 1: symplectic_euler(h, method); break;
            case 2: explicit_midpoint(h, method); break;
            default: explicit_euler(h, method); break;
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

    void read_parameters(float h_read, float f_scaler_read, float range_read, Obstacle obstacle_read, bool drawCircles)
    {
        f_scaler = f_scaler_read;
        h = h_read;
        range = range_read;
        obstacle = obstacle_read;
        obstacle_enabled = drawCircles;
    }

    void read_cursor(TV target_pos_read)
    {
        target_pos = target_pos_read;
    }

    void circular_vel_init()
    {
        Eigen::Matrix2f permutation;
        permutation << 0.0, -1.0 , 1.0, 0.0;
        velocities = permutation * positions;
    }

    void explicit_euler(float h, std::function<TVStack(TVStack)> method)
    {
        TVStack positions_temp = positions;
        positions += h * velocities;
        TVStack f = method(positions_temp);
        velocities += h * f;
    }

    void symplectic_euler(float h, std::function<TVStack(TVStack)> method)
    {
        positions += h * velocities;
        TVStack positions_temp = positions;
        TVStack f = method(positions_temp);
        velocities += h * f;
    }

    void explicit_midpoint(float h, std::function<TVStack(TVStack)> method)
    {
        TVStack positions_temp = positions;
        TVStack positions_temp_half = positions_temp + h / 2 * velocities;
        TVStack f_temp_half = method(positions_temp);
        TVStack velocities_temp_half = velocities + h / 2 * f_temp_half;
        positions += h * velocities_temp_half;
        TVStack f = method(positions_temp_half);
        velocities += h * f;
    }

    TVStack freefall(TVStack positions_temp)
    {
        Vector<T, dim> g(0, 9.8);
        TVStack f = g.replicate(1, n);
        return f;
    }

    TVStack separation(TVStack positions_temp) 
    {
        TVStack f = TVStack::Zero(dim, n);
        for (int i = 0; i < positions_temp.cols(); i++)
        {
            for (int j = 0; j < positions_temp.cols(); j++)
            {
                if ((positions_temp.col(i) - positions_temp.col(j)).norm() <= range)
                {
                    f_scaler = 1 / pow((positions_temp.col(i) - positions_temp.col(j) + TV(2.0f, 2.0f)).norm(), 2);
                    f.col(i) += f_scaler * (positions_temp.col(i) - positions_temp.col(j)).normalized();
                }
            }
        }
        return f;
    }    

    TVStack alignment(TVStack positions_temp) 
    {
        TVStack f_pos = TVStack::Zero(dim, n);
        TVStack f_vel = TVStack::Zero(dim, n);
        TVStack f = TVStack::Zero(dim, n);
        f_pos = cohesion(positions_temp);
        for (int i = 0; i < positions_temp.cols(); i++)
        {
            TVStack neighbor = TVStack::Zero(dim, n);
            int neighbor_count = 0;
            for (int j = 0; j < positions_temp.cols(); j++)
            {
                if ((positions_temp.col(i) - positions_temp.col(j)).norm() <= range)
                {
                    neighbor.col(j) = velocities.col(j);
                    neighbor_count ++;
                }
            }
            TV center = neighbor.rowwise().sum() / neighbor_count;
            f_vel.col(i) = f_scaler * (center - center.dot(velocities.col(i)) * velocities.col(i).normalized());
        }
        f = f_pos + f_vel;
        return f;
    }

    TVStack cohesion(TVStack positions_temp) 
    {
        TVStack f = TVStack::Zero(dim, n);
        for (int i = 0; i < positions_temp.cols(); i++)
        {
            TVStack neighbor = TVStack::Zero(dim, n);
            int neighbor_count = 0;
            for (int j = 0; j < positions_temp.cols(); j++)
            {
                if ((positions_temp.col(i) - positions_temp.col(j)).norm() <= range)
                {
                    neighbor.col(j) = positions_temp.col(j);
                    neighbor_count ++;
                }
            }
            TV center = neighbor.rowwise().sum() / neighbor_count;
            f.col(i) = f_scaler * (center - positions_temp.col(i)) - 0.2f * velocities.col(i);
        }
        return f;
    }

    TVStack leading(TVStack positions_temp)
    {
        TVStack f = TVStack::Zero(dim, n);
        TVStack f_repulsion = TVStack::Zero(dim, n);
        TVStack f_obs = TVStack::Zero(dim, n);
        f.col(0) = f_scaler * (target_pos - positions_temp.col(0)) - 2.0f * velocities.col(0);
        for (int i = 1; i < positions_temp.cols(); i++)
        {
            if ((positions_temp.col(0) - positions_temp.col(i)).norm() <= range)
            {
                f.col(i) += f_scaler * (positions_temp.col(0) - positions_temp.col(i)) + 0.8f * (velocities.col(0) - velocities.col(i));
            }
        }
        range = 0.05f;
        f_repulsion = separation(positions_temp);
        if (obstacle_enabled) f_obs = collision_avoidance(positions_temp);
        return f + f_repulsion + f_obs;
    }

    TVStack circular(TVStack positions_temp)
    {
        TVStack f = TVStack::Zero(dim, n);
        for (int i = 0; i < positions_temp.cols(); i++)
        {
            f.col(i) = -positions_temp.col(i);
        }
        return f;
    }

    TVStack collision_avoidance(TVStack positions_temp)
    {
        TVStack f = TVStack::Zero(dim, n);
        float safety_coefficient = 1.001f;
        for (int i = 0; i < positions_temp.cols(); i++)
        {
            if ((positions_temp.col(i) - obstacle.pos).norm() <= obstacle.radius)
                {
                    positions.col(i) = obstacle.pos + (positions_temp.col(i) - obstacle.pos).normalized() * (safety_coefficient * obstacle.radius);
                    f_scaler = 1.0f / pow((positions_temp.col(i) - obstacle.pos + TV(0.6f, 0.6f)).norm(), 2);
                    f.col(i) = f_scaler * (positions_temp.col(i) - obstacle.pos).normalized();
                }
        }
        return f;
    }

};
#endif
