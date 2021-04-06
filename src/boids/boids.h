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
        FREEFALL=0, SEPARATION=1, ALIGNMENT=2, COHESION=3, LEADING=4, CIRCULAR=5, COLLISION_AVOIDANCE=6, COLLABORATION_ADVERSARY=7
    };

enum UpdateTypes {
        EXPLICIT_EULER=0, SYMPLECTIC_EULER=1, EXPLICIT_MIDPOINT=2
    };

enum ControlGroup {
        RED=0, BLUE=1, BOTH=2
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
    // position of the cursor click, which the leader should approach
    bool obstacle_enabled = false;
    Eigen::MatrixXi group_label;
    // 2 * n matrix, each column corresponds to one boid
    // the first entry denotes the group, 0 = red, 1 = blue
    // the second entry denotes reproduction permission, 0 = reproduction allowed, 1 = reproduction limited
    ControlGroup control_group;

public:
    Boids() :n(1) {}
    Boids(int n) :n(n) {
        initializePositions(ZERO);
        initializeGroupLabel();
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

    void initializeGroupLabel()
    // initialize group division and reproduction permission
    {
        group_label = Eigen::MatrixXi::Zero(2, n);
        group_label.block(0, 0, 1, n / 2) = Eigen::MatrixXi::Ones(1, n / 2);
    }

    void updateBehavior(MethodTypes type, UpdateTypes rule)
    {
        std::function<TVStack(TVStack)> method;
        if (!update)
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
            case 7: method = std::bind(&Boids::collaboration_adversary, this, std::placeholders::_1); break;
            default: method = std::bind(&Boids::freefall, this, std::placeholders::_1); break;
        }
        switch (rule)
        {
            case 0: explicit_euler(method); break;
            case 1: symplectic_euler(method); break;
            case 2: explicit_midpoint(method); break;
            default: explicit_euler(method); break;
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

    Eigen::MatrixXi getGroupLabel()
    {
        return group_label;
    }

    void read_parameters(float h_read, float f_scaler_read, float range_read, Obstacle obstacle_read, bool drawCircles, ControlGroup control_group_read)
    {
        f_scaler = f_scaler_read;
        h = h_read;
        range = range_read;
        obstacle = obstacle_read;
        obstacle_enabled = drawCircles;
        control_group = control_group_read;
    }

    void read_cursor(TV target_pos_read)
    {
        target_pos = target_pos_read;
    }

    void circular_vel_init()
    // generate velocities orthogonal to positions
    {
        Eigen::Matrix2f permutation;
        permutation << 0.0, -1.0 , 1.0, 0.0;
        velocities = permutation * positions;
    }

    template <typename DerivedA, typename DerivedB>
    void add_column(Eigen::EigenBase<DerivedA>& matrix, Eigen::EigenBase<DerivedB>& col_to_add)
    // add a column to the end of a matrix
    {
        matrix.derived().conservativeResize(matrix.rows(), matrix.cols() + 1);
        matrix.derived().col(matrix.cols() - 1) = col_to_add.derived();
    }

    template <typename DerivedC>
    void remove_column(Eigen::EigenBase<DerivedC>& matrix, int col_ind)
    // remove a column of a matrix at index col_ind
    {
        int row_num = matrix.rows();
        int col_num = matrix.cols() - 1;
        if (col_ind < col_num)
            matrix.derived().block(0, col_ind, row_num, col_num - col_ind) = matrix.derived().block(0, col_ind + 1, row_num, col_num - col_ind);
        matrix.derived().conservativeResize(row_num, col_num);
    }

    int removed_cols_before_count(std::vector<int> removed_cols_ind_collection, int index)
    // calculate the number of columns in list removed_cols_ind_collection prior to index
    {
        int count = 0;
        for (int i = 0; i < size(removed_cols_ind_collection); i++)
        {
            if (removed_cols_ind_collection[i] < index)
            {
                count++;
            }
        }
        return count;
    }

    void explicit_euler(std::function<TVStack(TVStack)> method)
    {
        TVStack positions_temp = positions;
        positions += h * velocities;
        TVStack f = method(positions_temp);
        velocities += h * f;
    }

    void symplectic_euler(std::function<TVStack(TVStack)> method)
    {
        positions += h * velocities;
        TVStack positions_temp = positions;
        TVStack f = method(positions_temp);
        velocities += h * f;
    }

    void explicit_midpoint(std::function<TVStack(TVStack)> method)
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
        TV g(0, 9.8);
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

    TVStack collaboration_adversary(TVStack positions_temp)
    {
        TVStack f = TVStack::Zero(dim, n);
        Eigen::MatrixXi group_label_copy = group_label;
        // backup group_label since it will resized during the iteration
        std::vector<int> removed_cols_ind_collection;
        // collection of the indices of columns to be removed
        for (int i = 0; i < positions_temp.cols(); i++)
        {
            TVStack neighbor = TVStack::Zero(dim, n);
            // collection of alerting neighbors of boid i within 2.0f * range, where a repulsive force will be applied
            int neighbor_count = 0;
            // count of alerting neighbors described above
            int neighbor_count_ad = 0;
            // count of adversarial neighbors of boid i within range, where boid i will be removed if neighbor_count_ad >= 3
            int control_flag_ad = 0;
            // adversarial behavior control flag, 1 = enabled
            int removed_cols_count_before_i = removed_cols_before_count(removed_cols_ind_collection, i);
            // number of columns removed in list removed_cols_ind_collection prior to i
            for (int j = 0; j < positions_temp.cols(); j++)
            {
                float dist = (positions_temp.col(i) - positions_temp.col(j)).norm();
                // distance between boid i and j
                int removed_cols_count_before_j = removed_cols_before_count(removed_cols_ind_collection, j);
                // number of columns removed in list removed_cols_ind_collection prior to j
                int control_flag_co = 0;
                // collaborative behavior control flag, 1 = enabled
                if (i != j && dist <= 5.0f * range && group_label_copy(0, i) == group_label_copy(0, j))
                // check if dist <= 5.0f * range within which attraction force is applied and check if they are from the same group
                {
                    switch (control_group)
                    // apply control strategy to diffent control group
                    {
                        case RED: if (group_label_copy(0, i) == 0) control_flag_co = 1; break;
                        case BLUE: if (group_label_copy(0, i) == 1) control_flag_co = 1; break;
                        case BOTH: control_flag_co = 1; break;
                        default: control_flag_co = 1; break;
                    }
                    if (control_flag_co)
                    // collaborative behavior control enabled only when control_flag_co = 1
                        f.col(i - removed_cols_count_before_i) = f_scaler * (positions_temp.col(j) - positions_temp.col(i)) - 0.5f * velocities.col(i - removed_cols_count_before_i);
                        // force is calculated using PD control law
                    if (dist <= range  && group_label_copy(1, i) == 0 && group_label_copy(1, j) == 0)
                    // check further if dist <= range and boid i and j are both allowed to reproduce
                    {
                        group_label_copy(1, i) = 1;
                        group_label_copy(1, j) = 1;
                        // set reproduction permission to limited
                        group_label(1, i - removed_cols_count_before_i) = 1;
                        group_label(1, j - removed_cols_count_before_j) = 1;
                        // set reproduction permission to limited
                        add_column(positions, TV((positions_temp.col(i) + positions_temp.col(j)) / 2));
                        // add a column to positions with average position between boid i and j
                        add_column(group_label, (Eigen::MatrixXi(2, 1) << group_label_copy(0, i), 0).finished());
                        // add a column to group_label assigning the same group as boid i and reproduction permission
                        add_column(velocities, TV((velocities.col(i - removed_cols_count_before_i) + velocities.col(j - removed_cols_count_before_j)) / 2));
                        // add a column to velocities with average velocities between boid i and j
                        add_column(f, TV(0.0f, 0.0f));
                        // add a column to f with zeros
                        add_column(neighbor, TV(0.0f, 0.0f));
                        // add a column to neighbor with zeros
                        n++;
                        // increase the total number of boids by 1
                    }
                }

                if (i != j && dist <= 2.0f * range && group_label_copy(0, i) != group_label_copy(0, j))
                // check if dist <= 2.0f * range within which repulsion force is applied and check if they are from different groups
                {
                    neighbor.col(j - removed_cols_count_before_j) = positions_temp.col(j);
                    // collect alerting neighbors of boid i
                    neighbor_count++;
                    // increase the number of alerting neighbors of boid i by 1
                    if (dist <= range)
                    // check further if dist <= range
                        neighbor_count_ad++;
                        // increase the number of adversarial neighbors of boid i by 1
                }
            }

            if (neighbor_count >= 10)
            // check if the number of alerting neighbors of boid i >= 10
            {
                TV center = neighbor.rowwise().sum() / neighbor_count;
                // calculate the average position of alerting neighbors of boid i
                f_scaler = 1 / pow((positions_temp.col(i) - center + TV(10.0f, 10.0f)).norm(), 2);
                // calculate f_scaler inversely proportional to the sqaure of distance between boid i and the average position
                switch (control_group)
                // apply control strategy to diffent control group
                {
                    case RED: if (group_label_copy(0, i) == 0) control_flag_ad = 1; break;
                    case BLUE: if (group_label_copy(0, i) == 1) control_flag_ad = 1; break;
                    case BOTH: control_flag_ad = 1; break;
                    default: control_flag_ad = 1; break;
                }
                if (control_flag_ad)
                    // adversarial behavior control enabled only when control_flag_ad = 1
                    f.col(i - removed_cols_count_before_i) += f_scaler * (positions_temp.col(i) - center).normalized();
                    // force is calculated as the product of f_scaler and the direction pointing from the average position to boid i
            }
            
            if (neighbor_count_ad >= 3)
            // check if the number of adversarial neighbors of boid i >= 3
            {
                remove_column(positions, i - removed_cols_count_before_i);
                remove_column(group_label, i - removed_cols_count_before_i);
                remove_column(velocities, i - removed_cols_count_before_i);
                remove_column(f, i - removed_cols_count_before_i);
                remove_column(neighbor, i - removed_cols_count_before_i);
                // remove the column corresponding to index i in positions_temp
                n--;
                // decrease the total number of boids by 1
                removed_cols_ind_collection.push_back(i);
                // collect the index of the removed column
            }
        }
        TVStack f_repulsion = TVStack::Zero(dim, n);
        range = 0.05f;
        f_repulsion = separation(positions);
        // calculate repulsion in case to prevent being too close to each other
        return f + f_repulsion;
    }

};
#endif
