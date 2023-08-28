#pragma once
#include "math_headers.h"
#include "Constraint.h"
#include <memory>
#include <queue>


typedef enum
{
    INTEGRATION_IMPLICIT_EULER,
    INTEGRATION_IMPLICIT_BDF2,
    INTEGRATION_IMPLICIT_MIDPOINT,
    INTEGRATION_IMPLICIT_NEWMARK_BETA,
    INTEGRATION_QUASI_STATICS,
    INTEGRATION_TOTAL_NUM
} IntegrationMethod;

typedef enum
{
    OPTIMIZATION_METHOD_GRADIENT_DESCENT,
    OPTIMIZATION_METHOD_NEWTON,
    OPTIMIZATION_METHOD_LBFGS,
    OPTIMIZATION_METHOD_TOTAL_NUM
} OptimizationMethod;

typedef enum
{
    SOLVER_TYPE_DIRECT_LLT,
    SOLVER_TYPE_CG,
    SOLVER_TYPE_TOTAL_NUM
} SolverType;

typedef enum
{
    LBFGS_H0_IDENTITY,
    LBFGS_H0_LAPLACIAN,
    LBFGS_H0_TOTAL_NUM
} LBFGSH0Type;

class RigidBody;
class SoftBody;
class Contact;
class CollisionDetect;
class CollisionSolver;

class Simulation
{
public:
    Simulation();
    ~Simulation();

    const std::vector<std::unique_ptr<Contact>>& getContacts() const;

    void Reset();
    void Update();

    // set material property for all elements
    void SetMaterialProperty();
    void SetMaterialProperty(std::vector<Constraint*>& constraints);

    // inline functions
    inline void SetReprefactorFlag() 
    {
        m_precomputing_flag = false;
        m_prefactorization_flag = false;
        m_prefactorization_flag_newton = false;
    }
    inline void SetSoftBody(SoftBody *sb) { m_softbody = sb; };
    inline SoftBody* GetSoftBody() { return m_softbody; };
    // inline void SetRigidBody(RigidBody *rb) { m_rigidbody = rb; };
    // inline RigidBody* GetRigidBody() { return m_rigidbody; }
    
protected:
    // sim constants
    ScalarType m_h; // time_step
    unsigned int m_sub_stepping; // 
    bool m_step_mode;

    ScalarType m_gravity_constant;
    MaterialType m_material_type;
    ScalarType m_stiffness_attachment;
    ScalarType m_stiffness_stretch;
    ScalarType m_stiffness_bending;
    ScalarType m_stiffness_kappa;
    bool m_stiffness_auto_laplacian_stiffness;
    ScalarType m_stiffness_laplacian;
    ScalarType m_damping_coefficient;
    //ScalarType m_restitution_coefficient;
    //ScalarType m_friction_coefficient;

    // integration and optimization method
    IntegrationMethod m_integration_method;
    OptimizationMethod m_optimization_method;

    // key simulation components
    SoftBody* m_softbody;
    //RigidBody * m_rigidbody;

    // key simulation components: constraints
    std::vector<Constraint*> m_constraints;

    // Collision
    // std::unique_ptr<CollisionDetect> m_collisionDetect;
    // std::unique_ptr<CollisionSolver> m_collisionSolver;
    // AttachmentConstraint* m_selected_attachment_constraint;
    // collision constraints
    // std::vector<CollisionSpringConstraint> m_collision_constraints;

    // partial material control
    // std::vector<Constraint*> m_selected_constraints;

    // for Newton's method
    bool m_definiteness_fix;

    // solver type
    SolverType m_solver_type;
    int m_iterative_solver_max_iteration;

    // LBFGS
    bool m_lbfgs_restart_every_frame;
    bool m_lbfgs_need_update_H0;
    int m_lbfgs_m; // back-track history length
    VectorX m_lbfgs_last_x;
    VectorX m_lbfgs_last_gradient;
    LBFGSH0Type m_lbfgs_H0_type;
    // QueueLBFGS* m_lbfgs_queue;
    std::deque<VectorX> m_lbfgs_y_queue;
    std::deque<VectorX> m_lbfgs_s_queue;

    // volume
    ScalarType m_restshape_volume;
    ScalarType m_current_volume;

    // verbose
    bool m_verbose_show_converge;
    bool m_verbose_show_factorization_warning;
    bool m_verbose_show_energy;

    // constant term in optimization:
    // 0.5(x-y)^2 M (x-y) + (c) * h^2 * E(x) - h^2 * x^T * z;
    VectorX m_y;
    VectorX m_z;

    // external force (gravity, wind, etc...)
    VectorX m_external_force;

    // for optimization method, number of iterations
    unsigned int m_iterations_per_frame;
    // for optimization method
    unsigned int m_current_iteration;

    // line search 
    bool m_enable_line_search;
    ScalarType m_ls_alpha;
    ScalarType m_ls_beta;
    ScalarType m_ls_step_size;

    // prefetched instructions in linesearch
    bool m_ls_is_first_iteration;
    VectorX m_ls_prefetched_gradient;
    ScalarType m_ls_prefetched_energy;

    // local global method
    bool m_enable_openmp;
    VectorX m_last_descent_dir;

    // for prefactorization
    SparseMatrix m_weighted_laplacian_1D;
    SparseMatrix m_weighted_laplacian;

    bool m_precomputing_flag;
    bool m_prefactorization_flag;
    bool m_prefactorization_flag_newton;

    Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> m_prefactored_solver_1D;
    Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> m_prefactored_solver;
    Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> m_newton_solver;
    Eigen::ConjugateGradient<SparseMatrix> m_preloaded_cg_solver_1D;
    Eigen::ConjugateGradient<SparseMatrix> m_preloaded_cg_solver;

private:
    void clearConstraints(); // cleanup all constraints
    void setupConstraints(); // initialize constraints
    void dampVelocity(); // damp velocity at the end of each iteration.
    void calculateExternalForce(); // wind force is propotional to the area of triangles projected on the tangential plane
    
    void integrateImplicitMethod();

    void collisionDetection();
    void collisionSolve(VectorX& x);

    // all those "OneIteration" functions will be called in a loop
    // x is initially passed as the initial guess of the next postion (i.e. inertia term): x = y = current_pos + current_vel*h
    // x will be changed during these subroutines in EVERY iteration
    // the final value of x will be the next_pos that we used to update all vertices.
    bool performGradientDescentOneIteration(VectorX& x);
    bool performNewtonsMethodOneIteration(VectorX& x);
    bool performLBFGSOneIteration(VectorX& x);
    void LBFGSKernelLinearSolve(VectorX& r, VectorX gf_k, ScalarType scaled_identity_constant);

    // key initializations and constants computations
    void computeConstantVectorsYandZ();
    void updatePosAndVel(const VectorX& new_pos);

    // evaluate energy
    ScalarType evaluateEnergy(const VectorX& x);
    // evaluate gradient
    void evaluateGradient(const VectorX& x, VectorX& gradient, bool enable_omp = false);
    // evaluate gradient and energy
    ScalarType evaluateEnergyAndGradient(const VectorX& x, VectorX& gradient);
    // evaluate Hessian Matrix
    void evaluateHessian(const VectorX& x, SparseMatrix& hessian_matrix);
    // evaluate Weighted Laplacian Matrix
    void evaluateLaplacian(SparseMatrix& laplacian_matrix);
    // evaluate Weighted Laplacian Matrix nxn
    void evaluateLaplacian1D(SparseMatrix& laplacian_matrix_1d);

    // energy conservation
    ScalarType evaluatePotentialEnergy(const VectorX& x);
    ScalarType evaluateKineticEnergy(const VectorX& v);

    // basic building blocks
    ScalarType evaluateEnergyPureConstraint(const VectorX& x, const VectorX& f_ext);
    void evaluateGradientPureConstraint(const VectorX& x, const VectorX& f_ext, VectorX& gradient);
    ScalarType evaluateEnergyAndGradientPureConstraint(const VectorX& x, const VectorX& f_ext, VectorX& gradient);
    void evaluateHessianPureConstraint(const VectorX& x, SparseMatrix& hessian_matrix);
    void evaluateLaplacianPureConstraint(SparseMatrix& laplacian_matrix);
    void evaluateLaplacianPureConstraint1D(SparseMatrix& laplacian_matrix_1d);

    // line search
    ScalarType lineSearch(const VectorX& x, const VectorX& gradient_dir, const VectorX& descent_dir);
    ScalarType linesearchWithPrefetchedEnergyAndGradientComputing(const VectorX& x, const ScalarType current_energy, const VectorX& gradient_dir
        , const VectorX& descent_dir, ScalarType& next_energy, VectorX& next_gradient_dir);
    // matrices and prefactorizations
    void precomputeLaplacianWeights();
    void setWeightedLaplacianMatrix();
    void setWeightedLaplacianMatrix1D();
    void prefactorize();

    // volume
    ScalarType getVolume(const VectorX& x);

    // linear solver
    ScalarType linearSolve(VectorX& x, const SparseMatrix& A, const VectorX& b, char* msg = "");
    // conjugate gradient solver
    ScalarType conjugateGradientWithInitialGuess(VectorX& x, const SparseMatrix& A, const VectorX& b, const unsigned int max_it = 200, const ScalarType tol = 1e-5);
    // factorize matrix A using LLT decomposition
    void factorizeDirectSolverLLT(const SparseMatrix& A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver, char* warning_msg = ""); 
};
