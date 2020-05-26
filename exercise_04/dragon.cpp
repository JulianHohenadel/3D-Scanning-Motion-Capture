#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct RegistrationCostFunction
{
    RegistrationCostFunction(const Point2D& bf_trans_, const Point2D& af_trans_, const Weight& weight_i_)
        : p_i(bf_trans_), q_i(af_trans_), w_i(weight_i_)
    {
    }

    template<typename T>
        bool operator()(const T* const deg, const T* const tx, const T* const ty, T* residual) const {
            T rad = *deg * T(M_PI/180.0); 
            T Tp_ix = T(cos(rad) * p_i.x - sin(rad) * p_i.y) + *tx; 
            T Tp_iy = T(sin(rad) * p_i.x + cos(rad) * p_i.y) + *ty;
            residual[0] = T(w_i.w * (pow(Tp_ix - q_i.x, 2.0) + pow(Tp_iy - q_i.y, 2.0))); 

            return true;
        }

private:
    const Point2D p_i;
    const Point2D q_i;
    const Weight w_i; 
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// TODO: Read data points and the weights. Define the parameters of the problem
	const std::string file_path_1 = "../data/points_dragon_1.txt";
	const std::string file_path_2 = "../data/points_dragon_2.txt";
	const std::string file_path_weights = "../data/weights_dragon.txt";

   const auto bf_trans = read_points_from_file<Point2D>(file_path_1);
   const auto af_trans = read_points_from_file<Point2D>(file_path_2);
   const auto weight_i = read_points_from_file<Weight>(file_path_weights);

   const double deg_initial = 45;
   const double tx_initial = 1250;
   const double ty_initial = 500;

   double deg = deg_initial;
   double tx = tx_initial;
   double ty = ty_initial;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block
    for(int i = 0; i < bf_trans.size(); i++)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(
                new RegistrationCostFunction(bf_trans[i], af_trans[i], weight_i[i])),
                    nullptr, &deg, &tx, &ty  
        ); 
    }

	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// TODO: Output the final values of the translation and rotation (in degree)
    std::cout << "Initial deg: " << deg_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
	std::cout << "Final deg: " << deg << "\ttx: " << tx << "\tty: " << ty << std::endl;
	std::cout << "python3 plot_dragon.py --deg " << deg << " --tx " << tx << " --ty " << ty << std::endl;
	
	return 0;
}
