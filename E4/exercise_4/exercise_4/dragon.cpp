#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct RegistrationCostFunction
{

};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// TODO: Read data points and the weights. Define the parameters of the problem
	const std::string file_path_1 = "../data/points_dragon_1.txt";
	const std::string file_path_2 = "../data/points_dragon_2.txt";
	const std::string file_path_weights = "../data/weights_dragon.txt";


	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block


	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// TODO: Output the final values of the translation and rotation (in degree)

	system("pause");
	return 0;
}