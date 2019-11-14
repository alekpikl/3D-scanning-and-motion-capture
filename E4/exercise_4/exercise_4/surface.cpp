#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct SurfaceCostFunction
{

};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// TODO: Read 3D surface data points and define the parameters of the problem
	const std::string file_path = "../data/points_surface.txt";

	ceres::Problem problem;

	// TODO: For each data point create one residual block


	ceres::Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;
	
	// TODO: Output the final values of the parameters


	system("pause");
	return 0;
}