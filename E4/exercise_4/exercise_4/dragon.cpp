#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct RegistrationCostFunction
{
	RegistrationCostFunction(const Point2D& p_, const Point2D& q_, const Weight& w_)
		: p(p_), w(w_), q(q_)
	{
	}

	template<typename T>
	bool operator()(const T* const tx, const T* const ty, const T* const theta, T* residual) const
	{
		// TODO: Implement the cost function
		T tx_ = *tx;
		T ty_ = *ty;
		T theta_ = *theta;
		T tmp1 = ceres::cos(theta_)*p.x - ceres::sin(theta_)*p.y + tx_ - q.x;
		T tmp2 = ceres::sin(theta_)*p.x + ceres::cos(theta_)*p.y + ty_ - q.y;
		residual[0] = w.w * (tmp1*tmp1 + tmp2*tmp2);
		return true;
	}

private:
	const Point2D p;
	const Point2D q;
	const Weight w;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// TODO: Read data points and the weights. Define the parameters of the problem
	const std::string file_path_1 = "../../data/points_dragon_1.txt";
	const std::string file_path_2 = "../../data/points_dragon_2.txt";
	const std::string file_path_weights = "../../data/weights_dragon.txt";
	const auto points_1 = read_points_from_file<Point2D>(file_path_1);
	const auto points_2 = read_points_from_file<Point2D>(file_path_2);
	const auto weights = read_points_from_file<Weight>(file_path_weights);

	double theta = 0.0;
	double tx 	 = 0.0;
	double ty	 = 0.0;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block
	for (size_t i = 0; i < points_1.size(); ++i) {
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1> (
				new RegistrationCostFunction(points_1[i], points_2[i], weights[i])), 
			nullptr, &tx, &ty, &theta
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
	// TODO: is it in degree
	std::cout << "Final theta: " << theta*180.0/3.14 << "\ttx: " << tx << "\tty: " << ty << std::endl;

	system("pause");
	return 0;
}