#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct SurfaceCostFunction
{
	SurfaceCostFunction(const Point3D& point_)
		: point(point_)
	{
	}

	template<typename T>
	bool operator()(const T* const a, const T* const b, const T* const c, T* residual) const
	{
		// TODO: Implement the cost function
		T _c = *c;
		T _b = *b;
		T _a = *a;
		residual[0] = _c * point.z - T(1.) / _a * point.x * point.x + T(1.) / _b * point.y * point.y;
		return true;
	}

private:
	const Point3D point;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// TODO: Read 3D surface data points and define the parameters of the problem
	const std::string file_path = "../../data/points_surface.txt";
	const auto points = read_points_from_file<Point3D>(file_path);
	ceres::Problem problem;

	const double a_init  = 1.;
	const double b_init  = 1.;
	const double c_init  = 1.;

	double a = a_init;
	double b = b_init;
	double c = c_init;

	// TODO: For each data point create one residual block
	// For each data point create one residual block
	for (auto& point : points)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<SurfaceCostFunction, 1, 1, 1, 1>(
				new SurfaceCostFunction(point)),
			nullptr, &a, &b, &c
		);
	}

	ceres::Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;
	
	// TODO: Output the final values of the parameters
	std::cout << "Initial a: " << a_init << "\tb: " << b_init << "\tc: " << c_init << std::endl;
	std::cout << "Final a: " << a << "\tb: " << b << "\tc: " << c << std::endl;

	return 0;
}