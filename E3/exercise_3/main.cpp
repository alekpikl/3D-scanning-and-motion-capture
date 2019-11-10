#include <iostream>
#include <fstream>

#include "Eigen.h"
#include "SimpleMesh.h"
#include "ProcrustesAligner.h"


int alignBunnyWithProcrustes() {
	// Load the source and target mesh.
	// Make sure the paths are correct.
	const std::string filenameSource = std::string("../data/bunny/bunny.off");
	const std::string filenameTarget = std::string("../data/bunny/bunny_trans.off");

	SimpleMesh sourceMesh;
	if (!sourceMesh.loadMesh(filenameSource)) {
		std::cout << "Mesh file wasn't read successfully at location: " << filenameSource << std::endl;
		return -1;
	}

	SimpleMesh targetMesh;
	if (!targetMesh.loadMesh(filenameTarget)) {
		std::cout << "Mesh file wasn't read successfully at location: " << filenameTarget << std::endl;
		return -1;
	}

	// Fill in the matched points: sourcePoints[i] is matched with targetPoints[i].
	std::vector<Vector3f> sourcePoints; 
	sourcePoints.push_back(Vector3f(-0.0106867f, 0.179756f, -0.0283248f)); // left ear
	sourcePoints.push_back(Vector3f(-0.0639191f, 0.179114f, -0.0588715f)); // right ear
	sourcePoints.push_back(Vector3f(0.0590575f, 0.066407f, 0.00686641f)); // tail
	sourcePoints.push_back(Vector3f(-0.0789843f, 0.13256f, 0.0519517f)); // mouth
	
	std::vector<Vector3f> targetPoints;
	targetPoints.push_back(Vector3f(-0.02744f, 0.179958f, 0.00980739f)); // left ear
	targetPoints.push_back(Vector3f(-0.0847672f, 0.180632f, -0.0148538f)); // right ear
	targetPoints.push_back(Vector3f(0.0544159f, 0.0715162f, 0.0231181f)); // tail
	targetPoints.push_back(Vector3f(-0.0854079f, 0.10966f, 0.0842135f)); // mouth
		
	// Estimate the pose from source to target mesh with Procrustes alignment.
	ProcrustesAligner aligner;
	Matrix4f estimatedPose = aligner.estimatePose(sourcePoints, targetPoints);

	// Visualize the resulting joined mesh. We add triangulated spheres for point matches.
	SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, estimatedPose);
	for (const auto& sourcePoint : sourcePoints) {
		resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::sphere(sourcePoint, 0.002f), resultingMesh, estimatedPose);
	}
	for (const auto& targetPoint : targetPoints) {
		resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::sphere(targetPoint, 0.002f), resultingMesh, Matrix4f::Identity());
	}
	resultingMesh.writeMesh(std::string("../results/bunny_procrustes.off"));
	std::cout << "Resulting mesh written." << std::endl;
	
	return 0;
}


int main() 
{
	const auto result = alignBunnyWithProcrustes();

	return result;
}
