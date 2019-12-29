#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"

#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

// Returns true, if a triangle, defined by the idx1/2/3 is valid: edge smaller than edgeThreshold.
bool ValidTriangle(Vertex* vertices, unsigned idx1, unsigned idx2, unsigned idx3, float edgeThreshold) {
	if ( (vertices[idx1].position - vertices[idx2].position).norm() < edgeThreshold 
		&& (vertices[idx1].position - vertices[idx3].position).norm() < edgeThreshold 
		&& (vertices[idx2].position - vertices[idx3].position).norm() < edgeThreshold) 
	{
		return true;
	}
	return false;
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller than edgeThreshold
	
	// TODO: Get number of vertices
	unsigned int nVertices = width * height;
	
	// TODO: Determine number of valid faces
	unsigned nFaces = 0;
	unsigned maxNFaces = (width-1)*2 * (height-1);
	std::vector<std::string> faces = std::vector<std::string>(maxNFaces);
	for (int row = 0; row < height-1; ++row) {
		for (int col = 0; col < width-1; ++col)
		{
			unsigned idx1 = row*width + col ; // 0
			unsigned idx2 = (row + 1)*width + col ; // 5
			unsigned idx3 = row*width + col + 1; // 1 
			unsigned idx4 = (row + 1)*width + col + 1; // 6
			if (ValidTriangle(vertices, idx1, idx2, idx3, edgeThreshold))
			{
				faces[nFaces] = std::to_string(3) + " " + std::to_string(idx1) + " " + std::to_string(idx2) + " " + std::to_string(idx3);
				++nFaces;
			}
			if (ValidTriangle(vertices, idx2, idx3, idx4, edgeThreshold))
			{
				faces[nFaces] = std::to_string(3) + " " + std::to_string(idx2) + " " + std::to_string(idx4) + " " + std::to_string(idx3);
				++nFaces;			
			}
		}
	}

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for (int idx = 0; idx < width * height; ++idx) {
		if (vertices[idx].position[0] != MINF && vertices[idx].position[1] != MINF && vertices[idx].position[2] != MINF){
			outFile << vertices[idx].position[0] 
					<< " " << vertices[idx].position[1] 
					<< " " << vertices[idx].position[2] 
					<< " " << (int)vertices[idx].color[0]
					<< " " << (int)vertices[idx].color[1] 
					<< " " << (int)vertices[idx].color[2]
					<< " " << (int)vertices[idx].color[3]
					<< std::endl;	
		} else {
			outFile << 0.0 << " " << 0.0 << " "
					<< 0.0 << " " << (int)vertices[idx].color[0]
					<< " " << (int)vertices[idx].color[1] 
					<< " " << (int)vertices[idx].color[2]
					<< " " << (int)vertices[idx].color[3]
					<< std::endl;	
		}
	}
	

	// TODO: save valid faces
	for (unsigned idx = 0; idx < nFaces; ++idx) {
		outFile << faces[idx] << std::endl;
	}
	
	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		Matrix3f depthIntrinsicsInv = sensor.GetDepthIntrinsics().inverse();
		MatrixXf extendedId = MatrixXf::Identity(4,3);
		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap

		/* NOTE: have depth and color information for pixels in an image. Need to back-project this image-pixel to get the coordinates for the 
		 real-world points (where the pixels are in the real-world coordinate frame) and then write the color/depth values to the vertices. 
		 Those vertices then contruct a mesh - 3D object. */
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		for (int row = 0; row < sensor.GetDepthImageHeight() ; ++row) {
			for (int col = 0; col < sensor.GetDepthImageWidth(); ++col) {
				int idx = row * sensor.GetDepthImageWidth() + col;
				// std::cout << idx << std::endl;
				if (depthMap[idx] == MINF) {
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0,0,0,0);
				} else {
					// apply back-projection. 
					// TODO Depth is multiplied by 5000. Should I scale it down?
					// Vector4f p_camera = depthIntrinsicsInv * p_image; -> don't have depthIntrinsicInv. Using 
					// https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
					float scaling_factor = 1.0;
					float depth = depthMap[idx];
					Vector3f p_image = Vector3f(col * depth, row * depth, depth);
					Vector4f p_camera = extendedId * depthIntrinsicsInv * p_image;
					// Now I have a 3D Point Cloud from a 2D image and a depth Map.
					// Next, transform the point from the local to the world coordinate system.
					// get camera pose -> trajectoryInv
					Vector4f p_world = trajectoryInv * depthExtrinsicsInv * p_camera;
					p_world[3] = 1.0f; // Does this make sense? Is it necessary? What does it do anyway?
					vertices[idx].position = p_world;
					// NOTE color is saved in an array of dim 4 * height * width -> 4 values for each pixel
					vertices[idx].color[0] = colorMap[4*idx + 0];
					vertices[idx].color[1] = colorMap[4*idx + 1];
					vertices[idx].color[2] = colorMap[4*idx + 2];
					vertices[idx].color[3] = colorMap[4*idx + 3];
				}
			}
		}


		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}
