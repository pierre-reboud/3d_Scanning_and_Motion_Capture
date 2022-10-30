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

void check_dist(int& i, Vertex* vertices, bool *returns, float& edgeThreshold, unsigned int& width){
	// Sets the two elements of the returns array to true if both faces have 
	// sufficiently short edges. Else, set element to false.
	returns[0] = true;
	returns[1] = true;
	float norms[5] = {
		(vertices[i].position - vertices[i+1].position).norm(), 
		(vertices[i].position - vertices[i+width].position).norm(),
		(vertices[i+width].position - vertices[i+1].position).norm(),
		(vertices[i+width].position - vertices[i+width+1].position).norm(),
		(vertices[i+1].position - vertices[i+width+1].position).norm()
	};
	if (norms[2] > edgeThreshold){
		returns[0] = false;
		returns[1] = false;
	}
	else {
		if (norms[0] > edgeThreshold || norms[1] > edgeThreshold){
			returns[0] = false;
		}
		if (norms[3] > edgeThreshold || norms[4] > edgeThreshold){
			returns[1] = false;
		}
	}
}

void check_isinf(int& i, Vertex* vertices, bool *returns, unsigned int& width){
	returns[0] = true;
	if (std::isinf(std::abs(vertices[i].position.maxCoeff()))) {
		returns[0] = false;
	}
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
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width*height;//0;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0;

	// Write off file
	std::ofstream outFile(filename);
	std::stringstream faces_stream, vertices_stream;
	if (!outFile.is_open()) return false;

	// faces_stream << "# list of faces" << std::endl;
	// faces_stream << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;
	

	bool return_faces[2];

	for (int i = 0; i<(height-1)*(width-1); i++){
		check_dist(i, vertices, return_faces, edgeThreshold, width);
		if (return_faces[0] == true){
			faces_stream << 3 << " " << i << " " << i + width << " " << i + 1 << std::endl;
			++nFaces;
		}
		if (return_faces[1] == true){
			faces_stream << 3 << " " << i + width << " " << i + width + 1 << " " << i + 1 << std::endl;
			++nFaces;
		}
	}


	// write header
	vertices_stream << "COFF" << std::endl;

	// vertices_stream << "# numVertices numFaces numEdges" << std::endl;

	vertices_stream << nVertices << " " << nFaces << " 0" << std::endl;

	// vertices_stream << "# list of vertices" << std::endl << "# X Y Z R G B A" << std::endl;

	for (int i = 0; i<height*width; i++){
		check_isinf(i, vertices, return_faces,width);
		if (return_faces[0]){
			for (int j = 0; j<3; j++){
				vertices_stream << vertices[i].position[j] << " ";
			}
		}

		else{
			vertices_stream <<"0 0 0 ";
		}
		for (int j = 0; j<3; j++){
			vertices_stream << (unsigned int) vertices[i].color[j] << " ";
		}
		vertices_stream << (unsigned int) vertices[i].color[3] << std::endl;
	}

	// TODO: save valid faces
	outFile << vertices_stream.str();
	outFile << faces_stream.str();
	

	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../Data/rgbd_dataset_freiburg1_xyz/";
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
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		for (int idx = 0; idx<sensor.GetDepthImageWidth()*sensor.GetDepthImageHeight(); idx++){
			float z_prime = depthMap[idx];
			Vector3f image_point = Vector3f(idx % sensor.GetDepthImageWidth(), idx / sensor.GetDepthImageHeight(), 1.0f) * z_prime;
			Vector3f extrinsic_point = depthIntrinsicsInv*image_point;
			Vector4f homogeneous_point = Vector4f(extrinsic_point[0], extrinsic_point[1], extrinsic_point[2], 0.0f);
			Vector4f world_point = trajectoryInv*depthExtrinsicsInv*homogeneous_point;
			world_point[3] = 1.0f;
			vertices[idx].position = world_point;
			vertices[idx].color[0] = colorMap[idx*4];
			vertices[idx].color[1] = colorMap[idx*4+1];
			vertices[idx].color[2] = colorMap[idx*4+2];
			vertices[idx].color[3] = colorMap[idx*4+3];
			if(depthMap[idx] == MINF || std::isnan(std::abs(world_point.maxCoeff()))){
				// || std::isnan(std::abs(world_point.maxCoeff()))
				vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
				vertices[idx].color = Vector4uc(0, 0, 0, 0);
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