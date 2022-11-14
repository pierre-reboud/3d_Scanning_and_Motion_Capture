#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);
		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		
		Matrix4f estimatedPose = Matrix4f::Zero();
		estimatedPose.block(0,0,3,3) = rotation;
		estimatedPose.block(0,3,3,1) = translation;
		estimatedPose(3,3) = 1.0f;

		return estimatedPose;
	}


private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		// Hint: You can use the .size() method to get the length of a vector.
		Vector3f mean = Vector3f::Zero();
		for (auto& point : points){
			mean += point;
		}
		mean /= points.size();
		return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Hint: You can initialize an Eigen matrix with "MatrixXf m(num_rows,num_cols);" and access/modify parts of it using the .block() method (see above).

		MatrixXf m = MatrixXf::Zero(sourcePoints[0].size(), targetPoints[0].size()), crosscov = MatrixXf::Zero(sourcePoints[0].size(), targetPoints[0].size());
		for(size_t i = 0; i<sourcePoints.size();i++){
			crosscov += (targetPoints[i] - targetMean) * (sourcePoints[i] - sourceMean).transpose(); 
		}
		// Not necessary due to only using U and V of SVD.
		crosscov /= sourcePoints.size();
		Eigen::JacobiSVD<MatrixXf> svd(crosscov, ComputeFullV | ComputeFullU);
		std::cout << svd.computeU() << std::endl;
		std::cout << svd.computeV() << std::endl;
		Matrix3f rotation = svd.matrixU()*svd.matrixV().transpose();
		if (rotation.determinant() == -1.0f){
			Matrix3f help = Matrix3f::Identity();
			help(2,2) = -1.0f;
			rotation = svd.matrixU() * help * svd.matrixV().transpose();
		}
        return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.

		Vector3f translation = Vector3f::Zero();
		translation = - rotation * sourceMean + targetMean;
        return translation;
	}
};
