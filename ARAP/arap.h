#pragma once

#include "Eigen.h"
#include "OpenMeshHelper.h"

//#define USE_DENSE_SYSTEM_MATRIX

class ARAP
{
public:
	ARAP() : m_verticesBaseMesh(nullptr), m_verticesDeformed(nullptr), m_rotations(nullptr)
	{
	}

	~ARAP()
	{
		// free memory
		SAFE_DELETE_ARRAY(m_verticesBaseMesh);
		SAFE_DELETE_ARRAY(m_verticesDeformed);
		SAFE_DELETE_ARRAY(m_rotations);
	}

	void SetBaseMesh(const Mesh& baseMesh)
	{
		// free memory
		SAFE_DELETE_ARRAY(m_verticesBaseMesh);
		SAFE_DELETE_ARRAY(m_verticesDeformed);
		SAFE_DELETE_ARRAY(m_rotations);

		// set base mesh
		m_baseMesh = baseMesh;

		// mesh dimensions
		m_nVertices = (unsigned int) m_baseMesh.n_vertices();
		
		// allocate memory for vertices
		m_verticesBaseMesh = new Eigen::Vector3f[m_nVertices];
		m_verticesDeformed = new Eigen::Vector3f[m_nVertices];
		for (unsigned int i = 0; i < m_nVertices; i++)
		{
			VertexHandle vh(i);
			const Mesh::Point p = m_baseMesh.point(vh);
			m_verticesBaseMesh[i] = Eigen::Vector3f(p[0], p[1], p[2]);

			m_baseMesh.set_color(vh, Mesh::Color(0, 0, 0));
		}
		memcpy(m_verticesDeformed, m_verticesBaseMesh, sizeof(Eigen::Vector3f)*m_nVertices);

		// allocate memory for rotations
		m_rotations = new Eigen::Matrix3f[m_nVertices];

		// init system matrix
		InitSystemMatrix();
	}

	void DeformMesh(const std::vector<std::pair<unsigned int, Eigen::Vector3f>>& constraints, unsigned int nIter=10)
	{
		VERBOSE("Deform mesh ...");
		// reset to unmodified mesh
		memcpy(m_verticesDeformed, m_verticesBaseMesh, sizeof(Eigen::Vector3f)*m_nVertices);

		// apply constraints
		if (constraints.empty()) return;
		for (auto c : constraints)
		{
			m_verticesDeformed[c.first] = c.second;
		
			VertexHandle vh(c.first);
			m_baseMesh.set_color(vh, Mesh::Color(255, 0, 0));
		}

		// run flip-flop optimization
		ProgressBar("Deform mesh", 0.0f);
		for (unsigned int iter = 0; iter < nIter; ++iter)
		{
			// compute rotations
			SolveForRotations();
		
			// compute optimal vertex positions
			SolveForVertexPositions(constraints);

			ProgressBar("Deform mesh", iter / (nIter - 1.0f));
		}

		VERBOSE("Deform mesh ... DONE!");
	}

	Mesh GetDeformedMesh()
	{
		// copy base mesh
		Mesh result = m_baseMesh;

		// set deformed vertices
		for (unsigned int i = 0; i < m_nVertices; i++)
		{
			result.set_point(VertexHandle(i), Mesh::Point(m_verticesDeformed[i].x(), m_verticesDeformed[i].y(), m_verticesDeformed[i].z()));
		}

		return result;
	}

private:
	void InitSystemMatrix()
	{
		VERBOSE("Init system matrix ...");

		// allocate memory
#ifdef USE_DENSE_SYSTEM_MATRIX
		m_systemMatrix = Eigen::MatrixXf(m_nVertices, m_nVertices);
		m_systemMatrix.setZero();
#else
		m_systemMatrixSparse = Eigen::SparseMatrix<float>(m_nVertices, m_nVertices);
#endif
		m_rhsX = Eigen::VectorXf(m_nVertices);
		m_rhsY = Eigen::VectorXf(m_nVertices);
		m_rhsZ = Eigen::VectorXf(m_nVertices);

		// reset matrix
		m_rhsX.setZero();
		m_rhsY.setZero();
		m_rhsZ.setZero();

		// fill matrix
		for (unsigned int i = 0; i < m_nVertices; ++i)
		{
			m_systemMatrixSparse.insert(i, i) = 0.0f;
			for (Mesh::VertexVertexIter vv_it = m_baseMesh.vv_iter(VertexHandle(i)); vv_it; ++vv_it)
			{
				unsigned int j = vv_it.handle().idx();
				float w_ij = 1.0f;
#ifdef USE_DENSE_SYSTEM_MATRIX
				m_systemMatrix(i, i) += w_ij;
				m_systemMatrix(i, j) = -w_ij;
#else
				m_systemMatrixSparse.coeffRef(i, i) += w_ij;
				m_systemMatrixSparse.insert(i, j) = -w_ij;
#endif
			}
		}

		VERBOSE("Init system matrix ... DONE!");
	}

	void ComputeRightHandSide()
	{
		VERBOSE("Compute right hand side ...");

		// reset rhs
		m_rhsX.setZero();
		m_rhsY.setZero();
		m_rhsZ.setZero();

		#pragma omp parallel for
		for (int i = 0; i < (int)m_nVertices; ++i)
		{
			Eigen::Vector3f sum(0.0f, 0.0f, 0.0f);
			for (Mesh::VertexVertexIter vv_it = m_baseMesh.vv_iter(VertexHandle(i)); vv_it; ++vv_it)
			{
				unsigned int j = vv_it.handle().idx();
				float w_ij = 1.0f;
				sum += w_ij / 2.0f * (m_rotations[i] + m_rotations[j]) * (m_verticesBaseMesh[i] - m_verticesBaseMesh[j]);
			}
			m_rhsX(i) = sum.x();
			m_rhsY(i) = sum.y();
			m_rhsZ(i) = sum.z();
		}

		VERBOSE("Compute right hand side ... DONE!");
	}

	void SolveForRotations()
	{
		VERBOSE("Solve for rotations ...");

		#pragma omp parallel for
		for (int i = 0; i < (int)m_nVertices; ++i)
		{
			std::vector<Eigen::Vector3f> basePositions;
			std::vector<Eigen::Vector3f> currentPositions;

			for (Mesh::VertexVertexIter vv_it = m_baseMesh.vv_iter(VertexHandle(i)); vv_it; ++vv_it)
			{
				unsigned int j = vv_it.handle().idx();
				basePositions.push_back(m_verticesBaseMesh[i] - m_verticesBaseMesh[j]);
				currentPositions.push_back(m_verticesDeformed[i] - m_verticesDeformed[j]);
			}

			m_rotations[i] = Procrustes(currentPositions, basePositions, false, false).block<3, 3>(0, 0);
		}

		VERBOSE("Solve for rotations ... DONE!");
	}

	void SolveForVertexPositions(const std::vector<std::pair<unsigned int, Eigen::Vector3f>>& constraints)
	{
		VERBOSE("Solve for rotations ...");

		// compute rhs
		ComputeRightHandSide();

		// solve system
#ifdef USE_DENSE_SYSTEM_MATRIX
		Eigen::MatrixXf systemMatrix = m_systemMatrix;
#else
		Eigen::SparseMatrix<float> systemMatrixSparse = m_systemMatrixSparse;
#endif

		// adjust system matrix
		for (auto c : constraints)
		{
			// adapt rhs
			for (unsigned int i = 0; i < m_nVertices; ++i)
			{
#ifdef USE_DENSE_SYSTEM_MATRIX
				m_rhsX(i) -= systemMatrix(i, c.first) * c.second.x();
				m_rhsY(i) -= systemMatrix(i, c.first) * c.second.y();
				m_rhsZ(i) -= systemMatrix(i, c.first) * c.second.z();
#else
				if (systemMatrixSparse.coeff(i, c.first) != 0.0f)
				{
					m_rhsX(i) -= systemMatrixSparse.coeff(i, c.first) * c.second.x();
					m_rhsY(i) -= systemMatrixSparse.coeff(i, c.first) * c.second.y();
					m_rhsZ(i) -= systemMatrixSparse.coeff(i, c.first) * c.second.z();
				}
#endif
			}
			m_rhsX(c.first) = c.second.x();
			m_rhsY(c.first) = c.second.y();
			m_rhsZ(c.first) = c.second.z();

			// delete row and column
#ifdef USE_DENSE_SYSTEM_MATRIX
			for (unsigned int i = 0; i < m_nVertices; ++i) systemMatrix(c.first, i) = systemMatrix(i, c.first) = 0.0f;
			systemMatrix(c.first, c.first) = 1.0f;
#else
			for (unsigned int i = 0; i < m_nVertices; ++i) if(systemMatrixSparse.coeff(c.first,i) != 0.0f) systemMatrixSparse.coeffRef(c.first, i) = systemMatrixSparse.coeffRef(i, c.first) = 0.0f;
			systemMatrixSparse.coeffRef(c.first, c.first) = 1.0f;
#endif

		}

#ifdef USE_DENSE_SYSTEM_MATRIX
		static Eigen::JacobiSVD<Eigen::MatrixXf> svd(systemMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
#else
		/*static*/ Eigen::SimplicialCholesky<Eigen::SparseMatrix<float>> svd(systemMatrixSparse);
#endif
		Eigen::VectorXf x = svd.solve(m_rhsX);
		Eigen::VectorXf y = svd.solve(m_rhsY);
		Eigen::VectorXf z = svd.solve(m_rhsZ);

		for (unsigned int i = 0; i < m_nVertices; ++i)
		{
			m_verticesDeformed[i] = Eigen::Vector3f(x(i), y(i), z(i));
		}

		VERBOSE("Solve for rotations ... DONE!");
	}

	void ProgressBar(std::string titel, float progress)
	{
		int barWidth = 70;

		std::cout << titel << " [";
		int pos = int(barWidth * progress);
		for (int i = 0; i < barWidth; ++i) {
			if (i < pos) std::cout << "=";
			else if (i == pos) std::cout << ">";
			else std::cout << " ";
		}
		std::cout << "] " << int(progress * 100.0) << " %\r";
		std::cout.flush();

		if (progress == 1.0f)	std::cout << std::endl;
	}

	// base mesh
	Mesh m_baseMesh;

	// nVertices
	unsigned int m_nVertices;

	// vertices of the base mesh
	Eigen::Vector3f* m_verticesBaseMesh;

	// vertices of the current deformed mesh
	Eigen::Vector3f* m_verticesDeformed;

	// rotations of the vertices
	Eigen::Matrix3f* m_rotations;

	// system matrix
#ifdef USE_DENSE_SYSTEM_MATRIX
	Eigen::MatrixXf m_systemMatrix;
#else
	Eigen::SparseMatrix<float> m_systemMatrixSparse;
#endif

	// right hand side
	Eigen::VectorXf m_rhsX;
	Eigen::VectorXf m_rhsY;
	Eigen::VectorXf m_rhsZ;
};