#pragma once
#include <Eigen/Eigen>
#include "Mesh.h"

using MyMesh = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;


class HRBF {


public:
	HRBF() {}
	void Calculate(std::vector<MyMesh::Point>& points, std::vector<MyMesh::Normal>& normal)
	{
		int n = points.size();
		_node_centers.resize(3, n);
		_betas.resize(3, n);
		_alphas.resize(n);

		A.resize(4 * n, 4 * n);
		X.resize(4 * n);
		B.resize(4 * n);
		
		for (int i = 0; i < n; i++)
		{
			Eigen::Vector<float, 3> vec;
			vec(0) = points[i][0];
			vec(1) = points[i][1];
			vec(2) = points[i][2];
			_node_centers.col(i) = vec;
		}
		for (int i = 0; i < n; ++i)
		{
			Eigen::Vector<float, 3> po;
			po(0) = points[i][0];
			po(1) = points[i][1];
			po(2) = points[i][2];
			Eigen::Vector<float, 3> no;
			no(0) = normal[i][0];
			no(1) = normal[i][1];
			no(2) = normal[i][2];
			int io = (3 + 1) * i;
			B(io) = 0;
			B.template segment<3>(io + 1) = no;
			for (int j = 0; j < n; j++)
			{
				int jo = (3 + 1) * j;
				Eigen::Vector<float, 3> diff = po - _node_centers.col(j); // d: x-pi
				float l = diff.norm();
				if (l == 0) {
					A.template block<3 + 1, 3 + 1>(io, jo).setZero();
				}
				else {

					//float ci = c(po, _node_centers.col(j));
					float w =  l * l * l;//x^3
					float dw_l = 3 * l * l;//3*x^2
					float ddw = 6 * l;//6x
					Eigen::Vector<float, 3> g = diff * dw_l;
					A(io, jo) = w;
					A.row(io).template segment<3>(jo + 1) = g.transpose();
					A.col(jo).template segment<3>(io + 1) = g;
					A.template block<3, 3>(io + 1, jo + 1) = (ddw - dw_l) / (l * l) * (diff * diff.transpose());
					A.template block<3, 3>(io + 1, jo + 1).diagonal().array() += dw_l;

				}

			}


		}
		auto L = eigenToStdVector(A);
		X = A.lu().solve(B);
		Eigen::Map< Eigen::Matrix<float, 3 + 1, Eigen::Dynamic> > mx(X.data(), 3 + 1, n);

		_alphas = mx.row(0);
		_betas = mx.template bottomRows<3>();

		std::vector<float> AL(B.size());

		for (int i = 0;i< B.size(); i++)
		{
			AL[i]= B(i);
		}

		
	}


	float c(Eigen::Vector<float, 3> x, Eigen::Vector<float, 3> pi) {
		Eigen::Vector<float, 3> diff = x - pi; // d: x-pi
		float l = diff.norm();
		float pow2 = l*l;
		float first_part = 1 / pow2;
		float second_derivative = 6 * l; // 6x
		float first_derivative = 3 * pow2;// 3x^2
		float second_part = second_derivative - (first_derivative / l);

		return first_part * second_part;

	}


	std::vector<std::vector<float>> eigenToStdVector(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& eigenMatrix) {
		std::vector<std::vector<float>> result(eigenMatrix.rows(), std::vector<float>(eigenMatrix.cols()));
		for (int i = 0; i < eigenMatrix.rows(); ++i) {
			for (int j = 0; j < eigenMatrix.cols(); ++j) {
				result[i][j] = eigenMatrix(i, j);
			}
		}
		return result;
	}


	float eval(const MyMesh::Point& x) const
	{
		float ret = 0;
		int nb_nodes = _node_centers.cols();

		Eigen::Vector<float, 3> xi;
		xi(0) = x[0];
		xi(1) = x[1];
		xi(2) = x[2];

		for (int i = 0; i < nb_nodes; ++i)
		{
			Eigen::Vector<float, 3> diff = xi - _node_centers.col(i);
			float l = diff.norm();

			if (l > 0)
			{
				ret += _alphas(i) * (l * l * l);
				ret += _betas.col(i).dot(diff) * 3*(l * l) / l;
			}
		}
		return ret;
	}

	MyMesh::Point grad(const MyMesh::Point& x) const
	{
		Eigen::Vector<float, 3> xi;
		xi(0) = x[0];
		xi(1) = x[1];
		xi(2) = x[2];
		Eigen::Vector<float, 3> grad = Eigen::Vector<float, 3>::Zero();
		int nb_nodes = _node_centers.cols();
		for (int i = 0; i < nb_nodes; i++)
		{
			Eigen::Vector<float, 3>  node = _node_centers.col(i);
			Eigen::Vector<float, 3>  beta = _betas.col(i);
			float  alpha = _alphas(i);
			Eigen::Vector<float, 3>  diff = xi - node;

			Eigen::Vector<float, 3> diffNormalized = diff;
			float l = diff.norm();

			if (l > 0.00001f)
			{
				diffNormalized.normalize();
				float dphi = 3*(l*l);
				float ddphi = 6*(l);

				float alpha_dphi = alpha * dphi;

				float bDotd_l = beta.dot(diff) / l;
				float squared_l = diff.squaredNorm();

				grad += alpha_dphi * diffNormalized;
				grad += bDotd_l * (ddphi * diffNormalized - diff * dphi / squared_l) + beta * dphi / l;
			}
		}
		MyMesh::Point p;
		p[0] = grad(0);
		p[1] = grad(1);
		p[2] = grad(2);
		return p;
	}




	Eigen::Matrix<float, 3, Eigen::Dynamic> _node_centers;
	Eigen::Matrix<float, 3, Eigen::Dynamic>_betas;
	Eigen::Matrix<float, Eigen::Dynamic, 1> _alphas;

	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A;
	Eigen::Matrix<float, Eigen::Dynamic,1> X;
	Eigen::Matrix<float, Eigen::Dynamic,1> B;
};
