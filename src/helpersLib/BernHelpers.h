#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <memory>
#include <stdexcept>
#include <nanovg.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::MatrixXd;
typedef Eigen::SparseMatrix<double> SparseMatrixd;
using Eigen::Triplet;
typedef Eigen::SimplicialLDLT<SparseMatrixd, Eigen::Lower> Solver;
using std::cout;
using std::endl; 
using std::string;
using std::vector;
using std::pair;
using std::array;
using std::min;
using std::max;
template<typename T> void pop_front(std::vector<T>& vec) { vec.erase(vec.begin()); }
template<typename T> void push_back_pop_front(std::vector<T> &vec, const T &el) { vec.push_back(el); pop_front(vec); }
template<typename T> void concat_in_place(vector<T> &modify_me, const vector<T> &vector2) { modify_me.insert(modify_me.end(), vector2.begin(), vector2.end()); }
const auto min_element_wrapper = [](const auto &v) { return *std::min_element(v.begin(), v.end()); }; 
const auto max_element_wrapper = [](const auto &v) { return *std::max_element(v.begin(), v.end()); };
static void toggle(bool &b) { b = !b; }; 
static bool setFalseButReturnOriginalValue(bool &b) { bool ret = b; b = false; return ret; }
static bool  setTrueButReturnOriginalValue(bool &b) { bool ret = b; b = true; return ret; } 
double PI = 3.14159265358979323846264338327;
NVGcolor RED    = nvgRGB(249,  38, 114);
NVGcolor ORANGE = nvgRGB(253, 151,  31);
NVGcolor YELLOW = nvgRGB(230, 219, 116);
NVGcolor GREEN  = nvgRGB(166, 226,  46);
NVGcolor BLUE   = nvgRGB(102, 217, 239);
NVGcolor PURPLE = nvgRGB(174, 129, 255);
NVGcolor GRAY   = nvgRGB( 39,  40,  34); 
NVGcolor BLACK  = nvgRGB(  0,   0,   0); 
NVGcolor WHITE  = nvgRGB(255, 255, 255); 
NVGcolor DARK_RED  = nvgRGB(249 / 2,  38 / 2, 114 / 2);
NVGcolor DARK_YELLOW = nvgRGB(230 / 2, 219 / 2, 116 / 2);
NVGcolor DARK_BLUE   = nvgRGB(102 / 2, 217 / 2, 239 / 2); 
NVGcolor LIGHT_RED  = nvgRGB(252, 147, 185);
bool isNaN(const double &x) { return (x != x); } 
double clamp(const double &a, const double left, const double right) { if (a < left) { return left; } else if (a > right) { return right; } else { return a; } }; 
double clamp01(const double &a) { return clamp(a, 0., 1.); }; 
Vector2d e_theta(const double &theta) { return Vector2d(cos(theta), sin(theta)); };
double atan2_wrapper(const Vector2d &xy) { return atan2(xy.y(), xy.x()); };
Vector2d from_(const Vector2d &a, const Vector2d &b) { return b - a; }
Vector2d rotate_(const Vector2d &p, const double &theta) { return Vector2d(p[0] * cos(theta) - p[1] * sin(theta), p[1] * cos(theta) + p[0] * sin(theta)); }; 
template<typename T> T lerp(const double &f, const T &p, const T &q){ return p + (q - p)*f; }; 
double inverseLerp(const double x, const double left, const double right) { return (x - left) / (right - left); }; 
template<typename ... Args> string string_format(const std::string& format, Args ... args) {
	// https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf#comment61134428_2342176 
	unsigned int size = snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
	std::unique_ptr<char[]> buf(new char[size]);
	snprintf(buf.get(), size, format.c_str(), args ...);
	return string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
} 

template<typename T> vector<T> linspace(const int N, const T &left, const T &right) {
	if (N == 1) { return vector<T>({ left }); }
	vector<T> X;
	for (int i = 0; i < N; ++i) {
		double f = double(i) / (N - 1);
		T x = lerp<T>(f, left, right);
		X.push_back(x);
	}
	return X;
} 

NVGcolor colorMap(double f) {
	const auto clamp_access = [](const vector<Eigen::Vector3d> &v, int i) { return v[std::min(std::max(i, 0), int(v.size() - 1))]; }; // FORNOW
	vector<Eigen::Vector3d> plasma = { { 0.050383,0.029803,0.527975 },{ 0.171574,0.019706,0.580806 },{ 0.261183,0.013308,0.617911 },{ 0.35015,0.004382,0.646298 },{ 0.429719,0.000831,0.659425 },{ 0.512206,0.018833,0.655209 },{ 0.584391,0.068579,0.632812 },{ 0.65558,0.129725,0.592317 },{ 0.714883,0.187299,0.546338 },{ 0.771958,0.249237,0.494813 },{ 0.819651,0.306812,0.448306 },{ 0.866078,0.36966,0.400126 },{ 0.904601,0.429797,0.356329 },{ 0.940313,0.497642,0.309197 },{ 0.966798,0.564396,0.265118 },{ 0.986345,0.640969,0.217948 },{ 0.994324,0.716681,0.177208 },{ 0.989587,0.803205,0.146529 },{ 0.970533,0.887896,0.145919 },{ 0.940015,0.975158,0.131326 } };
	int N = plasma.size(); 
	f = clamp01(f); 
	double i_plus_frac = f * (N - 1);
	int i = (int)floor(i_plus_frac);
	double g = i_plus_frac - i; 
	auto L = clamp_access(plasma, i);
	auto R = clamp_access(plasma, i + 1);
	auto tmp = lerp(g, L, R);
	auto Q = [](const double &c) {return int(255 * clamp01(c)); };
	return nvgRGB(Q(tmp[0]), Q(tmp[1]), Q(tmp[2]));
} 

vector<double> VectorXd2vecDub(const VectorXd &in) {
	vector<double> out;
	for (int i = 0; i < in.size(); ++i) {
		out.push_back(in[i]);
	}
	return out;
}

VectorXd vecDub2VectorXd(const vector<double> &in) {
	VectorXd out; out.setZero(in.size());
	for (int i = 0; i < in.size(); ++i) {
		out[i] = in[i];
	}
	return out;
}


VectorXd stack_(const vector<VectorXd> &in) { 
	int N = 0; for (auto &dvec : in) { N += int(dvec.size()); }
	VectorXd out; out.setZero(N); 
	int o = 0;
	for (const auto &v : in) {
		out.segment(o, int(v.size())) = v;
		o += int(v.size());
	} 
	return out; 
}

#define seg2(y, i) y.segment<2>(2 * i)

const auto vecFD = [](VectorXd s0, auto O_of_s, double d = 1e-5) -> VectorXd {
	// ~ dOds|s0
	int N = int(s0.size());
	VectorXd dOds; dOds.setZero(N); 
	for (int i = 0; i < N; ++i) {
		double s0i = s0[i]; 
		s0[i] -= d; double lft = O_of_s(s0); s0[i] = s0i; 
		s0[i] += d; double ryt = O_of_s(s0); s0[i] = s0i; 
		dOds[i] = (ryt - lft) / (2. * d);
	}
	return dOds;
};

const auto matFD = [](VectorXd s0, auto G_of_s, double d = 1e-5) -> MatrixXd {
	// ~ dGds|s0 
	int R = int(G_of_s(s0).size());
	int C = int(s0.size()); 
	MatrixXd dGds; dGds.setZero(R, C); 
	for (int j = 0; j < C; ++j) {
		double s0j = s0[j]; 
		s0[j] -= d; auto lft = G_of_s(s0); s0[j] = s0j; 
		s0[j] += d; auto ryt = G_of_s(s0); s0[j] = s0j; 
		// dGds(i, j) = Dj(Gi) <=> dGds.col(j) = Dj(G) 
		auto DjG = (ryt - lft) / (2. * d);
		dGds.col(j) = DjG;
	} 
	return dGds;
};

SparseMatrixd dense2sparse(const MatrixXd &in) {
	SparseMatrixd out; out.resize(int(in.rows()), int(in.cols()));
	for (int r = 0; r < int(in.rows()); ++r) {
		for (int c = 0; c < int(in.cols()); ++c) {
			const double &val = in(r, c);
			if (fabs(val) > 1e-10) {
				out.insert(r, c) = val;
			}
		}
	}
	return out;
}

template<class MATType>
void writeSparseMatrixDenseBlockAdd(SparseMatrixd &hes, int startX, int startY, const MATType &block, bool dropZeroes = true, bool writeOnlyLowerDiagonalValues = false) {
	for (int i = 0; i < block.rows(); i++) {
		for (int j = 0; j < block.cols(); j++) {
			if (startX + i >= startY + j || !writeOnlyLowerDiagonalValues) {
				if (dropZeroes && fabs(block(i, j)) < 1e-10) { continue; }
				hes.coeffRef(startX + i, startY + j) += block(i, j);
			}
		}
	}
}
 
bool checkVectorXd(const VectorXd &G_FD, const VectorXd &G_analytic, const bool &printEverything = false, const double &scale = 1.) {
	if (G_FD.size() != G_analytic.size()) { cout << string_format("VectorXd size mismatch (%d, %d)", G_FD.size(), G_analytic.size()) << endl; throw std::runtime_error(""); }
	// --
	bool ret = true;
	for (int i = 0; i < G_FD.size(); i++) {
		double g1 = scale * G_FD[i];
		double g2 = scale * G_analytic[i];
		double err = g1 - g2;
		bool match = true;
		bool print = printEverything;
		if ((fabs(err) > 0.0001 && (2. * fabs(err) / (fabs(g1) + fabs(g2))) > 0.001)
			|| isNaN(g1) || isNaN(g2) || isinf(g1) || isinf(g2)) {
			ret = false;
			match = false;
			print = true;
		}
		if (print) {
			cout << string_format(string((match) ? "[MATCH]    " : "[MISMATCH] ") + "--> %lf * g_%d: FD: %lf, anal: %lf, helpers_error: %lf", scale, i, g1, g2, err) << endl;
		}
	}
	return ret;
};

bool checkMatrixXd(const MatrixXd &H_FD, const MatrixXd &H_analytic, const bool &isHessian = false, const bool &printEverything = false, const double &scale = 1.) {
	if ((H_FD.rows() != H_analytic.rows()) || (H_FD.cols() != H_analytic.cols())) { cout << string_format("MatrixXd size mismatch ((%d, %d), (%d, %d))", H_FD.rows(), H_FD.cols(), H_analytic.rows(), H_analytic.cols()) << endl; throw std::runtime_error(""); }
	// --
	bool ret = true;
	for (int i = 0; i < int(H_FD.rows()); i++) {
		int j_bound = (isHessian) ? i + 1 : int(H_FD.cols());
		for (int j = 0; j < j_bound; j++) {
			double h1 = H_FD(i, j);
			double h2 = H_analytic(i, j);
			double err = h1 - h2;
			bool match = true;
			bool print = printEverything;
			if ((fabs(err) > 0.0001) // *
				|| isNaN(h1) || isNaN(h2) || isinf(h1) || isinf(h2)) {
				ret = false;
				match = false;
				print = true;
			}
			if (print) {
				cout << string_format(string((match) ? "[MATCH]    " : "[MISMATCH] ") + "--> %lf * H_{%d,%d}: FD: %lf, anal val: %lf, helpers_error: %lf", scale, i, j, h1, h2, err) << endl;
			}
		}
	}
	return ret;
};

