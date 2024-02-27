#include "Bspline.h"

BSBasis::BSBasis() {
}

BSBasis::BSBasis(size_t degree, const std::vector<double>& knots) : p_(degree), knots_(knots) {
}

size_t
BSBasis::degree() const {
    return p_;
}

void
BSBasis::setDegree(size_t degree) {
    p_ = degree;
}

const std::vector<double>&
BSBasis::knots() const {
    return knots_;
}

std::vector<double>&
BSBasis::knots() {
    return knots_;
}

double
BSBasis::low() const {
    return knots_[p_];
}

double
BSBasis::high() const {
    return knots_[knots_.size() - p_ - 1];
}

void
BSBasis::reverse() {
    size_t k = knots_.size();
    std::vector<double> new_knots;
    new_knots.reserve(k);

    double curr = knots_.front();
    for (size_t i = 1, j = k - 1; i < k; ++i, --j) {
        new_knots.push_back(curr);
        curr += knots_[j] - knots_[j - 1];
    }
    new_knots.push_back(curr);

    knots_ = new_knots;
}

void
BSBasis::normalize() {
    size_t k = knots_.size();
    double low = knots_.front(), high = knots_.back(), len = high - low;
    for (size_t i = 0; i < k; ++i) {
        knots_[i] = (knots_[i] - low) / len;
    }
}

size_t
BSBasis::findSpan(double u) const {
    if (u >= knots_[knots_.size() - p_ - 1])
        return knots_.size() - p_ - 2;
    return (std::upper_bound(knots_.begin() + p_ + 1, knots_.end(), u) - knots_.begin()) - 1;
}

size_t
BSBasis::findSpanWithMultiplicity(double u, size_t& multi) const
{
    auto range = std::equal_range(knots_.begin(), knots_.end(), u);
    multi = range.second - range.first;

    if (u >= knots_[knots_.size() - p_ - 1])
        return knots_.size() - p_ - 2;
    return (range.second - knots_.begin()) - 1;
}


void
BSBasis::basisFunctions(size_t i, double u, std::vector<double>& coeff) const {
    coeff.clear(); coeff.reserve(p_ + 1);
    coeff.push_back(1.0);
    std::vector<double> left(p_ + 1), right(p_ + 1);
    for (size_t j = 1; j <= p_; ++j) {
        left[j] = u - knots_[i + 1 - j];
        right[j] = knots_[i + j] - u;
        double saved = 0.0;
        for (size_t r = 0; r < j; ++r) {
            double tmp = coeff[r] / (right[r + 1] + left[j - r]);
            coeff[r] = saved + tmp * right[r + 1];
            saved = tmp * left[j - r];
        }
        coeff.push_back(saved);
    }
}

void
BSBasis::basisFunctionsAll(size_t i, double u, DoubleMatrix& coeff) const {
    coeff.clear(); coeff.resize(p_ + 1);
    coeff[0].push_back(1.0);
    std::vector<double> left(p_ + 1), right(p_ + 1);
    for (size_t j = 1; j <= p_; ++j) {
        coeff[j].reserve(j + 1);
        left[j] = u - knots_[i + 1 - j];
        right[j] = knots_[i + j] - u;
        double saved = 0.0;
        for (size_t r = 0; r < j; ++r) {
            double tmp = coeff[j - 1][r] / (right[r + 1] + left[j - r]);
            coeff[j].push_back(saved + tmp * right[r + 1]);
            saved = tmp * left[j - r];
        }
        coeff[j].push_back(saved);
    }
}

void
BSBasis::basisFunctionDerivatives(size_t i, double u, size_t nr_der, DoubleMatrix& coeff) const {
    coeff.clear(); coeff.resize(nr_der + 1);
    DoubleMatrix ndu(p_ + 1);
    ndu[0].push_back(1);
    std::vector<double> left(p_ + 1), right(p_ + 1);
    for (size_t j = 1; j <= p_; ++j) {
        left[j] = u - knots_[i + 1 - j];
        right[j] = knots_[i + j] - u;
        double saved = 0;
        for (size_t r = 0; r < j; ++r) {
            ndu[j].push_back(right[r + 1] + left[j - r]);
            double tmp = ndu[r][j - 1] / ndu[j][r];
            ndu[r].push_back(saved + right[r + 1] * tmp);
            saved = tmp * left[j - r];
        }
        ndu[j].push_back(saved);
    }
    for (size_t j = 0; j <= p_; ++j)
        coeff[0].push_back(ndu[j][p_]);
    std::vector<double> a[2]; a[0].resize(p_ + 1); a[1].resize(p_ + 1);
    for (size_t r = 0; r <= p_; ++r) {
        size_t s1 = 0, s2 = 1;
        a[0][0] = 1;
        for (size_t k = 1; k <= nr_der; ++k) {
            double d = 0;
            size_t pk = p_ - k;
            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][r - k];
                d = a[s2][0] * ndu[r - k][pk];
            }
            size_t j1 = r >= k - 1 ? 1 : k - r;
            size_t j2 = r <= pk + 1 ? k - 1 : p_ - r;
            for (size_t j = j1; j <= j2; ++j) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][r + j - k];
                d += a[s2][j] * ndu[r + j - k][pk];
            }
            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            coeff[k].push_back(d);
            std::swap(s1, s2);
        }
    }
    size_t r = p_;
    for (size_t k = 1; k <= nr_der; ++k) {
        for (size_t j = 0; j <= p_; ++j)
            coeff[k][j] *= r;
        r *= p_ - k;
    }
}







BSpline::BSpline() {
}




BSpline::BSpline(size_t deg_u, size_t deg_v, const std::vector<Vec>& cpts)
	: nu(deg_u), nv(deg_v), control_points(cpts)
{
	basis_u_.setDegree(deg_u);
	basis_v_.setDegree(deg_v);
	basis_u_.knots().reserve(2 * (deg_u + 1));
	basis_u_.knots().insert(basis_u_.knots().end(), deg_u + 1, 0.0);
	basis_u_.knots().insert(basis_u_.knots().end(), deg_u + 1, 1.0);
	basis_v_.knots().reserve(2 * (deg_v + 1));
	basis_v_.knots().insert(basis_v_.knots().end(), deg_v + 1, 0.0);
	basis_v_.knots().insert(basis_v_.knots().end(), deg_v + 1, 1.0);
}


BSpline::BSpline(size_t deg_u, size_t deg_v,
	const std::vector<double>& knots_u, const std::vector<double>& knots_v,
	const std::vector<Vec>& cpts)
	: nu(knots_u.size() - deg_u - 2), nv(knots_v.size() - deg_v - 2), control_points(cpts)
{
	basis_u_.setDegree(deg_u);
	basis_v_.setDegree(deg_v);
	basis_u_.knots() = knots_u;
	basis_v_.knots() = knots_v;
}

Vec
BSpline::eval(double u, double v) const {
    size_t p_u = basis_u_.degree(), p_v = basis_v_.degree();
    size_t span_u = basis_u_.findSpan(u), span_v = basis_v_.findSpan(v);
    std::vector<double> coeff_u, coeff_v;
    basis_u_.basisFunctions(span_u, u, coeff_u);
    basis_v_.basisFunctions(span_v, v, coeff_v);
    Vec point(0.0, 0.0, 0.0);
    for (size_t i = 0; i <= p_u; ++i) {
        size_t base = (span_u - p_u + i) * (nv + 1);
        for (size_t j = 0; j <= p_v; ++j)
            point += control_points[base + span_v - p_v + j] * coeff_u[i] * coeff_v[j];
    }
    return point;
}

Vec
BSpline::eval(double u, double v, size_t nr_der, VectorMatrix& der) const {
    der.clear(); der.resize(nr_der + 1);
    size_t p_u = basis_u_.degree(), p_v = basis_v_.degree();
    size_t du = std::min(nr_der, p_u), dv = std::min(nr_der, p_v);
    size_t span_u = basis_u_.findSpan(u), span_v = basis_v_.findSpan(v);
    DoubleMatrix coeff_u, coeff_v;
    basis_u_.basisFunctionDerivatives(span_u, u, du, coeff_u);
    basis_v_.basisFunctionDerivatives(span_v, v, dv, coeff_v);
    std::vector<Vec> tmp(p_v + 1);
    for (size_t k = 0; k <= du; ++k) {
        for (size_t s = 0; s <= p_v; ++s) {
            tmp[s] = Vec(0.0, 0.0, 0.0);
            for (size_t r = 0; r <= p_u; ++r)
                tmp[s] += control_points[(span_u - p_u + r) * (nv + 1) + span_v - p_v + s] * coeff_u[k][r];
        }
        size_t dd = std::min(nr_der - k, dv);
        for (size_t l = 0; l <= dd; ++l) {
            Vec point(0.0, 0.0, 0.0);
            for (size_t s = 0; s <= p_v; ++s)
                point += tmp[s] * coeff_v[l][s];
            der[k].push_back(point);
        }
    }
    for (size_t k = p_u + 1; k <= nr_der; ++k)
        for (size_t l = 0; l <= nr_der - k; ++l)
            der[k].emplace_back(0.0, 0.0, 0.0);
    for (size_t l = p_v + 1; l <= nr_der; ++l)
        for (size_t k = 0; k <= nr_der - l; ++k)
            der[k].emplace_back(0.0, 0.0, 0.0);
    return der[0][0];
}




