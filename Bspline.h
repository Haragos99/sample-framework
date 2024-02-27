#pragma once
#include <QGLViewer/qglviewer.h>
#include <vector>
#include <Eigen/Eigen>
using qglviewer::Vec;



using DoubleMatrix = std::vector<std::vector<double>>;
using VectorMatrix = std::vector<std::vector<Vec>>;

struct BSBasis {
public:
    // Constructors
    BSBasis();
    BSBasis(const BSBasis&) = default;
    BSBasis(size_t degree, const std::vector<double>& knots);
    BSBasis& operator=(const BSBasis&) = default;

    // Properties
    size_t degree() const;
    void setDegree(size_t degree);
    const std::vector<double>& knots() const;
    std::vector<double>& knots();
    double low() const;
    double high() const;

    // Utilities
    void reverse();
    void normalize();
    size_t findSpan(double u) const;
    size_t findSpanWithMultiplicity(double u, size_t& multi) const;
    void basisFunctions(size_t i, double u, std::vector<double>& coeff) const;
    void basisFunctionsAll(size_t i, double u, DoubleMatrix& coeff) const;
    void basisFunctionDerivatives(size_t i, double u, size_t nr_der, DoubleMatrix& coeff) const;

private:
    size_t p_;
    std::vector<double> knots_;
};





struct BSpline {
    size_t du;
    size_t dv;
    size_t nu;
    size_t nv;
    BSBasis basis_u_, basis_v_;
    std::vector<double> knots;
    std::vector<Vec> control_points;
    BSpline();
    BSpline(const BSpline&) = default;
    BSpline(size_t deg_u, size_t deg_v, const std::vector<Vec>& cpts);
    BSpline(size_t deg_u, size_t deg_v, const std::vector<double>& knots_u, const std::vector<double>& knots_v,
        const std::vector<Vec>& cpts);
    Vec eval(double u, double v) const;
    Vec eval(double u, double v, size_t nr_der, VectorMatrix& der) const;
    void open(std::string& filname);

};
