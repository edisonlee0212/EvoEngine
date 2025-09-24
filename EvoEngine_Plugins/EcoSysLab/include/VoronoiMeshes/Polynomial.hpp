#pragma once
#include <iostream>

// resolve a macro conflict with /usr/include/X11/X.h:350:21
#pragma push_macro("Success")
#undef Success

#include "../unsupported/Eigen/Polynomials"
#include "Dense"

#pragma pop_macro("Success")

namespace kinDS {
struct Monomial {
  double coefficient;  // Coefficient of the term
  int exponent;        // Exponent of the term
  Monomial(double coeff, int exp = 0) : coefficient(coeff), exponent(exp) {
  }

  // unary minus for CoefficientTerm
  Monomial operator-() const {
    return {-coefficient, exponent};
  }
};

// Allow polyonomials to be expressed symbolically
struct Var {
  // overload ^ operator for powers
  Monomial operator^(int exp) const {
    return {1, exp};  // coefficient 1, exponent `exp`
  }

  // allow X by itself: X == X^1
  operator Monomial() const {
    return {1, 1};
  }
};

// allow multiplication: 3 * (X^2)
template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
Monomial operator*(T coeff, const Monomial& term) {
  return {coeff * term.coefficient, term.exponent};
}

// allow multiplication: (X^2) * 3
template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
Monomial operator*(const Monomial& term, T coeff) {
  return {coeff * term.coefficient, term.exponent};
}

/**
 * \brief Class representing a polynomial with real coefficients.
 */
class Polynomial {
  Eigen::VectorXd coeffs;

 private:
  static Eigen::VectorXd add_poly(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    int n = std::max(a.size(), b.size());
    Eigen::VectorXd result = Eigen::VectorXd::Zero(n);
    result.head(a.size()) += a;
    result.head(b.size()) += b;
    return result;
  }

  static Eigen::VectorXd multiply_poly(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    Eigen::VectorXd result = Eigen::VectorXd::Zero(a.size() + b.size() - 1);
    for (int i = 0; i < a.size(); ++i) {
      for (int j = 0; j < b.size(); ++j) {
        result[i + j] += a[i] * b[j];
      }
    }
    return result;
  }

 public:
  Polynomial() {
    assert(coeffs.size() == 0 && "Default constructor should not initialize coefficients.");
  }

  Polynomial(const Eigen::VectorXd& c) : coeffs(c) {
  }

  Polynomial(std::initializer_list<Monomial> init) {
    // Determine the maximum exponent
    int max_exp = 0;
    for (auto& [coeff, exp] : init) {
      if (exp > max_exp) {
        max_exp = exp;
      }
    }

    coeffs = Eigen::VectorXd::Zero(max_exp + 1);

    for (auto& [coeff, exp] : init) {
      if (exp < 0) {
        throw std::invalid_argument("Negative exponent in polynomial initialization.");
      }
      coeffs[exp] += coeff;  // Add coefficients for the same exponent
    }
  }

  Polynomial(const Var& x) : coeffs(Eigen::VectorXd::Zero(2)) {
    coeffs[1] = 1;  // x^1
  }

  Polynomial(double constant) : coeffs(Eigen::VectorXd::Zero(1)) {
    coeffs[0] = constant;  // Constant polynomial
  }

  Polynomial(int constant) : coeffs(Eigen::VectorXd::Zero(1)) {
    coeffs[0] = static_cast<double>(constant);  // Constant polynomial
  }

  Polynomial(std::function<Polynomial(const Var&)> func) {
    Var X;
    *this = func(X);
  }

  double operator()(double x) const {
    double result = 0;
    for (int i = coeffs.size() - 1; i >= 0; --i) {
      result = result * x + coeffs[i];
    }
    return result;
  }

  Eigen::VectorXcd roots() const {
    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeffs);
    Eigen::VectorXcd result = solver.roots();
    return result;
  }

  std::vector<double> realRoots() const {
    size_t deg = degree();
    if (deg == 0) {
      return {};
    } else if (deg == -1) {
      return {std::numeric_limits<double>::quiet_NaN()};
    }

    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeffs);
    const Eigen::VectorXcd& complex_roots = solver.roots();

    std::vector<double> real_roots;
    for (auto& root : complex_roots) {
      if (root.imag() == 0) {
        real_roots.emplace_back(root.real());
      }
    }

    return real_roots;
  }

  Polynomial derivative(size_t order = 1) const {
    if (order == 0) {
      return *this;  // The zeroth derivative is the polynomial itself
    }
    Eigen::VectorXd derived_coeffs = coeffs;

    // do computation in place
    for (size_t i = 0; i < order; ++i) {
      for (int j = 1; j < derived_coeffs.size(); ++j) {
        derived_coeffs[j - 1] = derived_coeffs[j] * j;
      }
      derived_coeffs.tail(1).setZero();  // Set the last coefficient to zero after differentiation
      derived_coeffs.conservativeResize(derived_coeffs.size() - 1);
    }
    return Polynomial(derived_coeffs);
  }

  // Utility: Remove leading zeros
  void trim() {
    coeffs.conservativeResize(degree() + 1);
  }

  // Polynomial degree, will return -1 for the zero polynomial
  int degree() const {
    int deg = -1;

    for (size_t i = 0; i < coeffs.size(); ++i) {
      if (std::abs(coeffs[i]) >= std::numeric_limits<double>::epsilon()) {
        deg = i;
      }
    }

    return deg;
  }

  // Polynomial division: dividend = divisor * quotient + remainder
  static void divide(const Polynomial& dividend, const Polynomial& divisor, Polynomial& quotient,
                     Polynomial& remainder) {
    remainder = dividend;
    remainder.trim();

    Polynomial div = divisor;
    div.trim();

    int n = remainder.degree();
    int d = div.degree();

    if (d < 0) {
      throw std::runtime_error("Division by zero polynomial");
    }

    quotient = Polynomial(Eigen::VectorXd::Zero(n - d + 1));

    for (int i = n - d; i >= 0; --i) {
      double coeff = remainder.coeffs[d + i] / div.coeffs[d];
      quotient.coeffs[i] = coeff;
      for (int j = 0; j <= d; ++j) {
        remainder.coeffs[i + j] -= coeff * div.coeffs[j];
      }
    }

    quotient.trim();
    remainder.trim();
  }

  // Polynomial GCD using Euclidean algorithm
  static Polynomial gcd(Polynomial a, Polynomial b) {
    a.trim();
    b.trim();
    while (b.coeffs.size() != 0) {
      Polynomial q, r;
      divide(a, b, q, r);
      a = b;
      b = r;
    }

    // Normalize GCD to have leading coefficient 1
    if (a.coeffs.size() != 0) {
      double lead = *(a.coeffs.end() - 1);
      for (double& c : a.coeffs)
        c /= lead;
    }

    return a;
  }

  // Reduce rational function p/q to lowest degree
  static void reduce(Polynomial& p, Polynomial& q) {
    p.trim();
    q.trim();
    Polynomial g = gcd(p, q);
    if (g.coeffs.size() == 0)
      return;  // nothing to reduce

    Polynomial quotient;
    Polynomial dummy;
    divide(p, g, quotient, dummy);
    p = quotient;

    divide(q, g, quotient, dummy);
    q = quotient;
  }

  Polynomial operator+(const Polynomial& other) const {
    return Polynomial(add_poly(coeffs, other.coeffs));
  }
  Polynomial operator-(const Polynomial& other) const {
    return Polynomial(add_poly(coeffs, -other.coeffs));
  }

  // unary minus
  Polynomial operator-() const {
    return Polynomial(-coeffs);
  }

  Polynomial operator*(const Polynomial& other) const {
    return Polynomial(multiply_poly(coeffs, other.coeffs));
  }

  // allow multiplication with a scalar
  template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
  Polynomial operator*(T scalar) const {
    return Polynomial(coeffs * scalar);
  }

  // allow multiplication with a scalar before a term
  template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
  friend Polynomial operator*(T scalar, const Polynomial& poly) {
    return Polynomial(poly.coeffs * scalar);
  }

  // Operators for adding symbolic terms
  /*Polynomial operator+(const CoefficientTerm& term) const {
    Polynomial result = *this;
    // Ensure the polynomial has enough coefficients
    if (result.coeffs.size() <= term.exponent) {
      size_t old_size = result.coeffs.size();
      result.coeffs.conservativeResize(term.exponent + 1);
      result.coeffs.tail(term.exponent + 1 - old_size).setZero(); // Ensure new coefficients are zero
    }

    result.coeffs[term.exponent] += term.coefficient;
    return result;
  }*/

  std::string to_string() const {
    std::string result;

    for (int i = coeffs.size() - 1; i >= 0; --i) {
      if (coeffs[i] != 0) {
        result += (i != coeffs.size() - 1 && coeffs[i] > 0 ? "+" : "") + std::to_string(coeffs[i]);

        if (i > 1)
          result += "x^" + std::to_string(i) + " ";
        else if (i == 1)
          result += "x ";
      }
    }

    return result.empty() ? "0" : result;  // If all coefficients are zero, return "0"
  }

  void print(std::ostream& stringstream = std::cout) const {
    stringstream << to_string() << "\n";
  }

  // getter
  const Eigen::VectorXd& getCoefficients() const {
    return coeffs;
  }
};

// combine coeffient terms to a polynomial
Polynomial operator+(const Monomial& term1, const Monomial& term2);

Polynomial operator-(const Monomial& term1, const Monomial& term2);

inline std::ostream& operator<<(std::ostream& os, const Polynomial& p) {
  return os << p.to_string();
}

#define POLYNOMIAL(expr)                \
  kinDS::Polynomial([&](kinDS::Var x) { \
    return (kinDS::Polynomial{(expr)}); \
  })
}  // namespace kinDS
