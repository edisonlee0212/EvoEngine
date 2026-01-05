#include "Polynomial.hpp"

kinDS::Polynomial kinDS::operator+(const kinDS::Monomial& term1, const kinDS::Monomial& term2) {
  return kinDS::Polynomial({term1}) + kinDS::Polynomial({term2});
}

kinDS::Polynomial kinDS::operator-(const kinDS::Monomial& term1, const kinDS::Monomial& term2) {
  return kinDS::Polynomial({term1}) - kinDS::Polynomial({term2});
}