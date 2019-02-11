#pragma once

#include <istream>
#include <ostream>
#include <stdexcept>

#include "../Objects/ITMDepthCorrection.h"

namespace ITMLib
{
namespace Objects
{

/** @brief Parse a file looking like:
 *     Polynomial 2 1
 *     Matrix 2 2
 *     1.07471 -0.0108775
 *     1.06566 -0.00387967
 *     1.07497 -0.0053418
 *     1.06591 0.00165605
 *
 *     With:
 *       - "Polynomial 2 1" describing a file of polynomial of degree 2, minimal degree 1
 *       - "Matrix 2 2" being the dimension of the polynomial matrix
 *       - "1.07471 -0.0108775" the numbers being in the reverse order of the polynomial order. For instance, this one is
 *         "1.07471 * x - 0.0108775 * x^2"
 */
template <typename _CorrectionModel>
class CorrectionModelDeserializer
{
public:
    static _CorrectionModel deserialize(std::istream &is, Size2 imageSize)
    {
        CorrectionModelDeserializer deserializer(is, imageSize);
        return deserializer.deserialize();
    }

private:
    CorrectionModelDeserializer(std::istream &is, Size2 imageSize)
            : _is (is)
            , _imageSize (imageSize)
    {}

    _CorrectionModel deserialize()
    {
        Size2 polynomialSize = parsePolynomialSize();

        if (polynomialSize.x() != _CorrectionModel::Polynomial::degree())
            throw std::invalid_argument(std::string("Invalid polynomial degree: expected ")
                                        + std::to_string(_CorrectionModel::Polynomial::degree())
                                        + ", got " + std::to_string(polynomialSize.x()));

        Size2 matrixSize = parseMatrixSize();

        std::vector<typename _CorrectionModel::Polynomial> polynomials;
        polynomials.reserve(matrixSize.prod());
        Eigen::size_t numberOfPolynomials = matrixSize.prod();
        for (Eigen::size_t i = 0; i < numberOfPolynomials; ++i)
        {
            polynomials.push_back(parsePolynomial(polynomialSize.y()));
        }

        return _CorrectionModel(_imageSize, matrixSize, std::move(polynomials));
    }

    Size2 parsePolynomialSize()
    {
        return parseKeywordAndSize("Polynomial");
    }

    Size2 parseMatrixSize()
    {
        return parseKeywordAndSize("Matrix");
    }

    Size2 parseKeywordAndSize(const std::string &keyword)
    {
        std::string parsedKeyword;
        Size2 size;

        if (!(_is >> parsedKeyword))
            throw std::invalid_argument(std::string("Could not parse keyword ") + keyword);

        if (parsedKeyword != keyword)
            throw std::invalid_argument(std::string("Expected \"") + keyword + "\" got \"" + parsedKeyword + '"');

        if (!(_is >> size.x() >> size.y()))
            throw std::invalid_argument(std::string("Could not parse size after keyword") + parsedKeyword);

        return size;
    }

    typename _CorrectionModel::Polynomial parsePolynomial(Eigen::size_t minDegree)
    {
        constexpr Eigen::size_t degree = _CorrectionModel::Polynomial::degree();
        typename _CorrectionModel::Polynomial::Coefficients coefficients = _CorrectionModel::Polynomial::Coefficients::Zero();

        for (Eigen::size_t i = minDegree; i <= degree; ++i)
            _is >> coefficients[i];

        if (!_is)
            throw std::invalid_argument("Failed to parse a polynomial");

        return typename _CorrectionModel::Polynomial(coefficients);
    }

private:
    std::istream &_is;
    Size2 _imageSize;
};

template<typename _Scalar, size_t _MaxDegree>
std::ostream &operator<<(std::ostream &out, const Polynomial<_Scalar, _MaxDegree> &polynomial)
{
    const typename Polynomial<_Scalar, _MaxDegree>::Coefficients &coefficients = polynomial.coefficients();

    for (size_t i = 0; i <= _MaxDegree; ++i) {
        out << coefficients[i];
        if (i != 0)
        {
            out << 'x';

            if (i > 1)
                out << '^' << i;
        }

        if (i + 1 <= _MaxDegree)
            out << " + ";
    }

    return out;
}

}

}