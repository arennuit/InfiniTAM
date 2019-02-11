//
// Created by laloge_h on 07/02/19.
//

#pragma once

#include <memory>
#include <new>
#include <vector>

#include "Core/EigenExtensions/EigenGeometryAndPlugins.h"

#include "../../ORUtils/MemoryBlock.h"

namespace ITMLib
{
namespace Objects
{
// Copy of polynomials module, because they don't use EIGEN_DEVICE_FUNC
/** \ingroup Polynomials_Module
 * \returns the evaluation of the polynomial at x using Horner algorithm.
 *
 * \param[in] poly : the vector of coefficients of the polynomial ordered
 *  by degrees i.e. poly[i] is the coefficient of degree i of the polynomial
 *  e.g. \f$ 1 + 3x^2 \f$ is stored as a vector \f$ [ 1, 0, 3 ] \f$.
 * \param[in] x : the value to evaluate the polynomial at.
 *
 * \note for stability:
 *   \f$ |x| \le 1 \f$
 */
template<typename Polynomials, typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
T device_poly_eval_horner(const Polynomials &poly, const T &x)
{
    T val = poly[poly.size() - 1];
    for (Eigen::DenseIndex i = poly.size() - 2; i >= 0; --i)
    {
        val = val * x + poly[i];
    }
    return val;
}

/** \ingroup Polynomials_Module
 * \returns the evaluation of the polynomial at x using stabilized Horner algorithm.
 *
 * \param[in] poly : the vector of coefficients of the polynomial ordered
 *  by degrees i.e. poly[i] is the coefficient of degree i of the polynomial
 *  e.g. \f$ 1 + 3x^2 \f$ is stored as a vector \f$ [ 1, 0, 3 ] \f$.
 * \param[in] x : the value to evaluate the polynomial at.
 */
template<typename Polynomials, typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
T device_poly_eval(const Polynomials &poly, const T &x)
{
    typedef typename Eigen::NumTraits<T>::Real Real;

    if (Eigen::numext::abs2(x) <= Real(1))
    {
        return device_poly_eval_horner(poly, x);
    } else
    {
        T val = poly[0];
        T inv_x = T(1) / x;
        for (Eigen::DenseIndex i = 1; i < poly.size(); ++i)
        {
            val = val * inv_x + poly[i];
        }

        return Eigen::numext::pow(x, (T) (poly.size() - 1)) * val;
    }
}

using Size2 = Eigen::Array<size_t, 2, 1, Eigen::DontAlign>;

/**
 * @brief A polynomial function
 * @tparam _Scalar Real type to use
 * @tparam _MaxDegree Degree of the polynomial
 */
template<typename _Scalar, size_t _MaxDegree>
class Polynomial
{
public:
    using Coefficients = Eigen::Matrix<_Scalar, _MaxDegree + 1, 1, Eigen::DontAlign>;

    /// \brief Linearity
    Polynomial()
            : _coefficients (Coefficients::Zero())
    {
        if (_coefficients.rows() > 1)
            _coefficients[1] = _Scalar(1);
    }

    explicit Polynomial(const Coefficients &coefficients)
            : _coefficients(coefficients)
    {}

    Polynomial(const Polynomial &) = default;

    Polynomial(Polynomial &&) noexcept = default;

    Polynomial &operator=(const Polynomial &) = default;

    Polynomial &operator=(Polynomial &&) = default;

    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE _Scalar evaluate(_Scalar x) const
    {
        return device_poly_eval(_coefficients, x);
    }

    const Coefficients &coefficients() const
    { return _coefficients; }

    static constexpr inline size_t degree()
    {
        return _MaxDegree;
    }

private:
    Coefficients _coefficients;
};

template<typename _Scalar>
using LocalPolynomial = Polynomial<_Scalar, 2>;

template<typename _Scalar>
using GlobalPolynomial = Polynomial<_Scalar, 2>;

/**
 *
 * @tparam _Scalar Real type to use
 * @tparam _Polynomial Polynomial class
 * @tparam _Allocator Allocator, a template class for allocation
 */
template<typename _Scalar, typename _Polynomial>
class CorrectionModel
{
    struct LookupTableData
    {
        Size2 index;
        _Scalar weight;
    };

    using LookupBin = LookupTableData[4];

public:
    using Scalar = _Scalar;
    using Polynomial = _Polynomial;

    /// \brief Default constructor. Initialize a correction model that will do nothing.
    CorrectionModel()
            : CorrectionModel({1, 1}, {2, 2}, {Polynomial(), Polynomial(), Polynomial(), Polynomial()})
    {}

    /**
     * Main constructor, to fully initialize the correction model
     * @param imageSize Size of the image
     * @param matrixSize Size of the polynomial matrix.
     * @param polynomials List of the polynomial. It is actually a matrix flattened into a vector
     */
    CorrectionModel(Size2 imageSize, Size2 matrixSize, const std::vector<_Polynomial> &polynomials)
            : _imageSize(std::move(imageSize)),
              _matrixSize(std::move(matrixSize)),
              _polynomials(polynomials.size(), true, true),
              _lookupTable(_imageSize.prod(), true, true)
    {
        _binSize = _imageSize / (_matrixSize - Size2(1, 1));
        assert(_matrixSize.prod() == polynomials.size());
        assert((_binSize.x() == 1 || _binSize.x() % 2 == 0) && (_binSize.y() == 1 || _binSize.y() % 2 == 0));

        std::copy(polynomials.begin(), polynomials.end(), _polynomials.GetData(MEMORYDEVICE_CPU));

        buildLookupTable();
        _polynomials.UpdateDeviceFromHost();
        _lookupTable.UpdateDeviceFromHost();
    }

    CorrectionModel(const CorrectionModel &other) = default;

    CorrectionModel(CorrectionModel &&other) noexcept = default;

    CorrectionModel &operator=(const CorrectionModel &other) = default;

    CorrectionModel &operator=(CorrectionModel &&other) noexcept = default;

    ~CorrectionModel() = default;

    /**
     * Resize the correction model, so it will works for another format of image.
     * @param imageSize Size of the images to process
     */
    void resize(Size2 imageSize)
    {
        _imageSize = imageSize;
        _binSize = _imageSize / (_matrixSize - Size2(1, 1));
        assert((_binSize.x() == 1 || _binSize.x() % 2 == 0) && (_binSize.y() == 1 || _binSize.y() % 2 == 0));

        _lookupTable = ORUtils::MemoryBlock<LookupBin >(_imageSize.prod(), true, true);
        buildLookupTable();
        _lookupTable.UpdateDeviceFromHost();
    }

    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE _Scalar undistort(size_t x, size_t y, _Scalar depth) const
    {
        const LookupBin &lookupBin = lookup(x, y);
        _Scalar tmp_depth = 0.0;
        _Scalar weight_sum = 0.0;

#pragma unroll
        for (size_t i = 0; i < 4; ++i)
        {
            const LookupTableData &ltData = lookupBin[i];
            weight_sum += ltData.weight;
            tmp_depth += ltData.weight * polynomial(ltData.index.x(), ltData.index.y()).evaluate(depth);
        }

        return tmp_depth / weight_sum;
    }

    // Getters
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Size2 &imageSize() const
    { return _imageSize; }

    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Size2 &binSize() const
    { return _binSize; }

    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const _Polynomial &polynomial(size_t x, size_t y) const
    {
        MemoryDeviceType currentDevice =
#ifdef __CUDA_ARCH__
                MemoryDeviceType::MEMORYDEVICE_CUDA;
#else
                MemoryDeviceType::MEMORYDEVICE_CPU;
#endif

        return _polynomials.GetData(currentDevice)[y * _matrixSize.x() + x];
    }

private:
    /**
     * @see Heavily based on the polynomial_matrix_model.h in the ROS package calibration_toolkit:
     *    https://github.com/iaslab-unipd/calibration_toolkit/blob/indigo_v0.3/kinect/include/kinect/depth/polynomial_matrix_model.h
     */
    void buildLookupTable()
    {
        for (size_t y = 0; y < _imageSize.y(); ++y)
        {
            for (size_t x = 0; x < _imageSize.x(); ++x)
            {
                LookupBin &ltData = lookup(x, y);

                Size2 index(x, y);
                Eigen::Array<size_t, 2, 2> bin;
                Eigen::Array<_Scalar, 2, 2> weight;

                bin.col(0) = index / _binSize;
                bin.col(1) = bin.col(0) + Size2(1, 1);

                weight.col(1) =
                        (index - bin.col(0) * _binSize).template cast<_Scalar>() / _binSize.template cast<_Scalar>();
                weight.col(0) = Eigen::Array<_Scalar, 2, 1>(_Scalar(1.0), _Scalar(1.0)) - weight.col(1);

#pragma unroll
                for (size_t i = 0; i < 2; ++i)
                {
#pragma unroll
                    for (size_t j = 0; j < 2; ++j)
                    {
                        LookupTableData &lt = ltData[i * 2 + j];
                        lt.index = Size2(bin(0, i), bin(1, j));
                        lt.weight = weight(0, i) * weight(1, j);
                    }
                }
            }
        }
    }

    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE LookupBin &lookup(size_t x, size_t y)
    {
        constexpr MemoryDeviceType currentDevice =
#ifdef __CUDA_ARCH__
                MemoryDeviceType::MEMORYDEVICE_CUDA;
#else
                MemoryDeviceType::MEMORYDEVICE_CPU;
#endif

        return _lookupTable.GetData(currentDevice)[y * _imageSize.x() + x];
    }

    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const LookupBin &lookup(size_t x, size_t y) const
    {
        constexpr MemoryDeviceType currentDevice =
#ifdef __CUDA_ARCH__
                MemoryDeviceType::MEMORYDEVICE_CUDA;
#else
                MemoryDeviceType::MEMORYDEVICE_CPU;
#endif

        return _lookupTable.GetData(currentDevice)[y * _imageSize.x() + x];
    }

    Size2 _imageSize;
    Size2 _matrixSize;
    Size2 _binSize;
    ORUtils::MemoryBlock<Polynomial> _polynomials;
    ORUtils::MemoryBlock<LookupBin> _lookupTable;
};

template<typename _Scalar, typename _LocalModel, typename _GlobalModel>
class TwoStepsCorrectionModel
{
public:
    using LocalModel = _LocalModel;
    using GlobalModel = _GlobalModel;

    /// @brief Default constructor
    TwoStepsCorrectionModel() = default;
    ~TwoStepsCorrectionModel() = default;
    TwoStepsCorrectionModel(const TwoStepsCorrectionModel &other) = default;
    TwoStepsCorrectionModel(TwoStepsCorrectionModel &&other) noexcept = default;
    TwoStepsCorrectionModel &operator=(const TwoStepsCorrectionModel &other) = default;
    TwoStepsCorrectionModel &operator=(TwoStepsCorrectionModel &&other) noexcept = default;

    TwoStepsCorrectionModel(LocalModel &&localModel, GlobalModel &&globalModel)
            : _localModel(std::move(localModel)), _globalModel(std::move(globalModel))
    {}

    const LocalModel &localModel() const
    {
        return _localModel;
    }

    const GlobalModel &globalModel() const
    {
        return _globalModel;
    }

    void setLocalModel(LocalModel localModel)
    {
        _localModel = std::move(localModel);
    }

    void setGlobalModel(GlobalModel globalModel)
    {
        _globalModel = std::move(globalModel);
    }

    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE _Scalar undistord(size_t x, size_t y, _Scalar depth) const
    {
        return _globalModel.undistort(x, y, _localModel.undistort(x, y, depth));
    }

private:
    LocalModel _localModel;
    GlobalModel _globalModel;
};

using DepthCorrectionModel = TwoStepsCorrectionModel<float,
        CorrectionModel<float, LocalPolynomial<float> >,
        CorrectionModel<float, GlobalPolynomial<float> > >;

} // namespace Object.

} // namespace ITMLib.
