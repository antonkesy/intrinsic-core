// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTRINSIC_MATH_SIGNALS_NUMERICAL_DIFFERENTIATION_H_
#define INTRINSIC_MATH_SIGNALS_NUMERICAL_DIFFERENTIATION_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <ostream>
#include <type_traits>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/ipow.h"

namespace intrinsic {

namespace numdiff_internal {

// Constants and coefficients for uniformly-spaced FiniteDifferences. See also
// https://en.wikipedia.org/wiki/Finite_difference.
struct FiniteDifferenceTable {
  // Maximum supported accuracy order for finite differences. Below coefficient
  // tables must implement all coefficients up to `kMaxOrder` to achieve
  // compatibility between different schemes.
  static constexpr int kMaxOrder = 6;
  // The corresponding number of implemented finite-differencing coefficients
  // for forward and backward schemes.
  static constexpr int kNumCoeffsForwardBackward = 8;
  // The corresponding number of implemented finite-differencing coefficients
  // for central schemes.
  static constexpr int kNumCoeffsCentral = 7;

  // First derivative coefficients for backward finite differences. See also
  // https://en.wikipedia.org/wiki/Finite_difference_coefficient#Backward_finite_difference.
  // Note that backward coefficients can be easily reused as forward
  // coefficients, so an explicit implementation of forward coefficients is
  // omitted.
  static constexpr std::array<std::array<double, kNumCoeffsForwardBackward>,
                              kMaxOrder>
      kFirstDerivativeBackwardCoeffs{{
          /*order=1*/ {1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          /*order=2*/ {1.5, -2.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0},
          /*order=3*/ {11.0 / 6.0, -3.0, 1.5, -1.0 / 3.0, 0.0, 0.0, 0.0, 0.0},
          /*order=4*/
          {25.0 / 12.0, -4.0, 3.0, -4.0 / 3.0, 1.0 / 4.0, 0.0, 0.0, 0.0},
          /*order=5*/
          {137.0 / 60.0, -5.0, 5.0, -10.0 / 3.0, 5.0 / 4.0, -1.0 / 5.0, 0.0,
           0.0},
          /*order=6*/
          {49.0 / 20.0, -6.0, 15.0 / 2.0, -20.0 / 3.0, 15.0 / 4.0, -6.0 / 5.0,
           1.0 / 6.0, 0.0},
      }};

  // Second derivative coefficients for backward finite differences. See also
  // https://en.wikipedia.org/wiki/Finite_difference_coefficient#Backward_finite_difference
  static constexpr std::array<std::array<double, kNumCoeffsForwardBackward>,
                              kMaxOrder>
      kSecondDerivativeBackwardCoeffs{{
          /*order=1*/ {1.0, -2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          /*order=2*/ {2.0, -5.0, 4.0, -1.0, 0.0, 0.0, 0.0, 0.0},
          /*order=3*/
          {35.0 / 12.0, -26.0 / 3.0, 19.0 / 2.0, -14.0 / 3.0, 11.0 / 12.0, 0.0,
           0.0, 0.0},
          /*order=4*/
          {15.0 / 4.0, -77.0 / 6.0, 107.0 / 6.0, -13.0, 61.0 / 12.0, -5.0 / 6.0,
           0.0, 0.0},
          /*order=5*/
          {203.0 / 45.0, -87.0 / 5.0, 117.0 / 4.0, -254.0 / 9.0, 33.0 / 2.0,
           -27.0 / 5.0, 137.0 / 180.0, 0.0},
          /*order=6*/
          {469.0 / 90.0, -223.0 / 10.0, 879.0 / 20.0, -949.0 / 18.0, 41.0,
           -201.0 / 10.0, 1019.0 / 180.0, -7.0 / 10.0},
      }};

  // First derivative coefficients for central finite differences. See also
  // https://en.wikipedia.org/wiki/Finite_difference_coefficient#Central_finite_difference
  static constexpr std::array<std::array<double, kNumCoeffsCentral>, kMaxOrder>
      kFirstDerivativeCentralCoeffs{{
          /*order=2*/ {0.0, 0.0, -0.5, 0.0, 0.5, 0.0, 0.0},
          /*order=4*/
          {0.0, 1.0 / 12.0, -2.0 / 3.0, 0.0, 2.0 / 3.0, -1.0 / 12.0, 0.0},
          /*order=6*/
          {-1.0 / 60.0, 3.0 / 20.0, -3.0 / 4.0, 0.0, 3.0 / 4.0, -3.0 / 20.0,
           1.0 / 60.0},
      }};

  // Second derivative coefficients for central finite differences. See also
  // https://en.wikipedia.org/wiki/Finite_difference_coefficient#Central_finite_difference
  static constexpr std::array<std::array<double, kNumCoeffsCentral>, kMaxOrder>
      kSecondDerivativeCentralCoeffs{{
          /*order=2*/ {0.0, 0.0, 1.0, -2.0, 1.0, 0.0, 0.0},
          /*order=4*/
          {0.0, -1.0 / 12.0, 4.0 / 3.0, -5.0 / 2.0, 4.0 / 3.0, -1.0 / 12.0,
           0.0},
          /*order=6*/
          {1. / 90., -3. / 20., 3. / 2., -49. / 18., 3. / 2., -3. / 20.,
           1. / 90.},
      }};
};

// Since the '%' operator in C/C++ does not handle negative input arguments in
// a modulo sense (in fact C/C++'s '%' operator should be appropriately called
// 'remainder' operator instead of 'modulo'), we implement our own 'modulo'
// functionality for handling negative numbers correctly here.
// Examples:
// modulo(-2, 6) will return 4;
// modulo(8,6) will return 2;
// This method is employed to get the ring buffer index using ring buffer
// size 'N' and a (incremented) counter value 'x'.
constexpr int Modulo(int x, int N) { return (x % N + N) % N; }

}  // namespace numdiff_internal

// Numerical differentiation for data sampled at real-time with a constant
// sampling time 'dt_sec'. Computes first and second derivatives up to accuracy
// order 'kMaxOrder'. For a detailed explanation about finite differences,
// please see https://en.wikipedia.org/wiki/Finite_difference. Maintains a
// fixed-size ringbuffer of the last 'kNumCoeffs' samples internally, to which
// new samples can be added via "Append()". 'T' is expected to be a
// floating-point type like double or float, or a floating-point based
// Eigen-type like eigenmath::VectorXd.
template <typename T>
class TimeSeriesNumDiff {
 public:
  // Maximum supported accuracy order.
  static constexpr int kMaxOrder =
      numdiff_internal::FiniteDifferenceTable::kMaxOrder;

  // Constructs a numerical differentiator for time series data sampled at
  // equally spaced time increments 'dt_sec'.
  explicit TimeSeriesNumDiff(double dt_sec) : dt_sec_(std::abs(dt_sec)) {}

  // Appends a new time series data point to the internal ring buffer.
  void Append(T current);

  // Computes first derivative using backward finite difference of
  // 'accuracy_order' on currently stored time series data. Use this method for
  // computing first derivatives of "past" data, for example in a control loop.
  // If the number of samples appended is not enough to provide an approximation
  // of 'accuracy_order', the derivative with the highest accuracy_order
  // currently possible is computed instead. Will return zero if only one sample
  // has been appended so far.
  icon::RealtimeStatusOr<T> FirstDerivativeBackward(int accuracy_order);

  // Computes second derivative using backward finite difference of
  // 'accuracy_order' on currently stored time series data. Use this method for
  // computing first derivatives of "past" data, for example in a control loop.
  // If the number of samples appended is not enough to provide an approximation
  // of 'accuracy_order', the second derivative with the highest accuracy_order
  // currently possible is computed instead. Will return zero if only two
  // samples have been appended so far.
  icon::RealtimeStatusOr<T> SecondDerivativeBackward(int accuracy_order);

  // Resets internal sample counter to zero. After calling Reset(), the internal
  // sample memory is built up from the beginning.
  void Reset() { sample_count_ = 0; }

 private:
  // Index pointing to the latest sample added to 'samples_'.
  int latest_sample_index_ = 0;
  // Sampling time in seconds.
  double dt_sec_;
  // Fixed-size array containing the added time series samples, evaluated in
  // ring-buffer fashion.
  std::array<T,
             numdiff_internal::FiniteDifferenceTable::kNumCoeffsForwardBackward>
      samples_;
  int sample_count_ = 0;
};

template <typename T>
void TimeSeriesNumDiff<T>::Append(T current) {
  if (sample_count_ < 1) {
    samples_.fill(current);  // Initialze ring buffer once, resizes Eigen types.
  }

  // Increment the index of the latest sample in the ring buffer, and append
  // new data point.
  latest_sample_index_ =
      (latest_sample_index_ + 1) %
      numdiff_internal::FiniteDifferenceTable::kNumCoeffsForwardBackward;
  samples_[latest_sample_index_] = std::move(current);

  // Only increase the sample_count_ up to values relevant to the computation
  // routine.
  sample_count_ = std::min(
      sample_count_ + 1,
      numdiff_internal::FiniteDifferenceTable::kNumCoeffsForwardBackward);
}

template <typename T>
icon::RealtimeStatusOr<T> TimeSeriesNumDiff<T>::FirstDerivativeBackward(
    int accuracy_order) {
  if (accuracy_order < 1 ||
      accuracy_order > numdiff_internal::FiniteDifferenceTable::kMaxOrder) {
    return icon::InvalidArgumentError(
        icon::FixedStrCat<icon::RealtimeStatus::kMaxMessageLength>(
            "Invalid 'accuracy_order', order must be in between 1 and ",
            numdiff_internal::FiniteDifferenceTable::kMaxOrder, ", but got ",
            accuracy_order));
  }

  // At least 2 samples are required for the lowest-order first derivative
  // finite difference scheme to be applicable. Return zero first derivative
  // otherwise.
  if (sample_count_ < 2) {
    // Multiplying a sample by zero is a trick to automatically retrieve correct
    // dimensions for Eigen-types.
    return T(0.0 * samples_.front());
  }

  // At least 'k+1' samples are required for derivative scheme of order 'k' to
  // be applicable. Example: a second-order first derivative requires 3
  // samples.
  const int num_samples_required = accuracy_order + 1;
  if (sample_count_ < num_samples_required) {
    // Directly jump to the differentiation order which works given the
    // current number of samples.
    return FirstDerivativeBackward(sample_count_ - 1);
  }

  // Get a reference to the finite-difference coefficients for the selected
  // accuracy order.
  const auto& coeffs = numdiff_internal::FiniteDifferenceTable::
      kFirstDerivativeBackwardCoeffs[accuracy_order - 1];

  // Initialize derivative to zero. Multiplying a sample by zero is a trick to
  // automatically retrieve correct dimensions for Eigen-types.
  T derivative = 0.0 * samples_.front();

  // Multiply finite difference coefficients with past time series. Since for
  // the selected order, only 'num_samples_required' multiplications are
  // necessary.
  for (int i = 0;
       i <
       std::min(
           num_samples_required,
           numdiff_internal::FiniteDifferenceTable::kNumCoeffsForwardBackward);
       ++i) {
    derivative +=
        coeffs[i] *
        samples_[numdiff_internal::Modulo(
            latest_sample_index_ - i, numdiff_internal::FiniteDifferenceTable::
                                          kNumCoeffsForwardBackward)];
  }

  // First derivative is normalized by sampling time.
  return T(derivative / dt_sec_);
}

template <typename T>
icon::RealtimeStatusOr<T> TimeSeriesNumDiff<T>::SecondDerivativeBackward(
    int accuracy_order) {
  if (accuracy_order < 1 ||
      accuracy_order > numdiff_internal::FiniteDifferenceTable::kMaxOrder) {
    return icon::InvalidArgumentError(
        icon::FixedStrCat<icon::RealtimeStatus::kMaxMessageLength>(
            "Invalid 'accuracy_order', order must be in between 1 and ",
            numdiff_internal::FiniteDifferenceTable::kMaxOrder, " but got ",
            accuracy_order));
  }

  // At least 3 samples are required for the lowest-order second derivative
  // finite difference scheme to be applicable. Return zero second derivative
  // otherwise.
  if (sample_count_ < 3) {
    return T(0.0 * samples_.front());
  }

  // At least 'k+2' samples are required for derivative scheme of order 'k' to
  // be applicable. Example: a second-order second derivative requires 4
  // samples.
  const int num_samples_required = accuracy_order + 2;
  if (sample_count_ < num_samples_required) {
    // Directly jump to the differentiation order which works given the
    // current number of samples.
    return SecondDerivativeBackward(sample_count_ - 2);
  }

  // Get a reference to the finite-difference coefficients for the selected
  // accuracy order.
  const auto& coeffs = numdiff_internal::FiniteDifferenceTable::
      kSecondDerivativeBackwardCoeffs[accuracy_order - 1];

  // Initialize derivative by zero. Multiplying a sample by zero is a trick to
  // automatically retrieve correct dimensions for Eigen-types.
  T derivative = 0.0 * samples_.front();

  // Multiply finite difference coefficients with past time series. Since for
  // the selected order, only 'num_samples_required' multiplications are
  // necessary.
  for (int i = 0;
       i <
       std::min(
           num_samples_required,
           numdiff_internal::FiniteDifferenceTable::kNumCoeffsForwardBackward);
       ++i) {
    derivative +=
        coeffs[i] *
        samples_[numdiff_internal::Modulo(
            latest_sample_index_ - i, numdiff_internal::FiniteDifferenceTable::
                                          kNumCoeffsForwardBackward)];
  }

  // Second derivative is normalized by sampling time squared.
  return T(derivative / (dt_sec_ * dt_sec_));
}

namespace numdiff_internal {

// Implementation of the first derivative using central finite differences.
// Central finite differences of order `accuracy_order` are computed around the
// point in `data` specified by `data_idx`. Writes the result into
// `derivative`. Returning the result via reference was chosen to enable easy
// generalization to different types (see below comment in the implementation).
// The caller must bound-check the inputs, otherwise there will be errors.
template <typename T>
void CentralFiniteDiffFirstDerivativeImpl(const int data_idx,
                                          absl::Span<const T> data,
                                          const double sampling_time_sec,
                                          const int accuracy_order,
                                          T& derivative) {
  // Initialize derivative to zero. Multiplying a sample by zero is a trick to
  // automatically retrieve correct dimensions for Eigen-types.
  derivative *= 0.0;

  // Get a reference to the finite-difference coefficients for the selected
  // accuracy order. For central differences, the accuracy_order is either 2, 4
  // or 6, and this maps those numbers to array indices in
  // kFirstDerivativeCentralCoeffs.
  const auto& coeffs = numdiff_internal::FiniteDifferenceTable::
      kFirstDerivativeCentralCoeffs[accuracy_order / 2 - 1];

  // The number of coefficients required to compute derivative for a given
  // `accuracy_order`.
  const int num_active_coeffs = accuracy_order + 1;
  // Index of the first coefficient of the coefficient table which is used in
  // the computation, given a specific accuracy_order.
  const int first_coeff_index =
      (numdiff_internal::FiniteDifferenceTable::kMaxOrder - accuracy_order) / 2;

  const int leftmost_data_idx = data_idx - accuracy_order / 2;
  QCHECK(leftmost_data_idx >= 0)
      << "data_idx out of lower bound, data_idx is " << data_idx;
  QCHECK(leftmost_data_idx + num_active_coeffs - 1 < data.size())
      << "data_idx out of upper bound, data_idx is " << data_idx
      << " while data.size() is " << data.size();

  // Multiply finite difference coefficients and data series in active window.
  for (int i = 0; i < num_active_coeffs; ++i) {
    derivative +=
        coeffs.at(first_coeff_index + i) * data.at(leftmost_data_idx + i);
  }

  derivative /= sampling_time_sec;
}

// Implementation of the first derivative using backward finite differences.
// Backward finite differences of order `accuracy_order` are computed to the
// left of the point in `data` specified by `data_idx`.  Writes the result into
// `derivative`. Returning the result via reference was chosen to enable easy
// generalization to different types (see below comment in the implementation).
// The caller must bound-check the inputs, otherwise there will be errors.
template <typename T>
void BackwardFiniteDiffFirstDerivativeImpl(const int data_idx,
                                           absl::Span<const T> data,
                                           const double sampling_time_sec,
                                           const int accuracy_order,
                                           T& derivative) {
  // Initialize derivative to zero. Multiplying a sample by zero is a trick to
  // automatically retrieve correct dimensions for Eigen-types.
  derivative *= 0.0;

  // Get a reference to the finite-difference coefficients for the selected
  // accuracy order.
  const auto& coeffs = numdiff_internal::FiniteDifferenceTable::
      kFirstDerivativeBackwardCoeffs[accuracy_order - 1];

  // The number of coefficients required to compute derivative for a given
  // `accuracy_order`.
  const int num_active_coeffs = accuracy_order + 1;

  QCHECK(data_idx < data.size())
      << "data_idx out of upper bound,  data_idx is " << data_idx
      << " while data.size() is " << data.size();
  QCHECK(data_idx - num_active_coeffs + 1 >= 0)
      << "data_idx out of lower bound, data_idx is " << data_idx;

  // Multiply finite difference coefficients and time series in active window.
  // For backward finite differences, the coefficient array starts at index 0.
  for (int i = 0; i < num_active_coeffs; ++i) {
    derivative += coeffs.at(i) * data.at(data_idx - i);
  }
  derivative /= sampling_time_sec;
}

// Implementation of the first derivative using forward finite differences.
// Forward finite differences of order `accuracy_order` are computed to the
// right of the point in `data` specified by `data_idx`. Writes the result into
// `derivative`. Returning the result via reference was chosen to enable easy
// generalization to different types (see below comment in the implementation).
// The caller must bound-check the inputs, otherwise there will be errors.
template <typename T>
void ForwardFiniteDiffFirstDerivativeImpl(const int data_idx,
                                          absl::Span<const T> data,
                                          const double sampling_time_sec,
                                          const int accuracy_order,
                                          T& derivative) {
  // Get a reference to the finite-difference coefficients for the selected
  // accuracy order. Important note: forward finite differences exploits the
  // duality of the backward and forward coefficients, so we do not need to
  // define a separate table for forward finite differences coefficients. To get
  // the coefficients of the forward approximations from those of the backward
  // ones, give all odd derivatives the opposite sign, whereas for even
  // derivatives the signs stay the same. Conclusion: for first derivatives, use
  // opposite sign.
  const auto& coeffs = numdiff_internal::FiniteDifferenceTable::
      kFirstDerivativeBackwardCoeffs[accuracy_order - 1];

  // Initialize derivative to zero. Multiplying a sample by zero is a trick to
  // automatically retrieve correct dimensions for Eigen-types.
  derivative *= 0.0;

  // The number of coefficients required to compute derivative for a given
  // `accuracy_order`.
  const int num_active_coeffs = accuracy_order + 1;

  QCHECK(data_idx + num_active_coeffs - 1 < data.size())
      << "data_idx out of upper bound,  data_idx is " << data_idx
      << " while data.size() is " << data.size();
  QCHECK(data_idx >= 0) << "data_idx out of lower bound, data_idx is "
                        << data_idx;

  // Multiply finite difference coefficients and time series in active window.
  for (int i = 0; i < num_active_coeffs; ++i) {
    derivative += coeffs.at(i) * data.at(data_idx + i);
  }
  derivative /= sampling_time_sec;

  // Note negative sign here.
  derivative *= -1.0;
}

// Implementation of the second derivative using central finite differences.
// Central finite differences of order `accuracy_order` are computed around the
// point in `data` specified by `data_idx`. Writes the result into
// `derivative`. Returning the result via reference was chosen to enable easy
// generalization to different types (see below comment in the implementation).
// The caller must bound-check the inputs, otherwise there will be errors.
template <typename T>
void CentralFiniteDiffSecondDerivativeImpl(const int data_idx,
                                           absl::Span<const T> data,
                                           const double sampling_time_sec,
                                           const int accuracy_order,
                                           T& derivative) {
  // Initialize derivative to zero. Multiplying a sample by zero is a trick to
  // automatically retrieve correct dimensions for Eigen-types.
  derivative *= 0.0;

  // Get a reference to the finite-difference coefficients for the selected
  // accuracy order. For central differences, the accuracy_order is either 2, 4
  // or 6, and this maps those numbers to array indices in
  // kSecondDerivativeCentralCoeffs.
  const auto& coeffs = numdiff_internal::FiniteDifferenceTable::
      kSecondDerivativeCentralCoeffs[accuracy_order / 2 - 1];

  // The number of coefficients required to compute derivative for a given
  // `accuracy_order`.
  const int num_active_coeffs = accuracy_order + 1;
  // Index of the first coefficient of the coefficient table which is used in
  // the computation.
  const int first_coeff_index =
      (numdiff_internal::FiniteDifferenceTable::kMaxOrder - accuracy_order) / 2;

  const int leftmost_data_idx = data_idx - accuracy_order / 2;
  QCHECK(leftmost_data_idx >= 0)
      << "data_idx out of lower bound, data_idx is " << data_idx;
  QCHECK(leftmost_data_idx + num_active_coeffs - 1 < data.size())
      << "data_idx out of upper bound, data_idx is " << data_idx
      << " while data.size() is " << data.size();

  // Multiply finite difference coefficients and time series in active window.
  for (int i = 0; i < num_active_coeffs; ++i) {
    derivative +=
        coeffs.at(first_coeff_index + i) * data.at(leftmost_data_idx + i);
  }

  derivative /= ::intrinsic::IPow(sampling_time_sec, 2);
}

// Implementation of the second derivative using backward finite differences.
// Backward finite differences of order `accuracy_order` are computed to the
// left of the point in `data` specified by `data_idx`. Writes the result into
// `derivative`. Returning the result via reference was chosen to enable easy
// generalization to different types (see below comment in the implementation).
// The caller must bound-check the inputs, otherwise there will be errors.
template <typename T>
void BackwardFiniteDiffSecondDerivativeImpl(const int data_idx,
                                            absl::Span<const T> data,
                                            const double sampling_time_sec,
                                            const int accuracy_order,
                                            T& derivative) {
  // Initialize derivative to zero. Multiplying a sample by zero is a trick to
  // automatically retrieve correct dimensions for Eigen-types.
  derivative *= 0.0;

  // Get a reference to the finite-difference coefficients for the selected
  // accuracy order.
  const auto& coeffs = numdiff_internal::FiniteDifferenceTable::
      kSecondDerivativeBackwardCoeffs[accuracy_order - 1];

  // The number of coefficients required to compute derivative for a given
  // `accuracy_order`.
  const int num_active_coeffs = accuracy_order + 2;

  QCHECK(data_idx < data.size())
      << "data_idx out of upper bound,  data_idx is " << data_idx
      << " while data.size() is " << data.size();
  QCHECK(data_idx - num_active_coeffs + 1 >= 0)
      << "data_idx out of lower bound, data_idx is " << data_idx;

  // Multiply finite difference coefficients and time series in active window.
  for (int i = 0; i < num_active_coeffs; ++i) {
    derivative += coeffs.at(i) * data.at(data_idx - i);
  }
  derivative /= ::intrinsic::IPow(sampling_time_sec, 2);
}

// Implementation of the second derivative using forward finite differences.
// Forward finite differences of order `accuracy_order` are computed to the
// right of the point in `data` specified by `data_idx`. Writes the result into
// `derivative`. Returning the result via reference was chosen to enable easy
// generalization to different types (see below comment in the implementation).
// The caller must bound-check the inputs, otherwise there will be errors.
template <typename T>
void ForwardFiniteDiffSecondDerivativeImpl(const int data_idx,
                                           absl::Span<const T> data,
                                           const double sampling_time_sec,
                                           const int accuracy_order,
                                           T& derivative) {
  // Get a reference to the finite-difference coefficients for the selected
  // accuracy order. Important note: forward finite differences exploits the
  // duality of the backward and forward coefficients, so we do not need to
  // define a separate table for forward finite differences coefficients. To get
  // the coefficients of the forward approximations from those of the backward
  // ones, give all odd derivatives the opposite sign, whereas for even
  // derivatives the signs stay the same. Conclusion: for second derivatives,
  // use the same sign.
  const auto& coeffs = numdiff_internal::FiniteDifferenceTable::
      kSecondDerivativeBackwardCoeffs[accuracy_order - 1];

  // Initialize derivative to zero. Multiplying a sample by zero is a trick to
  // automatically retrieve correct dimensions for Eigen-types.
  derivative *= 0.0;
  // The number of coefficients required to compute derivative for a given
  // `accuracy_order`.
  const int num_active_coeffs = accuracy_order + 2;

  QCHECK(data_idx + num_active_coeffs - 1 < data.size())
      << "data_idx out of upper bound,  data_idx is " << data_idx
      << " while data.size() is " << data.size();
  QCHECK(data_idx >= 0) << "data_idx out of lower bound, data_idx is "
                        << data_idx;

  // Multiply finite difference coefficients and time series in active window.
  for (int i = 0; i < num_active_coeffs; ++i) {
    derivative += coeffs.at(i) * data.at(data_idx + i);
  }

  derivative /= ::intrinsic::IPow(sampling_time_sec, 2);
}

}  // namespace numdiff_internal

// Approximates the first derivative of uniformly sampled time-series `data`
// with Central Finite Differences of `accuracy_order`. At the left and right
// boundaries of `data`, forward and backward finite differences are used to
// compute approximations of the first derivative. Reduces `accuracy_order` to a
// feasible number in case the user-provided value is too high for the number of
// provided samples. `T` is expected to be a floating-point type like double or
// float, or a floating-point based Eigen-type like eigenmath::VectorXd.Returns
// a uniformly spaced time series with the first derivative, or a non-ok status
// code in case of failure.
template <typename T>
absl::StatusOr<std::vector<T>> ComputeFirstDerivativesCentral(
    absl::Span<const T> data, const double sampling_time_sec,
    int accuracy_order) {
  // At least 2 data points are required to run finite differences
  if (data.size() < 2) {
    return absl::InvalidArgumentError(
        absl::StrCat("Data too short, data needs to contain at least 2 points "
                     "to compute a first derivative, but got size",
                     data.size()));
  }

  if (accuracy_order < 2 ||
      accuracy_order > numdiff_internal::FiniteDifferenceTable::kMaxOrder ||
      accuracy_order % 2 != 0) {
    return absl::InvalidArgumentError(
        absl::StrCat("'accuracy_order' order must be one of [2, 4, 6] but got ",
                     accuracy_order));
  }
  if (sampling_time_sec <= 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Expected sampling time > 0.0, but got ", sampling_time_sec));
  }

  std::vector<T> derivatives;
  derivatives.resize(data.size(), 0.0 * data.front());

  // Special case data.size() == 2: set order to 1 and skip central differences.
  if (data.size() == 2) {
    numdiff_internal::ForwardFiniteDiffFirstDerivativeImpl(
        0, data, sampling_time_sec, /*accuracy_order=*/1, derivatives.at(0));
    numdiff_internal::BackwardFiniteDiffFirstDerivativeImpl(
        1, data, sampling_time_sec, /*accuracy_order=*/1, derivatives.at(1));
    return derivatives;
  }

  // Check which is the highest order method we can apply to compute the
  // derivative using the given dataset size.
  int highest_possible_order = std::floor(static_cast<double>(data.size()) / 2);
  const int remainder = highest_possible_order % /*multiple*/ 2;
  if (remainder != 0) {
    highest_possible_order =
        highest_possible_order + /*multiple*/ 2 - remainder;
  }

  // Select the minimum of the highest possible order and the user-provided
  // order, but never choose an order less than 2.
  accuracy_order =
      std::max(2, std::min(accuracy_order, highest_possible_order));

  // Central differences cannot be applied to the left boundary of the data
  // interval directly. Here we apply forward finite differences with same
  // `accuracy_order`.
  for (int p = 0; p < accuracy_order / 2; ++p) {
    numdiff_internal::ForwardFiniteDiffFirstDerivativeImpl(
        p, data, sampling_time_sec, accuracy_order, derivatives.at(p));
  }

  // Loop over all indices for which to compute the derivative using the central
  // differences scheme. The interval for which central differences can be
  // applied without modification.
  for (int p = accuracy_order / 2; p < data.size() - accuracy_order / 2; ++p) {
    numdiff_internal::CentralFiniteDiffFirstDerivativeImpl(
        p, data, sampling_time_sec, accuracy_order, derivatives.at(p));
  }

  // Central differences cannot be applied to the right boundary of the data
  // interval directly. Here we apply backward finite differences with same
  // `accuracy_order`.
  for (int p = data.size() - accuracy_order / 2; p < data.size(); ++p) {
    numdiff_internal::BackwardFiniteDiffFirstDerivativeImpl(
        p, data, sampling_time_sec, accuracy_order, derivatives.at(p));
  }

  return derivatives;
}

// Approximates the second derivative of uniformly sampled time-series `data`
// with Central Finite Differences of `accuracy_order`. At the left and right
// boundaries of `data`, forward and backward finite differences are used to
// compute approximations of the first derivative. Reduces `accuracy_order` to a
// feasible number in case the user-provided value is too high for the number of
// provided samples. `T` is expected to be a floating-point type like double or
// float, or a floating-point based Eigen-type like eigenmath::VectorXd. Returns
// a uniformly spaced time series with the first derivative, or a non-ok status
// code in case of failure.
template <typename T>
absl::StatusOr<std::vector<T>> ComputeSecondDerivativesCentral(
    absl::Span<const T> data, const double sampling_time_sec,
    int accuracy_order) {
  // At least 2 data points are required to run finite differences
  if (data.size() < 3) {
    return absl::InvalidArgumentError(
        absl::StrCat("Data too short, data needs to contain at least 3 points "
                     "to compute a second derivative, but got size",
                     data.size()));
  }

  if (accuracy_order < 2 ||
      accuracy_order > numdiff_internal::FiniteDifferenceTable::kMaxOrder ||
      accuracy_order % 2 != 0) {
    return absl::InvalidArgumentError(
        absl::StrCat("'accuracy_order' order must be one of [2, 4, 6] but got ",
                     accuracy_order));
  }
  if (sampling_time_sec <= 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Expected sampling time > 0.0, but got ", sampling_time_sec));
  }

  std::vector<T> derivatives;
  derivatives.resize(data.size(), 0.0 * data.front());

  // Special case data.size() == 2: set order to 1 for forward/backward finite
  // differences, and use order two central differences.
  if (data.size() == 3) {
    numdiff_internal::ForwardFiniteDiffSecondDerivativeImpl(
        0, data, sampling_time_sec, /*accuracy_order=*/1, derivatives.at(0));
    numdiff_internal::CentralFiniteDiffSecondDerivativeImpl(
        1, data, sampling_time_sec, /*accuracy_order=*/2, derivatives.at(1));
    numdiff_internal::BackwardFiniteDiffSecondDerivativeImpl(
        2, data, sampling_time_sec, /*accuracy_order=*/1, derivatives.at(2));
    return derivatives;
  }

  // The highest order method we can apply to compute the derivative using the
  // given data:
  int highest_possible_order =
      std::floor(static_cast<double>(data.size() - 1) / 2);
  const int remainder = highest_possible_order % /*multiple*/ 2;
  if (remainder != 0) {
    highest_possible_order =
        highest_possible_order + /*multiple*/ 2 - remainder;
  }

  // Select the minimum of the highest possible order and the user-provided
  // order, but never choose an order less than 2.
  accuracy_order =
      std::max(2, std::min(accuracy_order, highest_possible_order));

  // Central differences cannot be applied to the left boundary of the data
  // interval directly. Here we apply forward finite differences with same
  // `accuracy_order`.
  for (int p = 0; p < accuracy_order / 2; ++p) {
    numdiff_internal::ForwardFiniteDiffSecondDerivativeImpl(
        p, data, sampling_time_sec, accuracy_order, derivatives.at(p));
  }

  // Loop over all indices for which to compute the derivative using the central
  // differences scheme. The interval for which central differences can be
  // applied without modification.
  for (int p = accuracy_order / 2; p < data.size() - accuracy_order / 2; ++p) {
    numdiff_internal::CentralFiniteDiffSecondDerivativeImpl(
        p, data, sampling_time_sec, accuracy_order, derivatives.at(p));
  }

  // Central differences cannot be applied to the right boundary of the data
  // interval directly. Here we apply backward finite differences with same
  // `accuracy_order`.
  for (int p = data.size() - accuracy_order / 2; p < data.size(); ++p) {
    numdiff_internal::BackwardFiniteDiffSecondDerivativeImpl(
        p, data, sampling_time_sec, accuracy_order, derivatives.at(p));
  }

  return derivatives;
}

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SIGNALS_NUMERICAL_DIFFERENTIATION_H_
