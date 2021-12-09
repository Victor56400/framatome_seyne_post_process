/**
  * \file matrix.hpp
  * \brief Template header file of the Matrix class
  */

#ifndef FUTILITIES_MATRIX_HPP
#define FUTILITIES_MATRIX_HPP

#include <sstream>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <forssea_utilities/helpers.hpp>
#include <boost/container/vector.hpp>


namespace forssea_utilities {

/**
 * This class implements a simple version of a matrix. It is based on a STL vector.
    The implemented matrix i column-based, meaning that its values are stored column
    after column in the STL vector.
    For example: [[1 2];[3 4]] is stored as [1 3 2 4]
 * \brief Simple matrix class, implementing most common operations, and designed to be used with CGAL optimizers, which require
 * a column based matrix.
 * @tparam T type of the matrix content
 */
template<typename T>
class Matrix {
private:
  uint8_t row_; /**< Number of rows in the matrix*/
  uint8_t col_; /**< Number of columns in the matrix*/
  std::vector<std::vector<T> > mat_; /**< Container of the matrix values*/
  std::vector<typename std::vector<T>::iterator> mat_it_; /**< Container of the matrix's column iterators*/

public:
  static const uint8_t NONE; /**< Value indicating that the the matrix does not have a special layout.*/
  static const uint8_t DIAG; /**< Value indicating that the matrix is square and diagonal.*/

  // Constructors
  /**
   * Default constructor.
   */
  Matrix();

  /**
   * Constructor initializing the whole matrix with a default value or as a diagonal matrix, which value is the default
   * value.
   * @param row number of rows
   * @param col number of columns (ignored if shape is set to \ref DIAG)
   * @param value default value
   * @param shape shape parameter of the matrix (\ref NONE or \ref DIAG)
   */
  Matrix(const uint8_t &row, const uint8_t &col, const T &value = T(), const uint8_t &shape = NONE);

  /**
   * Constructor initializing the whole matrix with default values or as a diagonal matrix, which values are the input
   * default values.
   * @param row number of rows
   * @param col number of columns (ignored if shape is set to \ref DIAG)
   * @param values vector containing the default values used to initialize the matrix or its diagonal.
   * @param shape shape parameter of the matrix (\ref NONE or \ref DIAG)
   */
  Matrix(const uint8_t &row, const uint8_t &col, const std::vector<T> &values, const uint8_t &shape = NONE);

  /**
   * Returns a reference to the element of the matrix stored on line row and column col
   * @param row row index of the element
   * @param col col index of the element
   * @return reference to the element
   */
  T &at(const uint8_t &row, const uint8_t &col);

  /**
   * Returns a const reference to the element of the matrix stored on line row and column col
   * @param row row index of the element
   * @param col col index of the element
   * @return const reference to the element
   */
  const T &at(const uint8_t &row, const uint8_t &col) const;

  /**
   * Returns a reference to the i-th element of the matrix (elements are counted column by column). A negative index
   * starts at the end of the matrix (last row of last column).
   * @param index index of the element in the matrix
   * @return reference to the element
   */
  T &at(int index);

  /**
   * Returns a const reference to the i-th element of the matrix (elements are counted column by column). A negative
   * index starts at the end of the matrix (last row of last column).
   * @param index index of the element in the matrix
   * @return const reference to the element
   */
  const T &at(int index) const;

  /**
   * Returns an iterator pointing to the first column (which is a std::vector) of the matrix.
   * @return iterator to the first column
   */
  typename std::vector<std::vector<T> >::iterator begin() { return mat_.begin(); }

  /**
   * Returns an iterator pointing after the last column (which is a std::vector) of the matrix.
   * @return iterator to the past-the-end column
   */
  typename std::vector<std::vector<T> >::iterator end() { return mat_.end(); }

  /**
   * Returns a const_iterator pointing to the first column (which is a std::vector) of the matrix.
   * @return const_iterator to the first column
   */
  typename std::vector<std::vector<T> >::const_iterator begin() const { return mat_.begin(); }

  /**
   * Returns a const_iterator pointing after the last column (which is a std::vector) of the matrix.
   * @return const_iterator to the past-the-end column
   */
  typename std::vector<std::vector<T> >::const_iterator end() const { return mat_.end(); }

  /**
   * Returns an iterator pointing to the first element of the i-th column of the matrix.
   * @param i index of the column of the matrix
   * @return iterator pointing to the i-th column's first element
   */
  typename std::vector<T>::iterator begin(const uint8_t &i) { return mat_.at(i).begin(); }

  /**
   * Returns an iterator pointing after the last element of the i-th column of the matrix.
   * @param i index of the column of the matrix
   * @return iterator pointing after o-th column's last element
   */
  typename std::vector<T>::iterator end(const uint8_t &i) { return mat_.at(i).end(); }

  /**
   * Returns a const_iterator pointing to the first element of the i-th column of the matrix.
   * @param i index of the column of the matrix
   * @return const iterator pointing to the i-th column's first element
   */
  typename std::vector<T>::const_iterator begin(const uint8_t &i) const { return mat_.at(i).begin(); }

  /**
   * Returns a const_iterator pointing after the last element of the i-th column of the matrix.
   * @param i index of the column of the matrix
   * @return const_iterator pointing after o-th column's last element
   */
  typename std::vector<T>::const_iterator end(const uint8_t &i) const { return mat_.at(i).end(); }

  /**
   * Returns an iterator to the first element of the vector of column iterators of the matrix
   * @return iterator to the first element of \ref mat_it_
   */
  typename std::vector<typename std::vector<T>::iterator>::iterator col_it();

  /**
   * Returns a std::pair containing the number of rows and columns of the matrix.
   * @return std::pair containing the number of rows and columns of the matrix
   */
  const std::pair<const uint8_t, const uint8_t> size() const { return {row_, col_}; }

  /**
   * Returns the number of rows of the matrix
   * @return number of rows of the matrix
   */
  const uint8_t &rows() const { return row_; }

  /**
   * Returns the number of columns of the matrix
   * @return number of columns of the matrix
   */
  const uint8_t &cols() const { return col_; }

  /**
   * Concatenates the input matrix on the right of the current matrix.
   * @param m matrix to concatenate on the right
   * @return reference to current matrix with the added column
   */
  Matrix<T> &concatRight(const Matrix<T> &m);

  /**
   * Concatenates the input matrix under the current matrix.
   * @param m matrix to concatenate under the current one
   * @return reference to current matrix with the added row
   */
  Matrix<T> &concatDown(const Matrix<T> &m);

  /**
   * Remove the index-th column of the matrix
   * @param index of the column to remove
   * @return reference to current matrix without the removed column
   */
  Matrix<T> &removeCol(int index);

  /**
   * Remove the index-th row of the matrix
   * @param index of the row to remove
   * @return reference to current matrix without the removed row
   */
  Matrix<T> &removeRow(int index);

  /**
   * Remove the columns corresponding to the input indexes
   * @param indexes indexes of the columns to remove. Will be reordered.
   * @return reference to current matrix without the removed columns
   */
  Matrix<T> &removeCols(std::vector<int> indexes);

  /**
   * Remove the rows corresponding to the input indexes
   * @param indexes indexes of the rows to remove. Will be reordered.
   * @return reference to current matrix without the removed rows
   */
  Matrix<T> &removeRows(std::vector<int> indexes);

  /**
   * Remove the columns masked by the boolean mask
   * @param mask boolean mask used to remove columns from the matrix (a column is removed when its corresponding boolean
   * value in the mask is false)
   * @return reference to current matrix without the removed columns
   */
  Matrix<T> &removeCols(const boost::container::vector<bool> &mask);

  /**
   * Remove the rows masked by the boolean mask
   * @param mask boolean mask used to remove rows from the matrix (a row is removed when its corresponding boolean
   * value in the mask is false)
   * @return reference to current matrix without the removed rows
   */
  Matrix<T> &removeRows(const boost::container::vector<bool> &mask);

  /**
   * Transpose current matrix
   * @return reference to current matrix
   */
  Matrix<T> &transpose();

  /**
   * Return the result of the cross product between the current matrix and the input one.
   * @param m input matrix
   * @return matrix resulting from the cross-product between the current matrix and the input one
   */
  Matrix<T> cross(const Matrix<T> &m) const;

  /**
   * Returns the rank of the current matrix
   * @return rank of the current matrix
   */
  const T rank() const;

  /**
   * Removes nan and inf from matrix.
   * nan become 0 and inf become big numbers
   * @return current matrix corrected
   */
  Matrix<T> &correct();

  /**
   * Returns a diagonal matrix from a vector
   * @return diagonal matrix
   */
  Matrix<T> diag();

  /**
   * Returns a reference to the element of the matrix stored on line row and column col
   * @param row row index of the element
   * @param col col index of the element
   * @return reference to the element
   */
  T &operator()(const uint8_t &row, const uint8_t &col) { return at(row, col); }

  /**
   * Returns a reference to the i-th element of the matrix (elements are counted column by column). A negative index
   * starts at the end of the matrix (last row of last column).
   * @param index index of the element in the matrix
   * @return reference to the element
   */
  T &operator()(const int &index) { return at(index); }

  /**
  * Returns a const reference to the element of the matrix stored on line row and column col
  * @param row row index of the element
  * @param col col index of the element
  * @return const reference to the element
  */
  const T &operator()(const uint8_t &row, const uint8_t &col) const { return at(row, col); }

  /**
   * Returns a const reference to the i-th element of the matrix (elements are counted column by column). A negative
   * index starts at the end of the matrix (last row of last column).
   * @param index index of the element in the matrix
   * @return const reference to the element
   */
  const T &operator()(const int &index) const { return at(index); }

  /**
   * Result the result of the addition of the current matrix and the input one.
   * @param m input matrix
   * @return matrix resulting from the addition of the current one and the input one
   */
  Matrix<T> operator+(const Matrix<T> &m) const;

  /**
   * Result the result of the subtraction of the current matrix and the input one.
   * @param m input matrix
   * @return matrix resulting from the subtraction of the current one and the input one
   */
  Matrix<T> operator-(const Matrix<T> &m) const;

  /**
   * Adds the input matrix to the current one and returns a reference to the latter
   * @param m input matrix
   * @return reference to current matrix to which the input one has been added
   */
  Matrix<T> &operator+=(const Matrix<T> &m);

  /**
   * Subtracts the input matrix to the current one and returns a reference to the latter
   * @param m input matrix
   * @return reference to current matrix to which the input one has been subtracted
   */
  Matrix<T> &operator-=(const Matrix<T> &m);

  /**
   * Checks value equality between the current matrix and the input one
   * @param m input matrix
   * @return true if the current matrix and the input one are equal (mathematical equality), false otherwise
   */
  bool operator==(const Matrix<T> &m) const;

  /**
   * Returns the opposite of the current matrix
   * @return opposite of the current matrix
   */
  Matrix<T> operator-() const;

  /**
   * Returns the result of the matrix multiplication between the current one and the input one.
   * @param m input matrix
   * @return result of the matrix multiplication
   */
  Matrix<T> operator*(const Matrix<T> &m) const;

  /**
   * Displays the input matrix
   * @param strm output stream
   * @param m input matrix
   * @return reference to input output stream
   */
  friend std::ostream &operator<<(std::ostream &strm, const Matrix<T> &m) {
    strm << std::scientific;
    for (uint8_t i = 0; i < m.row_; ++i) {
      strm << "[";
      for (uint8_t j = 0; j < m.col_; ++j) {
        strm << std::setw(11) << std::setprecision(3) << m(i, j);
      }
      strm << "]\n";
    }
    strm << std::defaultfloat;
    return strm;
  }
};

template<typename T>
const uint8_t Matrix<T>::NONE = 0;
template<typename T>
const uint8_t Matrix<T>::DIAG = 1;

template<typename T>
Matrix<T>::Matrix() {
  row_ = 0;
  col_ = 0;
  mat_.clear();
}

template<typename T>
Matrix<T>::Matrix(const uint8_t &row, const uint8_t &col, const T &value, const uint8_t &shape) {
  row_ = row;
  col_ = col;
  switch (shape) {
    case Matrix::NONE:mat_.resize(col, std::vector<T>(row, value));
      break;
    case Matrix::DIAG:
      // Creates a diagonal square matrix of size col.
      mat_.resize(col, std::vector<T>(col, T()));
      for (uint8_t i = 0; i < col; ++i) {
        at(i, i) = value;
      }
      break;
    default:mat_.resize(col, std::vector<T>(row, value));
      break;
  }
}

template<typename T>
Matrix<T>::Matrix(const uint8_t &row, const uint8_t &col, const std::vector<T> &values, const uint8_t &shape) {
  row_ = row;
  col_ = col;
  switch (shape) {
    case Matrix::NONE:
      if (values.size() != 0 and row * col != values.size()) {
        std::ostringstream ss;
        ss << "The number of input values is different from the size of the expected matrix.";
        throw std::out_of_range(ss.str().c_str());
      } else if (values.size() == 0)
        mat_.resize(col, std::vector<T>(row, 0));
      else {
        mat_.resize(col);
        for (uint8_t i = 0; i < col; ++i) {
          mat_.at(i).insert(mat_.at(i).begin(), values.begin() + i * row, values.begin() + (i + 1) * row);
        }
      }
      break;
    case Matrix::DIAG:
      // Creates a diagonal square matrix of size col.
      mat_.resize(col, std::vector<T>(col, T()));
      for (uint8_t i = 0; i < col; ++i) {
        at(i, i) = values.at(i);
      }
      break;
    default:
      if (values.size() != 0 and row * col != values.size()) {
        std::ostringstream ss;
        ss << "The number of input values is different from the size of the expected matrix.";
        throw std::out_of_range(ss.str().c_str());
      } else if (values.size() == 0)
        mat_.resize(col, std::vector<T>(row, 0));
      else {
        mat_.resize(col);
        for (uint8_t i = 0; i < col; ++i) {
          mat_.at(i).insert(mat_.at(i).begin(), values.begin() + i * row, values.begin() + (i + 1) * row);
        }
      }
      break;
  }
}

template<typename T>
inline T &Matrix<T>::at(const uint8_t &row, const uint8_t &col) {
  if (row < row_ && col < col_) {
    return mat_.at(col).at(row);
  } else {
    std::ostringstream ss;
    ss << "Out-of-bounds coordinates: row (" << row << ") must be in [0, " << row_ - 1 << "] and col (" << col
       << ") must be in [0, " << col_ - 1 << "]";
    throw std::out_of_range(ss.str().c_str());
  }
}

template<typename T>
inline const T &Matrix<T>::at(const uint8_t &row, const uint8_t &col) const {
  if (row < row_ && col < col_) {
    return mat_.at(col).at(row);
  } else {
    std::ostringstream ss;
    ss << "Out-of-bounds coordinates: row (" << row << ") must be in [0, " << row_ - 1 << "] and col (" << col
       << ") must be in [0, " << col_ - 1 << "]";
    throw std::out_of_range(ss.str().c_str());
  }
}

template<typename T>
inline T &Matrix<T>::at(int index) {
  uint16_t n = row_ * col_;
  if ((index < 0 and n >= -index) or (index >= 0 and index < n)) {
    if (index < 0)
      index = (index + n) % n;
    return mat_.at(index / row_).at(index % row_);
  } else {
    std::ostringstream ss;
    ss << "Out-of-bounds index: when calling 'at' with index (" << index << ") must be in [" << -int(n)
       << ", " << n - 1 << "]";
    throw std::out_of_range(ss.str().c_str());
  }
}

template<typename T>
inline const T &Matrix<T>::at(int index) const {
  uint16_t n = row_ * col_;
  if ((index < 0 and n >= -index) or (index >= 0 and index < n)) {
    if (index < 0)
      index = (index + n) % n;
    return mat_.at(index / row_).at(index % row_);
  } else {
    std::ostringstream ss;
    ss << "Out-of-bounds index: when calling 'at' with index (" << index << ") must be in [" << -int(n)
       << ", " << n - 1 << "]";
    throw std::out_of_range(ss.str().c_str());
  }
}

template<typename T>
typename std::vector<typename std::vector<T>::iterator>::iterator Matrix<T>::col_it() {
  mat_it_.clear();
  mat_it_.reserve(col_);
  for (auto c_it = mat_.begin(); c_it != mat_.end(); ++c_it)
    mat_it_.push_back((*c_it).begin());
  return mat_it_.begin();
}

template<typename T>
Matrix<T> &Matrix<T>::concatRight(const Matrix<T> &m) {
  if (row_ == m.row_) {
    mat_.insert(end(), m.begin(), m.end());
    col_ += m.col_;
    return *this;
  } else if (row_ == 0 && col_ == 0) {
    *this = m; // Copies input matrix
    return *this;
  } else {
    throw std::out_of_range("Matrix dimensions not consistent (lines).");
  }
}

template<typename T>
Matrix<T> &Matrix<T>::concatDown(const Matrix<T> &m) {
  if (col_ == m.col_) {
    uint8_t cnt = 0;
    // auto = typename std::vector<std::vector<T> >::iterator
    for (auto c_it = mat_.begin(); c_it != mat_.end(); ++c_it) {
      (*c_it).reserve(row_ + m.row_); // Avoids too much memory reallocation
      (*c_it).insert((*c_it).end(), m.begin(cnt), m.end(cnt));
      ++cnt;
    }
    row_ += m.row_;
    return *this;
  } else if (row_ == 0 && col_ == 0) {
    *this = m; // Copies input matrix
    return *this;
  } else {
    throw std::out_of_range("Matrix dimensions not consistent (columns).");
  }
}

template<typename T>
Matrix<T> &Matrix<T>::removeCol(int index) {
  if ((index < 0 and col_ >= -index) or (index >= 0 and index < col_)) {
    if (index < 0)
      index = (index + col_) % col_;
    mat_.erase(begin() + index, begin() + index + 1);
    --col_;
    return *this;
  } else {
    std::ostringstream ss;
    ss << "Out-of-bounds index: when calling 'removeCol' with index (" << index << ") must be in ["
       << -int(col_) << ", " << col_ - 1 << "]";
    throw std::out_of_range(ss.str().c_str());
  }
}

template<typename T>
Matrix<T> &Matrix<T>::removeRow(int index) {
  if ((index < 0 and row_ >= -index) or (index >= 0 and index < row_)) {
    if (index < 0)
      index = (index + row_) % row_;
    // auto = typename std::vector<std::vector<T> >::iterator
    for (auto c_it = mat_.begin(); c_it != mat_.end(); ++c_it) {
      (*c_it).erase((*c_it).begin() + index);
    }
    --row_;
    return *this;
  } else {
    std::ostringstream ss;
    ss << "Out-of-bounds index: when calling 'removeRow' with index (" << index << ") must be in ["
       << -int(row_) << ", " << row_ - 1 << "]";
    throw std::out_of_range(ss.str().c_str());
  }
}

template<typename T>
Matrix<T> &Matrix<T>::removeCols(std::vector<int> indexes) {
  // removes columns beginning by the end of the matrix, to conserve other indexes.
  (indexes += int(col_)) %= int(col_);
  std::sort(indexes.rbegin(), indexes.rend());
  for (const int &index : indexes) {
    removeCol(index);
  }
  return *this;
}

template<typename T>
Matrix<T> &Matrix<T>::removeRows(std::vector<int> indexes) {
  // removes rows beginning by the end of the matrix, to conserve other indexes.
  (indexes += int(row_)) %= int(row_);
  std::sort(indexes.rbegin(), indexes.rend());
  for (const int &index : indexes) {
    removeRow(index);
  }
  return *this;
}

template<typename T>
Matrix<T> &Matrix<T>::removeCols(const boost::container::vector<bool> &mask) {
  if (mask.size() == col_) {
    // removes columns beginning by the end of the matrix, to conserve other indexes.
    for (uint8_t i = mask.size(); i > 0; --i) {
      if (not mask.at(i - 1))
        removeCol(i - 1);
    }
    return *this;
  } else {
    std::ostringstream ss;
    ss << "Size of input mask does not match the number of columns of the matrix.";
    throw std::out_of_range(ss.str().c_str());
  }
}

template<typename T>
Matrix<T> &Matrix<T>::removeRows(const boost::container::vector<bool> &mask) {
  if (mask.size() == row_) {
    // removes rows beginning by the end of the matrix, to conserve other indexes.
    for (uint8_t i = mask.size(); i > 0; --i) {
      if (not mask.at(i - 1))
        removeRow(i - 1);
    }
    return *this;
  } else {
    std::ostringstream ss;
    ss << "Size of input mask does not match the number of rows of the matrix.";
    throw std::out_of_range(ss.str().c_str());
  }
}

template<typename T>
Matrix<T> &Matrix<T>::transpose() {
  Matrix<T> temp(col_, row_);
  for (uint8_t i = 0; i < row_; ++i) {
    for (uint8_t j = 0; j < col_; ++j) {
      temp(j, i) = at(i, j);
    }
  }
  *this = temp;
  return *this;
}

template<typename T>
Matrix<T> Matrix<T>::cross(const Matrix<T> &m) const {
  if (row_ * col_ == 3 && m.row_ * m.col_ == 3) {
    Matrix<T> res(3, 1);
    res(0) = at(1) * m(2) - at(2) * m(1);
    res(1) = at(2) * m(0) - at(0) * m(2);
    res(2) = at(0) * m(1) - at(1) * m(0);
    return res;
  } else {
    throw std::out_of_range("The matrices provided should be 3x1 or 1x3.");
  }
}

template<typename T>
const T Matrix<T>::rank() const {
  // Creates an Eigen duplicate
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> e_m(row_, col_);
  for (uint8_t i = 0; i < row_; ++i) {
    for (uint8_t j = 0; j < col_; ++j) {
      e_m(i, j) = at(i, j);
    }
  }
  Eigen::FullPivLU<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > lu_decomp(e_m);
  return lu_decomp.rank();
}

template<typename T>
Matrix<T> &Matrix<T>::correct() {
  for (uint16_t i = 0; i < row_ * col_; ++i) {
    T &val = at(i);
    if (std::isnan(val)) {
      val = 0;
    } else if (std::isinf(val) and std::signbit(val)) {
      val = std::numeric_limits<T>::min();
    } else if (std::isinf(val) and not std::signbit(val)) {
      val = std::numeric_limits<T>::max();
    }
  }
  return *this;
}

template<typename T>
Matrix<T> Matrix<T>::diag() {
  if (col_ == 1) {
    return Matrix<T>(row_ * col_, row_ * col_, this->mat_.at(0), DIAG);
  } else if (row_ == 1) {
    std::vector<T> diag(row_ * col_);
    std::transform(this->mat_.begin(),
                   this->mat_.end(),
                   diag.begin(),
                   [](const typename std::vector<T> &x) { return x.at(0); });
    return Matrix<T>(row_ * col_, row_ * col_, diag, DIAG);
  }
}

template<typename T>
Matrix<T> Matrix<T>::operator+(const Matrix<T> &m) const {
  if (row_ == m.row_ && col_ == m.col_) {
    Matrix<T> res = *(this); // copy current matrix
    return res += m;
  } else {
    throw std::out_of_range("Matrix dimensions not consistent.");
  }
}

template<typename T>
Matrix<T> Matrix<T>::operator-(const Matrix<T> &m) const {
  if (row_ == m.row_ && col_ == m.col_) {
    Matrix<T> res = *(this); // copy current matrix
    return res -= m;
  } else {
    throw std::out_of_range("Matrix dimensions not consistent.");
  }
}

template<typename T>
Matrix<T> &Matrix<T>::operator+=(const Matrix<T> &m) {
  if (row_ == m.row_ && col_ == m.col_) {
    for (uint16_t i = 0; i < row_ * col_; ++i) at(i) += m(i);
    return *(this);
  } else {
    throw std::out_of_range("Matrix dimensions not consistent.");
  }
}

template<typename T>
Matrix<T> &Matrix<T>::operator-=(const Matrix<T> &m) {
  if (row_ == m.row_ && col_ == m.col_) {
    for (uint16_t i = 0; i < row_ * col_; ++i) at(i) -= m(i);
    return *(this);
  } else {
    throw std::out_of_range("Matrix dimensions not consistent.");
  }
}

template<typename T>
bool Matrix<T>::operator==(const Matrix<T> &m) const {
  if (row_ == m.row_ && col_ == m.col_) {
    bool res = true;
    for (uint16_t i = 0; i < row_ * col_; ++i) {
      res &= (at(i) == m(i));
      if (!res) break;
    }
    return res;
  } else {
    throw std::out_of_range("Matrix dimensions not consistent.");
  }
}

template<typename T>
Matrix<T> Matrix<T>::operator-() const {
  Matrix<T> res(row_, col_);
  for (uint16_t i = 0; i < row_ * col_; ++i) res(i) = -at(i);
  return res;
}

template<typename T>
Matrix<T> Matrix<T>::operator*(const Matrix<T> &m) const {
  if (col_ == m.row_) {
    Matrix<T> res(row_, m.col_, T());
    for (uint8_t i = 0; i < res.row_; ++i) {
      for (uint8_t j = 0; j < res.col_; ++j) {
        for (uint8_t k = 0; k < col_; ++k) {
          res(i, j) += at(i, k) * m(k, j);
        }
      }
    }
    return res;
  } else {
    throw std::out_of_range("Matrix dimensions not consistent.");
  }
}
}

#endif
