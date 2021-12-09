#include <forssea_utilities/helpers.hpp>
#include <forssea_utilities/matrix.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include <boost/container/vector.hpp>


using namespace std;
using forssea_utilities::operator<<;
using forssea_utilities::operator+=;
using forssea_utilities::operator%=;
using forssea_utilities::sgn;
using forssea_utilities::angle;

typedef forssea_utilities::Matrix<float> Mf;

TEST(GeneralTests, sign) {
  EXPECT_EQ(-1, sgn(-1.54684));
  EXPECT_EQ(0, sgn(0.00000));
  EXPECT_EQ(1, sgn(156.00000));
  EXPECT_EQ(1, sgn(156));
  EXPECT_EQ(-1, sgn(-15));
  EXPECT_EQ(0, sgn(0));
}

TEST(GeneralTests, angle) {
  EXPECT_FLOAT_EQ(-5 * M_PI / 6, angle(7.0 * M_PI / 6.0));
  EXPECT_FLOAT_EQ(4 * M_PI / 6, angle(-8.0 * M_PI / 6.0));
  EXPECT_FLOAT_EQ(0.0, angle(-24.0 * M_PI / 6.0));
  EXPECT_FLOAT_EQ(-M_PI / 6, angle(-13.0 * M_PI / 6.0));
}

TEST(VectorTests, display) {
  try {
    vector<int> v1 = {1, 2, 3, 4, 5};
    vector<float> v2 = {1.2, 0.52, 2.3, 5.6004, 5.489568};
    vector<bool> v3 = {true, false, true, false};
    ostringstream s;
    s << v1 << endl << v2 << endl << v3 << endl;
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(VectorTests, addInt) {
  vector<int> v0 = {1, 2, 3, 4, 5};
  vector<int> v1 = {1, 2, 3, 4, 5};
  vector<int> v2 = {3, 4, 5, 6, 7};
  EXPECT_EQ(v2, v1 += 2);
  EXPECT_EQ(v0, v1 += -2);
}

TEST(VectorTests, modInt) {
  vector<int> v1 = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  vector<int> v2 = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
  EXPECT_EQ(v2, v1 %= 2);
}

TEST(MatrixTests, constructors) {
  try {
    Mf m1;
    EXPECT_EQ(0, m1.rows());
    EXPECT_EQ(0, m1.cols());
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m2(2, 4, 0);
    EXPECT_EQ(2, m2.rows());
    EXPECT_EQ(4, m2.cols());
    for (int i = 0; i < m2.rows(); i++) {
      for (int j = 0; j < m2.cols(); j++) {
        EXPECT_FLOAT_EQ(0, m2.at(i, j));
      }
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m3(3, 3, 1, Mf::DIAG);
    EXPECT_EQ(3, m3.rows());
    EXPECT_EQ(3, m3.cols());
    for (int i = 0; i < m3.rows(); i++) {
      for (int j = 0; j < m3.cols(); j++) {
        if (i != j) {
          EXPECT_FLOAT_EQ(0, m3.at(i, j));
        } else {
          EXPECT_FLOAT_EQ(1, m3.at(i, j));
        }
      }
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m4(2, 4);
    EXPECT_EQ(2, m4.rows());
    EXPECT_EQ(4, m4.cols());
    for (int i = 0; i < m4.rows(); i++) {
      for (int j = 0; j < m4.cols(); j++) {
        EXPECT_FLOAT_EQ(0, m4.at(i, j));
      }
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m5(3, 5, {});
    EXPECT_EQ(3, m5.rows());
    EXPECT_EQ(5, m5.cols());
    for (int i = 0; i < m5.rows(); i++) {
      for (int j = 0; j < m5.cols(); j++) {
        EXPECT_FLOAT_EQ(0, m5.at(i, j));
      }
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m6(3, 3, {1, 2, 3}, Mf::DIAG);
    EXPECT_EQ(3, m6.rows());
    EXPECT_EQ(3, m6.cols());
    EXPECT_FLOAT_EQ(1, m6.at(0, 0));
    EXPECT_FLOAT_EQ(2, m6.at(1, 1));
    EXPECT_FLOAT_EQ(3, m6.at(2, 2));
    for (int i = 0; i < m6.rows(); i++) {
      for (int j = 0; j < m6.cols(); j++) {
        if (i != j)
          EXPECT_FLOAT_EQ(0, m6.at(i, j));
      }
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m7(1, 3, {1, 2, 3}, Mf::NONE);
    EXPECT_EQ(1, m7.rows());
    EXPECT_EQ(3, m7.cols());
    EXPECT_FLOAT_EQ(1, m7.at(0, 0));
    EXPECT_FLOAT_EQ(2, m7.at(0, 1));
    EXPECT_FLOAT_EQ(3, m7.at(0, 2));
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m8(3, 1, {1, 2, 3}, Mf::NONE);
    EXPECT_EQ(3, m8.rows());
    EXPECT_EQ(1, m8.cols());
    EXPECT_FLOAT_EQ(1, m8.at(0, 0));
    EXPECT_FLOAT_EQ(2, m8.at(1, 0));
    EXPECT_FLOAT_EQ(3, m8.at(2, 0));
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, maxConstructors) {
  try {
    Mf m2(uint8_t(-1), uint8_t(-1), 0);
    EXPECT_EQ(uint8_t(-1), m2.rows());
    EXPECT_EQ(uint8_t(-1), m2.cols());
    for (int i = 0; i < m2.rows(); i++) {
      for (int j = 0; j < m2.cols(); j++) {
        EXPECT_FLOAT_EQ(0, m2.at(i, j));
      }
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m3(uint8_t(-1), uint8_t(-1), 1, Mf::DIAG);
    EXPECT_EQ(uint8_t(-1), m3.rows());
    EXPECT_EQ(uint8_t(-1), m3.cols());
    for (int i = 0; i < m3.rows(); i++) {
      for (int j = 0; j < m3.cols(); j++) {
        if (i != j) {
          EXPECT_FLOAT_EQ(0, m3.at(i, j));
        } else {
          EXPECT_FLOAT_EQ(1, m3.at(i, j));
        }
      }
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m4(uint8_t(-1), uint8_t(-1));
    EXPECT_EQ(uint8_t(-1), m4.rows());
    EXPECT_EQ(uint8_t(-1), m4.cols());
    for (int i = 0; i < m4.rows(); i++) {
      for (int j = 0; j < m4.cols(); j++) {
        EXPECT_FLOAT_EQ(0, m4.at(i, j));
      }
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m5(uint8_t(-1), uint8_t(-1), {});
    EXPECT_EQ(uint8_t(-1), m5.rows());
    EXPECT_EQ(uint8_t(-1), m5.cols());
    for (int i = 0; i < m5.rows(); i++) {
      for (int j = 0; j < m5.cols(); j++) {
        EXPECT_FLOAT_EQ(0, m5.at(i, j));
      }
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m6(uint8_t(-1), uint8_t(-1), vector<float>(uint8_t(-1), 1.5), Mf::DIAG);
    EXPECT_EQ(uint8_t(-1), m6.rows());
    EXPECT_EQ(uint8_t(-1), m6.cols());
    for (int i = 0; i < m6.rows(); i++) {
      for (int j = 0; j < m6.cols(); j++) {
        if (i != j)
          EXPECT_FLOAT_EQ(0, m6.at(i, j));
        else
          EXPECT_FLOAT_EQ(1.5, m6.at(i, j));
      }
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m7(1, uint8_t(-1), vector<float>(uint8_t(-1), -1.56), Mf::NONE);
    EXPECT_EQ(1, m7.rows());
    EXPECT_EQ(uint8_t(-1), m7.cols());
    for (uint8_t i = 0; i < m7.cols(); ++i)
      EXPECT_FLOAT_EQ(-1.56, m7.at(0, i));
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
  try {
    Mf m8(uint8_t(-1), 1, vector<float>(uint8_t(-1), -1.56), Mf::NONE);
    EXPECT_EQ(uint8_t(-1), m8.rows());
    EXPECT_EQ(1, m8.cols());
    for (uint8_t i = 0; i < m8.cols(); ++i)
      EXPECT_FLOAT_EQ(-1.56, m8.at(i, 0));
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, accessors) {
  try {
    Mf m1(2, 2, 0);
    for (int i = -4; i < 4; i++) {
      EXPECT_FLOAT_EQ(0, m1.at(i));
    }
    for (int i = -4; i < 4; i++) {
      m1.at(i) += 1;
    }
    for (int i = -4; i < 4; i++) {
      EXPECT_FLOAT_EQ(2, m1.at(i));
    }
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        EXPECT_FLOAT_EQ(2, m1(i, j));
      }
    }
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        m1(i, j) += 1;
      }
    }
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        EXPECT_FLOAT_EQ(3, m1(i, j));
      }
    }
    for (int i = -4; i < 4; i++) {
      EXPECT_FLOAT_EQ(3, m1(i));
    }
    for (int i = -4; i < 4; i++) {
      m1(i) += 1;
    }
    for (int i = -4; i < 4; i++) {
      EXPECT_FLOAT_EQ(5, m1(i));
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, maxAccessors) {
  try {
    Mf m1(uint8_t(-1), uint8_t(-1), 0);
    for (int i = -uint8_t(-1) * uint8_t(-1); i < uint8_t(-1) * uint8_t(-1); i++) {
      EXPECT_FLOAT_EQ(0, m1.at(i));
    }
    for (int i = -uint8_t(-1) * uint8_t(-1); i < uint8_t(-1) * uint8_t(-1); i++) {
      m1.at(i) += 1;
    }
    for (int i = -uint8_t(-1) * uint8_t(-1); i < uint8_t(-1) * uint8_t(-1); i++) {
      EXPECT_FLOAT_EQ(2, m1.at(i));
    }
    for (uint8_t i = 0; i < uint8_t(-1); i++) {
      for (uint8_t j = 0; j < uint8_t(-1); j++) {
        EXPECT_FLOAT_EQ(2, m1(i, j));
      }
    }
    for (uint8_t i = 0; i < uint8_t(-1); i++) {
      for (uint8_t j = 0; j < uint8_t(-1); j++) {
        m1(i, j) += 1;
      }
    }
    for (uint8_t i = 0; i < uint8_t(-1); i++) {
      for (uint8_t j = 0; j < uint8_t(-1); j++) {
        EXPECT_FLOAT_EQ(3, m1(i, j));
      }
    }
    for (int i = -uint8_t(-1) * uint8_t(-1); i < uint8_t(-1) * uint8_t(-1); i++) {
      EXPECT_FLOAT_EQ(3, m1(i));
    }
    for (int i = -uint8_t(-1) * uint8_t(-1); i < uint8_t(-1) * uint8_t(-1); i++) {
      m1(i) += 1;
    }
    for (int i = -uint8_t(-1) * uint8_t(-1); i < uint8_t(-1) * uint8_t(-1); i++) {
      EXPECT_FLOAT_EQ(5, m1(i));
    }
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, display) {
  try {
    Mf m1(2, 2, 1);
    ostringstream s;
    s << m1;
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, maxDisplay) {
  try {
    Mf m1(uint8_t(-1), uint8_t(-1), 1);
    ostringstream s;
    s << m1;
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, addition) {
  Mf m1(2, 2, 1);
  Mf m2(2, 2, 1);
  Mf m3(2, 2, 2);
  try {
    EXPECT_EQ(m3, m1 + m2);
    m1 += m2;
    EXPECT_EQ(m3, m1);
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    cout << "m3:\n" << m3 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, maxAddition) {
  Mf m1(uint8_t(-1), uint8_t(-1), 1);
  Mf m2(uint8_t(-1), uint8_t(-1), 1);
  Mf m3(uint8_t(-1), uint8_t(-1), 2);
  try {
    EXPECT_EQ(m3, m1 + m2);
    m1 += m2;
    EXPECT_EQ(m3, m1);
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    cout << "m3:\n" << m3 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, unaryMinus) {
  Mf m1(2, 2, 1);
  Mf m2(2, 2, -1);
  try {
    EXPECT_EQ(m2, -m1);
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, maxUnaryMinus) {
  Mf m1(uint8_t(-1), uint8_t(-1), 1);
  Mf m2(uint8_t(-1), uint8_t(-1), -1);
  try {
    EXPECT_EQ(m2, -m1);
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, product) {
  Mf m1(3, 3, 1);
  Mf m2(3, 3, 1);
  Mf m3(3, 3, 3);
  try {
    EXPECT_EQ(m3, m1 * m2);
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    cout << "m3:\n" << m3 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, maxProduct) {
  Mf m1(uint8_t(-1), uint8_t(-1), 1);
  Mf m2(uint8_t(-1), uint8_t(-1), 1);
  Mf m3(uint8_t(-1), uint8_t(-1), uint8_t(-1));
  try {
    EXPECT_EQ(m3, m1 * m2);
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    cout << "m3:\n" << m3 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, rank) {
  Mf m1(2, 2, 1, Mf::DIAG);
  Mf m2(2, 2, 1);
  try {
    EXPECT_FLOAT_EQ(2, m1.rank());
    EXPECT_FLOAT_EQ(1, m2.rank());
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, maxRank) {
  Mf m1(uint8_t(-1), uint8_t(-1), 1, Mf::DIAG);
  Mf m2(uint8_t(-1), uint8_t(-1), 1);
  try {
    EXPECT_FLOAT_EQ(uint8_t(-1), m1.rank());
    EXPECT_FLOAT_EQ(1, m2.rank());
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, columnOperations) {
  Mf m0(3, 2, 0);
  m0.at(0) = 1;
  m0.at(4) = 1;
  Mf m1(3, 2, 0);
  m1.at(0) = 1;
  m1.at(4) = 1;
  Mf m2(3, 1, 0);
  m2.at(2) = 1;
  Mf m3(3, 3, 1, Mf::DIAG);
  Mf m4(3, 1, 0);
  m4.at(1) = 1;
  Mf m5(3, 3, 1, Mf::DIAG);
  try {
    EXPECT_EQ(m3, m1.concatRight(m2));
    EXPECT_EQ(m0, m1.removeCol(-1));
    EXPECT_EQ(m4, m1.removeCol(0));
    EXPECT_EQ(m4, m3.removeCols(std::vector<int>{0, -1}));
    EXPECT_EQ(m4, m5.removeCols(boost::container::vector<bool>{false, true, false}));
  } catch (logic_error &e) {
    cout << "m0:\n" << m0 << endl;
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    cout << "m3:\n" << m3 << endl;
    cout << "m5:\n" << m5 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, maxColumnOperations) {
  Mf m0(uint8_t(-1), uint8_t(-1), 0);
  for (uint8_t i = 0; i < uint8_t(-1); ++i) {
    m0.at(i, uint8_t(-2)) = 1;
  }
  Mf m1(uint8_t(-1), uint8_t(-2), 0);
  Mf m2(uint8_t(-1), 1, 1);
  Mf m3(uint8_t(-1), uint8_t(-1), 0);
  for (uint8_t i = 0; i < uint8_t(-1); ++i) {
    for (uint8_t j = 0; j < uint8_t(-1); ++j) {
      m3.at(i, j) = i;
    }
  }
  Mf m4(uint8_t(-1), uint8_t(-2), 0);
  for (uint8_t i = 0; i < uint8_t(-1); ++i) {
    for (uint8_t j = 0; j < uint8_t(-2); ++j) {
      m4.at(i, j) = i;
    }
  }
  Mf m5(uint8_t(-1), uint8_t(-3), 0);
  for (uint8_t i = 0; i < uint8_t(-1); ++i) {
    for (uint8_t j = 1; j < uint8_t(-2); ++j) {
      m5.at(i, j - uint8_t(1)) = i;
    }
  }
  Mf m6 = m3;
  try {
    EXPECT_EQ(m0, m1.concatRight(m2));
    EXPECT_EQ(m0.removeCol(uint8_t(-2)), m1.removeCol(-1));
    EXPECT_EQ(m5, m3.removeCols(std::vector<int>{0, -1}));
    m3 = m6;
    boost::container::vector<bool> mask(uint8_t(-1), true);
    mask.at(0) = false;
    mask.at(uint8_t(-2)) = false;
    EXPECT_EQ(m5, m3.removeCols(mask));
  } catch (logic_error &e) {
    cout << "m0:\n" << m0 << endl;
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    cout << "m3:\n" << m3 << endl;
    cout << "m5:\n" << m5 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, rowOperations) {
  Mf m0(2, 3, 0);
  m0.at(0) = 1;
  m0.at(3) = 1;
  Mf m1(2, 3, 0);
  m1.at(0) = 1;
  m1.at(3) = 1;
  Mf m2(1, 3, 0);
  m2.at(2) = 1;
  Mf m3(3, 3, 1, Mf::DIAG);
  Mf m4(1, 3, 0);
  m4.at(1) = 1;
  Mf m5(3, 3, 1, Mf::DIAG);
  try {
    EXPECT_EQ(m3, m1.concatDown(m2));
    EXPECT_EQ(m0, m1.removeRow(-1));
    EXPECT_EQ(m4, m1.removeRow(0));
    EXPECT_EQ(m4, m3.removeRows(std::vector<int>{0, -1}));
    EXPECT_EQ(m4, m5.removeRows(boost::container::vector<bool>{false, true, false}));
  } catch (logic_error &e) {
    cout << "m0:\n" << m0 << endl;
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    cout << "m3:\n" << m3 << endl;
    cout << "m5:\n" << m5 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, maxRowOperations) {
  Mf m0(uint8_t(-1), uint8_t(-1), 0);
  for (uint8_t i = 0; i < uint8_t(-1); ++i) {
    m0.at(uint8_t(-2), i) = 1;
  }
  Mf m1(uint8_t(-2), uint8_t(-1), 0);
  Mf m2(1, uint8_t(-1), 1);
  Mf m3(uint8_t(-1), uint8_t(-1), 0);
  for (uint8_t i = 0; i < uint8_t(-1); ++i) {
    for (uint8_t j = 0; j < uint8_t(-1); ++j) {
      m3.at(i, j) = i;
    }
  }
  Mf m4(uint8_t(-2), uint8_t(-1), 0);
  for (uint8_t i = 0; i < uint8_t(-2); ++i) {
    for (uint8_t j = 0; j < uint8_t(-1); ++j) {
      m4.at(i, j) = i;
    }
  }
  Mf m5(uint8_t(-3), uint8_t(-1), 0);
  for (uint8_t i = 1; i < uint8_t(-2); ++i) {
    for (uint8_t j = 0; j < uint8_t(-1); ++j) {
      m5.at(i - uint8_t(1), j) = i;
    }
  }
  Mf m6 = m3;
  try {
    EXPECT_EQ(m0, m1.concatDown(m2));
    EXPECT_EQ(m0.removeRow(uint8_t(-2)), m1.removeRow(-1));
    EXPECT_EQ(m5, m3.removeRows(std::vector<int>{0, -1}));
    m3 = m6;
    boost::container::vector<bool> mask(uint8_t(-1), true);
    mask.at(0) = false;
    mask.at(uint8_t(-2)) = false;
    EXPECT_EQ(m5, m3.removeRows(mask));
  } catch (logic_error &e) {
    cout << "m0:\n" << m0 << endl;
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    cout << "m3:\n" << m3 << endl;
    cout << "m5:\n" << m5 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, transpose) {
  Mf m1(3, 1, {1, 2, 3});
  Mf m2(1, 3, {1, 2, 3});
  Mf m3(1, 3, {1, 2, 3});
  try {
    EXPECT_EQ(m1, m2.transpose());
    EXPECT_EQ(m3, m2.transpose());
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, maxTranspose) {
  std::vector<float> vals(uint8_t(-1));
  std::generate(vals.begin(), vals.end(), std::rand);
  Mf m1(uint8_t(-1), 1, vals);
  Mf m2(1, uint8_t(-1), vals);
  Mf m3(1, uint8_t(-1), vals);
  try {
    EXPECT_EQ(m1, m2.transpose());
    EXPECT_EQ(m3, m2.transpose());
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, diag) {
  Mf m1(3, 1, {1, 2, 3});
  Mf m2(3, 3, {1, 2, 3}, Mf::DIAG);
  try {
    EXPECT_EQ(m2, m1.diag());
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, maxDiag) {
  std::vector<float> diag(uint8_t(-1));
  std::generate(diag.begin(), diag.end(), std::rand);
  Mf m1(uint8_t(-1), 1, diag);
  Mf m2(uint8_t(-1), uint8_t(-1), diag, Mf::DIAG);
  try {
    EXPECT_EQ(m2, m1.diag());
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(MatrixTests, crossProduct) {
  Mf m1(3, 1, 0);
  m1.at(0) = 1;
  Mf m2(3, 1, 0);
  m2.at(1) = 1;
  Mf m3(3, 1, 0);
  m3.at(2) = 1;
  try {
    EXPECT_EQ(m3, m1.cross(m2));
    EXPECT_EQ(m1, m2.cross(m3));
    EXPECT_EQ(m2, m3.cross(m1));
    EXPECT_EQ(-m3, m2.cross(m1));
    EXPECT_EQ(-m1, m3.cross(m2));
    EXPECT_EQ(-m2, m1.cross(m3));
  } catch (logic_error &e) {
    cout << "m1:\n" << m1 << endl;
    cout << "m2:\n" << m2 << endl;
    cout << "m3:\n" << m3 << endl;
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(ParamTests, paramGetter) {
  ros::param::set("/integer", 1);
  ros::param::set("~private_integer", 2);
  int i, j, k, l, m = 10;
  std::vector<int> v(5, 0);
  std::vector<int> w(5, 1);
  try {
    // Calls no default function with existing parameter
    EXPECT_TRUE(forssea_utilities::getParam("integer", i));
    EXPECT_EQ(i, 1);
    for (int a = 0; a < 5; ++a) {
      EXPECT_TRUE(forssea_utilities::getParam("integer", v[a], 5));
    }
    EXPECT_EQ(v, w);
    // Calls default param function with lvalue as default and non existing parameter
    EXPECT_TRUE(forssea_utilities::getParam("integer_", j, m));
    EXPECT_EQ(j, m);
    // Calls default param function with rvalue as default and non existing parameter
    EXPECT_TRUE(forssea_utilities::getParam("integer_", k, 5));
    EXPECT_EQ(k, 5);
    // Calls no default param function with non existing parameter
    EXPECT_FALSE(forssea_utilities::getParam("integer_", l));
    EXPECT_TRUE(forssea_utilities::getParam("private_integer", l));
    EXPECT_EQ(l, 2);
  } catch (logic_error &e) {
    ADD_FAILURE() << "Uncaught exception: " << e.what();
  }
}

TEST(ConversionTests, quaternionToEuler)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion q;
  q.w = 1.0;
  forssea_utilities::getRPY(q, roll, pitch, yaw);
  EXPECT_EQ(roll, 0.0);
  EXPECT_EQ(pitch, 0.0);
  EXPECT_EQ(yaw, 0.0);

  q.x = 0.4662279;
  q.y = 0.22729203;
  q.z = 0.66958072;
  q.w = 0.53163103;
  forssea_utilities::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(roll, M_PI/3, 0.001);
  EXPECT_NEAR(pitch, -M_PI/8, 0.001);
  EXPECT_NEAR(yaw, M_PI/2, 0.001);


}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tests");
  return RUN_ALL_TESTS();
}
