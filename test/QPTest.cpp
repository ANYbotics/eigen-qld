/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2012-2019, CNRS-UM LIRMM, CNRS-AIST JRL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Dense>

// EigenQP
#include "eigen-qld/QLD.h"

struct QP1 {
  QP1() {
    nrvar = 6;
    nreq = 3;
    nrineq = 2;

    Q.resize(nrvar, nrvar);
    Aeq.resize(nreq, nrvar);
    Aineq.resize(nrineq, nrvar);
    A.resize(nreq + nrineq, nrvar);

    C.resize(nrvar);
    Beq.resize(nreq);
    Bineq.resize(nrineq);
    B.resize(nreq + nrineq);
    XL.resize(nrvar);
    XU.resize(nrvar);
    X.resize(nrvar);

    Aeq << 1., -1., 1., 0., 3., 1., -1., 0., -3., -4., 5., 6., 2., 5., 3., 0., 1., 0.;
    Beq << 1., 2., 3.;

    Aineq << 0., 1., 0., 1., 2., -1., -1., 0., 2., 1., 1., 0.;
    Bineq << -1., 2.5;

    A.topRows(nreq) = Aeq;
    A.bottomRows(nrineq) = -Aineq;

    B.head(nreq) = -Beq;
    B.tail(nrineq) = Bineq;

    // with  x between ci and cs:
    XL << -1000., -10000., 0., -1000., -1000., -1000.;
    XU << 10000., 100., 1.5, 100., 100., 1000.;

    // and minimize 0.5*x'*Q*x + p'*x with
    C << 1., 2., 3., 4., 5., 6.;
    Q.setIdentity();

    X << 1.7975426, -0.3381487, 0.1633880, -4.9884023, 0.6054943, -3.1155623;
  }

  int nrvar, nreq, nrineq;
  Eigen::MatrixXd Q, Aeq, Aineq, A;
  Eigen::VectorXd C, Beq, Bineq, B, XL, XU, X;
};

void ineqWithXBounds(Eigen::MatrixXd& Aineq, Eigen::VectorXd& Bineq, const Eigen::VectorXd& XL, const Eigen::VectorXd& XU) {
  double inf = std::numeric_limits<double>::infinity();

  std::vector<std::pair<int, double>> lbounds, ubounds;

  for (int i = 0; i < XL.rows(); ++i) {
    if (XL[i] != -inf) lbounds.emplace_back(i, XL[i]);
    if (XU[i] != inf) ubounds.emplace_back(i, XU[i]);
  }

  long int nrconstr = Bineq.rows() + static_cast<long int>(lbounds.size()) + static_cast<long int>(ubounds.size());

  Eigen::MatrixXd A(Eigen::MatrixXd::Zero(nrconstr, Aineq.cols()));
  Eigen::VectorXd B(Eigen::VectorXd::Zero(nrconstr));

  A.block(0, 0, Aineq.rows(), Aineq.cols()) = Aineq;
  B.segment(0, Bineq.rows()) = Bineq;

  int start = static_cast<int>(Aineq.rows());

  for (int i = 0; i < static_cast<int>(lbounds.size()); ++i) {
    const auto& b = lbounds[i];
    A(start, b.first) = -1.;
    B(start) = -b.second;
    ++start;
  }

  for (int i = 0; i < static_cast<int>(ubounds.size()); ++i) {
    const auto& b = ubounds[i];
    A(start, b.first) = 1.;
    B(start) = b.second;
    ++start;
  }

  Aineq = A;
  Bineq = B;
}

TEST(QPTest, QLD) {  // NOLINT
  QP1 qp1;
  Eigen::QLD qld(qp1.nrvar, qp1.nreq, qp1.nrineq);
  qld.solve(qp1.Q, qp1.C, qp1.Aeq, qp1.Beq, qp1.Aineq, qp1.Bineq, qp1.XL, qp1.XU);
  ASSERT_NEAR((qld.result() - qp1.X).norm(), 0., 1e-6);
  Eigen::QLDDirect qldd(qp1.nrvar, qp1.nreq, qp1.nrineq);
  qldd.solve(qp1.Q, qp1.C, qp1.A, qp1.B, qp1.XL, qp1.XU, 3);
  ASSERT_NEAR((qld.result() - qp1.X).norm(), 0., 1e-6);
}

TEST(QPTest, QLDSize) {  // NOLINT
  QP1 qp1;
  Eigen::QLD qld(qp1.nrvar, qp1.nreq + 10, qp1.nrineq + 22);
  qld.solve(qp1.Q, qp1.C, qp1.Aeq, qp1.Beq, qp1.Aineq, qp1.Bineq, qp1.XL, qp1.XU);
  ASSERT_NEAR((qld.result() - qp1.X).norm(), 0., 1e-6);
}

TEST(QPTest, IneqWithXBounds) {  // NOLINT
  QP1 qp1;
  ineqWithXBounds(qp1.Aineq, qp1.Bineq, qp1.XL, qp1.XU);
  double inf = std::numeric_limits<double>::infinity();
  for (int i = 0; i < qp1.nrvar; ++i) {
    qp1.XL[i] = -inf;
    qp1.XU[i] = inf;
  }
  int nrineq = static_cast<int>(qp1.Aineq.rows());
  Eigen::QLD qld(qp1.nrvar, qp1.nreq, nrineq);
  qld.solve(qp1.Q, qp1.C, qp1.Aeq, qp1.Beq, qp1.Aineq, qp1.Bineq, qp1.XL, qp1.XU);
  ASSERT_NEAR((qld.result() - qp1.X).norm(), 0., 1e-6);
}
