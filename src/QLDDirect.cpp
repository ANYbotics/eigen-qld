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

// associated header
#include "eigen-qld/QLD.h"

namespace Eigen {

QLDDirect::QLDDirect() : fdOut_(0), verbose_(false), fail_(0), U_(), WAR_(), IWAR_() {}

QLDDirect::QLDDirect(int nrvar, int nreq, int nrineq, int ldq, int lda, bool verbose)
    : fdOut_(0), verbose_(verbose ? 1 : 0), fail_(0), U_(), WAR_(), IWAR_() {
  problem(nrvar, nreq, nrineq, ldq, lda);
}

void QLDDirect::fdOut(int fd) {
  fdOut_ = fd;
}

int QLDDirect::fdOut() const {
  return fdOut_;
}

void QLDDirect::verbose(bool v) {
  verbose_ = v ? 1 : 0;
}

bool QLDDirect::verbose() const {
  return verbose_;
}

int QLDDirect::fail() const {
  return fail_;
}

void QLDDirect::problem(int nrvar, int nreq, int nrineq, int ldq, int lda) {
  int nrconstr = nreq + nrineq;

  if (ldq < nrvar) ldq = nrvar;
  if (lda < nrconstr) lda = nrconstr;

  int MMAX = lda == 0 ? 1 : lda;
  int NMAX = ldq == 0 ? 1 : ldq;

  X_.resize(nrvar);

  U_.resize(nrconstr + 2 * nrvar);
  int LWAR = static_cast<int>(std::ceil((3. * NMAX * NMAX) / 2. + 10. * NMAX + MMAX + nrconstr + 1.));
  WAR_.resize(LWAR);
  IWAR_.resize(nrvar);
}

const VectorXd& QLDDirect::result() const {
  return X_;
}

const VectorXd& QLDDirect::multipliers() const {
  return U_;
}

}  // namespace Eigen
