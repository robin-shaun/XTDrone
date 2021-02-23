/*
 * Copyright (C) 2019  Rhys Mainwaring
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <iostream>
#include <string>

#include "wave_gazebo_plugins/Wavefield.hh"

using namespace asv;

///////////////////////////////////////////////////////////////////////////////
// Utilities
std::ostream& operator<<(std::ostream& os, const std::vector<double>& _vec)
{
  for (auto&& v : _vec ) // NOLINT
    os << v << ", ";
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const std::vector<Eigen::Vector2d>& _vec)
{
  for (auto&& v : _vec ) // NOLINT
    os << v.transpose() << ", ";
  return os;
}

///////////////////////////////////////////////////////////////////////////////
// Define tests

// One dimension, one wave component
TEST(Wavefield, WaveSolver1D)
{
  struct WaveParams
  {
    double a;
    double k;
    double omega;
    double phi;
  };

  auto dispersion = [=](auto omega)
  {
    return omega * omega / 9.8;
  };

  auto wave = [=](auto x, auto t, auto& wp)
  {
    const double theta = wp.k * x - wp.omega * t;
    const double s = std::sin(theta);
    const double px = x - wp.a * s;
    return px;
  };

  auto wave_f = [=](auto x, auto p, auto t, auto& wp)
  {
    const double theta = wp.k * x - wp.omega * t;
    const double s = std::sin(theta);
    const double f = p - x + wp.a * s;
    return f;
  };

  auto wave_df = [=](auto x, auto p, auto t, auto& wp)
  {
    const double theta = wp.k * x - wp.omega * t;
    const double c = std::cos(theta);
    const double df = wp.a * wp.k * c - 1;
    return df;
  };

  auto solver = [=](auto& func, auto& dfunc, auto x0, auto p,
                    auto t, auto& wp, auto tol, auto nmax)
  {
    int n = 0;
    double err = 1;
    double xn = x0;
    while (std::abs(err) > tol && n < nmax)
    {
      const double f = func(x0, p, t, wp);
      const double df = dfunc(x0, p, t, wp);
      const double d = -f/df;
      xn = x0 + d;
      x0 = xn;
      err = f;
      n++;
    }
    return xn;
  };

  WaveParams wp;
  wp.a = 3.0;
  wp.omega = 2*M_PI/4.0;
  wp.k = dispersion(wp.omega);
  wp.phi = 0.0;

  double t = 0;
  double x0 = 2.0;

  double p = wave(x0, t, wp);
  double f  = wave_f(x0, p, t, wp);
  double df = wave_df(x0, p, t, wp);

  const double tol = 1E-8;
  const double nmax = 20;
  double x = solver(wave_f, wave_df, p, p, t, wp, tol, nmax);

  const double eps = 1E-8;
  EXPECT_NEAR(wp.a, 3.0, eps);
  EXPECT_NEAR(wp.k, 0.251775622, eps);
  EXPECT_NEAR(wp.omega, 1.570796327, eps);
  EXPECT_NEAR(x0, 2.0, eps);
  EXPECT_NEAR(p, 0.552382941, eps);
  EXPECT_LE(f, eps);
  EXPECT_NEAR(df, -0.338428477, eps);
  EXPECT_NEAR(x, 2.0, eps);

  // std::cout << "a:      " << wp.a << std::endl;
  // std::cout << "k:      " << wp.k << std::endl;
  // std::cout << "omega:  " << wp.omega << std::endl;
  // std::cout << "x0:     " << x0 << std::endl;
  // std::cout << "p:      " << p << std::endl;
  // std::cout << "f:      " << f << std::endl;
  // std::cout << "df:     " << df << std::endl;
  // std::cout << "x:      " << x << std::endl;
}

// Two dimensions, one wave component
TEST(Wavefield, WaveSolver2D)
{
  struct WaveParams
  {
    double a;
    double k;
    double omega;
    double phi;
    Eigen::Vector2d dir;
  };

  auto dispersion = [=](auto omega)
  {
    return omega * omega / 9.8;
  };

  auto wave = [=](auto x, auto t, auto& wp)
  {
    const double theta = wp.k * x.dot(wp.dir)  - wp.omega * t;
    const double s = std::sin(theta);
    const double c = std::cos(theta);
    const double dx = wp.dir.x();
    const double dy = wp.dir.y();
    const Eigen::Vector3d p(
      x.x() - wp.a * dx * s,
      x.y() - wp.a * dy * s,
      wp.a * c);
    return p;
  };

  auto wave_f = [=](auto x, auto p, auto t, auto& wp)
  {
    const double theta = wp.k * x.dot(wp.dir)  - wp.omega * t;
    const double s = std::sin(theta);
    const double dx = wp.dir.x();
    const double dy = wp.dir.y();
    const Eigen::Vector2d f(
      p.x() - x.x() + wp.a * dx * s,
      p.y() - x.y() + wp.a * dy * s);
    return f;
  };

  auto wave_df = [=](auto x, auto p, auto t, auto& wp)
  {
    const double theta = wp.k * x.dot(wp.dir)  - wp.omega * t;
    const double c = std::cos(theta);
    const double dx = wp.dir.x();
    const double dy = wp.dir.y();
    const double akc = wp.a * wp.k * c;
    const double df1x = akc * dx * dx - 1.0;
    const double df1y = akc * dx * dy;
    const double df2x = df1y;
    const double df2y = akc * dy * dy - 1.0;
    Eigen::Matrix2d J;
    J(0, 0) = df1x;
    J(0, 1) = df1y;
    J(1, 0) = df2x;
    J(1, 1) = df2y;
    return J;
  };

  auto solver = [=](auto& func, auto& dfunc, auto x0, auto p,
                    auto t, auto& wp, auto tol, auto nmax)
  {
    int n = 0;
    double err = 1;
    auto xn = x0;
    while (std::abs(err) > tol && n < nmax)
    {
      const auto F = func(x0, p, t, wp);
      const auto J = dfunc(x0, p, t, wp);
      xn = x0 - J.inverse() * F;
      x0 = xn;
      err = F.norm();
      n++;
    }
    return xn;
  };

  WaveParams wp;
  wp.a = 3.0;
  wp.omega = 2*M_PI/4.0;
  wp.k = dispersion(wp.omega);
  wp.phi = 0.0;
  wp.dir = Eigen::Vector2d(1, 0);

  double t = 0;
  Eigen::Vector2d x0(2.0, 3.0);

  auto p3 = wave(x0, t, wp);
  Eigen::Vector2d p(p3(0), p3(1));
  auto F  = wave_f(x0, p, t, wp);
  auto J = wave_df(x0, p, t, wp);

  const double tol = 1E-8;
  const double nmax = 20;
  auto x = solver(wave_f, wave_df, p, p, t, wp, tol, nmax);

  const double eps = 1E-8;
  EXPECT_NEAR(wp.a, 3.0, eps);
  EXPECT_NEAR(wp.k, 0.251775622, eps);
  EXPECT_NEAR(wp.omega, 1.570796327, eps);
  EXPECT_DOUBLE_EQ(wp.dir.x(), 1.0);
  EXPECT_DOUBLE_EQ(wp.dir.y(), 0.0);
  EXPECT_DOUBLE_EQ(x0.x(), 2.0);
  EXPECT_DOUBLE_EQ(x0.y(), 3.0);
  EXPECT_DOUBLE_EQ(x.x(), 2.0);
  EXPECT_DOUBLE_EQ(x.y(), 3.0);
  EXPECT_LE(F.norm(), eps);

  // std::cout << "a:      " << wp.a << std::endl;
  // std::cout << "k:      " << wp.k << std::endl;
  // std::cout << "omega:  " << wp.omega << std::endl;
  // std::cout << "x0:     " << x0.transpose() << std::endl;
  // std::cout << "p3:     " << p3.transpose() << std::endl;
  // std::cout << "p:      " << p.transpose() << std::endl;
  // std::cout << "F:      " << F.transpose() << std::endl;
  // std::cout << "J:      " << J << std::endl;
  // std::cout << "x:      " << x.transpose() << std::endl;
}

// Two dimensions, many wave components
TEST(Wavefield, NWaveSolver2D)
{
  struct WaveParams
  {
    std::vector<double> a;
    std::vector<double> k;
    std::vector<double> omega;
    std::vector<double> phi;
    std::vector<Eigen::Vector2d> dir;
  };

  auto dispersion = [=](auto omega)
  {
    return omega * omega / 9.8;
  };

  auto wave = [=](auto x, auto t, auto& wp)
  {
    Eigen::Vector3d p(x.x(), x.y(), 0.0);
    const size_t n = wp.a.size();
    for (auto&& i = 0; i < n; ++i) // NOLINT
    {
      const double theta = wp.k[i] * x.dot(wp.dir[i])  - wp.omega[i] * t;
      const double s = std::sin(theta);
      const double c = std::cos(theta);
      const double dx = wp.dir[i].x();
      const double dy = wp.dir[i].y();
      const double a = wp.a[i];
      p += Eigen::Vector3d(
        - a * dx * s,
        - a * dy * s,
        a * c);
    }
    return p;
  };

  auto wave_f = [=](auto x, auto p, auto t, auto& wp)
  {
    Eigen::Vector2d f(p.x() - x.x(), p.y() - x.y());
    const size_t n = wp.a.size();
    for (auto&& i = 0; i < n; ++i) // NOLINT
    {
      const double theta = wp.k[i] * x.dot(wp.dir[i])  - wp.omega[i] * t;
      const double s = std::sin(theta);
      const double dx = wp.dir[i].x();
      const double dy = wp.dir[i].y();
      const double a = wp.a[i];
      f += Eigen::Vector2d(
        a * dx * s,
        a * dy * s);
    }
    return f;
  };

  auto wave_df = [=](auto x, auto p, auto t, auto& wp)
  {
    Eigen::Matrix2d J;
    J(0, 0) = -1;
    J(0, 1) =  0;
    J(1, 0) =  0;
    J(1, 1) = -1;
    const size_t n = wp.a.size();
    for (auto&& i = 0; i < n; ++i) // NOLINT
    {
      const double theta = wp.k[i] * x.dot(wp.dir[i])  - wp.omega[i] * t;
      const double c = std::cos(theta);
      const double dx = wp.dir[i].x();
      const double dy = wp.dir[i].y();
      const double akc = wp.a[i] * wp.k[i] * c;
      const double df1x = akc * dx * dx;
      const double df1y = akc * dx * dy;
      const double df2x = df1y;
      const double df2y = akc * dy * dy;
      J(0, 0) += df1x;
      J(0, 1) += df1y;
      J(1, 0) += df2x;
      J(1, 1) += df2y;
    }
    return J;
  };

  auto solver = [=](auto& func, auto& dfunc, auto x0, auto p,
                    auto t, auto& wp, auto tol, auto nmax)
  {
    int n = 0;
    double err = 1;
    auto xn = x0;
    while (std::abs(err) > tol && n < nmax)
    {
      const auto F = func(x0, p, t, wp);
      const auto J = dfunc(x0, p, t, wp);
      xn = x0 - J.inverse() * F;
      x0 = xn;
      err = F.norm();
      n++;
    }
    return xn;
  };

  WaveParams wp;
  // wp.a = { 3.0, 0.0, 0.0 };
  // wp.omega = { 2*M_PI/4.0, 2*M_PI, 2*M_PI };
  wp.a = { 1.0, 2.0, 3.0 };
  wp.omega = { 2*M_PI/50.0, 2*M_PI/10.0, 2*M_PI/20.0 };
  wp.k = { dispersion(wp.omega[0]), dispersion(wp.omega[1]),
           dispersion(wp.omega[2]) };
  wp.phi = { 0.0, 0.0, 0.0 };
  wp.dir = { Eigen::Vector2d(1, 0), Eigen::Vector2d(1, 0),
             Eigen::Vector2d(1, 0) };

  double t = 0;
  Eigen::Vector2d x0(2.0, 3.0);

  auto p3 = wave(x0, t, wp);
  Eigen::Vector2d p(p3(0), p3(1));
  auto F  = wave_f(x0, p, t, wp);
  auto J = wave_df(x0, p, t, wp);

  const double tol = 1E-8;
  const double nmax = 20;
  auto x = solver(wave_f, wave_df, p, p, t, wp, tol, nmax);

  const double eps = 1E-8;
  EXPECT_DOUBLE_EQ(x0.x(), 2.0);
  EXPECT_DOUBLE_EQ(x0.y(), 3.0);
  EXPECT_DOUBLE_EQ(x.x(), 2.0);
  EXPECT_DOUBLE_EQ(x.y(), 3.0);
  EXPECT_LE(F.norm(), eps);

  // std::cout << "a:      " << wp.a << std::endl;
  // std::cout << "k:      " << wp.k << std::endl;
  // std::cout << "omega:  " << wp.omega << std::endl;
  // std::cout << "dir:    " << wp.dir << std::endl;
  // std::cout << "x0:     " << x0.transpose() << std::endl;
  // std::cout << "p3:     " << p3.transpose() << std::endl;
  // std::cout << "p:      " << p.transpose() << std::endl;
  // std::cout << "F:      " << F.transpose() << std::endl;
  // std::cout << "J:      " << J << std::endl;
  // std::cout << "x:      " << x.transpose() << std::endl;
}

// Two dimensions, many wave components, combined function
// and jacobian calculation
TEST(Wavefield, NWaveFdFSolver2D)
{
  struct WaveParams
  {
    std::vector<double> a;
    std::vector<double> k;
    std::vector<double> omega;
    std::vector<double> phi;
    std::vector<Eigen::Vector2d> dir;
  };

  auto dispersion = [=](auto omega)
  {
    return omega * omega / 9.8;
  };

  auto wave = [=](auto x, auto t, auto& wp)
  {
    Eigen::Vector3d p(x.x(), x.y(), 0.0);
    const size_t n = wp.a.size();
    for (auto&& i = 0; i < n; ++i) // NOLINT
    {
      const double theta = wp.k[i] * x.dot(wp.dir[i])  - wp.omega[i] * t;
      const double s = std::sin(theta);
      const double c = std::cos(theta);
      const double dx = wp.dir[i].x();
      const double dy = wp.dir[i].y();
      const double a = wp.a[i];
      p += Eigen::Vector3d(
        - a * dx * s,
        - a * dy * s,
        a * c);
    }
    return p;
  };

  auto wave_fdf = [=](auto x, auto p, auto t, auto& wp, auto& F, auto& J)
  {
    F(0) = p.x() - x.x();
    F(1) = p.y() - x.y();
    J(0, 0) = -1;
    J(0, 1) =  0;
    J(1, 0) =  0;
    J(1, 1) = -1;
    const size_t n = wp.a.size();
    for (auto&& i = 0; i < n; ++i) // NOLINT
    {
      const double theta = wp.k[i] * x.dot(wp.dir[i])  - wp.omega[i] * t;
      const double s = std::sin(theta);
      const double c = std::cos(theta);
      const double dx = wp.dir[i].x();
      const double dy = wp.dir[i].y();
      const double a  = wp.a[i];
      const double akc = a * wp.k[i] * c;
      const double df1x = akc * dx * dx;
      const double df1y = akc * dx * dy;
      const double df2x = df1y;
      const double df2y = akc * dy * dy;
      F(0) += a * dx * s;
      F(1) += a * dy * s;
      J(0, 0) += df1x;
      J(0, 1) += df1y;
      J(1, 0) += df2x;
      J(1, 1) += df2y;
    }
  };

  auto solver = [=](auto& fdfunc, auto x0, auto p, auto t, auto& wp,
                    auto tol, auto nmax)
  {
    int n = 0;
    double err = 1;
    auto xn = x0;
    Eigen::Vector2d F;
    Eigen::Matrix2d J;
    while (std::abs(err) > tol && n < nmax)
    {
      fdfunc(x0, p, t, wp, F, J);
      xn = x0 - J.inverse() * F;
      x0 = xn;
      err = F.norm();
      n++;
    }
    return xn;
  };

  WaveParams wp;
  wp.a = { 1.0, 2.0, 3.0 };
  wp.omega = { 2*M_PI/50.0, 2*M_PI/10.0, 2*M_PI/20.0 };
  wp.k = { dispersion(wp.omega[0]), dispersion(wp.omega[1]),
           dispersion(wp.omega[2]) };
  wp.phi = { 0.0, 0.0, 0.0 };
  wp.dir = { Eigen::Vector2d(1, 0), Eigen::Vector2d(1, 0),
             Eigen::Vector2d(1, 0) };

  double t = 0;
  Eigen::Vector2d x0(2.0, 3.0);

  auto p3 = wave(x0, t, wp);
  Eigen::Vector2d p(p3(0), p3(1));
  Eigen::Vector2d F;
  Eigen::Matrix2d J;
  wave_fdf(x0, p, t, wp, F, J);

  const double tol = 1E-8;
  const double nmax = 20;
  auto x = solver(wave_fdf, p, p, t, wp, tol, nmax);

  const double eps = 1E-8;
  EXPECT_DOUBLE_EQ(x0.x(), 2.0);
  EXPECT_DOUBLE_EQ(x0.y(), 3.0);
  EXPECT_DOUBLE_EQ(x.x(), 2.0);
  EXPECT_DOUBLE_EQ(x.y(), 3.0);
  EXPECT_LE(F.norm(), eps);
}

///////////////////////////////////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

