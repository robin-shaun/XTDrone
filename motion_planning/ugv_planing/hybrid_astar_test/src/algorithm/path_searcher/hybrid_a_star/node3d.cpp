#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/node3d.h"

using namespace HybridAStar;

const int Node3D::dir = 3;      // 单一方向分成等分
// R = 6, 6.75 DEG
const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};

// 在珊格范围内
bool Node3D::isOnGrid(const int width, const int height) const {
  return (x >= 0 && x < width  &&                                       // 珊格x范围内
          y >= 0 && y < height &&                                       // 珊格y范围内
          (int)(t / Constants::deltaHeadingRad) >= 0 &&                 //
          (int)(t / Constants::deltaHeadingRad) < Constants::headings);
}
// 在？？范围内
bool Node3D::isInRange(const Node3D& goal) const {
  int random = rand() % 10 + 1;
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;
  return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;
}
// 阔列点
Node3D* Node3D::createSuccessor(const int i) {
//    std::cout << "i : "<< i << std::endl;
//    std::cout << x << ", " << y << ", " << Helper::normalizeHeadingRad(t) << std::endl;
    float xSucc;
    float ySucc;
    float tSucc;

    float d;
    if(i < 3) d = Constants::cellSize * 0.7068582;
    else d = - Constants::cellSize * 0.7068582;

    static const float alpha_i[] = { 0.0, 1.0, -1.0, 0.0, 1.0, -1.0};
    float alpha = alpha_i[i] * Constants::vehicle_max_rudder * M_PI / 180.0;
    float beta = d * std::tan(alpha) / Constants::vehicle_wheelbase;
    float theta = t;
    if (t > M_PI)
        theta   = - (2 * M_PI - t);
    if (std::abs(beta) < 0.001) {
        xSucc = x + d * cos(theta);
        ySucc = y + d * sin(theta);
        tSucc = Helper::normalizeHeadingRad(theta);
    }
    else{
        float r = Constants::vehicle_wheelbase / std::tan(alpha);
        xSucc = x + r * sin(theta + beta) - r * sin(theta);
        ySucc = y - r * cos(theta + beta) + r * cos(theta);
        beta = Helper::normalizeHeadingRad(beta);
        tSucc = Helper::normalizeHeadingRad(theta + beta);
    }
//    std::cout << xSucc << ", " << ySucc << ", " << tSucc << std::endl;

//    if (i < 3) {
//        xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
//        ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
//        tSucc = Helper::normalizeHeadingRad(t + dt[i]);
//    }
//    // backwards
//    else {
//        xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
//        ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
//        tSucc = Helper::normalizeHeadingRad(t - dt[i - 3]);
//    }
//    std::cout << xSucc << ", " << ySucc << ", " << tSucc << std::endl;
//    std::cout << std::endl;

    return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}

// 预瞄点的代价
void Node3D::updateG() {
    // 相对于上一个点是前进的
    if (prim < 3) {
        // 相对于上一个点，有方向偏移
        if (pred->prim != prim) {
            // 上一个点是倒退的，但是该点是前进的，速度方向相反了
            if (pred->prim > 2)
                g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
            // 上一个点是前进的，该点也是前进的，速度方向不变
            else
                g += dx[0] * Constants::penaltyTurning;
        }
        // 相当于上一个点，方向不变
        else
            g += dx[0];
    }
    // 相对于上一个点是后退的
    else {
        // 相对于上一个点，有方向偏移
        if (pred->prim != prim) {
            // 上一个点是前进的，但是该点是后退的，速度方向相反了
            if (pred->prim < 3)
                g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
            // 上一个点是后退的，但是该点是后退的，速度方向不变
            else
                g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
        }
        // 相当于上一个点，方向不变，但是是后退的
        else
            g += dx[0] * Constants::penaltyReversing;
    }
}

// 判断是否是同一个珊格点
bool Node3D::operator == (const Node3D& rhs) const {
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}
