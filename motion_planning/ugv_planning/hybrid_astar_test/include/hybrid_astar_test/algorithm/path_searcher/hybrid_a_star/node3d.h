#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include "hybrid_astar_test/global_definition/constants.hpp"
#include "hybrid_astar_test/utility/helper.hpp"

namespace HybridAStar {
class Node3D {
public:
    Node3D(): Node3D(0, 0, 0, 0, 0, nullptr) {}
    Node3D(float x, float y, float t, float g, float h, const Node3D* pred, int prim = 0) {
        this->x = x;
        this->y = y;
        this->t = t;
        this->g = g;
        this->h = h;
        this->pred = pred;
        this->o = false;
        this->c = false;
        this->idx = -1;
        this->prim = prim;
    }

public:
    // get params
    float getX() const { return x; }
    float getY() const { return y; }
    float getT() const { return t; }
    float getG() const { return g; }
    float getH() const { return h; }
    float getC() const { return g + h; }
    int getIdx() const { return idx; }
    int getPrim() const { return prim; }
    bool isOpen() const { return o; }
    bool isClosed() const { return c; }
    const Node3D* getPred() const { return pred; }

public:
    // write params
    void setX(const float& x) { this->x = x; }
    void setY(const float& y) { this->y = y; }
    void setT(const float& t) { this->t = t; }
    void setG(const float& g) { this->g = g; }
    void setH(const float& h) { this->h = h; }
    int setIdx(int width, int height) { this->idx = (int)(t / Constants::deltaHeadingRad) * width * height + (int)(y) * width + (int)(x); return idx;}
    void open() { o = true; c = false;}
    void close() { c = true; o = false; }
    void setPred(const Node3D* pred) { this->pred = pred; }

    // UPDATE METHODS
    void updateG();
    bool operator == (const Node3D& rhs) const;   

    // RANGE CHECKING
    bool isInRange(const Node3D& goal) const;
    bool isOnGrid(const int width, const int height) const;
    Node3D* createSuccessor(const int i);

    static const int dir;
    static const float dx[];
    static const float dy[];
    static const float dt[];

private:
    float x;        // x
    float y;        // y
    float t;        // theta
    float g;        // the cost-so-far  
    float h;        // the cost-to-go   
    int idx;        //   x + map_width * y
    bool o;         
    bool c;         
    int prim;           
    const Node3D* pred; 
};
}
#endif // NODE3D_H
