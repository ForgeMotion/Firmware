// Quaternion Math library partially taken from I2CDev library
// Quaternion to extrinsic X-Y-Z Euler angles function and 
// yaw/pitch/roll deviation functions written and added by Tyler McGahee 2016

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _QUATERNION_MATH_H_
#define _QUATERNION_MATH_H_

class Quaternion {
    public:
        float w;
        float x;
        float y;
        float z;
        
        Quaternion() {
            w = 1.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }
        
        Quaternion(float nw, float nx, float ny, float nz) {
            w = nw;
            x = nx;
            y = ny;
            z = nz;
        }

        Quaternion getProduct(Quaternion q) {
            // Quaternion multiplication is defined by:
            //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
            return Quaternion(
                w*q.w - x*q.x - y*q.y - z*q.z,  // new w
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w); // new z
        }

        Quaternion getConjugate() {
            return Quaternion(w, -x, -y, -z);
        }
        
        float getNorm() {
            return w*w + x*x + y*y + z*z;
        }
        
        float getMagnitude() {
            return sqrt(w*w + x*x + y*y + z*z);
        }
        
        void normalize() {
            float m = getMagnitude();
            w /= m;
            x /= m;
            y /= m;
            z /= m;
        }
        
        Quaternion getNormalized() {
            Quaternion r(w, x, y, z);
            r.normalize();
            return r;
        }
};

// Revised function: Gives right handed extrinsic X-Y-Z (same as intrinsic Z-Y'-X'')
void getEulerFromQuat(float *data, Quaternion *q) {
    data[0] = atan2(2 * (q->x * q->y + q->w * q->z), 2 * (q->w * q->w + q->x * q->x) - 1);
    data[1] = -asin(-2 * (q->x * q->z - q->w * q->y));
    data[2] = atan2(2 * (q->y * q->z + q->w * q->x), 2 * (q->w * q->w + q->z * q->z) - 1);
}

void getYPRDev(Quaternion *initPose, Quaternion *currentPose, float *yprDev){
    //qDev =  conjugate(qInitPos) * qCurrent
    Quaternion qDev = initPose->getConjugate();
    qDev = qDev.getProduct(*currentPose);
    
    getEulerFromQuat(yprDev, &qDev);
    for(int i=0; i<3; i++) yprDev[i] *= 180.0/M_PI;
}


#endif /* _QUATERNION_MATH_H_ */
