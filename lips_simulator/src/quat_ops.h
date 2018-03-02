#pragma once
#include <string>
#include <sstream>
#include <iostream>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#pragma GCC diagnostic pop
#include <Eigen/Dense>


/**
Vertex based on
**/

using namespace std;

inline Eigen::Matrix<double,3,3> rotx(double t) {

    Eigen::Matrix<double, 3, 3> r;
    double ct = cos(t);
    double st = sin(t);
    r << 1.0, 0.0, 0.0,
            0, ct, -st,
            0, st, ct;
    return r;
}

inline Eigen::Matrix<double,3,3> roty(double t){

    Eigen::Matrix<double, 3, 3> r;
    double ct = cos(t);
    double st = sin(t);
    r <<    ct,	0.0,	st,
            0.0,	1.0,	0.0,
            -st,	0.0, 	ct;

    return r;
}

inline Eigen::Matrix<double,3,3> rotz(double t){

    Eigen::Matrix<double, 3, 3> r;
    double ct = cos(t);
    double st = sin(t);
    r <<    ct,	-st, 0.0,
            st,	ct,	0.0,
            0.0, 0.0, 1.0;

    return r;
}


inline Eigen::Matrix<double, 4, 1> rot_2_quat(const Eigen::MatrixXd &rot) {

    assert(rot.cols() == 3);
    assert(rot.rows() == 3);
    Eigen::Matrix<double, 4, 1> q;
    double T = rot.trace();
    if ((rot(0, 0) > T) && (rot(0, 0) > rot(1, 1)) && (rot(0, 0) > rot(2, 2))) {
        //cout << "case 1- " << endl;
        q(0) = sqrt((1 + (2 * rot(0, 0)) - T) / 4);
        q(1) = (1 / (4 * q(0))) * (rot(0, 1) + rot(1, 0));
        q(2) = (1 / (4 * q(0))) * (rot(0, 2) + rot(2, 0));
        q(3) = (1 / (4 * q(0))) * (rot(1, 2) - rot(2, 1));

    }
    else if ((rot(1, 1) >= T) && (rot(1, 1) >= rot(0, 0)) && (rot(1, 1) >= rot(2, 2))) {
        //cout << "case 2- " << endl;
        q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
        q(0) = (1 / (4 * q(1))) * (rot(0, 1) + rot(1, 0));
        q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
        q(3) = (1 / (4 * q(1))) * (rot(2, 0) - rot(0, 2));
    }
    else if ((rot(2, 2) > T) && (rot(2, 2) > rot(0, 0)) && (rot(2, 2) > rot(1, 1))) {
        //cout << "case 3- " << endl;
        q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
        q(0) = (1 / (4 * q(2))) * (rot(0, 2) + rot(2, 0));
        q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
        q(3) = (1 / (4 * q(2))) * (rot(0, 1) - rot(1, 0));
    }
    else {
        //cout << "case 4- " << endl;
        q(3) = sqrt((1 + T) / 4);
        q(0) = (1 / (4 * q(3))) * (rot(1, 2) - rot(2, 1));
        q(1) = (1 / (4 * q(3))) * (rot(2, 0) - rot(0, 2));
        q(2) = (1 / (4 * q(3))) * (rot(0, 1) - rot(1, 0));
    }
    if (q(3) < 0) {
        q = -q;
    }



    q = q / (q.norm());
    return q;
}


inline Eigen::MatrixXd skew_x(Eigen::Matrix<double, 3, 1> w) {
    Eigen::Matrix<double, 3, 3> w_x;
    w_x << 0, -w(2), w(1),
            w(2), 0, -w(0),
            -w(1), w(0), 0;
    return w_x;
}

inline Eigen::MatrixXd quat_2_Rot(Eigen::Matrix<double, 4, 1> q) {
    Eigen::Matrix<double, 3, 3> q_x = skew_x(q.block(0, 0, 3, 1));
    Eigen::MatrixXd Rot = (2.0 * pow(q(3, 0),2) - 1.0) * Eigen::MatrixXd::Identity(3, 3) - 2.0 * q(3, 0) * q_x +
                          2.0 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());


    //cout << "Rot1- " << endl << Rot << endl << endl;
    //cout << "Rot2- " << endl << Eigen::MatrixXd::Identity(3, 3)-2*q(3,0)*q_x
    return Rot;
}


inline Eigen::Matrix<double, 4, 1> quat_multiply(Eigen::Matrix<double, 4, 1> q, Eigen::Matrix<double, 4, 1> p) {
    Eigen::Matrix<double, 4, 1> q_t;
    Eigen::Matrix<double, 4, 4> Qm;
    Qm.block(0, 0, 3, 3) = q(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skew_x(q.block(0, 0, 3, 1));
    Qm.block(0, 3, 3, 1) = q.block(0, 0, 3, 1);
    Qm.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();
    Qm(3, 3) = q(3, 0);
    q_t = Qm * p;
    if (q_t(3,0) <0){
        q_t*=-1;;
    }
    return q_t/q_t.norm();

}


inline Eigen::Matrix<double, 3, 1> vee(Eigen::Matrix<double, 3, 3> w_x) {
    Eigen::Matrix<double, 3, 1> w;
    w << w_x(2, 1), w_x(0, 2), w_x(1, 0);
    return w;
}

inline Eigen::Matrix<double, 3, 3> Exp(Eigen::Matrix<double, 3, 1> w) {

    Eigen::Matrix<double, 3, 3> w_x = skew_x(w);

    double theta = w.norm();

    Eigen::Matrix<double, 3, 3> R;

    if (theta ==0){
        R= Eigen::MatrixXd::Identity(3,3);
    }
    else{
        R= Eigen::MatrixXd::Identity(3, 3) + (sin(theta) / theta) * (w_x) + ((1 - cos(theta)) / pow(theta,2)) * w_x * w_x;}

    /*sleep(1);
    cout << "Input of Exp- " << endl;
    cout <<  w << endl << endl;

    cout << w_x << endl;

    cout << theta << endl;

    cout << "(sin(theta) / theta) * (w_x)- " << endl << (sin(theta) / theta) * (w_x) << endl << endl;

    cout << "((1 - cos(theta)) / pow(theta,2)) * w_x * w_x-" << endl << ((1 - cos(theta)) / pow(theta,2)) * w_x * w_x << endl << endl;

    cout << "Output of Exp- " << endl;
    cout << R << endl;

    sleep(1);*/


    return R;



}

inline Eigen::Matrix<double, 3, 3> Log(Eigen::Matrix<double, 3, 3> R) {

    Eigen::Matrix<double, 3, 3> w_x;

    double theta = acos(.5 * (R.trace() - 1));

    w_x = (theta / (2.0 * sin(theta))) * (R - R.transpose());

    if (R != Eigen::MatrixXd::Identity(3,3)) {

        return w_x;
    }
    else{
        return Eigen::MatrixXd::Zero(3,3);
    }

}

inline Eigen::Matrix<double,4,1> Inv(Eigen::Matrix<double,4,1> q){
    Eigen::Matrix<double,4,1> qinv;

    qinv.block(0,0,3,1)= -q.block(0,0,3,1);
    qinv(3,0)= q(3,0);
    return qinv;

}



