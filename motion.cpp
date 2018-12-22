#include "motion.hpp"

bool isIdentity(const Eigen::Matrix3d &M) {
	return (M*M.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-5);
}

bool isOrthogonal(const Eigen::Matrix3d &M) {
	return isIdentity(M) && M.determinant();
}

double mixedProduct(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z) {
    Eigen::Matrix3d m;
    m << x(0), x(1), x(2),
         y(0), y(1), y(2),
         z(0), z(1), z(2);

    return m.determinant();
}

std::pair<Eigen::Vector3d, double> A2AngleAxis(const Eigen::Matrix3d &A) {

	if(!isOrthogonal(A)) {
		std::cerr << "Given matrix A is not orthogonal!" << std::endl;
		exit(EXIT_FAILURE);
	}

	Eigen::Matrix3d diff = A-Eigen::Matrix3d::Identity();

	Eigen::Vector3d v1, v2;
	v1 << diff(0,0), diff(0,1), diff(0,2);
	v2 << diff(1,0), diff(1,1), diff(1,2);

	Eigen::Vector3d p = v1.cross(v2);
    p.normalize();

	Eigen::Vector3d u = v1;
    u.normalize();
	Eigen::Vector3d up = A*u;
    up.normalize();

    double phi = acos(u.dot(up));

    if(mixedProduct(u,up,p) < 0)
        p = -p;

	return std::pair<Eigen::Vector3d, double>(p, phi);
}

Eigen::Matrix3d Rodriguez(const Eigen::Vector3d &p, const double phi) {
	if(!p.isUnitary(1e-5)) {
		std::cerr << "Given vector is not unitary!" << std::endl;
		exit(EXIT_FAILURE);
	}

	Eigen::Matrix3d px;
	px << 0.0, -p(2), p(1),
		  p(2), 0.0, -p(0),
		 -p(1), p(0), 0.0;

	Eigen::Matrix3d A;
	A = p*p.transpose()+cos(phi)*(Eigen::Matrix3d::Identity()-p*p.transpose())+sin(phi)*px;

	return A;
}

Eigen::Vector3d A2Euler(const Eigen::Matrix3d &A) {

	if(!isOrthogonal(A)) {
		std::cerr << "Given matrix A is not orthogonal!" << std::endl;
		exit(EXIT_FAILURE);
	}

	Eigen::Vector3d angles;
	double phi, theta, psi;

	if(A(2,0) < 1) {
		if(A(2,0) > -1) {
			psi = atan2(A(1,0), A(0,0));
			theta = asin(-A(2,0));
			phi = atan2(A(2,1), A(2,2));
		} else {
			psi = atan2(-A(0,1), A(1,1));
			theta = M_PI/2.0;
			phi = 0.0;
		}
	} else {
		phi = 0.0;
		theta = -M_PI/2.0;
		psi = atan2(-A(0,1), A(1,1));
	}

	angles << phi, theta, psi;
	return angles;
}

Eigen::Matrix3d Euler2A(const Eigen::Vector3d &angles) {

	if (!(std::abs(angles(0)) >= 0 && std::abs(angles(0)) < 2*M_PI)) {
		std::cerr << "Phi must take value from [0, 2*PI)" << std::endl;
		exit(EXIT_FAILURE);
	}

	if (!(angles(1) >= -M_PI/2.0 && angles(1) <= M_PI/2.0)) {
		std::cerr << "Theta must take value from [-PI/2, PI/2]" << std::endl;
		exit(EXIT_FAILURE);
	}

	if (!(std::abs(angles(2)) >= 0 && std::abs(angles(2)) < 2*M_PI)) {
		std::cerr << "Psi must take value from [0, 2*PI)" << std::endl;
		exit(EXIT_FAILURE);
	}

	Eigen::Matrix3d Rx, Ry, Rz;

	double phi = angles(0);
	double theta = angles(1);
	double psi = angles(2);

	Rx << 1.0, 0.0, 0.0,
		  0.0, cos(phi), -sin(phi),
		  0.0, sin(phi), cos(phi);

	Ry << cos(theta), 0.0, sin(theta),
		  0.0, 1.0, 0.0,
		  -sin(theta), 0.0, cos(theta);

	Rz << cos(psi), -sin(psi), 0.0,
		  sin(psi), cos(psi), 0.0,
		  0.0, 0.0, 1.0;

	return Rz*Ry*Rx;
}

std::pair<Eigen::Vector3d, double> Q2AngleAxis(const Eigen::Quaterniond& q) {
	Eigen::Quaterniond q1 = q;

	q1.normalize();
	if(q1.w() < 0) {
		q1.w() = -q1.w();
		q1.x() = -q1.x();
		q1.y() = -q1.y();
		q1.z() = -q1.z();
	}

	double phi = 2*acos(q1.w());

	Eigen::Vector3d p;
	if(abs(q1.w()) == 1)
		p << 1,0,0;
	else {
		p << q1.vec()(0), q1.vec()(1), q1.vec()(2);
		p.normalize();
	}

	return std::pair<Eigen::Vector3d, double>(p, phi);
}

Eigen::Quaterniond AngleAxis2Q(const Eigen::Vector3d &p, const double phi) {
	if (!(std::abs(phi) >= 0 && std::abs(phi) < 2*M_PI)) {
		std::cerr << "Phi must take value from [0, 2*PI)" << std::endl;
		exit(EXIT_FAILURE);
	}

	double w = cos(phi/2);
	Eigen::Vector3d p1 = p.normalized();
	double x, y, z;
	x = sin(phi/2)*p1(0);
	y = sin(phi/2)*p1(1);
	z = sin(phi/2)*p1(2);
	return Eigen::Quaterniond(w, x, y, z);
}

Eigen::Quaterniond SlerpInterpolation(const Eigen::Quaterniond &q1,
                                      const Eigen::Quaterniond &q2,
                                      double tm, double t) {
	Eigen::Quaterniond _q1 = q1.normalized();
	Eigen::Quaterniond _q2 = q2.normalized();

	double alfa = _q1.dot(_q2);

	if(alfa < 0) {
		_q1.w() = -_q1.w();
		_q1.x() = -_q1.x();
		_q1.y() = -_q1.y();
		_q1.z() = -_q1.z();
		alfa = -alfa;
	}
	if(alfa > 0.95) {
		return _q1;
	}

	double phi = acos(alfa);
	Eigen::Quaterniond result_quaternion;

	double pom1 = sin(phi*(1-t/tm))/sin(phi);
	double pom2 = sin(phi*t/tm)/sin(phi);

	result_quaternion.w() = pom1*_q1.w()+pom2*_q2.w();
	result_quaternion.x() = pom1*_q1.x()+pom2*_q2.x();
	result_quaternion.y() = pom1*_q1.y()+pom2*_q2.y();
	result_quaternion.z() = pom1*_q1.z()+pom2*_q2.z();

	return result_quaternion;
}


void writeEigenVector3d(const std::string &name, const Eigen::Vector3d &v) {
	std::cout << "Vector " << name << " = (" << v(0) << ", " << v(1) << ", " << v(2) << ")." << std::endl;
}

void writeEigenMatrix3d(const std::string &name, const Eigen::Matrix3d &m) {
	std::cout << "Matrix " << name << std::endl;
	std::cout << m << std::endl;
}

void writeQuaternion(const std::string &name, const Eigen::Quaterniond &q) {
	std::cout << "Quaternion " << name << " = "
			  << q.vec()(0) << "*i + " << q.vec()(1) << "*j + " << q.vec()(2) << "*k + " << q.w() << std::endl;
}
