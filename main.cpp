#include "motion.hpp"
#include <GL/glut.h>

#define TIMER_ID 0
#define TIMER_INTERVAL 25

/* Declaratios of callback functions. */
static void on_keyboard(unsigned char key, int x, int y);
static void on_reshape(int width, int height);
static void on_timer(int value);
static void on_display(void);

/* Extra functions. */
static void draw_coosys();
Eigen::Quaterniond getQuaternion(const Eigen::Vector3d& angles);
static void update(double &x_t, double &y_t, double &z_t, double &phi, double &theta, double &psi);

/* Start and end coordinates of object. */
Eigen::Vector3d startCentar(0.0, 3.0, 0.0);

Eigen::Vector3d endCentar(10.0, 4.0, 2.0);

/* Start and end angles of object. */
Eigen::Vector3d startEuler(30.0, 45.0, 0.0);

Eigen::Vector3d endEuler(60.0, -45.0, 0.0);

/* Translation parameters. */
double x_t, y_t, z_t;

/* Rotation parameters. */
double phi, theta, psi;

/* Time passed. */
static double t;
static double total_time = 4;

/* Flag for knowing if animation is on. */
static int animation_ongoing;

int main(int argc, char **argv) {

	/* Initialization of GLUT. */
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    /* Create a window. */
    glutInitWindowSize(500, 450);
    glutInitWindowPosition(100, 100);
    glutCreateWindow(argv[0]);

    /* Registration of callback functions. */
    glutKeyboardFunc(on_keyboard);
    glutReshapeFunc(on_reshape);
    glutDisplayFunc(on_display);

    /* Initialization of OpenGL. */
    glClearColor(0.25, 0.25, 0.25, 0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    // x_t = startCentar[0];
    // y_t = startCentar[1];
    // z_t = startCentar[2];
    // phi = startEuler[0];
    // theta = startEuler[1];
    // psi = startEuler[2];
    x_t = 0;
    y_t = 0;
    z_t = 0;
    phi = 0;
    theta = 0;
    psi = 0;
    animation_ongoing = 0;
    t = 0;

    /* Program entering main loop. */
    glutMainLoop();
	return 0;
}

static void on_display(void) {

    /* Setting the color of all pixels on given background color. */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* Setting a visual point. */
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(
        12, 2, 12,
        6, 2, 0,
        0, 1, 0);

    /* Drawing ending coordinating system. */
    glPushMatrix();
        glTranslatef(endCentar[0], endCentar[1], endCentar[2]);
        glRotatef(endEuler[0], 1, 0, 0);
        glRotatef(endEuler[1], 0, 1, 0);
        glRotatef(endEuler[2], 0, 0, 1);
        draw_coosys();
    glPopMatrix();

    /* Update translation coordinates and rotation angles. */
    update(x_t, y_t, z_t, phi, theta, psi);

    /* Drawing moving coordinating system. */
    glPushMatrix();
        glTranslatef(x_t, y_t, z_t);
        glRotatef(phi, 1, 0, 0);
        glRotatef(theta, 0, 1, 0);
        glRotatef(psi, 0, 0, 1);
        draw_coosys();
    glPopMatrix();

    /* Drawing starting coordinating system. */
    glPushMatrix();
        glTranslatef(startCentar[0], startCentar[1], startCentar[2]);
        glRotatef(startEuler[0], 1, 0, 0);
        glRotatef(startEuler[1], 0, 1, 0);
        glRotatef(startEuler[2], 0, 0, 1);
        draw_coosys();
    glPopMatrix();

    /* Setting new frame on the window */
    glutSwapBuffers();
}

Eigen::Quaterniond getQuaternion(const Eigen::Vector3d& angles) {
    Eigen::Matrix3d A = Euler2A(angles);
    std::pair<Eigen::Vector3d, double> res = A2AngleAxis(A);
    Eigen::Quaterniond q = AngleAxis2Q(res.first, res.second);
    return q;
}

static void update(double &x_t, double &y_t, double &z_t, double &phi, double &theta, double &psi) {

    Eigen::Quaterniond q1 = getQuaternion((startEuler)*M_PI/180);
    Eigen::Quaterniond q2 = getQuaternion((endEuler)*M_PI/180);
    Eigen::Quaterniond q_res = SlerpInterpolation(q1, q2, total_time, t);

    std::pair<Eigen::Vector3d, double> res = Q2AngleAxis(q_res);
    Eigen::Matrix3d A = Rodriguez(res.first, res.second);
    Eigen::Vector3d angles = A2Euler(A);

    phi = angles[0]*180/M_PI;
    theta = angles[1]*180/M_PI;
    psi = angles[2]*180/M_PI;

    x_t = (1-t/total_time)*startCentar[0]+t/total_time*endCentar[0];
    y_t = (1-t/total_time)*startCentar[1]+t/total_time*endCentar[1];
    z_t = (1-t/total_time)*startCentar[2]+t/total_time*endCentar[2];
}

static void on_keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 27:
        /* Program ends. */
        exit(0);
        break;

    case 'g':
    case 'G':
        /* Animation starts. */
        if (!animation_ongoing) {
            glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID);
            animation_ongoing = 1;
        }
        break;

    case 's':
    case 'S':
        /* Animation stops. */
        animation_ongoing = 0;
        break;

    case 'r':
    case 'R':
        /* Animation resets. */
        t = 0;
        if (!animation_ongoing) {
            glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID);
            animation_ongoing = 1;
        }
        break;

    }
}

static void on_timer(int id) {
    if (id != TIMER_ID) return;

    /* Animation stops when time is up. */
    if(t > total_time){
        animation_ongoing = 0;
    }
    else{
         t += 0.08;
    }

    /* Forsira se ponovno iscrtavanje prozora. */
    glutPostRedisplay();

    /* Po potrebi se ponovo postavlja tajmer. */
    if (animation_ongoing) {
        glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID);
    }

}

static void on_reshape(int width, int height) {
    /* Setting viewport. */
    glViewport(0, 0, width, height);

    /* Setting parameters of projection. */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, (float) width / height, 1, 1500);
}


static void draw_coosys() {
    glLineWidth(GLfloat(3.0));
    glBegin(GL_LINES);

    glColor3f(1, 0, 0);
    glVertex3f(1, 0, 0);
    glVertex3f(0, 0, 0);

    glColor3f(0, 1, 0);
    glVertex3f(0, 1, 0);
    glVertex3f(0, 0, 0);

    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 1);
    glVertex3f(0, 0, 0);

    glEnd();
}
