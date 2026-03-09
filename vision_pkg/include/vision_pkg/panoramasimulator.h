#ifndef VISION_PKG_PANORAMASIMULATOR_H
#define VISION_PKG_PANORAMASIMULATOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <functional>

#include <EGL/egl.h>
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "vision_pkg/fisheye.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class PanoramaSimulator
{
public:
    using PanoramaReadyCallback = std::function<void(const cv::Mat&)>;

    PanoramaSimulator();
    ~PanoramaSimulator();

    void initialize();

    void setPerspective(float x, float y);
    void setPerspective();

    void processFrames(const cv::Mat &frame1, const cv::Mat &frame2);
    cv::Mat captureRenderedFrame();

    void setPanoramaReadyCallback(PanoramaReadyCallback cb) { m_callback = std::move(cb); }

    float getCameraDistance() const { return cameraDistance; }
    float getCameraAngleX() const { return cameraAngleX; }
    float getCameraAngleY() const { return cameraAngleY; }

private:
    void initializeEGL();
    void initializeGL();
    void drawSphere();
    void loadTexture(const cv::Mat &image);
    cv::Mat combineImages(const cv::Mat &A, const cv::Mat &B);
    void updateTexture();

    void initShaders();
    void initSphereGeometry();

    // EGL context
    EGLDisplay eglDisplay = EGL_NO_DISPLAY;
    EGLContext eglContext = EGL_NO_CONTEXT;
    EGLSurface eglSurface = EGL_NO_SURFACE;
    GLuint fbo = 0;
    GLuint rboColor = 0;
    GLuint rboDepth = 0;

    float cameraDistance = 0.01f;
    float cameraAngleX = 0.0f;
    float cameraAngleY = 0.0f;
    float fovy = 60.0f;
    float sphereAngleX = 0.0f;
    float sphereAngleY = 90.0f;
    float sphereAngleZ = 90.0f;
    const float yawSpeed = 12.36f;
    const float pitchSpeed = 12.36f;
    GLuint textureID = 0;

    GLuint vaoID = 0, vboID = 0, iboID = 0;
    GLuint shaderProgram = 0;
    GLsizei indexCount = 0;

    int windowWidth = 540;
    int windowHeight = 270;

    cv::Mat m_frame1, m_frame2;
    glm::mat4 projectionMatrix;
    glm::mat4 viewMatrix;

    PanoramaReadyCallback m_callback;
};

#endif // VISION_PKG_PANORAMASIMULATOR_H
