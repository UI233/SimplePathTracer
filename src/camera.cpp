#include "mycamera.h"

#include <Eigen/LU>

namespace simple_pt
{
Camera::Camera(float fovy, const Eigen::Vector3f& eye, const Eigen::Vector3f& lookat, const Eigen::Vector3f& up, size_t p_width, size_t p_height, float z_near, float z_far) :
    width(p_width),
    height(p_height),
    origin(eye)
    {
        //todo: check pers matrix
        Eigen::Matrix4f view, pers_dir;
        auto up_n = up.normalized();
        Eigen::Vector3f forward = (lookat - eye).normalized();
        Eigen::Vector3f side = forward.cross(up).normalized();
        // Eigen::Vector3f cam_up = forward.cross(side);
        view << side.x(), up_n.x(), forward.x(), 0.0f,
            side.y(), up_n.y(), forward.y(), 0.0f,
            side.z(), up_n.z(), forward.z(), 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;
        float aspect_ratio = (float)width / height;
        fovy = fovy / 180.0 * M_PI;
        float inv_tan = 1.0f / std::tan(fovy * 0.5);
        pers_dir << inv_tan, 0.0f, 0.0f, 0.0f,
            0.0f, inv_tan, 0.0f, 0.0f,
            0.0f, 0.0f, z_far / (z_far - z_near), -z_far*z_near / (z_far - z_near),
            0.0f, 0.0f, 1.0f, 0.0f;
        Eigen::Matrix4f inv = pers_dir.inverse();
        this->pers = view * inv;
    }

/**
 * Given a pixel coordinate (x, y), generate a ray that starts at the camera origin and goes through
 * the pixel
 * 
 * @param x The x coordinate of the pixel in the image.
 * @param y the y coordinate of the pixel in the image
 * 
 * @return A ray
 */
Ray Camera::generateRay(float x, float y) const {
    // todo: test this function
    float x_i = x / (float) width, y_i = y / (float) height;
    float aspect_ratio = (float)width / height;
    x_i = x_i * 2.0f * aspect_ratio - aspect_ratio;
    y_i = y_i * 2.0f - 1.0f;
    Eigen::Vector4f dir(x_i, y_i, 1.0f, 1.0f);
    dir = pers * dir;
    dir /= dir[3];
    Eigen::Vector3f dir_h(dir[0], dir[1], dir[2]);
    dir_h -= origin;
    return Ray(origin, dir_h.normalized());
}
} // namespace simple_pt

