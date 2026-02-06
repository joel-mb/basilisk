/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "MJSite.h"
#include "MJBody.h"
#include "MJScene.h"
#include "MJSpec.h"

#include <Eigen/Geometry>

#include <sstream>

MJSite::MJSite(mjsSite* site, MJBody& body) : MJObject(site), body(body) {}

void MJSite::setPositionRelativeToBody(const Eigen::Vector3d& position)
{
    // Copy new position in the mjSpec
    std::copy_n(position.data(), 3, mjsObject->pos);

    // If id exists, then this site has already been configured with
    // an mjModel. Thus, update the position in said mjModel
    if (this->id.has_value())
    {
        auto model = this->body.getSpec().getMujocoModel();
        std::copy_n(position.data(), 3, model->site_pos + 3 * this->getId());

        // However, this process will make the kinematics stale
        this->body.getSpec().getScene().markKinematicsAsStale();
    }
}

void MJSite::setAttitudeRelativeToBody(const Eigen::MRPd& attitude)
{
    auto mat = attitude.toRotationMatrix();
    auto quat = Eigen::Quaterniond(mat);
    auto quatVec = Eigen::Vector4d{quat.w(), quat.x(), quat.y(), quat.z()};

    // Copy new quat in the mjSpec
    std::copy_n(quatVec.data(), 4, mjsObject->quat);

    // If id exists, then this site has already been configured with
    // an mjModel. Thus, update the quat in said mjModel
    if (this->id.has_value())
    {
        auto model = this->body.getSpec().getMujocoModel();
        std::copy_n(quatVec.data(), 4, model->site_quat + 4 * this->getId());

        // However, this process will make the kinematics stale
        this->body.getSpec().getScene().markKinematicsAsStale();
    }
}

void MJSite::writeFwdKinematicsMessage(mjModel* model, mjData* data, uint64_t CurrentSimNanos)
{
    writeStatesMsgPayload(model, data, CurrentSimNanos);
    writeRelativeStatesMsgPayload(model, data, CurrentSimNanos);
}

void MJSite::writeStatesMsgPayload(mjModel* model, mjData* data, uint64_t CurrentSimNanos)
{
    SCStatesMsgPayload payload;

    std::copy_n(data->site_xpos + 3 * this->getId(), 3, payload.r_BN_N);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rot{data->site_xmat +
                                                                 9 * this->getId()};
    Eigen::Map<Eigen::MRPd> mrpd{payload.sigma_BN};
    mrpd = rot;

    double res_N[6], res_B[6];
    mj_objectVelocity(model, data, mjOBJ_SITE, static_cast<int>(this->getId()), res_N, 0);
    mj_objectVelocity(model, data, mjOBJ_SITE, static_cast<int>(this->getId()), res_B, 1);

    std::copy_n(res_B, 3, payload.omega_BN_B);
    std::copy_n(res_N + 3, 3, payload.v_BN_N);

    this->stateOutMsg.write(&payload, this->body.getSpec().getScene().moduleID, CurrentSimNanos);
}

void MJSite::writeRelativeStatesMsgPayload(mjModel* model, mjData* data, uint64_t CurrentSimNanos)
{
    SCRelativeStatesMsgPayload payload;

    const int site_id = static_cast<int>(this->getId());
    const int body_id   = model->site_bodyid[site_id];
    const int parent_id = model->body_parentid[body_id];

    // Body position and rotation in the inertial frame N
    Eigen::Vector3d r_BN_N(data->site_xpos + 3 * site_id);
    Eigen::Matrix3d sigma_BN =
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            data->site_xmat + 9 * site_id);

    // Parent body position and rotation in the inertial frame N
    Eigen::Vector3d r_PN_N(data->xpos + 3 * parent_id);
    Eigen::Matrix3d sigma_PN =
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            data->xmat + 9 * parent_id);

    // Transform to parent frame
    Eigen::Vector3d r_BP_P = sigma_PN.transpose() * (r_BN_N - r_PN_N);
    Eigen::Matrix3d sigma_BP = sigma_PN.transpose() * sigma_BN;

    std::copy_n(r_BP_P.data(), 3, payload.r_BP_P);
    Eigen::Map<Eigen::MRPd> mrpd{payload.sigma_BP};
    mrpd = sigma_BP;

    this->relativeStateOutMsg.write(&payload, this->body.getSpec().getScene().moduleID, CurrentSimNanos);
}
